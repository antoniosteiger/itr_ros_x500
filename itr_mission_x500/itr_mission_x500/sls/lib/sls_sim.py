import numpy as np
import cvxpy as cp
from scipy.signal import cont2discrete
import time

from itr_comms_x500 import Comms
from sls.sls import (
    SLS,
    make_delay_matrices_chain,
    error_augment_all,
    setpoint_augment_all,
    calc_RTT_max,
    delay_augment_all,
    build_phi_u_sparsity,
    horizon_augment_traj
)
from itr_description_x500 import LTI, X500, Quadcopter
from itr_mission_x500 import initialize, launch, make_comms, make_fsm, make_mission
from itr_statemachine_x500 import (
    OC_MISSION_FINISHED,
    Arm,
    Hover,
    MissionState,
    Takeoff,
)
from sls.trajectory import (
    generate_helix_trajectory,
    generate_waypoint_trajectory,
    generate_circle_trajectory,
    generate_figure8_trajectory
)
from sim.gz import GazeboControlNode

from px4_msgs.msg import VehicleStatus

# SOME HACKY PARAMETERS
MASS = 2.0 # from x500_base model
GRAVITY = 9.807232
HOVER_THRUST = MASS * GRAVITY
ARM_Y = 0.174
ARM_X = 0.174

MOTOR_CONSTANT   = 8.54858e-6   # [N / (rad/s)²]  F = k * omega²
MOMENT_CONSTANT  = 0.016        # [m]              tau_z = F * cm  (you already had this)
MAX_ROT_VEL      = 1000.0       # [rad/s]          maxRotVelocity from SDF
MIN_ROT_VEL      = 0.0          # idle speed (0 for SITL unless set otherwise)
N_MOTORS         = 4
HOVER_THROTTLE = -0.72 # empiric
MAX_SINGLE_THRUST = MOTOR_CONSTANT * MAX_ROT_VEL**2          # [N]
MAX_TOTAL_THRUST  = N_MOTORS * MAX_SINGLE_THRUST              # [N]
# Max differential torque: two motors on one side at max, two at zero
MAX_TAU_X = ARM_Y * 2 * MAX_SINGLE_THRUST   # roll
MAX_TAU_Y = ARM_X * 2 * MAX_SINGLE_THRUST   # pitch
MAX_TAU_Z = MOMENT_CONSTANT * 2 * MAX_SINGLE_THRUST  # yaw (two CW vs two CCW)

# MASS = 2.0 # from x500_base model
# GRAVITY = 9.807232
# HOVER_THRUST = MASS * GRAVITY
# MAX_THRUST = np.abs(HOVER_THRUST / HOVER_THROTTLE)
# ARM_Y = 0.174
# ARM_X = 0.174
# MOMENT_CONST = 0.016
# TAU_X_MAX = ARM_Y * (MAX_THRUST / 2) * 50
# TAU_Y_MAX = ARM_X * (MAX_THRUST / 2) * 50
# TAU_Z_MAX = MOMENT_CONST * (MAX_THRUST / 2) * 50

#SIM SETTINGS
TSTEP = 0.02
TSIM = int(5.0 / TSTEP)

SIM_SLOW_DOWN = 1

# MPC/SLS PROBLEM SETTINGS
HORIZON = 60
SOLVER = cp.CLARABEL # or cp.CLARABEL
MOSEK_PARAMS = {
    "MSK_IPAR_NUM_THREADS": 12,
    "MSK_DPAR_INTPNT_CO_TOL_REL_GAP": 1e-6,
    "MSK_DPAR_INTPNT_CO_TOL_PFEAS": 1e-6,
    "MSK_DPAR_INTPNT_CO_TOL_DFEAS": 1e-7,
    "MSK_IPAR_PRESOLVE_USE": "MSK_PRESOLVE_MODE_ON"
}

CLARABEL_PARAMS = {
    "tol_gap_abs": 1e-4,
    "tol_gap_rel": 1e-4,
    "tol_feas": 1e-4
}
MAX_ITER = 1000
INPUT_WEIGHTS = np.diag([0.5, 100.0, 100.0, 100.0])
STATE_WEIGHTS = np.diag([17.0, 17.0, 17.0, 7.5, 7.5, 7.5, 5.1, 5.1, 5.1, 4.5, 4.5, 4.5])

# Trajectory Settings
RADIUS = 1.0
PERIOD = 40.0
START = (0.0, 0.0, -1.0)
WAYPOINTS = np.array([
    [0.0, 1.0, -1.0],
    [0.0, 1.0, -1.0]
])
TIMES = [0.0, TSIM*TSTEP]
TRAJ = generate_waypoint_trajectory(WAYPOINTS, times=TIMES, dt=TSTEP, Tsim=TSIM, method="linear")
# TRAJ = generate_figure8_trajectory(width=2.0, height=1.0, period=PERIOD, Tsim=TSIM, dt=TSTEP, start_pos=START)
# TRAJ = generate_helix_trajectory(radius=RADIUS, climb_rate=0.025, period=PERIOD, Tsim=TSIM, dt=TSTEP, start_pos=START)
# TRAJ = generate_circle_trajectory(radius=RADIUS, period=PERIOD, Tsim=TSIM, dt=TSTEP, start_pos=START)
# TRAJ[3:, :] = 0

# Input and State Constraint Settings
ANGLE_LIMIT = np.deg2rad(15)
X_MIN =[
    None, None, None,
    None, None, None,
    -ANGLE_LIMIT, -ANGLE_LIMIT, None,
    None, None, None
]

X_MAX =[
    None, None, None,
    None, None, None,
    ANGLE_LIMIT, ANGLE_LIMIT, None,
    None, None, None
]

U_MIN = [
    -MAX_TOTAL_THRUST,
    -MAX_TAU_X,
    -MAX_TAU_Y,
    -MAX_TAU_Z
]

U_MAX = [
    MAX_TOTAL_THRUST,
    MAX_TAU_X,
    MAX_TAU_Y,
    MAX_TAU_Z
]


def rmse(array):
    array_len = len(array)
    array_sum = np.sum(array ** 2)

    rmse = 1/(array_len + 1) * array_sum

    return rmse

def normalize_thrust(thrust_delta_N: float) -> float:
    """
    Convert delta thrust [N] (output of SLS, linearized around hover)
    to PX4 normalized thrust in [-1, 0] (NED: negative = up).
    
    PX4 expects normalized motor velocity, not normalized force.
    thrust_total = hover_thrust + thrust_delta
    """
    # thrust_total = HOVER_THRUST + thrust_delta_N
    # thrust_total = np.clip(thrust_total, 0, MAX_TOTAL_THRUST)
    #
    # # Per-motor thrust at this collective command
    # thrust_per_motor = thrust_total / N_MOTORS
    #
    # # Invert F = k*omega² to get omega, then normalize to [0,1]
    # omega = np.sqrt(np.clip(thrust_per_motor / MOTOR_CONSTANT, 0, None))
    # normalized = (omega - MIN_ROT_VEL) / (MAX_ROT_VEL - MIN_ROT_VEL)
    # normalized = np.clip(normalized, 0.0, 1.0)
    normalized = thrust_delta_N / MAX_TOTAL_THRUST
    normalized += HOVER_THROTTLE

    return normalized  # negative = up in NED

def normalize_torques(torques: np.ndarray) -> np.ndarray:
    tau_x, tau_y, tau_z = torques
    return np.array([
        np.clip(tau_x / MAX_TAU_X, -1.0, 1.0),
        np.clip(tau_y / MAX_TAU_Y, -1.0, 1.0),
        np.clip(tau_z / MAX_TAU_Z, -1.0, 1.0),
    ])

# We will not use the ControllerState here, because it is real-time and periodic (real-time)
# We instead wrap the MissionState to add a more sim focused mission state
class SLS_Mission(MissionState):
    def __init__(
        self,
        oc_next_state: str,
        comms: Comms,
        sim: GazeboControlNode,
        Tsim: int,
        Tstep: float,
        A: np.ndarray,
        B: np.ndarray,
        Q: np.ndarray | None = None,
        R: np.ndarray | None = None,
        Delta: np.ndarray | None = None,
        Phi_u_sparsity: np.ndarray | None = None,
        traj: np.ndarray | None = None,
        x_min: list[float | None] | None = None,
        x_max: list[float | None] | None = None,
        u_min: list[float | None] | None = None,
        u_max: list[float | None] | None = None,
        error_dim: int = 0,
        setpoint_dim: int = 0,
        horizon: int = 10,
        solver: str = cp.CLARABEL,
        max_iter: int = 1000,
        solver_params: dict | None = None,
        debug: bool = False,
    ):
        """
        Initialize the SLS simulation loop.

        Args:
            oc_next_state: YASMIN state machine outcome, i.e. the state to transition to after simulation completes.
            comms: Communication layer for interfacing with the drone.
            sim: Communication layer for interfacing with the Gazebo physics simulator.
            Tsim: Number of simulation steps to run.
            Tstep: Simulation time step in seconds (e.g. 0.1). Used for discretization.
            A: Continuous state transition matrix.
            B: Continuous input matrix.
            Delta: Delay-weighted adjacency matrix of the network.
            Q: State weighting matrix. Defaults to identity if None.
            R: Input weighting matrix. Defaults to identity if None.
            Delta: Delay-weighted adjacency matrix of the network.
            Phi_u_sparsity: Binary matrix setting the structure of the controller.
            traj: Reference trajectory of shape (Nx, Tsim). Defaults to zero if None.
            x_min: Lower bounds on states. Each element is a float or None (unconstrained).
            x_max: Upper bounds on states. Each element is a float or None (unconstrained).
            u_min: Lower bounds on inputs. Each element is a float or None (unconstrained).
            u_max: Upper bounds on inputs. Each element is a float or None (unconstrained).
            error_dim: How many error states to add to the state vector.
            setpoint_dim: How many setpoint inputs to add to the input vector.
            horizon: Prediction horizon (number of steps, e.g. 10).
            solver: CVXPY solver to use (e.g. cp.MOSEK, cp.CLARABEL).
            max_iter: Maximum number of solver iterations.
            solver_params: Dictionary of solver-specific parameters (e.g. tolerances).
            debug: Enable debug output.
        """
        super().__init__(oc_next_state)

        self.comms = comms
        self.sim = sim
        self._debug = debug
        self._input_type = "thrust_and_torque"

        self.Tstep = Tstep
        self.Tsim = Tsim
        self.step = 0

        self.Nx_orig = A.shape[0]
        self.Nu_orig = B.shape[1]
        self.Nx_e = self.Nx_orig + error_dim
        self.Nu_e = self.Nu_orig + error_dim
        self.Nx_s = self.Nx_e + setpoint_dim
        self.Nu_s = self.Nu_e + setpoint_dim
        x_min = x_min if x_min is not None else []
        x_max = x_max if x_max is not None else []
        u_min = u_min if u_min is not None else []
        u_max = u_max if u_max is not None else []


        self.horizon = horizon

        if Delta is None:
            Delta = np.array([
                [0, 0, 0],
                [0, 0, 0],
                [0, 0, 0]
            ])
        if Phi_u_sparsity is None:
            Phi_u_sparsity = np.ones((self.horizon, self.Nu_orig, self.Nx_orig))

        Delta_s, Delta_a = make_delay_matrices_chain(self.Nx_orig, self.Nu_orig, Delta)
        traj = horizon_augment_traj(traj, horizon) # Augment for last sim step
        rtt_max = calc_RTT_max(Delta_s, Delta_a)
        self.Nx_d = self.Nx_s * (rtt_max + 1)
        self.Nu_d = self.Nu_s

        ## Error-Augment System
        # TODO: This can be made nicer by categorizing the arguments:
        # System, Trajectory, Network Information, constraints, optionals
        if error_dim > 0:
            A, B, Q, R, traj, Phi_u_sparsity, Delta_a, Delta_s, x_min, x_max = error_augment_all(
                error_dim,
                A, B, Q, R,
                traj,
                Phi_u_sparsity, Delta_a, Delta_s,
                x_min, x_max,
                error_weight=0.0,
                ref_weight=0.0,
                traj_augment_style="zeros",
                sparse_augment_style="zeros",
                delta_augment_style="repeat"
            )

        ## Setpoint Augment System
        if setpoint_dim > 0:
            A, B, Q, R, traj = setpoint_augment_all(
                setpoint_dim,
                A, B, Q, R,
                traj,
                error_weight=0.1,
                setpoint_weight=0.5,
                traj_augment_style="zeros",
            )

        # DISCRETIZE
        Cdyn = np.eye(A.shape[0])
        Ddyn = np.zeros((A.shape[0], B.shape[1]))
        A, B, _, _, _= cont2discrete((A, B, Cdyn, Ddyn), self.Tstep, method="euler")

        # Delay Augment
        if rtt_max > 0:
            A, B, Q, R, traj = delay_augment_all(
                rtt_max,
                A, B, Q, R,
                traj,
                traj_augment_style="shift",
                Q_augment_style="scaled",
                Q_augment_scale=0.0
            )

        # Build sparsity in yolo fashion
        if error_dim > 0 and setpoint_dim > 0:
            Phi_u_sparsity = build_phi_u_sparsity(Nx_orig=self.Nx_orig, Nu_orig=self.Nu_orig, error_dim=error_dim, setpoint_dim=setpoint_dim, D=Delta, horizon=horizon)

        # Augmentation Pipeline finished
        self.A = A
        self.B = B
        self.traj = traj

        # Create Controller
        self.sls = SLS(
            A,
            B,
            Q,
            R,
            Phi_u_sparsity,
            x_min=x_min,
            x_max=x_max,
            u_min=u_min,
            u_max=u_max,
            horizon=horizon,
            solver=solver,
            max_iter=max_iter,
            solver_params=solver_params,
            debug=debug
        )
        
        # Set up data storage
        self.x_VAL = np.zeros((A.shape[0], Tsim+1))
        self.u_VAL = np.zeros((B.shape[1], Tsim))
        self.Phi_x_VAL = np.zeros((A.shape[0], A.shape[0], Tsim))
        self.Phi_u_VAL = np.zeros((B.shape[1], A.shape[0], Tsim))
        self.te_VAL = np.zeros((Tsim))
        self.ue_VAL = np.zeros((Tsim))
        self.te_rms = 0.0
        self.ue_rms = 0.0

    def task(self):
        # This main task is blocking and is called by the State Machine

        # Prime offboard mode
        self._log("Priming Offboard Mode")
        while self.comms.get_status()["nav"] != 14:
            self.sim.pause()
            self.comms.offboard_keepalive(self._input_type)
            self.comms.cmd_offboard_mode()
            self.comms.offboard_keepalive(self._input_type)
            self.comms.send_torque_setpoint(np.array([0.0, 0.0, 0.0]))
            self.comms.send_thrust_setpoint(HOVER_THROTTLE)
            navstate = self.comms.get_status()["nav"]
            # print(f"Navstate: {navstate}")
            time.sleep(0.1)
            self.sim.resume()
        
        # Get starting position and initialize
        obs = self._get_observation()
        obs_padded = np.zeros(self.Nx_s)
        obs_padded[:self.Nx_orig] = obs
        self.x_VAL[:, 0] = np.tile(obs_padded, self.Nx_d // self.Nx_s)

        # Shift trajectory to the starting position
        # offset = obs[:3] - self.traj[:3, 0]
        # traj_shifted = self.traj.copy()
        # traj_shifted[:3, :] += offset[:, np.newaxis]
        # self.traj = traj_shifted
        
        # Main sim loop
        for k in range(self.Tsim):
            # Pause the simulation
            self.sim.pause()

            # Tell PX4 to stay in offboard mode
            self.comms.offboard_keepalive(self._input_type)
            self.comms.cmd_offboard_mode()
            self.comms.offboard_keepalive(self._input_type)

            # Run the actual control task
            self._ctrl_task()

            # Resume sim for one timestep
            self.sim.resume()
            time.sleep(self.Tstep / SIM_SLOW_DOWN)

            self.step += 1

        self.te_rms = rmse(self.te_VAL)
        self.ue_rms = rmse(self.ue_VAL)

        return self.oc_next_state

    def _ctrl_task(self):
        # Wrap the controller in the offboard-mode enable logic
        # Also wrap in Gazebo simulation start/stop logic
        # ros2 service call /world/itr/control ros_gz_interfaces/srv/ControlWorld "{world_control: {pause: false}}"

        # Get measurement
        obs = self._get_observation()
        self.x_VAL[:self.Nx_orig, self.step] = obs
        obs = self.x_VAL[:, self.step]

        # Get reference (trajectory)
        ref = self._get_reference()
        
        # Compute control input
        u = self._ctrl(ref, obs)
        # Set ref (trajectory) part in u from error augmentation:
        u[self.Nu_orig:self.Nu_e] = self.traj[:self.Nu_e - self.Nu_orig, self.step]
        # store u
        self.u_VAL[:, self.step] = u

        # Get observation. Populate observations of delayed states by using delay-augmented dynamics
        self.x_VAL[:, self.step + 1] = self.A @ self.x_VAL[:, self.step] + self.B @ self.u_VAL[:, self.step]

        # Compute real tracking error:
        # TODO: allow passing a position-only trajectory and use traj dim here instead of hardcoded 3
        self.te_VAL[self.step] = np.linalg.norm(ref[:3, 0] - obs[:3])
        # Compute control effort:
        self.ue_VAL[self.step] = np.linalg.norm(u[:self.Nu_orig])
        
        # Apply control input
        self._apply_input(u)


    def _ctrl(self, ref, obs):
        u, Phi_x, Phi_u = self.sls(ref, obs)
        self.Phi_x_VAL[:, :, self.step] = Phi_x
        self.Phi_u_VAL[:, :, self.step] = Phi_u

        # u = np.zeros(self.Nu_d)
        # time.sleep(0.1)

        if self._debug:
            self._log(f"Control Input: {u}")

        self._log(f"Control Input: {u}")


        return u

    def _get_observation(self):
        pos = self.comms.get_position() # From Mocap (Gazebo), pre-converted to NED
        vel = self.comms.get_velocity() # From PX4, already in NED
        orientation = self.comms.get_orientation_euler() # From Mocap (Gazebo), pre-converted to NED
        body_rates = self.comms.get_angular_velocity() # From PX4, already in NED

        obs = np.concatenate([pos, vel, orientation, body_rates])

        # if self._debug:
        #     self._log(f"Observation: {obs}")
        self._log(f"Observation: {obs}")

        return obs

    def _get_reference(self):
        traj_section = self.traj[:, self.step : self.step + self.horizon]
        ref = traj_section

        # if self._debug:
        #     self._log(f"Current Trajectory Target: {ref[:self.Nx_orig, 0]}")

        self._log(f"Current Trajectory Target: {ref[:self.Nx_orig, 0]}")

        

        return ref

    def _apply_input(self, input):
        input = input[:self.Nu_orig] # Only use the actual inputs, no augmentations

        thrust_normalized = 0.0
        torques_normalized = np.zeros(3)
        
        thrust, tau_x, tau_y, tau_z = input

        thrust_normalized = normalize_thrust(thrust) # negative means up for PX4 (NED)
        # thrust_normalized += HOVER_THROTTLE # Linearization around hover
        torques_normalized = normalize_torques(np.array([tau_x, tau_y, tau_z]))
        # torques_normalized[2] *= -1

        if self._debug:
            self._log(f"Normalized Control Input: {np.concatenate([[thrust_normalized], torques_normalized])}")

        self._log(f"Normalized Control Input: {np.concatenate([[thrust_normalized], torques_normalized])}")


        self.comms.send_torque_setpoint(torques_normalized)
        self.comms.send_thrust_setpoint(thrust_normalized)

        return
    
    def _log(self, msg: str):
        print(f"\033[34m[ITR_CONTROLLER]: {msg} \033[0m")


def main() -> None:
    initialize(debug=True)

    # Create communication layer
    comms = make_comms()

    # Create physics sim management layer
    sim = GazeboControlNode()
    
    # Plant Model
    model = Quadcopter(X500, TSTEP)
    A = model.A # Continuous
    B = model.B # Continuous

    # Network Model
    Delta = np.array([ # delay weighted adjacency matrix
        [0, 0, 0],
        [0, 0, 0],
        [0, 0, 0],
    ])

    sls_mission = SLS_Mission(
        OC_MISSION_FINISHED,
        comms,
        sim,
        TSIM,
        TSTEP,
        A,
        B,
        Q = STATE_WEIGHTS,
        R = INPUT_WEIGHTS,
        Delta = Delta,
        Phi_u_sparsity = None,
        traj = TRAJ,
        x_min = X_MIN,
        x_max = X_MAX,
        # u_min = U_MIN,
        # u_max = U_MAX,
        # error_dim = 6,
        # setpoint_dim = 6,
        horizon = HORIZON,
        solver = SOLVER,
        max_iter = MAX_ITER,
        solver_params = CLARABEL_PARAMS,
        debug = False
    )

    mission = make_mission()
    mission.add_state(Arm("armed", comms), "ARM", "armed", "TAKEOFF")
    mission.add_state(Takeoff("took off", comms), "TAKEOFF", "took off", "HOVER")
    mission.add_state(Hover("stable", comms, 2), "HOVER", "stable", "SLS")
    mission.add_state(
        sls_mission,
        "SLS",
        OC_MISSION_FINISHED,
        OC_MISSION_FINISHED,
    )

    fsm = make_fsm(mission, comms)

    launch(fsm, comms, sim)


if __name__ == "__main__":
    main()
