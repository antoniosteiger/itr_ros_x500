from itr_mission_x500.scripts.sim.gz import GazeboControlNode
import numpy as np
import cvxpy as cp
from lib.sls import SLS, solve_sls_fir
from lib.px4 import normalize_thrust, normalize_torques, get_state, apply_thrust_and_torque
from lib.trajectory import horizon_augment_traj, generate_helix_trajectory, generate_waypoint_trajectory
from lib.plot import add_matrix, show_plots
from itr_description_x500 import Quadcopter, X500 
from itr_comms_x500 import Comms
from itr_statemachine_x500 import (
    OC_MISSION_FINISHED,
    Arm,
    Hover,
    MissionState,
    Takeoff,
    SimControllerState,
    RealtimeControllerState
)
from itr_mission_x500 import (
    initialize,
    make_mission,
    make_fsm,
    make_comms,
    make_sim,
    launch
)
from itr_controller_x500 import Controller

TSIM_DURATION = 20.0  # Total simulation duration in seconds
TSTEP_OUTER = 0.025 # IMPORTANT
TSTEP_INNER = 0.01 # IMPORTANT
TSIM_OUTER = int(TSIM_DURATION / TSTEP_OUTER)  # Outer loop steps (200)
TSIM_INNER = int(TSIM_DURATION / TSTEP_INNER)  # Inner loop steps (2000)
ONLINE_HORIZON = 20
OFFLINE_HORIZON = 50

RADIUS = 2.0
PERIOD = 40.0
CLIMB_RATE = 0.02
WAYPOINTS = np.array([
    [0.0, 1.0, -1.0],
    [0.0, 1.0, -1.0]
])
TIMES = [0.0, TSIM_INNER*TSTEP_INNER]
START = (0.0, 0.0, -1.0)  # NED frame: z=-2 means 2m altitude
# TRAJ = generate_helix_trajectory(
#     radius=RADIUS,
#     climb_rate=CLIMB_RATE,
#     period=PERIOD,
#     Tsim=TSIM_OUTER, # IMPORTANT
#     dt=TSTEP_OUTER, # IMPORTANT
#     start_pos=START,
#     nx_total=12
# )
TRAJ = generate_waypoint_trajectory(WAYPOINTS, times=TIMES, dt=TSTEP_OUTER, Tsim=TSIM_OUTER, method="linear")

CLARABEL_PARAMS = {
    "tol_gap_abs": 1e-4,
    "tol_gap_rel": 1e-4,
    "tol_feas": 1e-4
}

# Input and State Constraint Settings
ANGLE_LIMIT = np.deg2rad(20)
X_MIN =[
    None,
    None,
    None,
    -ANGLE_LIMIT,
    -ANGLE_LIMIT,
    None,
    None,
    None,
    None,
    None,
    None,
    None
]

X_MAX = [
    None,
    None,
    None,
    ANGLE_LIMIT,
    ANGLE_LIMIT,
    None,
    None,
    None,
    None,
    None,
    None,
    None
]

MASS = 2.0 # from x500_base model
GRAVITY = 9.807232
HOVER_THRUST = MASS * GRAVITY
ARM_Y = 0.174
ARM_X = 0.174
MOTOR_CONSTANT   = 8.54858e-6   # [N / (rad/s)²]  F = k * omega²
MOMENT_CONSTANT  = 0.016
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
U_MIN = [
    -MAX_TOTAL_THRUST,
    -MAX_TAU_X,
    -MAX_TAU_Y,
    -MAX_TAU_Z
]

U_MAX = [
    MASS * GRAVITY, # NED
    MAX_TAU_X,
    MAX_TAU_Y,
    MAX_TAU_Z
]


class OfflineSLS(Controller):
    def __init__(
        self,
        comms,
        K_sls,
        traj,
        debug = False
    ):
        super().__init__()
        self.K_sls = K_sls
        self.traj = traj
        self.step = 0
        self.debug = False
        self.comms = comms
        self.debug = debug

    def __call__(self, ref, obs):
        # if self.step == 0:
        #     # Shift trajectory to the starting position
        #     offset = obs[:3] - self.traj[:3, 0]
        #     traj_shifted = self.traj.copy()
        #     traj_shifted[:3, :] += offset[:, np.newaxis]
        #     self.traj = traj_shifted

        u = self.K_sls @ (obs - ref)

        self.step += 1

        return u
    
    def get_reference(self):
        ref = self.traj[:, self.step]
        return ref
    
    def get_observation(self):
        obs = get_state(self.comms, debug=False)
        return obs

    def is_finished(self):
        if self.step >= self.traj.shape[1] - 1:
            return True
        else:
            return False

    def apply_input(self, input):
        thrust = normalize_thrust(input[0])
        torques = normalize_torques(input[1:])
        input = np.concatenate([[thrust], torques])
        # print(f"[ITR_CONTROLLER]: Norm. Input: {input}")
        apply_thrust_and_torque(self.comms, input, debug=False)


class OnlineSLS(Controller):
    def __init__(
        self,
        comms,
        sls,
        traj,
        debug=False
    ):
        super().__init__()
        self.comms = comms
        self.sls = sls
        self.horizon = sls.horizon
        self.traj = horizon_augment_traj(traj, sls.horizon)
        self.step = 0
        self.debug = debug

    def __call__(self, ref, obs):
        u, _, _ = self.sls(ref, obs)
        self.step += 1
        return u

    def get_reference(self):
        ref_window = self.traj[:, self.step:self.step+self.horizon]
        return ref_window

    def get_observation(self):
        obs = get_state(self.comms, debug=self.debug)
        return obs

    def is_finished(self):
        if self.step >= self.traj.shape[1]:
            return True
        else:
            return False
    
    # Only works if used without offline_sls
    def apply_input(self, input):
        thrust = normalize_thrust(input[0])
        torques = normalize_torques(input[1:])
        input = np.concatenate([[thrust], torques])
        apply_thrust_and_torque(self.comms, input, debug=self.debug)

class ControllerWrapper(Controller):
    def __init__(
        self,
        comms: Comms,
        sls_offline,
        sls_online,
        tstep_inner = 0.01,
        tstep_outer = 0.05,
        debug = False
    ):
        super().__init__()
        self.ratio = int(tstep_outer / tstep_inner)
        self.step = 0
        self.debug = debug
        
        self.comms = comms
        self.sls_offline = sls_offline
        self.sls_online = sls_online

        self.x_ref = np.zeros(12)

    def __call__(self, ref, obs):
        # Compute optimal reference using MPC-style SLS
        if self.step == 0 or self.step % self.ratio == 0:
            self.x_ref = self.sls_online(ref, obs)
        
        # Compute drone input using infinite horizon SLS
        u = self.sls_offline(self.x_ref, obs)
        self.step += 1

        return u

    def get_reference(self):
        ref = self.sls_online.get_reference()
        return ref
        
    def get_observation(self):
        obs = get_state(self.comms, debug=self.debug)
        return obs

    def apply_input(self, input):
        thrust = normalize_thrust(input[0])
        torques = normalize_torques(input[1:])
        input = np.concatenate([[thrust], torques])
        apply_thrust_and_torque(self.comms, input, debug=self.debug)

    def is_finished(self):
        if self.sls_online.is_finished():
            return True
        else:
            return False


def setup_sls_mpc(comms, K_sls=None, debug=False):
    quad = Quadcopter(params=X500, d_t=TSTEP_OUTER, d_method="zoh")
    A = quad.Ad
    B = quad.Bd

    if K_sls is not None:
        # Closed-loop system: A_cl = A + B*K, input is now setpoint (12D)
        A = A + B @ K_sls  # K is same, but applied to outer loop dynamics
        B = -B @ K_sls  # Input matrix for setpoint tracking (now 12x12)

        # Define continuous-time state tolerances (cost per second)
        x_max_continuous = np.array([
            1.0, 1.0, 1.0,      # position (m) - relaxed tolerance
            2.0, 2.0, 2.0,      # velocity (m/s)
            0.4, 0.4, 0.6,      # angles (rad) - ~23°, 23°, 34°
            2.0, 2.0, 2.0       # rates (rad/s)
        ])

        # Setpoint tracking: input is now 12D state setpoint
        # Use same tolerances as states (symmetric cost)
        u_max_continuous = x_max_continuous.copy()

        # Convert to continuous-time costs
        Q_continuous = np.diag(1.0 / x_max_continuous**2)
        R_continuous = np.diag(1.0 / u_max_continuous**2)

        # Scale by sampling time (CRITICAL!)
        Q = Q_continuous * TSTEP_OUTER
        R = R_continuous * TSTEP_OUTER

        # Adjust for horizon length
        horizon_scaling = np.sqrt(ONLINE_HORIZON / 20.0)
        Q = Q / horizon_scaling
        R = R / horizon_scaling

        print(f"\n{'='*60}")
        print(f"Online SLS-MPC Setup (Outer Loop with K_sls feedback)")
        print(f"{'='*60}")
        print(f"Sampling time (Ts): {TSTEP_OUTER} s")
        print(f"Horizon: {ONLINE_HORIZON} steps ({ONLINE_HORIZON * TSTEP_OUTER} s)")
        print(f"Input dimension: {B.shape[1]} (setpoint tracking)")
        print(f"\nContinuous-time state tolerances: {x_max_continuous}")
        print(f"Discrete Q diagonal (first 6): {np.diag(Q)[:6]}")
        print(f"Discrete R diagonal (first 6): {np.diag(R)[:6]}")
        print(f"Q/R ratio (position): {Q[0,0] / R[0,0]:.2f}")
        print(f"{'='*60}\n")

    else:
        # Direct control case (no inner loop feedback)
        x_max_continuous = np.array([
            1.0, 1.0, 1.0,      # position (m)
            2.0, 2.0, 2.0,      # velocity (m/s)
            0.3, 0.3, 0.5,      # angles (rad)
            2.0, 2.0, 2.0       # rates (rad/s)
        ])

        u_max_continuous = np.array([
            15.0,  # thrust delta (N)
            5.0,   # τ_roll (Nm)
            5.0,   # τ_pitch (Nm)
            0.3    # τ_yaw (Nm)
        ])

        Q_continuous = np.diag(1.0 / x_max_continuous**2)
        R_continuous = np.diag(1.0 / u_max_continuous**2)

        Q = Q_continuous * TSTEP_OUTER
        R = R_continuous * TSTEP_OUTER

        horizon_scaling = np.sqrt(ONLINE_HORIZON / 20.0)
        Q = Q / horizon_scaling
        R = R / horizon_scaling

        print(f"\n{'='*60}")
        print(f"Online SLS-MPC Setup (Direct Control)")
        print(f"{'='*60}")
        print(f"Sampling time (Ts): {TSTEP_OUTER} s")
        print(f"Horizon: {ONLINE_HORIZON} steps ({ONLINE_HORIZON * TSTEP_OUTER} s)")
        print(f"Q/R ratio (position vs thrust): {Q[0,0] / R[0,0]:.2f}")
        print(f"{'='*60}\n")

    Nx = A.shape[0]
    Nu = B.shape[1]
    # Verify outer loop is also stable
    spectral_radius_outer = np.max(np.abs(np.linalg.eigvals(A)))
    print(f"Closed-loop system (outer): A_cl.shape={A.shape}, B_cl.shape={B.shape}")
    print(f"A_cl spectral radius (outer, dt={TSTEP_OUTER}s): {spectral_radius_outer:.6f}")
    if spectral_radius_outer >= 1.0:
        print("WARNING: Outer loop closed-loop system is unstable!")
    else:
        print("✓ Outer loop closed-loop system is stable")

    Phi_u_sparsity_online = np.ones((Nu, Nx))  # (12×12) at each timestep
    Phi_u_sparsity_online = np.tile(
        Phi_u_sparsity_online[np.newaxis, :, :],
        (ONLINE_HORIZON, 1, 1)
    )

    sls_controller = SLS(
        Ad=A,              # Closed-loop dynamics (0.1s discretization)
        Bd=B,              # Maps setpoints to states
        Phi_u_sparsity=Phi_u_sparsity_online,
        Q=Q,                 # State cost
        R=R,                 # Input (setpoint) cost
        horizon=ONLINE_HORIZON,
        x_min=X_MIN,                   # No constraints for basic version
        x_max=X_MAX,
        u_min=U_MIN,
        u_max=U_MAX,
        solver=cp.CLARABEL,
        max_iter=10000,
        solver_params=CLARABEL_PARAMS,
        debug=True                # Set True to see solver details
    )

    sls = OnlineSLS(
        comms, sls_controller, TRAJ,
        debug=debug
    )

    return sls

def setup_sls_fir(comms, debug=False):
    quad = Quadcopter(params=X500, d_t=TSTEP_INNER, d_method="zoh")
    A = quad.Ad
    B = quad.Bd
    
    x_max = np.array([0.02, 0.02, 0.025,  # positions (m) - increased for more tolerance
                      0.08, 0.08, 0.1,  # velocities (m/s) - increased
                      0.05, 0.05, 0.1,  # angles (rad) ≈ 17°, 17°, 29°
                      0.45, 0.45, 0.8])  # rates (rad/s) - increased
    # Use realistic control authority limits
    u_max = np.array([1.3,  # thrust delta (N) - realistic around hover
                      0.2,   # τ_roll (Nm) - closer to MAX_TAU_X
                      0.2,   # τ_pitch (Nm) - closer to MAX_TAU_Y
                      0.8]) # τ_yaw (Nm) - close to MAX_TAU_Z

    Q = np.diag(1.0 / x_max**2)
    R = np.diag(1.0 / u_max**2)

    # Q = np.diag([12.0, 8.0, 8.0, 2.5, 2.5, 2.5, 0, 0, 0, 0, 0, 0])
    # R = np.diag([0.5, 2.0, 2.0, 2.0])

    # Build sparsity manually (using inner loop dimensions)
    Phi_u_sparsity_t = np.ones((B.shape[1], A.shape[0]))
    # Phi_u_sparsity_t[:, :12] = 0
    Phi_u_sparsity = np.tile(Phi_u_sparsity_t[np.newaxis, :, :], (OFFLINE_HORIZON, 1, 1))

    K_sls, Phi_u, diagnostics = solve_sls_fir(A, B, Q, R, OFFLINE_HORIZON, Phi_u_sparsity=Phi_u_sparsity, fir_mode="hard", solver=cp.CLARABEL, verbose=False)

    sls = OfflineSLS(
        comms=comms,
        K_sls=K_sls,
        traj=TRAJ,
        debug=debug
    )

    return sls

def main():
    initialize(debug=False)

    comms = make_comms()
    sim = make_sim()

    sls_offline = setup_sls_fir(comms, debug=True)
    sls_online = setup_sls_mpc(
        comms,
        debug=True)
    controller = ControllerWrapper(
        comms,
        sls_offline,
        sls_online,
        TSTEP_INNER,
        TSTEP_OUTER,
        debug=True
    )

    missionState = SimControllerState(
        oc_next_state=OC_MISSION_FINISHED,
        comms=comms,
        sim=sim,
        controller=sls_online,
        Tsim = TSIM_OUTER, # IMPORTANT
        Tstep = TSTEP_OUTER, # IMPORTANT
        debug = True,
    )
    
    # missionState = RealtimeControllerState(
    #     oc_next_state=OC_MISSION_FINISHED,
    #     comms=comms,
    #     controller=sls_offline,
    #     Tsim = TSIM_INNER,
    #     Tstep = TSTEP_INNER,
    #     debug = True
    # )

    mission = make_mission()
    mission.add_state(Arm("armed", comms), "ARM", "armed", "TAKEOFF")
    mission.add_state(Takeoff("took off", comms), "TAKEOFF", "took off", "HOVER")
    mission.add_state(Hover("stable", comms, 2), "HOVER", "stable", "SLS")
    mission.add_state(
        missionState,
        "SLS",
        OC_MISSION_FINISHED,
        OC_MISSION_FINISHED,
    )

    # missionState.task()

    fsm = make_fsm(mission, comms)

    launch(fsm, comms, sim)


if __name__ == "__main__":
    main()
