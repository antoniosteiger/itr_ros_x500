import numpy as np
import numpy.typing as npt

from itr_comms_x500 import Comms
from itr_controller_x500 import MPC
from itr_description_x500 import LTI, X500, Quadcopter
from itr_mission_x500 import initialize, launch, make_comms, make_fsm, make_mission
from itr_statemachine_x500 import (
    OC_MISSION_FINISHED,
    Arm,
    ControllerState,
    Hover,
    Takeoff,
)

HOVER_THROTTLE = -0.725


def null_trajectory(length: int):
    vec = np.array(
        [
            [0.0],
            [0.0],
            [-1.8],
            [0.0],
            [0.0],
            [0.0],
            [0.0],
            [0.0],
            [0.0],
            [0.0],
            [0.0],
            [0.0],
        ]
    )
    traj = np.tile(vec, (1, length))
    return traj


class DoubleIntegrator(LTI):
    def __init__(self, d_t):
        A = np.zeros((6, 6))
        A[0:3, 3:6] = np.identity(3)

        B = np.zeros((6, 3))
        B[3:6, 0:3] = np.identity(3)

        C = np.identity(6)

        D = np.zeros((6, 3))

        super().__init__(A, B, C, D, d_t=d_t)


class MPC_State(ControllerState):
    def __init__(
        self,
        oc_next_state: str,
        mpc: MPC,
        comms: Comms,
        trajectory: npt.NDArray[np.float64],
        rate: int = 50,
        debug: bool = False,
    ):
        super().__init__(oc_next_state, mpc, "thrust_and_torque", comms, 50, debug)
        self.comms = comms
        self.mpc = mpc
        self.trajectory = trajectory
        self.step = 0
        self.debug = debug

    def get_observation(self):
        pos = self.comms.get_position()  # From Mocap, needs sign adjustment for z coord
        pos[2] = -1 * pos[2]
        vel = self.comms.get_velocity()  # From PX4, does not need sign adjustment
        # vel[2] = -1 * vel[2]
        orientation = self.comms.get_orientation_euler()
        body_rates = self.comms.get_angular_velocity()
        obs = np.concatenate([pos, vel, orientation, body_rates])

        if self.debug:
            print(f"Observation: {obs}")

        return obs

    def get_reference(self):
        # Send the current section of the trajectory to the MPC
        # Pad with the final point in the trajectory at the end
        H = self.mpc.H
        traj = self.trajectory
        step = self.step
        section = traj[:, step : step + H]

        # traj = np.append(traj, traj)
        if self.debug:
            print(f"Reference: {section}")
        return section

    def apply_input(self, input):
        # Normalize thrust and send. IMPORTANT: negative thrust means up
        thrust_normalized = -HOVER_THROTTLE - input[0] / X500.thrust_max
        self.comms.send_thrust_setpoint(thrust_normalized)
        # Normalize torques and send
        torques_normalized = [input[1] / X500.tau_x_max, input[2] / X500.tau_y_max, 0.0]
        self.comms.send_torque_setpoint(torques_normalized)

        if self.debug:
            print(f"Input Sent: {thrust_normalized}, {torques_normalized}")

    def is_finished(self):
        # TODO: Handle trajectory end better. (e.g. padding)
        if self.step >= len(self.trajectory[1]) - self.mpc.H - 1:
            return True
        else:
            return False

    def thrust2throttle(self, thrust):
        return (thrust - 4 * 0.6652) / (4 * 0.8496)


def main() -> None:
    initialize(debug=True)

    rate = 20
    horizon = 20
    comms = make_comms()

    Q = np.zeros((12, 12))
    Q[0:3, 0:3] = 20 * np.identity(3)
    R = 0.005 * np.identity(4)  # TODO: dial in weights

    model = Quadcopter(X500, 1 / rate)
    # Limit pitch and yaw to small-angle assumption (15 degrees or 0.26 rad)
    x_min = np.array(
        [None, None, None, None, None, None, -0.26, -0.26, None, None, None, None]
    )
    x_max = np.array(
        [None, None, None, None, None, None, 0.26, 0.26, None, None, None, None]
    )
    # Limit thrust and torques. Disallow yaw torque
    u_min = np.array(  # this contains actually the maximum upwards thrust
        [
            -(1 - HOVER_THROTTLE) * X500.thrust_max,
            -X500.tau_x_max,
            -X500.tau_y_max,
            None,
        ]
    )
    u_max = np.array(
        [(HOVER_THROTTLE) * X500.thrust_max, X500.tau_x_max, X500.tau_y_max, None]
    )
    mpc = MPC(
        model.Ad_sparse,
        model.Bd_sparse,
        Q,
        R,
        horizon,
        x_min=x_min,
        x_max=x_max,
        u_min=u_min,
        u_max=u_max,
        debug=True,
    )
    trajectory = null_trajectory(2000)

    mission = make_mission()
    mission.add_state(Arm("armed", comms), "ARM", "armed", "TAKEOFF")
    mission.add_state(Takeoff("took off", comms), "TAKEOFF", "took off", "HOVER")
    mission.add_state(Hover("stable", comms, 2), "HOVER", "stable", "MPC")
    mission.add_state(
        MPC_State(OC_MISSION_FINISHED, mpc, comms, trajectory, rate=50, debug=True),
        "MPC",
        OC_MISSION_FINISHED,
        OC_MISSION_FINISHED,
    )

    fsm = make_fsm(mission, comms)

    launch(fsm, comms)


if __name__ == "__main__":
    main()
