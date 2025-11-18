import numpy as np
import numpy.typing as npt

from itr_comms_x500 import Comms
from itr_controller_x500 import MPC
from itr_description_x500 import LTI
from itr_mission_x500 import initialize, launch, make_comms, make_fsm, make_mission
from itr_statemachine_x500 import (
    OC_MISSION_FINISHED,
    Arm,
    ControllerState,
    Hover,
    Takeoff,
)


def null_trajectory(length: int):
    vec = np.array([[0.0], [0.0], [-1.2], [0.0], [0.0], [0.0]])
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
    ):
        super().__init__(oc_next_state, mpc, "acceleration", comms, 50, True)
        self.comms = comms
        self.mpc = mpc
        self.trajectory = trajectory
        self.step = 0

    def get_observation(self):
        pos = self.comms.get_position()
        vel = self.comms.get_velocity()
        return np.concatenate([pos, vel])

    def get_reference(self):
        # Send the current section of the trajectory to the MPC
        # Pad with the final point in the trajectory at the end
        H = self.mpc.H
        traj = self.trajectory
        step = self.step

        # traj = np.append(traj, traj)

        return traj[:, step : step + H]

    def apply_input(self, input):
        self.comms.send_acceleration_setpoint(input)
        self.step += 1

    def is_finished(self):
        if self.step >= len(self.trajectory):
            return True
        else:
            return False


def main() -> None:
    initialize(debug=True)

    rate = 50
    horizon = 50
    comms = make_comms()

    Q = np.zeros((6, 6))
    Q[0:3, 0:3] = np.identity(3)
    R = np.identity(3)

    model = DoubleIntegrator(1 / rate)
    mpc = MPC(model.Ad_sparse, model.Bd_sparse, Q, R, horizon, debug=True)
    trajectory = null_trajectory(2000)

    mission = make_mission()
    mission.add_state(Arm("armed", comms), "ARM", "armed", "TAKEOFF")
    mission.add_state(Takeoff("took off", comms), "TAKEOFF", "took off", "HOVER")
    mission.add_state(Hover("stable", comms, 2), "HOVER", "stable", "MPC")
    mission.add_state(
        MPC_State(OC_MISSION_FINISHED, mpc, comms, trajectory, rate=50),
        "MPC",
        OC_MISSION_FINISHED,
        OC_MISSION_FINISHED,
    )

    fsm = make_fsm(mission, comms)

    launch(fsm, comms)


if __name__ == "__main__":
    main()
