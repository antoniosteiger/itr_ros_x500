import numpy as np
import numpy.typing as npt

from itr_comms_x500 import Comms
from itr_controller_x500 import Controller
from itr_mission_x500 import initialize, launch, make_comms, make_fsm, make_mission
from itr_statemachine_x500 import (
    OC_MISSION_FINISHED,
    Arm,
    ControllerState,
    Hover,
    Takeoff,
)

# class MPCState(MissionState):
#     def __init__(
#         self,
#         oc_next_state: str,
#         comms: Comms,
#         reference,
#         interval_s: float,
#         horizon: int = 20,
#     ):
#         super().__init__(oc_next_state, comms)

#         model = Quadcopter(X500, interval_s)

#         Q = np.diag([1.0, 1.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
#         R = np.diag([1.0, 0.5, 0.5, 0.5])
#         self.horizon = horizon
#         self.ctrl = MPC(model.Ad_sparse, model.Bd_sparse, Q, R, horizon, debug=True)

#         self.ref = reference

#         self.active = False

#     def task(self):
#         step = 0
#         self.comms.offboard_keepalive("thrust_and_torque")
#         self.comms.offboard_keepalive("thrust_and_torque")
#         self.comms.cmd_offboard_mode()

#         if self.comms.cmd_is_success():
#             pass
#         else:
#             return OC_MISSION_ABORTED
#         while True:
#             obs = np.zeros(12)
#             obs[0:3] = self.comms.get_position()
#             obs[3:6] = self.comms.get_velocity()
#             obs[6:9] = quat2euler(self.comms.get_orientation())
#             obs[9:12] = self.comms.get_angular_velocity()

#             if step * self.horizon + self.horizon > len(self.ref.T):
#                 self.active = False
#                 print("test")
#                 return self.oc_next_state
#             else:
#                 ref = self.ref[
#                     :, step * self.horizon : step * self.horizon + self.horizon
#                 ]
#                 action = self.ctrl(ref, obs)
#                 self.comms.send_thrust_setpoint(action[0])
#                 self.comms.send_torque_setpoint(action[1:4])
#                 self.comms.offboard_keepalive("thrust_and_torque")
#                 step += 1


class MPC(Controller):
    def __init__(self, horizon: int = 40):
        super().__init__()
        self.horizon = horizon

    def __call__(self, reference, observation):
        return


class MPC_State(ControllerState):
    def __init__(
        self,
        oc_next_state: str,
        mpc: MPC,
        comms: Comms,
        trajectory: npt.NDArray[np.float64],
        rate: int = 50,
    ):
        super().__init__(oc_next_state, mpc, comms, rate)
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
        H = self.mpc.horizon
        traj = self.trajectory
        step = self.step

        traj_extended = traj + [traj[-1]] * H

        section = traj_extended[step : step + H]

        return section

    def is_finished(self):
        return


def main() -> None:
    initialize(debug=True)

    comms = make_comms()

    mission = make_mission()
    mission.add_state(Arm("armed", comms), "ARM", "armed", "TAKEOFF")
    mission.add_state(Takeoff("took off", comms), "TAKEOFF", "took off", "HOVER")
    mission.add_state(Hover("stable", comms), "HOVER", "stable", "MPC")
    mission.add_state(
        MPC_State(OC_MISSION_FINISHED, comms),
        "MPC",
        OC_MISSION_FINISHED,
        OC_MISSION_FINISHED,
    )

    fsm = make_fsm(mission, comms)

    launch(fsm, comms)


if __name__ == "main":
    main()
