import numpy as np

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


class NothingController(Controller):
    def __init__(self, comms: Comms):
        self.comms = comms
        self.offboard_setpoint_counter = 0
        self.step = 0
        return

    def __call__(self, ref, obs):
        # Always send offboard

        # The ControllerState in the state machine executes this in a regular interval
        # and ensures the drone is in the offboard mode that enables external setpoints
        # Slowly lower the altitude
        self.comms.send_position_setpoint(np.array([0.0, 0.0, 0.001 * self.step]))

        # if self.offboard_setpoint_counter == 25:
        #     # self.comms.send_position_setpoint(np.array([0.0, 0.0, -1.5]))
        #     # self.comms.cmd_manual_position_mode()
        #     self.comms.cmd_offboard_mode()
        # if self.offboard_setpoint_counter == 26:
        # self.comms.offboard_keepalive("position")
        # self.comms.send_position_setpoint(self.comms.get_position())

        # if self.comms.get_status()["nav"] == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
        # self.comms.send_position_setpoint(np.array([0.0, 0.0, -0.75]))

        # if self.offboard_setpoint_counter < 41:
        #     self.offboard_setpoint_counter += 1


class NothingControllerState(ControllerState):
    def __init__(
        self, oc_next_state: str, comms: Comms, rate: int = 1, max_steps: int = 10
    ):
        super().__init__(
            oc_next_state, NothingController(comms), "position", comms, rate, debug=True
        )
        self.comms = comms
        self.counter = 0
        self.max = max_steps

    def get_observation(self):
        self.comms.get_position()
        return None

    def get_reference(self):
        self.counter += 1
        return None

    def is_finished(self):
        if self.counter >= self.max:
            return True
        else:
            return False


def nothing_mission():
    initialize(debug=True)

    comms = make_comms()

    mission = make_mission()
    mission.add_state(Arm("take off", comms), "ARM", "take off", "TAKEOFF")
    mission.add_state(
        Takeoff("takeoff completed", comms), "TAKEOFF", "takeoff completed", "HOVER"
    )
    mission.add_state(
        Hover("hover completed", comms, 2), "HOVER", "hover completed", "CONTROLLER"
    )
    mission.add_state(
        NothingControllerState(OC_MISSION_FINISHED, comms, rate=20, max_steps=100000),
        "CONTROLLER",
        OC_MISSION_FINISHED,
        OC_MISSION_FINISHED,
    )

    fsm = make_fsm(mission, comms)

    launch(fsm, comms)
    return


if __name__ == "__main__":
    nothing_mission()
