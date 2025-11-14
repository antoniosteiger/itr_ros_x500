from time import sleep

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
        return

    def __call__(self, ref, obs):
        # Always send offboard keepalive
        self.comms.offboard_keepalive("velocity")
        self.comms.send_velocity_setpoint(np.array([0.0, 0.0, 0.0]))

        self._log("Hello from NothingController")


class NothingControllerState(ControllerState):
    def __init__(
        self, oc_next_state: str, comms: Comms, rate: int = 1, max_steps: int = 10
    ):
        super().__init__(
            oc_next_state, NothingController(comms), comms, rate, debug=True
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

    def init_hook(self):
        i = 0
        while i <= 8:
            sleep(0.2)
            self.comms.offboard_keepalive("velocity")
            self.comms.send_velocity_setpoint(np.array([0.0, 0.0, -0.2]))
            i += 1

        self.comms.cmd_offboard_mode()

        if self.comms.cmd_is_success():
            return
        else:
            print("FAIL FAIL FAIL")
            raise Exception("Failed to enable offboard control mode.")


def nothing_mission():
    initialize(debug=True)

    comms = make_comms()

    mission = make_mission()
    mission.add_state(Arm("take off", comms), "ARM", "take off", "TAKEOFF")
    mission.add_state(Takeoff("hover", comms), "TAKEOFF", "hover", "HOVER")
    mission.add_state(
        Hover("start controller", comms, 2), "HOVER", "start controller", "CONTROLLER"
    )
    mission.add_state(
        NothingControllerState(OC_MISSION_FINISHED, comms, rate=10, max_steps=100),
        "CONTROLLER",
        OC_MISSION_FINISHED,
        OC_MISSION_FINISHED,
    )

    fsm = make_fsm(mission, comms)

    launch(fsm, comms)
    return


if __name__ == "__main__":
    nothing_mission()
