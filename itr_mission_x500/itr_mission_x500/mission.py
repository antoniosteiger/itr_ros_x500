import rclpy
from rclpy.node import Node
from yasmin import Blackboard

from itr_comms_x500 import Comms
from itr_controller_x500 import MPC, SLS
from itr_statemachine_x500 import (
    FSM,
    OC_MISSION_FINISHED,
    Arm,
    Hover,
    Mission,
    MissionState,
    Takeoff,
    start,
)

AVAILABLE_CONTROLLERS = {"sls": SLS, "mpc": MPC}


class ControllerState(MissionState):
    def __init__(self, oc_next_state: str, comms: Comms, controller: str):
        super().__init__(oc_next_state, comms)
        for key in AVAILABLE_CONTROLLERS:
            if controller.lower() == key.lower():
                self.controller = AVAILABLE_CONTROLLERS[controller]()

    def execute(self, blackboard: Blackboard):
        return


def main():
    rclpy.init()

    comms_node = Node("itr_x500_comms")
    comms = Comms(comms_node, debug=True)

    mission = Mission()
    mission.add_state(Arm("take off", comms), "ARM", "take off", "TAKEOFF")
    mission.add_state(Takeoff("hover", comms), "TAKEOFF", "hover", "HOVER")
    mission.add_state(
        Hover(OC_MISSION_FINISHED, comms, 10),
        "HOVER",
        OC_MISSION_FINISHED,
        OC_MISSION_FINISHED,
    )

    fsm = FSM(mission, comms, debug=True)

    start(fsm, comms_node)
    return


if __name__ == "__main__":
    main()
