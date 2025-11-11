from threading import Thread
from time import sleep

import rclpy
from rclpy.node import Node
from yasmin import (
    YASMIN_LOG_ERROR,
    Blackboard,
    State,
    StateMachine,
    logs,
)
from yasmin_ros import set_ros_loggers
from yasmin_viewer import YasminViewerPub

from itr_comms_x500 import Comms

# Outcome Constants
OC_END = "end"
OC_START = "start"
OC_IDLE = "idle"
OC_MISSION_FINISHED = "finished"
OC_MISSION_ABORTED = "aborted"

# State Name Constants
ST_MISSION = "MISSION"
ST_SAFE = "SAFE STATE"


def log(msg: str):
    print(f"\033[34m[ITR_STATEMACHINE]: {msg} \n")


class MissionState(State):
    def __init__(self, oc_next_state: str, comms: Comms):
        super().__init__([oc_next_state, OC_MISSION_ABORTED])
        self.oc_next_state = oc_next_state
        self.comms = comms


class Mission(StateMachine):
    def __init__(self):
        super().__init__([OC_MISSION_FINISHED, OC_MISSION_ABORTED])

    def add_state(
        self,
        state: MissionState,
        state_name: str,
        outcome_next: str,
        next_state_name: str,
    ):
        super().add_state(
            state_name,
            state,
            transitions={
                OC_MISSION_ABORTED: OC_MISSION_ABORTED,
                outcome_next: next_state_name,
            },
        )


class SafeState(State):
    def __init__(self, comms: Comms):
        super().__init__([OC_START, OC_IDLE, OC_END])
        self.comms = comms
        self.exit_flag = False

    def execute(self, blackboard: Blackboard):
        # Always send the landing command immediately
        self.comms.cmd_rtl()
        if not self.exit_flag:
            self.exit_flag = True
            log("Starting Statemachine.")
            sleep(2)
            return OC_START
        else:
            log("Reached end, exiting state machine.")
            return OC_END


class Arm(MissionState):
    def __init__(self, oc_next_state: str, comms: Comms):
        super().__init__(oc_next_state, comms)

    def execute(self, blackboard: Blackboard):
        self.comms.cmd_set_origin()
        sleep(2)
        self.comms.cmd_arm()
        sleep(2)
        if self.comms.get_status()["arm"]:
            log("Vehicle armed!")
            return self.oc_next_state
        else:
            YASMIN_LOG_ERROR("Vehicle could not be armed.")
            return OC_MISSION_ABORTED


class Takeoff(MissionState):
    def __init__(self, oc_next_state: str, comms: Comms, altitude: float = 1.5):
        super().__init__(oc_next_state, comms)
        self.altitude = altitude

    def execute(self, blackboard: Blackboard):
        self.comms.cmd_takeoff()

        max_checks = 15
        takeoff = False
        for i in range(max_checks):
            if self.comms.get_position()[2] > self.altitude - 0.2:
                takeoff = True
                break
            else:
                takeoff = False
            sleep(1)

        if takeoff:
            log("Takeoff detected.")
            return self.oc_next_state
        else:
            YASMIN_LOG_ERROR("Could not complete takeoff!")
            return OC_MISSION_ABORTED


class Hover(MissionState):
    def __init__(self, oc_next_state: str, comms: Comms, hovertime_s: int = 10):
        super().__init__(oc_next_state, comms)
        self.hovertime_s = hovertime_s

    def execute(self, blackboard: Blackboard):
        self.comms.cmd_hover()

        if self.comms.cmd_is_success():
            log(f"Hovering for {self.hovertime_s}")
            sleep(self.hovertime_s)
            return self.oc_next_state
        else:
            YASMIN_LOG_ERROR("Could not enable hovering.")
            return OC_MISSION_ABORTED


class FSM(Node):
    def __init__(self, mission: Mission, comms: Comms, debug=False):
        super().__init__("ITR_X500")

        self.comms = comms

        self.sm = StateMachine(outcomes=[OC_END])
        self.sm.add_state(
            ST_SAFE,
            SafeState(comms=self.comms),
            transitions={OC_END: OC_END, OC_IDLE: ST_SAFE, OC_START: ST_MISSION},
        )
        self.sm.add_state(
            ST_MISSION,
            mission,
            transitions={OC_MISSION_FINISHED: ST_SAFE, OC_MISSION_ABORTED: ST_SAFE},
        )

        set_ros_loggers()
        logs.set_log_level(1)
        YasminViewerPub("ITR_X500", self.sm)

    def start(self):
        try:
            self.sm()
        except KeyboardInterrupt:
            if self.sm.is_running():
                self.sm.cancel_state()


def start(fsm: FSM, comms_node: Node):
    def spin_node(node: Node):
        rclpy.spin(node)

    Thread(target=spin_node, args=(comms_node,), daemon=True).start()
    fsm.start()

    # Shutdown ROS 2 if it's running
    if rclpy.ok():
        rclpy.shutdown()


# Execute the FSM
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

    def spin_node(node: Node):
        rclpy.spin(node)

    Thread(target=spin_node, args=(comms_node,), daemon=True).start()
    fsm.start()

    # Shutdown ROS 2 if it's running
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__":
    main()
