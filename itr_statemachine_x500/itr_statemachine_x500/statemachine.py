from threading import Thread
from time import sleep

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger

from px4_msgs.msg import (
    VehicleStatus
)

from yasmin import State
from yasmin import StateMachine
from yasmin import Blackboard
from yasmin import YASMIN_LOG_DEBUG
from yasmin import YASMIN_LOG_ERROR
from yasmin import YASMIN_LOG_INFO
from yasmin import YASMIN_LOG_WARN
from yasmin_viewer import YasminViewerPub
from yasmin_ros import set_ros_loggers

from .comms import Comms

# Outcome Constants
OC_END = "end"
OC_START = "start"
OC_IDLE = "idle"
OC_MISSION_FINISHED = "finished"
OC_MISSION_ABORTED = "aborted"

# State Name Constants
ST_MISSION = "MISSION"
ST_SAFE = "SAFE STATE"


class Mission(StateMachine):
    def __init__(self):
        super().__init__([OC_MISSION_FINISHED, OC_MISSION_ABORTED])

class SafeState(State):
    def __init__(self):
        super().__init__([OC_START, OC_IDLE, OC_END])

    def execute(self, blackboard: Blackboard):
        # TODO: proper start handling
        sleep(2)
        return OC_START

class Arm(State):
    def __init__(self, oc_next_state: str, comms: Comms):
        super().__init__([oc_next_state, OC_MISSION_ABORTED])
        self.oc_next_state = oc_next_state
        self.comms = comms

    def execute(self, blackboard: Blackboard):
        self.comms.cmd_set_origin()
        sleep(2)
        self.comms.cmd_arm()
        sleep(2)
        if self.comms.get_status()["arm"]:
            return self.oc_next_state
        else:
            return OC_MISSION_ABORTED
        
class Takeoff(State):
    def __init__(self, oc_next_state: str, comms: Comms):
        super().__init__([oc_next_state, OC_MISSION_ABORTED])
        self.oc_next_state = oc_next_state
        self.comms = comms

    def execute(self, blackboard: Blackboard):
        self.comms.cmd_takeoff()
        sleep(10)
        if True:
            return self.oc_next_state
        else:
            return OC_MISSION_ABORTED


class FSM(Node):
    def __init__(self, mission: Mission, debug=False):
        super().__init__("ITR_X500")
        
        self.comms = Comms(self, debug=debug)

        self.sm = StateMachine(outcomes=[OC_END])
        self.sm.add_state(
            ST_SAFE,
            SafeState(),
            transitions={
                OC_END: OC_END,
                OC_IDLE: ST_SAFE,
                OC_START: ST_MISSION
            }
        )
        self.sm.add_state(
            ST_MISSION,
            mission,
            transitions={
                OC_MISSION_FINISHED: ST_SAFE,
                OC_MISSION_ABORTED: ST_SAFE
            }
        )

    def start(self):
        try:
            outcome = self.sm()
            YASMIN_LOG_INFO(outcome)
        except KeyboardInterrupt:
            if self.sm.is_running():
                self.sm.cancel_state()



# Execute the FSM
def main():    
    rclpy.init()
    set_ros_loggers()

    comms_node = Node("itr_x500_comms")
    comms = Comms(comms_node, debug=True)

    mission = Mission()
    mission.add_state(
        "ARM",
        Arm("take off", comms),
        transitions={
            "take off": "TAKE OFF",
            OC_MISSION_ABORTED: OC_MISSION_ABORTED
        }
    )
    mission.add_state(
        "TAKE OFF",
        Takeoff(OC_MISSION_FINISHED, comms),
        transitions={
            OC_MISSION_FINISHED: OC_MISSION_FINISHED,
            OC_MISSION_ABORTED: OC_MISSION_ABORTED
        }
    )

    fsm = FSM(mission, debug=True)    
    YasminViewerPub("ITR_X500", fsm.sm) 
    
    def spin_node(node: Node):
        rclpy.spin(node)
    
    Thread(target=spin_node, args=(comms_node,), daemon=True).start()
    fsm.start()

    # Shutdown ROS 2 if it's running
    if rclpy.ok():
        rclpy.shutdown()

if __name__ == "__main__":
    main()

