import time
from typing import Callable

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
import py_trees
import py_trees_ros

from px4_msgs.msg import OffboardControlMode
from px4_msgs.msg import VehicleCommand
from px4_msgs.msg import VehicleStatus
from px4_msgs.msg import TakeoffStatus

from .comms import Comms


class State(py_trees.behaviour.Behaviour):
    def __init__(self, name, comms: Comms = None, logger = None, debug=False):
        super().__init__(name)

        self.comms = comms
        self.logger = logger
        self.debug = debug

    def initialise(self):
        #self.__init__()
        return

    def update(self):
        #self.do()
        return py_trees.common.Status.SUCCESS
    
    def terminate(self, new_status):
        #self.exit()
        pass

    def log(self, msg):
        if self.debug:
            self.logger.info(msg)
    
    def error(self, msg):
        self.logger.error(msg)


class SafeState(State):
    def __init__(self, comms: Comms = None, logger = None, debug = False):
        super().__init__("Safe State", comms=comms, logger=logger, debug=debug)
    
    def update(self):
        # TODO: Check if drone has landed (likely through arming state)
        self.log("Safe State Active")
        return py_trees.common.Status.RUNNING
        
class HoverState(State):
    def __init__(self):
        super().__init__("Hover State")

class Arm(State):
    def __init__(self, name: str, comms: Comms, logger, debug: bool = False):
        super().__init__(name=name, comms=comms, logger=logger, debug=debug)
        self.comms = comms
        self.cmd_counter = 0
        self.cmd_max = 5

    def update(self):
        if self.comms.get_status()["arm"]:
            self.feedback_message = "Vehicle armed"
            self.log("Vehicle armed")         
            return py_trees.common.Status.SUCCESS
        else:
            if self.cmd_counter < self.cmd_max:
                self.comms.cmd_arm()
                self.cmd_counter += 1
                return py_trees.common.Status.RUNNING
            else:
                self.error(f"Failed to arm Vehicle, tried {self.cmd_max}")
                return py_trees.common.Status.FAILURE

class Takeoff(State):
    def __init__(self, name:str, comms: Comms, logger, debug: bool = False):
        super().__init__(name=name, comms=comms, logger=logger, debug=debug)
        self.comms = comms
        self.cmd_counter = 0
        self.cmd_max = 5

    def update(self):
        # if self.comms.get_status()["nav"] == VehicleStatus.NAVIGATION_STATE_AUTO_LOITER:
        #     self.feedback_message = "Vehicle Hovering"
        #     self.log("Vehicle hovering")
        #     return py_trees.common.Status.SUCCESS
        if (
            self.comms.get_status()["nav"] == VehicleStatus.NAVIGATION_STATE_AUTO_LOITER
            and self.comms.get_position()[2] > 0.5
        ):
            self.log("Vehicle Flying")
            return py_trees.common.Status.SUCCESS
        elif (
            self.comms.get_status()["nav"] == VehicleStatus.NAVIGATION_STATE_AUTO_TAKEOFF
        ):
            self.log("Taking off...")
            return py_trees.common.Status.RUNNING  
        elif (
            self.comms.get_status()["arm"] 
            and not self.comms.get_status()["nav"] == VehicleStatus.NAVIGATION_STATE_AUTO_TAKEOFF
            and self.comms.get_position()[2] < 0.5
        ):
            if self.cmd_counter < self.cmd_max:
                self.comms.cmd_takeoff()
                self.cmd_counter += 1
                return py_trees.common.Status.RUNNING
            else:
                self.error(f"Failed taking off, tried {self.cmd_max} times.")
                return py_trees.common.Status.FAILURE
        elif not self.comms.get_status()["arm"]:
            self.error(f"Vehicle not armed, aborting takeoff.")
            return py_trees.common.Status.FAILURE
        else:
            self.error(f"Failed taking off, unknown error.")
            return py_trees.common.Status.FAILURE

class Statemachine(Node):
    def __init__(self, name="tree", tick_interval_s:float=0.05, debug = False):
        super().__init__(name)
        self.tick_interval_s = tick_interval_s
        self.debug = debug

        self.comms = Comms(self, debug=debug)
        
        self.root = py_trees.composites.Selector("Root", memory=True)
        self.states = py_trees.composites.Sequence(name, memory=True)
        self.root.add_children([self.states, SafeState(comms=self.comms, logger=self.get_logger(), debug=debug)])

        

        self.tree = py_trees_ros.trees.BehaviourTree(root=self.root)
        self.tree.setup(node=self, timeout=15.0)
        
        self.set_parameters([
            Parameter("default_snapshot_stream", Parameter.Type.BOOL, True),
            Parameter("default_snapshot_blackboard_data", Parameter.Type.BOOL, True),
            Parameter("default_snapshot_blackboard_activity", Parameter.Type.BOOL, True),
            Parameter("default_snapshot_period", Parameter.Type.DOUBLE, tick_interval_s)
        ])


        self.timer = self.create_timer(self.tick_interval_s, self.run)

    def add_state(self, state: State):
        self.states.add_child(state)

    def run(self):
        try:
            self.tree.tick()
        except Exception as e:
            self.get_logger().error(f"Error ticking tree: {e}")

    
    def shutdown(self):
        self.tree.shutdown()
        
    
def main(args=None):
    rclpy.init(args=args)
    
    try:
        statemachine = Statemachine(tick_interval_s=0.5, debug=True)
        logger = statemachine.get_logger()
        armingState = Arm("Arming", comms= statemachine.comms, logger=logger, debug=True)
        takeoffState = Takeoff("Takeoff", comms = statemachine.comms, logger=logger, debug=True)
        statemachine.add_state(armingState)
        statemachine.add_state(takeoffState)
        rclpy.spin(statemachine)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            statemachine.shutdown()
            statemachine.destroy_node()
            rclpy.shutdown()

if __name__ == "__main__":
    main()
