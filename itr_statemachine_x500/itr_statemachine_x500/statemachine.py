import time
from typing import Callable

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
import py_trees
import py_trees_ros

from px4_msgs.msg import OffboardControlMode
from px4_msgs.msg import VehicleCommand

from .comms import Comms


class State(py_trees.behaviour.Behaviour):
    def __init__(self, name, comms: Comms = None):
        super().__init__(name)

        self.comms = comms

    def initialise(self):
        super().initialise()
        #self.entry()
        return

    def update(self):
        #self.do()
        return py_trees.common.Status.SUCCESS
    
    def terminate(self, new_status):
        #self.exit()
        pass


class SafeState(State):
    def __init__(self):
        super().__init__("Safe State")
    
    def setup(self, **kwargs):
        print("Safe State Setup")
        return
    
    def initialise(self):
        
        return
    
    def update(self):
        # TODO: Check if drone has landed (likely through arming state)
        print("Safe State Running")
        return py_trees.common.Status.RUNNING
        
class HoverState(State):
    def __init__(self):
        super().__init__("Hover State")

class Arm(State):
    def __Init__(self, name: str, comms: Comms):
        super().__init__(name, comms)
        self.comms = comms

    def initialise(self):
        self.comms.cmd_arm()
    
    def update(self):
        if self.comms.cmd_is_success() and self.comms.get_status()["arm"]:
            self.feedback_message = "Vehicle Armed"
            return py_trees.common.Status.SUCCESS
        else:
            return py_trees.common.Status.RUNNING


class Statemachine(Node):
    def __init__(self, name="tree", tick_interval_s:float=0.05):
        super().__init__(name)
        self.tick_interval_s = tick_interval_s
        
        self.root = py_trees.composites.Selector("Root", memory=False)
        self.states = py_trees.composites.Sequence(name, memory=False)
        self.root.add_children([self.states, SafeState()])

        self.comms = Comms(self, debug=True)

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
        statemachine = Statemachine(tick_interval_s=0.5)
        armingState = Arm("Arming", statemachine.comms)
        statemachine.add_state(armingState)
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
