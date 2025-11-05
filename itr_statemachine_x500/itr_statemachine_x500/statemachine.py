import time
from typing import Callable

import rclpy
from rclpy.node import Node
import py_trees
import py_trees_ros

from px4_msgs.msg import OffboardControlMode
from px4_msgs.msg import VehicleCommand

from itr_comms_x500 import Comms

UPDATE_INTERVAL_S = 0.05

class State(py_trees.behaviour.Behaviour):
    def __init__(self, name, entry:Callable = lambda: None, do:Callable = lambda: None, exit:Callable = lambda: None):
        super().__init__(name)

        self.entry = entry
        self.do = do
        self.exit = exit

    def setup(self):

        pass

    def initialise(self):
        super().initialise()
        self.entry()
        return

    def update(self):
        self.do()
        return py_trees.common.Status.SUCCESS
    
    def terminate(self, new_status):
        self.exit()
        pass


class SafeState(State):
    def __init__(self):
        super().__init__("Safe State")
        self.cmd_pub
    
    def initialise(self):
        
        return
    
    def update(self):
        # TODO: Check if drone has landed (likely through arming state)
        if self.is_landed is True:
            return py_trees.common.Status.SUCCESS
        else:
            return py_trees.common.Status.RUNNING
        
class HoverState(State):
    def __init__(self):
        super().__init__("Hover State")


class Statemachine(Node):
    def __init__(self, comms: Comms, name="Main", tick_interval_s:float=0.05):
        super().__init__("itr_statemachine_x500")
        self.tick_interval_s = tick_interval_s
        
        self.root = py_trees.composites.Selector("Root", memory=True)
        self.states = py_trees.composites.Sequence(name, memory=True)
        self.root.add_children([self.states, SafeState()])
        
        # ?
        self._parameter_overrides=[
            rclpy.parameter.Parameter(
                "default_snapshot_stream",
                rclpy.parameter.Parameter.Type.BOOL,
                True
            ),
            rclpy.parameter.Parameter(
                "default_snapshot_period",
                rclpy.parameter.Parameter.Type.DOUBLE,
                UPDATE_INTERVAL_S
            ),
        ]

    def add_state(self, state: State):
        self.states.add_child(state)

    def run(self):
        tree = py_trees_ros.trees.BehaviourTree(root=self.root)
        tree.setup(node=self.node, timeout=15.0)

        tree.tick_tock(int(self.tick_interval_s * 1000))

    