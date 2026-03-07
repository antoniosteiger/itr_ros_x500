from .scripts.sim.gz import GazeboControlNode
import signal
import sys
from threading import Thread, Event

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from itr_comms_x500 import Comms
from itr_statemachine_x500 import (
    FSM,
    OC_MISSION_FINISHED,
    Arm,
    Hover,
    Mission,
    Takeoff,
)

from ros_gz_interfaces.srv import ControlWorld

DEBUG_FLAG = False

GAZEBO_WORLD_NAME = "itr"          # change to match your world name
GAZEBO_SERVICE_NAME = f"/world/{GAZEBO_WORLD_NAME}/control"
GAZEBO_SERVICE_TIMEOUT = 5.0        # seconds to wait for the service to become available


class GazeboControlNode(Node):
    """Node that exposes pause() / resume() helpers for a Gazebo world."""

    def __init__(self, world_name: str = GAZEBO_WORLD_NAME, debug=False):
        super().__init__("gazebo_control_node")

        service_name = f"/world/{world_name}/control"
        self._client = self.create_client(ControlWorld, service_name)
        self.debug = debug

        self.get_logger().info(f"Waiting for service '{service_name}' …")
        if not self._client.wait_for_service(timeout_sec=GAZEBO_SERVICE_TIMEOUT):
            raise RuntimeError(
                f"Service '{service_name}' not available after "
                f"{GAZEBO_SERVICE_TIMEOUT} s. Is Gazebo running?"
            )
        self.get_logger().info("Service ready.")

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def pause(self) -> bool:
        """Pause the Gazebo simulation. Returns True on success."""
        return self._send_control(pause=True)

    def resume(self) -> bool:
        """Resume (un-pause) the Gazebo simulation. Returns True on success."""
        return self._send_control(pause=False)

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------

    def _send_control(self, pause: bool) -> bool:
        request = ControlWorld.Request()
        request.world_control.pause = pause

        future = self._client.call_async(request)

        # Wait for the result without spinning — the executor handles that
        event = Event()
        future.add_done_callback(lambda _: event.set())
        event.wait()

        if future.result() is not None:
            state = "paused" if pause else "resumed"
            if self.debug:
                self.get_logger().info(f"Simulation {state} successfully.")
            return True
        else:
            if self.debug:
                self.get_logger().error(f"Service call failed: {future.exception()}")
            return False


def log(msg: str):
    print(f"\033[33m[ITR_MISSION]: {msg} \033[0m")


def handle_interrupt(signum, frame, comms: Comms):
    """
    Handle Ctrl+C (SIGINT) signal before ROS shuts down.
    This function sends the RTL command and then shuts down ROS gracefully.
    """
    log("ABORT DETECTED, landing drone...")
    # Send RTL (Return to Launch) command to land the drone
    comms.cmd_rtl()

    # Gracefully shutdown ROS
    if rclpy.ok():
        rclpy.shutdown()

    # Exit the program after shutdown
    sys.exit(0)


def initialize(debug: bool = False) -> None:
    """
    Initialize the Drone Mission.

    Args:
        debug (bool, optional): A debug flag. If true,
                                communication layer and state machine will log
                                diagnostic information to the terminal.
                                Defaults to False.
    """
    global DEBUG_FLAG
    DEBUG_FLAG = debug
    rclpy.init()


def make_comms() -> Comms:
    """
    Create the communication layer for the drone.
    This layer is run in a separate thread.

    Returns:
        Comms: A Comms instance (communication layer interface, see itr_comms_x500)
    """
    return Comms(DEBUG_FLAG)


def make_mission() -> Mission:
    """
    Create the main mission for the drone.
    Add states to the mission like this:
    mission = make_mission()
    mission.add_state(...)

    Returns:
        Mission: A Mission Instance. (see itr_statemachine_x500)
    """
    return Mission()


def make_fsm(mission: Mission, comms: Comms) -> FSM:
    """
    Create a finite state machine for the drone.
    By default, it has a mission (which is itself a state machine)
    and a safe state. Should anything go wrong, the state machine
    automatically returns to the safe state, which lands the drone.
    The state machine can be monitored in the browser using the YASMIN
    viewer.

    Args:
        mission (Mission): a Mission instance. Describes the main states the drone shall go through.
        comms (Comms):

    Returns:
        FSM: A YASMIN finite state machine.
    """
    return FSM(mission, comms, debug=DEBUG_FLAG)

def make_sim() -> GazeboControlNode:
    """
    Create a node to start and stop Gazebo simulation
    """
    return GazeboControlNode()


def launch(fsm: FSM, comms: Comms, sim: GazeboControlNode = None) -> None:
    """
    Start the Drone Mission.
    Main Entry Point to anything you want to do with the X500 Drone at ITR.
    Press Ctrl+C in the terminal to abort the mission. The drone will immediately return
    to its home position.

    Args:
        fsm (FSM): Finite state machine for the drone created with make_fsm()
        comms (Comms): Communication layer for the drone created with make_comms()
    """
    # land the drone on ctrl+c
    signal.signal(
        signal.SIGINT, lambda signum, frame: handle_interrupt(signum, frame, comms)
    )

    def spin_node(node: Node):
        rclpy.spin(node)

    try:
        log("Starting Mission...")
        executor = MultiThreadedExecutor()
        executor.add_node(comms)
        if sim is not None:
            executor.add_node(sim)

        Thread(target=executor.spin, daemon=True).start()

        fsm.start()
    except Exception as e:
        log(f"Exception raised:\n{e}")
        # Send RTL command:
        comms.cmd_rtl()
    finally:
        if rclpy.ok():
            rclpy.shutdown()


def _basic_mission():
    # Initialize the Mission Environment
    initialize(debug=True)

    # Create a Communication Layer to the Drone
    comms = make_comms()

    # Create an empty Mission
    mission = make_mission()

    # Add States to the Mission in Sequence
    # Arm the Drone
    mission.add_state(Arm("take off", comms), "ARM", "take off", "TAKEOFF")
    # Let the drone take off
    mission.add_state(Takeoff("hover", comms), "TAKEOFF", "hover", "HOVER")
    # Let the drone hover for five seconds
    mission.add_state(
        Hover(OC_MISSION_FINISHED, comms, 5),
        "HOVER",
        OC_MISSION_FINISHED,
        OC_MISSION_FINISHED,
    )

    # Create a State Machine with the Mission
    fsm = make_fsm(mission, comms)

    # Start the Mission
    launch(fsm, comms)
    return


def main():
    _basic_mission()
    return


if __name__ == "__main__":
    main()
