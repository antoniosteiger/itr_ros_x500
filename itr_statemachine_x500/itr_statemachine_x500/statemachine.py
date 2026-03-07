import numpy as np
import sched
import time
import traceback
from abc import ABC, abstractmethod
from threading import Thread
from time import sleep
from typing import Literal

import rclpy
from rclpy.node import Node
from yasmin import (
    YASMIN_LOG_ERROR,
    Blackboard,
    State,
    StateMachine,
    set_log_level,
    LogLevel
)
from yasmin_ros import set_ros_loggers
from yasmin_viewer import YasminViewerPub

from itr_comms_x500 import Comms
from itr_controller_x500 import Controller
from px4_msgs.msg import VehicleStatus

# Outcome Constants
OC_END = "end"
OC_START = "start"
OC_IDLE = "idle"
OC_MISSION_FINISHED = "finished"
OC_MISSION_ABORTED = "aborted"

# State Name Constants
ST_MISSION = "MISSION"
ST_SAFE = "SAFE STATE"




class MissionState(State):
    def __init__(self, oc_next_state: str):
        super().__init__([oc_next_state, OC_MISSION_ABORTED])
        self.oc_next_state = oc_next_state

    def execute(self, blackboard: Blackboard):
        try:
            outcome = self.task()
            return outcome
        except Exception as e:
            self._log(f"\033[31mException in mission state, switching to safe state!\n{e}")
            traceback.print_exc()
            return OC_MISSION_ABORTED

    def task(self):
        raise NotImplementedError

    def _log(self, msg: str):
        print(f"\033[34m[ITR_STATEMACHINE]: {msg} \033[0m")


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
            self._log("Starting Statemachine.")
            sleep(2)
            return OC_START
        else:
            self._log("Reached end, exiting state machine.")
            return OC_END

    def _log(self, msg: str):
        print(f"\033[34m[ITR_STATEMACHINE]: {msg} \033[0m")


class Arm(MissionState):
    def __init__(self, oc_next_state: str, comms: Comms):
        super().__init__(oc_next_state)
        self.comms = comms

    def task(self):
        self.comms.cmd_set_origin()
        sleep(1)
        self.comms.cmd_hold()  # Needed to recover from other states that don't allow arming, such as safe recovery
        sleep(1)
        self.comms.cmd_arm()
        sleep(2)
        if self.comms.get_status()["arm"]:
            self._log("Vehicle armed!")
            return self.oc_next_state
        else:
            YASMIN_LOG_ERROR("Vehicle could not be armed.")
            return OC_MISSION_ABORTED


class Takeoff(MissionState):
    def __init__(self, oc_next_state: str, comms: Comms, altitude: float = 1.5):
        super().__init__(oc_next_state)
        self.comms = comms
        self.altitude = altitude

    def task(self):
        self.comms.cmd_takeoff()

        max_checks = 15
        check_count = 0
        sleep(3)
        while (
            self.comms.get_status()["nav"] != VehicleStatus.NAVIGATION_STATE_AUTO_LOITER
        ):
            check_count += 1
            sleep(0.2)
            if check_count >= max_checks:
                YASMIN_LOG_ERROR("Could not complete takeoff!")
                return OC_MISSION_ABORTED

        self._log("Takeoff Completed.")
        return self.oc_next_state


class Hover(MissionState):
    def __init__(self, oc_next_state: str, comms: Comms, hovertime_s: int = 10):
        super().__init__(oc_next_state)
        self.comms = comms
        self.hovertime_s = hovertime_s

    def task(self):
        self.comms.cmd_hold()

        if self.comms.cmd_is_success():
            self._log(f"Hovering for {self.hovertime_s}")
            sleep(self.hovertime_s)
            return self.oc_next_state
        else:
            YASMIN_LOG_ERROR("Could not enable hovering.")
            return OC_MISSION_ABORTED


class RealtimeControllerState(MissionState):
    def __init__(
        self,
        oc_next_state: str,
        comms: Comms,
        controller: Controller,
        Tsim: int,
        Tstep: float,
        input_type: Literal[
            "position",
            "velocity",
            "acceleration",
            "attitude",
            "body_rate",
            "thrust_and_torque",
            "direct_actuator",
        ] = "thrust_and_torque",
        hover_throttle=0.725,
        debug=False,
    ):
        super().__init__(oc_next_state)

        self._ctrl = controller
        self.comms = comms
        self._Tsim = Tsim
        self._Tstep = Tstep
        self._input_type = input_type
        self._debug = debug
        self.offboard_retry_counter = 0
        self.offboard_retry_max = 20
        self.hover_throttle = hover_throttle
        
        # Thread control variables
        self._offboard_thread = None
        self._stop_offboard = False
        self._offboard_active = False

        self._scheduler = sched.scheduler(timefunc=time.monotonic, delayfunc=time.sleep)
        self._period = self._Tstep

        self._next_time = time.monotonic() + self._period
        self.step = 0

    def _offboard_keepalive_thread(self):
        """Background thread to maintain offboard mode and send zero setpoints when needed."""
        while not self._stop_offboard:
            # Send offboard keepalive sequence
            self.comms.offboard_keepalive(self._input_type)
            self.comms.cmd_offboard_mode()
            self.comms.offboard_keepalive(self._input_type)

            # Check navigation mode
            nav_mode = self.comms.get_status()["nav"]

            if nav_mode == 14:
                # Offboard mode active - controller will handle setpoints
                self._offboard_active = True
            else:
                # Not in offboard yet - send zero setpoint to hover
                self.comms.send_torque_setpoint(np.array([0.0, 0.0, 0.0]))
                self.comms.send_thrust_setpoint(self.hover_throttle)

            time.sleep(0.1)  # Run at ~10Hz

    def task(self):
        # Start offboard keepalive thread
        self._log("Starting offboard keepalive thread")
        self._stop_offboard = False
        self._offboard_thread = Thread(target=self._offboard_keepalive_thread, daemon=True)
        self._offboard_thread.start()

        # Wait for offboard mode to activate
        self._log("Waiting for offboard mode activation")
        while not self._offboard_active:
            time.sleep(0.05)

        self._log("Offboard mode active, starting controller")

        # Spin off a periodic thread with the controller and wait until it exits
        self._next_time = time.monotonic() + self._period
        self._scheduler.enterabs(self._next_time, 1, self._ctrl_task)
        self._scheduler.run()  # Waits until no scheduled controller steps are left

        # Stop offboard thread
        self._stop_offboard = True
        if self._offboard_thread:
            self._offboard_thread.join(timeout=1.0)

        return self.oc_next_state

    def _ctrl_task(self):
        obs = self._ctrl.get_observation()
        ref = self._ctrl.get_reference()
        input = self._ctrl(ref, obs)
        self._ctrl.apply_input(input)

        if self._debug:
            self._log(f"Observation: {obs}")
            self._log(f"Reference: {ref}")
            self._log(f"Control Input: {input}")

        if self._ctrl.is_finished():
            return
        else:
            self.step += 1

        self._next_time += self._period

        if self.step >= self._Tsim:
            return

        self._scheduler.enterabs(self._next_time, 1, self._ctrl_task)

    def init_hook(self):
        return

    def exit_hook(self):
        return

    def _log(self, msg: str):
        print(f"\033[34m[ITR_CONTROLLER]: {msg} \033[0m")


# We will not use the ControllerState here, because it is real-time and periodic (real-time)
# We instead wrap the MissionState to add a more sim focused mission state
class SimControllerState(MissionState):
    def __init__(
        self,
        oc_next_state: str,
        comms: Comms,
        sim, # Gazebo Control Node
        controller: Controller,
        Tsim: int,
        Tstep: float,
        hover_throttle: float = 0.72,
        debug: bool = False,
    ):
        """
        """
        super().__init__(oc_next_state)

        self.comms = comms
        self.sim = sim
        self.controller = controller
        self._debug = debug
        self._input_type = "thrust_and_torque"
        self.hover_throttle = hover_throttle

        self.Tstep = Tstep
        self.Tsim = Tsim
        self.step = 0

        # Thread control variables
        self._offboard_thread = None
        self._stop_offboard = False
        self._offboard_active = False

    def _offboard_keepalive_thread(self):
        """Background thread to maintain offboard mode and send zero setpoints when needed."""
        while not self._stop_offboard:
            # Send offboard keepalive sequence
            self.comms.offboard_keepalive(self._input_type)
            self.comms.cmd_offboard_mode()
            self.comms.offboard_keepalive(self._input_type)

            # Check navigation mode
            nav_mode = self.comms.get_status()["nav"]

            if nav_mode == 14:
                # Offboard mode active - controller will handle setpoints
                self._offboard_active = True
            else:
                # Not in offboard yet - send zero setpoint to hover
                self.comms.send_torque_setpoint(np.array([0.0, 0.0, 0.0]))
                self.comms.send_thrust_setpoint(self.hover_throttle)

            time.sleep(0.1)  # Run at ~10Hz

    def task(self):
        # This main task is blocking and is called by the State Machine

        # Start offboard keepalive thread
        self._log("Starting offboard keepalive thread")
        self._stop_offboard = False
        self._offboard_thread = Thread(target=self._offboard_keepalive_thread, daemon=True)
        self._offboard_thread.start()

        # Wait for offboard mode to activate
        self._log("Waiting for offboard mode activation")
        while not self._offboard_active:
            time.sleep(0.05)

        self._log("Offboard mode active, starting controller")

        # Get starting position and initialize
        obs = self.controller.get_observation()
        # self.x_VAL[:, 0] = obs


        # Main sim loop
        for k in range(self.Tsim):
            self.sim.pause()

            # Compute control input at current state
            u = self._ctrl_task()

            self.sim.resume()

            # Apply control input at 1ms intervals using timer-based scheduling
            start_time = time.monotonic()
            next_time = start_time + 0.001
            num_substeps = int(self.Tstep / 0.001)

            for _ in range(num_substeps):
                self.controller.apply_input(u)

                # Sleep until next scheduled time
                sleep_duration = next_time - time.monotonic()
                if sleep_duration > 0:
                    time.sleep(sleep_duration)

                next_time += 0.001

            self.step += 1

        # Stop offboard thread
        self._stop_offboard = True
        if self._offboard_thread:
            self._offboard_thread.join(timeout=1.0)

        return self.oc_next_state

    def _ctrl_task(self):
        # Wrap the controller in the offboard-mode enable logic
        # Also wrap in Gazebo simulation start/stop logic
        # ros2 service call /world/itr/control ros_gz_interfaces/srv/ControlWorld "{world_control: {pause: false}}"

        # Get measurement
        obs = self.controller.get_observation()

        # Get reference (trajectory)
        ref = self.controller.get_reference()

        # Compute control input
        u = self._ctrl(ref, obs)

        if self._debug:
            self._log(f"Observation: {obs}")
            self._log(f"Reference: {ref}")
            self._log(f"Control Input: {u}")

        return u

    def _ctrl(self, ref, obs):
        u = self.controller(ref, obs)

        return u

    def _log(self, msg: str):
        print(f"\033[34m[ITR_CONTROLLER]: {msg} \033[0m")


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
        set_log_level(LogLevel.WARN)
        YasminViewerPub(self.sm, "ITR_X500")

    def start(self):
        try:
            self.sm()
        except KeyboardInterrupt:
            if self.sm.is_running():
                self.sm.cancel_state()


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
