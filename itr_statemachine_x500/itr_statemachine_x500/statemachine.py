import sched
import time
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
    logs,
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


def log(msg: str):
    print(f"\033[34m[ITR_STATEMACHINE]: {msg} \033[0m")


class MissionState(State):
    def __init__(self, oc_next_state: str):
        super().__init__([oc_next_state, OC_MISSION_ABORTED])
        self.oc_next_state = oc_next_state

    def execute(self, blackboard: Blackboard):
        try:
            outcome = self.task()
            return outcome
        except Exception as e:
            log(f"\033[31mException in mission state, switching to safe state!\n{e}")
            return OC_MISSION_ABORTED

    def task(self):
        raise NotImplementedError


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
            log("Vehicle armed!")
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

        log("Takeoff Completed.")
        return self.oc_next_state


class Hover(MissionState):
    def __init__(self, oc_next_state: str, comms: Comms, hovertime_s: int = 10):
        super().__init__(oc_next_state)
        self.comms = comms
        self.hovertime_s = hovertime_s

    def task(self):
        self.comms.cmd_hold()

        if self.comms.cmd_is_success():
            log(f"Hovering for {self.hovertime_s}")
            sleep(self.hovertime_s)
            return self.oc_next_state
        else:
            YASMIN_LOG_ERROR("Could not enable hovering.")
            return OC_MISSION_ABORTED


class ControllerState(MissionState, ABC):
    def __init__(
        self,
        oc_next_state: str,
        controller: Controller,
        input_type: Literal[
            "position",
            "velocity",
            "acceleration",
            "attitude",
            "body_rate",
            "thrust_and_torque",
            "direct_actuator",
        ],
        comms: Comms,
        rate: int = 50,
        debug=False,
    ):
        super().__init__(oc_next_state)

        self._ctrl = controller
        self.comms = comms
        self._rate = rate
        self._input_type = input_type
        self._debug = debug
        self.offboard_retry_counter = 0
        self.offboard_retry_max = 20

        self._scheduler = sched.scheduler(timefunc=time.monotonic, delayfunc=time.sleep)
        self._period = 1 / rate

        self._next_time = time.monotonic() + self._period
        self.step = 0

    def _ctrl_task(self):
        # Wrap the controller in the offboard-mode enable logic
        if self.comms.get_status()["nav"] != VehicleStatus.NAVIGATION_STATE_OFFBOARD:
            self.comms.offboard_keepalive(self._input_type)
            self.comms.cmd_offboard_mode()
            self.offboard_retry_counter += 1
            if self.offboard_retry_counter > self.offboard_retry_max:
                raise Exception("Could not enable offboard mode!")
        else:
            self.comms.offboard_keepalive(self._input_type)
            obs = self.get_observation()
            ref = self.get_reference()
            input = self._ctrl(ref, obs)
            self.apply_input(input)
        if self.is_finished():
            return
        else:
            self.step += 1
            self._next_time += self._period
            self._scheduler.enterabs(self._next_time, 1, self._ctrl_task)

    def task(self):
        # Call the init_hook before the controller starts
        self.init_hook()
        # Spin off a periodic thread with the controller and wait until it exits
        self._next_time = time.monotonic() + self._period
        self._scheduler.enterabs(self._next_time, 1, self._ctrl_task)
        self._scheduler.run()  # Waits until no scheduled controller steps are left

        # Call the end hook after the controller finished
        self.exit_hook()
        return self.oc_next_state

    def init_hook(self):
        return

    def exit_hook(self):
        return

    @abstractmethod
    def get_observation(self):
        raise NotImplementedError

    @abstractmethod
    def get_reference(self):
        raise NotImplementedError

    @abstractmethod
    def apply_input(self, input):
        raise NotImplementedError

    @abstractmethod
    def is_finished(self):
        raise NotImplementedError


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
