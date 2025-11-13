import signal
import sys
from threading import Thread

import rclpy
from rclpy.node import Node
from scipy.spatial.transform import Rotation

from itr_comms_x500 import Comms
from itr_controller_x500 import Controller
from itr_statemachine_x500 import (
    FSM,
    OC_MISSION_FINISHED,
    Arm,
    ControllerState,
    Hover,
    Mission,
    Takeoff,
)


def quat2euler(quat):
    r = Rotation.from_quat(quat)
    euler = r.as_euler("xyz", degrees=True)
    return euler


# class MPCState(MissionState):
#     def __init__(
#         self,
#         oc_next_state: str,
#         comms: Comms,
#         reference,
#         interval_s: float,
#         horizon: int = 20,
#     ):
#         super().__init__(oc_next_state, comms)

#         model = Quadcopter(X500, interval_s)

#         Q = np.diag([1.0, 1.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
#         R = np.diag([1.0, 0.5, 0.5, 0.5])
#         self.horizon = horizon
#         self.ctrl = MPC(model.Ad_sparse, model.Bd_sparse, Q, R, horizon, debug=True)

#         self.ref = reference

#         self.active = False

#     def task(self):
#         step = 0
#         self.comms.offboard_keepalive("thrust_and_torque")
#         self.comms.offboard_keepalive("thrust_and_torque")
#         self.comms.cmd_offboard_mode()

#         if self.comms.cmd_is_success():
#             pass
#         else:
#             return OC_MISSION_ABORTED
#         while True:
#             obs = np.zeros(12)
#             obs[0:3] = self.comms.get_position()
#             obs[3:6] = self.comms.get_velocity()
#             obs[6:9] = quat2euler(self.comms.get_orientation())
#             obs[9:12] = self.comms.get_angular_velocity()

#             if step * self.horizon + self.horizon > len(self.ref.T):
#                 self.active = False
#                 print("test")
#                 return self.oc_next_state
#             else:
#                 ref = self.ref[
#                     :, step * self.horizon : step * self.horizon + self.horizon
#                 ]
#                 action = self.ctrl(ref, obs)
#                 self.comms.send_thrust_setpoint(action[0])
#                 self.comms.send_torque_setpoint(action[1:4])
#                 self.comms.offboard_keepalive("thrust_and_torque")
#                 step += 1


class NothingController(Controller):
    def __init__(self):
        return

    def __call__(self, ref, obs):
        self._log("Hello from NothingController")


class NothingControllerState(ControllerState):
    def __init__(self, oc_next_state: str, comms: Comms):
        super().__init__(oc_next_state, NothingController(), 1)
        self.comms = comms
        self.counter = 0
        self.max = 100

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


def nothing_mission():
    rclpy.init()

    comms = Comms(debug=True)

    mission = Mission()
    mission.add_state(Arm("take off", comms), "ARM", "take off", "TAKEOFF")
    mission.add_state(Takeoff("hover", comms), "TAKEOFF", "hover", "HOVER")
    mission.add_state(
        Hover("start controller", comms, 2), "HOVER", "start controller", "CONTROLLER"
    )
    mission.add_state(
        NothingControllerState(OC_MISSION_FINISHED, comms),
        "CONTROLLER",
        OC_MISSION_FINISHED,
        OC_MISSION_FINISHED,
    )

    fsm = FSM(mission, comms, debug=True)

    launch(fsm, comms)
    return


def handle_interrupt(signum, frame, comms: Comms):
    """
    Handle Ctrl+C (SIGINT) signal before ROS shuts down.
    This function sends the RTL command and then shuts down ROS gracefully.
    """
    print("\nAbort detected, landing drone...")
    # Send RTL (Return to Launch) command to land the drone
    comms.cmd_rtl()

    # Gracefully shutdown ROS
    if rclpy.ok():
        rclpy.shutdown()

    # Exit the program after shutdown
    sys.exit(0)


def launch(fsm: FSM, comms: Comms):
    # land the drone on ctrl+c
    signal.signal(
        signal.SIGINT, lambda signum, frame: handle_interrupt(signum, frame, comms)
    )

    def spin_node(node: Node):
        rclpy.spin(node)

    try:
        Thread(target=spin_node, args=(comms,), daemon=True).start()
        fsm.start()
    except Exception as e:
        print(f"Exception raised:\n{e}")
        # Send RTL command:
        comms.cmd_rtl()
    finally:
        if rclpy.ok():
            rclpy.shutdown()


def mission_basic():
    rclpy.init()

    comms = Comms(debug=True)

    mission = Mission()
    mission.add_state(Arm("take off", comms), "ARM", "take off", "TAKEOFF")
    mission.add_state(Takeoff("hover", comms), "TAKEOFF", "hover", "HOVER")
    mission.add_state(
        Hover(OC_MISSION_FINISHED, comms, 5),
        "HOVER",
        OC_MISSION_FINISHED,
        OC_MISSION_FINISHED,
    )

    fsm = FSM(mission, comms, debug=True)

    launch(fsm, comms)
    return


def main():
    nothing_mission()
    return


if __name__ == "__main__":
    main()
