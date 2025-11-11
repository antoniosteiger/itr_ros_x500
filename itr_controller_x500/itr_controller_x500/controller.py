# from scipy.spatial.transform import Rotation as R
import time

import numpy as np

# ROS2 Interface
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from scipy.spatial.transform import Rotation as R

# PX4 Interface
from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleAttitudeSetpoint,
    VehicleCommand,
    VehicleStatus,
    VehicleThrustSetpoint,
)

QOS_PROFILE_PX4_SUB = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,  # As fast as possible, no retransmissions
    durability=QoSDurabilityPolicy.VOLATILE,  # Don't keep message history
    history=QoSHistoryPolicy.KEEP_LAST,  # Only keep last N messages
    depth=0,  # only latest message in history
)

QOS_PROFILE_PX4_PUB = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=0,
)


class Controller(Node):
    def __init__(self, name, sampling_interval_s=0.05, debug=False):
        super().__init__(name)
        self.debug = debug
        self.sampling_interval_s = sampling_interval_s
        self.state = 0
        self.start_time = None

        self.nav_state = VehicleStatus.NAVIGATION_STATE_MAX
        self.arming_state = VehicleStatus.ARMING_STATE_DISARMED

        self.theta = 0.0
        self.radius = 1.5
        self.omega = 0.2
        self.altitude = 1.5

        self.position = np.array([0.0, 0.0, 0.0])
        self.velocity = np.array([0.0, 0.0, 0.0])
        self.orientation = np.array([0.0, 0.0, 0.0, 1.0])
        self.angular_velocity = np.array([0.0, 0.0, 0.0])

        self.subs = {}
        self.pubs = {}

        self._init_topics()

        # Set EKF Origin (Makes Geofencing and RTL possible with MoCap)
        self.cmd_set_ekf_origin()
        time.sleep(1.0)  # Wait for command to complete.

        self.control_timer = self.create_timer(sampling_interval_s, self.run)

    def _pose_callback(self, msg: Odometry):
        self.position[0] = msg.pose.pose.position.x
        self.position[1] = msg.pose.pose.position.y
        self.position[2] = msg.pose.pose.position.z

        self.orientation[0] = msg.pose.pose.orientation.x
        self.orientation[1] = msg.pose.pose.orientation.y
        self.orientation[2] = msg.pose.pose.orientation.z
        self.orientation[3] = msg.pose.pose.orientation.w

        # if self.debug:
        #     self.get_logger().info(f"Position: {self.position}")
        #     self.get_logger().info(f"Orientation: {self.orientation}")

    def _status_callback(self, msg: VehicleStatus):
        # self.get_logger().info(f"NAV_STATUS: {msg.nav_state}")
        # self.get_logger().info(f"TARGET OFFBOARD STATUS: {VehicleStatus.NAVIGATION_STATE_OFFBOARD}")

        self.nav_state = msg.nav_state
        self.arming_state = msg.arming_state

    def get_euler_orientation(self):
        roll, pitch, yaw = R.from_quat(self.orientation).as_euler("xyz", degrees=False)
        return np.array([roll, pitch, yaw])

    def compute_control(self):
        thrust = [0, 0, 0.5]
        attitude = [0, 0, 0, 1]

        return thrust, attitude

    def run(self):
        if self.state == 0 and self.arming_state == VehicleStatus.ARMING_STATE_DISARMED:
            self.cmd_arm()
        if (
            self.arming_state == VehicleStatus.ARMING_STATE_ARMED
            and self.nav_state == VehicleStatus.NAVIGATION_STATE_AUTO_LOITER
            and self.position[2] < 1.2
        ):
            self.state = 1
            print(self.position[2])
            self.cmd_takeoff()
            self.start_time = int(self.get_clock().now().nanoseconds / 1000)
        if (
            self.arming_state == VehicleStatus.ARMING_STATE_ARMED
            and self.nav_state == VehicleStatus.NAVIGATION_STATE_AUTO_TAKEOFF
        ):
            pass
        if (
            self.arming_state == VehicleStatus.ARMING_STATE_ARMED
            and self.nav_state == VehicleStatus.NAVIGATION_STATE_AUTO_LOITER
            and self.position[2] > 1.2
        ):
            self.state = 2
            self.cmd_activate_offboard()
            self._send_offboard()
        if (
            self.arming_state == VehicleStatus.ARMING_STATE_ARMED
            and self.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD
        ):
            self._send_offboard()
            trajectory_msg = TrajectorySetpoint()
            trajectory_msg.position[0] = self.radius * np.cos(self.theta)
            trajectory_msg.position[1] = self.radius * np.sin(self.theta)
            trajectory_msg.position[2] = -self.altitude
            self.pubs["trajectory_setpoint"].publish(trajectory_msg)

            self.theta = self.theta + self.omega * self.sampling_interval_s
            if (
                int(self.get_clock().now().nanoseconds / 1000) - self.start_time
                > 10000000
            ):
                # self.cmd_hover()
                self.cmd_deactivate_offboard()
                self.state = 3
            if (
                self.state == 3
                and self.arming_state == VehicleStatus.ARMING_STATE_ARMED
                and self.nav_state == VehicleStatus.NAVIGATION_STATE_AUTO_LOITER
                and self.position[2] > 1.2
            ):
                self.cmd_precland()
                print("success")
                return

        # return
        # self.cmd_takeoff()

    def apply_control(self, thrust, attitude):
        # thrust_msg = VehicleThrustSetpoint()
        # thrust_msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        # thrust_msg.timestamp_sample = thrust_msg.timestamp
        # thrust_msg.xyz = thrust

        attitude_msg = VehicleAttitudeSetpoint()
        attitude_msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        attitude_msg.q_d = attitude
        attitude_msg.thrust_body = thrust

        # self.pubs["thrust_setpoint"].publish(thrust_msg)
        self.pubs["attitude_setpoint"].publish(attitude_msg)

        if self.debug:
            self.get_logger().info(f"Applied Control: {thrust}, {attitude}")
        return

    def _init_topics(self):
        self.subs["pose"] = self.create_subscription(
            Odometry,
            "/pose",
            self._pose_callback,
            10,
        )

        self.subs["vehicle_status"] = self.create_subscription(
            VehicleStatus,
            "fmu/out/vehicle_status_v1",
            self._status_callback,
            QOS_PROFILE_PX4_SUB,
        )

        self.pubs["thrust_setpoint"] = self.create_publisher(
            VehicleThrustSetpoint, "fmu/in/vehicle_thrust_setpoint", QOS_PROFILE_PX4_PUB
        )

        self.pubs["attitude_setpoint"] = self.create_publisher(
            VehicleAttitudeSetpoint,
            "fmu/in/vehicle_attitude_setpoint",
            QOS_PROFILE_PX4_PUB,
        )

        self.pubs["offboard_control"] = self.create_publisher(
            OffboardControlMode, "fmu/in/offboard_control_mode", QOS_PROFILE_PX4_PUB
        )

        self.pubs["trajectory_setpoint"] = self.create_publisher(
            TrajectorySetpoint, "fmu/in/trajectory_setpoint", QOS_PROFILE_PX4_PUB
        )

        self.pubs["vehicle_command"] = self.create_publisher(
            VehicleCommand, "fmu/in/vehicle_command", QOS_PROFILE_PX4_PUB
        )

    def make_cmd(
        self,
        cmd,
        param1=0.0,
        param2=0.0,
        param3=0.0,
        param4=0.0,
        param5=0.0,
        param6=0.0,
        param7=0.0,
    ):
        msg = VehicleCommand()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.command = cmd
        msg.param1 = param1
        msg.param2 = param2
        msg.param3 = param3
        msg.param4 = param4
        msg.param5 = param5
        msg.param6 = param6
        msg.param7 = param7
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True

        return msg

    def _send_offboard(self):
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        # msg.attitude = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)

        self.pubs["offboard_control"].publish(msg)

    def cmd_takeoff(self, altitude=1.5):
        msg = self.make_cmd(22, param7=altitude)
        self.pubs["vehicle_command"].publish(msg)
        if self.debug:
            print("[cmd] takeoff")

    def cmd_activate_offboard(self):
        msg = self.make_cmd(176, param1=1.0, param2=6.0)
        self.pubs["vehicle_command"].publish(msg)
        if self.debug:
            print("[cmd] activate offboard")

    def cmd_deactivate_offboard(self):
        msg = self.make_cmd(176, param1=0.0, param2=0.0)
        self.pubs["vehicle_command"].publish(msg)
        if self.debug:
            print("[cmd] deactivate offboard")

    def cmd_rtl(self):
        msg = self.make_cmd(20)
        self.pubs["vehicle_command"].publish(msg)
        if self.debug:
            print("[cmd] return to launch")

    def cmd_set_ekf_origin(self):
        msg = self.make_cmd(100000, param5=0.0, param6=0.0, param7=0.0)
        self.pubs["vehicle_command"].publish(msg)
        if self.debug:
            print("[cmd] set_ekf_origin")

    def cmd_arm(self):
        msg = self.make_cmd(400, param1=1.0)
        self.pubs["vehicle_command"].publish(msg)
        if self.debug:
            print("[cmd] arm")

    def cmd_hover(self):
        msg = self.make_cmd(17)
        self.pubs["vehicle_command"].publish(msg)
        if self.debug:
            print("[cmd] hover")

    def cmd_precland(self):
        msg = self.make_cmd(23)
        self.pubs["vehicle_command"].publish(msg)
        if self.debug:
            print("[cmd] hover")


def main(args=None):
    rclpy.init(args=args)
    controller = Controller("Test", debug=True)

    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        controller.cmd_rtl()
        controller.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
