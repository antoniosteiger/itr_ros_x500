from typing import Callable, Literal, TypedDict

import numpy as np
import numpy.typing as npt
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.publisher import Publisher
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from rclpy.subscription import Subscription
from scipy.spatial.transform import Rotation as R

from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleAttitudeSetpoint,
    VehicleCommand,
    VehicleCommandAck,
    VehicleOdometry,
    VehicleStatus,
    VehicleThrustSetpoint,
    VehicleTorqueSetpoint,
)

POSE_TOPIC = "/pose"
DEFAULT_TAKEOFF_ALTITUDE = 1.5

QOS_PROFILE_PX4_SUB = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,  # As fast as possible, no retransmissions
    durability=QoSDurabilityPolicy.VOLATILE,  # Don't keep message history
    history=QoSHistoryPolicy.KEEP_LAST,  # Only keep last N messages
    depth=1,  # only latest message in history
)

QOS_PROFILE_PX4_PUB = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=1,
)


class Status(TypedDict):
    nav: int
    arm: bool
    checks: bool


class Comms(Node):
    def __init__(self, debug=False):
        super().__init__("itr_comms_x500")

        self._pubs: dict[str, Publisher] = {}
        self._subs: dict[str, Subscription] = {}

        self.position = np.array([0.0, 0.0, 0.0])
        self.velocity = np.array([0.0, 0.0, 0.0])
        self.orientation = np.array([0.0, 0.0, 0.0, 1.0])
        self.angular_velocity = np.array([0.0, 0.0, 0.0])
        self.status: Status = {
            "nav": VehicleStatus.NAVIGATION_STATE_MAX,
            "arm": VehicleStatus.ARMING_STATE_DISARMED,
            "checks": False,
        }
        self._cmd_sent = 0
        self._cmd_ack_result = VehicleCommandAck.VEHICLE_CMD_RESULT_DENIED
        self._cmd_ack_id: int = -1

        self.set_debug(debug)
        self._init_subs()
        self._init_pubs()

    def set_debug(self, debug: bool = True):
        self.debug = debug

    def _log(self, msg: str):
        # self.get_logger().info(f"\033[35m{msg}")
        print(f"\033[35m[ITR_COMMS]: {msg} \033[0m")

    def _get_timestamp(self):
        return int(self.get_clock().now().nanoseconds / 1000)

    def _make_sub(self, type, topic: str, callback: Callable):
        self._subs[topic] = self.create_subscription(
            type,
            topic,
            callback,
            QOS_PROFILE_PX4_SUB,
        )

    def _init_subs(self):
        self._make_sub(Odometry, POSE_TOPIC, self._pose_callback)
        self._make_sub(
            VehicleStatus, "/fmu/out/vehicle_status_v1", self._status_callback
        )
        self._make_sub(
            VehicleCommandAck, "/fmu/out/vehicle_command_ack", self._cmd_ack_callback
        )
        self._make_sub(
            VehicleOdometry, "/fmu/out/vehicle_odometry", self._odometry_callback
        )

    def _pose_callback(self, msg: Odometry):
        self.position[0] = msg.pose.pose.position.x
        self.position[1] = msg.pose.pose.position.y
        self.position[2] = msg.pose.pose.position.z

        self.orientation[0] = msg.pose.pose.orientation.x
        self.orientation[1] = msg.pose.pose.orientation.y
        self.orientation[2] = msg.pose.pose.orientation.z
        self.orientation[3] = msg.pose.pose.orientation.w

    def _status_callback(self, msg: VehicleStatus):
        self.status["nav"] = msg.nav_state
        if msg.arming_state == VehicleStatus.ARMING_STATE_ARMED:
            self.status["arm"] = True
        else:
            self.status["arm"] = False
        self.status["checks"] = msg.pre_flight_checks_pass

    def _cmd_ack_callback(self, msg: VehicleCommandAck):
        self._cmd_ack_result = msg.result
        self._cmd_ack_id = msg.command

        if self.debug:
            if self.cmd_is_success():
                self._log(f"Command {self._cmd_ack_id} succeeded.")
            else:
                self._log(f"Command {self._cmd_ack_id} FAILED.")

    def _odometry_callback(self, msg: VehicleOdometry):
        self.velocity[0] = msg.velocity[0]
        self.velocity[1] = msg.velocity[1]
        self.velocity[2] = msg.velocity[2]

        self.angular_velocity[0] = msg.angular_velocity[0]
        self.angular_velocity[1] = msg.angular_velocity[1]
        self.angular_velocity[2] = msg.angular_velocity[2]

    def get_position(self) -> npt.NDArray[np.float64]:
        return self.position

    def get_velocity(self) -> npt.NDArray[np.float64]:
        return self.velocity

    def get_orientation_quat(self) -> npt.NDArray[np.float64]:
        return self.orientation

    def get_orientation_euler(self) -> npt.NDArray[np.float64]:
        quat = self.get_orientation_quat()
        r = R.from_quat(quat)
        euler = r.as_euler("xyz", degrees=False)
        return euler

    def get_angular_velocity(self) -> npt.NDArray[np.float64]:
        return self.angular_velocity

    def get_status(self) -> Status:
        return self.status

    def _make_pub(self, type, topic: str):
        self._pubs[topic] = self.create_publisher(type, topic, QOS_PROFILE_PX4_PUB)

    def _init_pubs(self):
        self._make_pub(VehicleCommand, "/fmu/in/vehicle_command")
        self._make_pub(OffboardControlMode, "/fmu/in/offboard_control_mode")
        self._make_pub(VehicleThrustSetpoint, "/fmu/in/vehicle_thrust_setpoint")
        self._make_pub(VehicleTorqueSetpoint, "/fmu/in/vehicle_torque_setpoint")
        self._make_pub(TrajectorySetpoint, "/fmu/in/trajectory_setpoint")
        self._make_pub(VehicleAttitudeSetpoint, "/fmu/in/vehicle_attitude_setpoint")

    def _make_cmd_msg(
        self,
        cmd,
        param1=0.0,
        param2=0.0,
        param3=0.0,
        param4=0.0,
        param5=0.0,
        param6=0.0,
        param7=0.0,
    ) -> VehicleCommand:
        msg = VehicleCommand()
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

    def _send_cmd(self, msg: VehicleCommand):
        self._cmd_sent = msg.command
        self._pubs["/fmu/in/vehicle_command"].publish(msg)
        if self.debug:
            self._log(f"Command {msg.command} has been sent.")

    def cmd_set_origin(
        self, latitude: float = 0.0, longitude: float = 0.0, altitude: float = 0.0
    ):
        cmd = VehicleCommand.VEHICLE_CMD_SET_GPS_GLOBAL_ORIGIN
        msg = self._make_cmd_msg(
            cmd, param5=latitude, param6=longitude, param7=altitude
        )
        self._send_cmd(msg)

    # def cmd_takeoff(
    #     self,
    #     latitude: float = 0.0,
    #     longitude: float = 0.0,
    #     altitude: float = DEFAULT_TAKEOFF_ALTITUDE,
    #     yaw: float = 0.0
    # ):
    #     cmd = VehicleCommand.VEHICLE_CMD_NAV_TAKEOFF
    #     msg = self._make_cmd_msg(cmd)
    #     self._send_cmd(msg)

    def cmd_rtl(self):
        cmd = VehicleCommand.VEHICLE_CMD_NAV_RETURN_TO_LAUNCH
        msg = self._make_cmd_msg(cmd)
        self._send_cmd(msg)

    def cmd_arm(self):
        cmd = VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM
        msg = self._make_cmd_msg(cmd, param1=1.0)
        self._send_cmd(msg)

    def cmd_offboard_mode(self):
        cmd = VehicleCommand.VEHICLE_CMD_DO_SET_MODE
        msg = self._make_cmd_msg(cmd, param1=1.0, param2=6.0)
        self._send_cmd(msg)

    def cmd_takeoff(self):
        cmd = VehicleCommand.VEHICLE_CMD_DO_SET_MODE
        msg = self._make_cmd_msg(cmd, param1=1.0, param2=4.0, param3=2.0)
        self._send_cmd(msg)

    def cmd_hold(self):
        cmd = VehicleCommand.VEHICLE_CMD_DO_SET_MODE
        msg = self._make_cmd_msg(cmd, param1=1.0, param2=4.0, param3=3.0)
        self._send_cmd(msg)

    def cmd_reboot(self):
        cmd = VehicleCommand.VEHICLE_CMD_PREFLIGHT_REBOOT_SHUTDOWN
        msg = self._make_cmd_msg(cmd, param1=1.0)
        self._send_cmd(msg)

    def cmd_is_success(self):
        if (
            self._cmd_sent == self._cmd_ack_id
            and self._cmd_ack_result == VehicleCommandAck.VEHICLE_CMD_RESULT_ACCEPTED
        ):
            return True
        else:
            return False

    def offboard_keepalive(
        self,
        desired_setpoint: Literal[
            "position",
            "velocity",
            "acceleration",
            "attitude",
            "body_rate",
            "thrust_and_torque",
            "direct_actuator",
        ],
    ):
        msg = OffboardControlMode()
        msg.timestamp = self._get_timestamp()
        msg.position = False
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.thrust_and_torque = False
        msg.direct_actuator = False
        match desired_setpoint:
            case "position":
                msg.position = True
            case "velocity":
                msg.velocity = True
            case "acceleration":
                msg.acceleration = True
            case "attitude":
                msg.attitude = True
            case "body_rate":
                msg.body_rate = True
            case "thrust_and_torque":
                msg.thrust_and_torque = True
            case "direct_actuator":
                msg.direct_actuator = True

        self._pubs["/fmu/in/offboard_control_mode"].publish(msg)
        if self.debug:
            self._log("Sent offboard keepalive message.")

    def send_thrust_setpoint(self, setpoint: float):
        msg = VehicleThrustSetpoint()
        msg.timestamp = self._get_timestamp()
        msg.timestamp_sample = self._get_timestamp()
        msg.xyz = np.array([0.0, 0.0, setpoint])
        self._pubs["/fmu/in/vehicle_thrust_setpoint"].publish(msg)

    def send_torque_setpoint(self, setpoint: npt.NDArray[np.float64]):
        msg = VehicleTorqueSetpoint()
        msg.timestamp = self._get_timestamp()
        msg.timestamp_sample = self._get_timestamp()
        msg.xyz = setpoint  # TODO: Torque normalization
        self._pubs["/fmu/in/vehicle_torque_setpoint"].publish(msg)

    def send_velocity_setpoint(self, setpoint: npt.NDArray[np.float64]):
        msg = TrajectorySetpoint()
        msg.timestamp = self._get_timestamp()
        msg.velocity = setpoint
        self._pubs["/fmu/in/trajectory_setpoint"].publish(msg)

        if self.debug:
            self._log(f"Sent velocity setpoint {setpoint}")

    def send_position_setpoint(self, setpoint: npt.NDArray[np.float64]):
        msg = TrajectorySetpoint()
        msg.timestamp = self._get_timestamp()
        msg.position = setpoint
        msg.yaw = 1.57079  # 90 degrees
        self._pubs["/fmu/in/trajectory_setpoint"].publish(msg)

    def send_attitude_setpoint(self, setpoint: npt.NDArray[np.float64]):
        msg = VehicleAttitudeSetpoint()
        msg.timestamp = self._get_timestamp()
        msg.thrust_body = [0.0, 0.0, 0.55]
        msg.q_d = setpoint
        self._pubs["/fmu/in/vehicle_attitude_setpoint"].publish(msg)

    def send_acceleration_setpoint(self, setpoint: npt.NDArray[np.float64]):
        msg = TrajectorySetpoint()
        msg.timestamp = self._get_timestamp()
        msg.acceleration = setpoint
        msg.yaw = 1.5709  # 90 deg
        self._pubs["/fmu/in/trajectory_setpoint"].publish(msg)
