
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)

from px4_msgs.msg import (
    VehicleCommand,
    OffboardControlMode,
    VehicleStatus
)

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
            depth=1
)

class Comms(Node):
    def __init__(self):
        self.cmd_pub = self.create_publisher(
            VehicleCommand,
            "/fmu/in/vehicle_command",
            QOS_PROFILE_PX4_PUB
        )

        self.ob_pub = self.create_publisher(
            OffboardControlMode,
            "/fmu/in/offboard_control_mode",
            QOS_PROFILE_PX4_PUB
        )

        self.status_sub = None
    
    def get_timestamp(self):
        return int(self.get_clock().now().nanoseconds / 1000)
    
    def make_cmd(self, cmd, param1 = 0, param2 = 0, param3 = 0, param4 = 0, param5 = 0, param6 = 0, param7 = 0):
        msg = VehicleCommand()
        msg.timestamp = self.get_timestamp()
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

    # For command number definitions, refer to: https://github.com/PX4/PX4-Autopilot/blob/main/msg/versioned/VehicleCommand.msg

    def cmd_takeoff(self, altitude = 1.5):
        msg = self.make_cmd(22, param7=altitude)
        self.cmd_pub.publish(msg)
    
    def cmd_rtl(self):
        msg = self.make_cmd(20)
        self.cmd_pub.publish(msg)

    def cmd_land(self, yaw=0.0):
        msg = self.make_cmd(21, param4=yaw)
        self.cmd_pub.publish(msg)

    def send_offboard_mode(self, mode=str):
        msg = OffboardControlMode()
        msg.timestamp = self.get_timestamp()
        msg[mode] = True
        self.ob_pub.publish(msg)
    
    def subscribe_vehicle_status(self, callback):
        self.status_sub = self.create_subscription(
            VehicleStatus,
            "/fmu/out/vehicle_status_v1",
            QOS_PROFILE_PX4_SUB,
            callback
        )
    