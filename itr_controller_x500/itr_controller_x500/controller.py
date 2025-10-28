# ROS2 Interface
import rclpy

# PX4 Message Definitions
from px4_msgs.msg import VehicleStatus
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)


class Controller(Node):
    def __init__(self):
        super().__init__("itr_controller_x500")

        # QoS settings used for PX4 topics:
        qos_profile_sub = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,  # As fast as possible, no retransmissions
            durability=QoSDurabilityPolicy.VOLATILE,  # Don't keep message history
            history=QoSHistoryPolicy.KEEP_LAST,  # Only keep last N messages
            depth=0,  # only latest message in history
        )

        self.status_sub = self.create_subscription(
            VehicleStatus,
            "fmu/out/vehicle_status_v1",
            self.vehicle_status_callback,
            qos_profile_sub,
        )

    def vehicle_status_callback(self, msg):
        self.get_logger().info(f"Arming State: {msg.arming_state}")


def main(args=None):
    rclpy.init(args=args)
    controller = Controller()

    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        controller.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
