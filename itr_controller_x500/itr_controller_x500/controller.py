import numpy as np
from scipy.spatial.transform import Rotation as R
# ROS2 Interface
import rclpy

# PX4 Message Definitions
from nav_msgs.msg import Odometry
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

        self.position = np.array([0, 0, 0])
        self.velocity = np.array([0, 0, 0])
        self.orientation = np.array([0, 0, 0, 1.0])
        self.angular_velocity = np.array([0, 0, 0])

        # QoS settings used for PX4 topics:
        qos_profile_sub = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,  # As fast as possible, no retransmissions
            durability=QoSDurabilityPolicy.VOLATILE,  # Don't keep message history
            history=QoSHistoryPolicy.KEEP_LAST,  # Only keep last N messages
            depth=1,  # only latest message in history
        )

        self.status_sub = self.create_subscription(
            Odometry,
            "/pose",
            self._pose_callback,
            10,
        )

    def _pose_callback(self, msg: Odometry):
        self.position[0] = msg.pose.pose.position.x
        self.position[1] = msg.pose.pose.position.y
        self.position[2] = msg.pose.pose.position.z

        self.orientation[0] = msg.pose.pose.orientation.x
        self.orientation[1] = msg.pose.pose.orientation.y
        self.orientation[2] = msg.pose.pose.orientation.z
        self.orientation[3] = msg.pose.pose.orientation.w

    def get_euler_orientation(self):
        roll, pitch, yaw = R.from_quat(self.orientation).as_euler('xyz', degrees=False)
        return np.array([roll, pitch, yaw])

    def compute_control(self):
        return
    
    def apply_control(self):
        return



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
