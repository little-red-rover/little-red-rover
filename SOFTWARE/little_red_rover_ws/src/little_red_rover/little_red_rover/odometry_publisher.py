import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg._joint_state import JointState
from nav_msgs.msg._odometry import Odometry

# LRR Odometry Publisher
# Takes in wheel position / velocity information from JointState messages on /joint_states
# Outputs integrated odometry as Odometry message on /odom
# https://control.ros.org/master/doc/ros2_controllers/doc/mobile_robot_kinematics.html#differential-drive-robot


class OdomPublisher(Node):
    def __init__(self):
        super().__init__("odom_publisher")
        self.subscription = self.create_subscription(
            JointState, "joint_states", self.listener_callback, qos_profile_sensor_data
        )
        self.subscription  # prevent unused variable warning

        self.publisher = self.create_publisher(
            Odometry, "odom", qos_profile_sensor_data
        )

    def listener_callback(self, msg: JointState):
        # self.get_logger().info("Got joint state message: %f" % msg.position[0])
        msg
        odom_msg = Odometry()
        odom_msg.child_frame_id = "base_link"
        self.publisher.publish(odom_msg)


def main(args=None):
    rclpy.init(args=args)

    odom_publisher = OdomPublisher()

    rclpy.spin(odom_publisher)

    odom_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
