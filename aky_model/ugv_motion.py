#!/usr/bin/env python3
"""Small differential-drive visual simulator used by the AKY RViz-only UGV demo."""

import math

import rclpy
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster


class UgvMotion(Node):
    """Convert /cmd_vel into odometry TF and wheel joint states.

    This is intentionally a kinematic visual simulator, not a hardware driver or
    physics simulator.  Keeping it separate from the URDF makes the RViz demo
    deterministic and lets keyboard commands visibly move both the rover and
    its four wheels.
    """

    def __init__(self):
        super().__init__('aky_motion')

        # These values must match urdf/aky_model.urdf.xacro.
        self.wheel_radius = 0.09       # metres
        self.wheel_separation = 0.51   # metres, left-wheel centre to right-wheel centre
        self.command_timeout = 0.75    # seconds; stops stale keyboard commands

        self.left_joints = ['front_left_wheel_joint', 'rear_left_wheel_joint']
        self.right_joints = ['front_right_wheel_joint', 'rear_right_wheel_joint']

        self.linear_velocity = 0.0
        self.angular_velocity = 0.0
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.left_wheel_angle = 0.0
        self.right_wheel_angle = 0.0

        now = self.get_clock().now()
        self.last_command_time = now
        self.last_update_time = now

        self.create_subscription(Twist, 'cmd_vel', self.command_callback, 10)
        self.joint_state_publisher = self.create_publisher(JointState, 'joint_states', 10)
        self.odom_publisher = self.create_publisher(Odometry, 'odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.create_timer(0.02, self.update)  # 50 Hz

        self.get_logger().info('AKY motion node is ready; waiting for /cmd_vel.')

    def command_callback(self, message):
        """Accept Twist commands and clamp them to safe visualization limits."""
        self.linear_velocity = max(-0.8, min(0.8, message.linear.x))
        self.angular_velocity = max(-1.8, min(1.8, message.angular.z))
        self.last_command_time = self.get_clock().now()

    def update(self):
        """Advance differential-drive kinematics, then publish TF, odometry, joints."""
        now = self.get_clock().now()
        dt = (now - self.last_update_time).nanoseconds / 1_000_000_000.0
        self.last_update_time = now
        dt = max(0.0, min(dt, 0.1))  # Protects the model from pauses or clock jumps.

        command_age = (now - self.last_command_time).nanoseconds / 1_000_000_000.0
        if command_age > self.command_timeout:
            linear_velocity = 0.0
            angular_velocity = 0.0
        else:
            linear_velocity = self.linear_velocity
            angular_velocity = self.angular_velocity

        # Differential-drive inverse kinematics.
        left_rate = (
            linear_velocity - 0.5 * self.wheel_separation * angular_velocity
        ) / self.wheel_radius
        right_rate = (
            linear_velocity + 0.5 * self.wheel_separation * angular_velocity
        ) / self.wheel_radius
        self.left_wheel_angle += left_rate * dt
        self.right_wheel_angle += right_rate * dt

        # Integrate base_footprint pose in the odom frame using the midpoint heading.
        midpoint_yaw = self.yaw + 0.5 * angular_velocity * dt
        self.x += linear_velocity * math.cos(midpoint_yaw) * dt
        self.y += linear_velocity * math.sin(midpoint_yaw) * dt
        self.yaw = math.atan2(
            math.sin(self.yaw + angular_velocity * dt),
            math.cos(self.yaw + angular_velocity * dt),
        )

        stamp = now.to_msg()
        quaternion_z = math.sin(self.yaw / 2.0)
        quaternion_w = math.cos(self.yaw / 2.0)

        self.publish_joint_states(stamp, left_rate, right_rate)
        self.publish_transform(stamp, quaternion_z, quaternion_w)
        self.publish_odometry(
            stamp,
            quaternion_z,
            quaternion_w,
            linear_velocity,
            angular_velocity,
        )

    def publish_joint_states(self, stamp, left_rate, right_rate):
        joints = JointState()
        joints.header.stamp = stamp
        joints.name = self.left_joints + self.right_joints
        joints.position = [self.left_wheel_angle] * len(self.left_joints) + [
            self.right_wheel_angle
        ] * len(self.right_joints)
        joints.velocity = [left_rate] * len(self.left_joints) + [right_rate] * len(
            self.right_joints
        )
        self.joint_state_publisher.publish(joints)

    def publish_transform(self, stamp, quaternion_z, quaternion_w):
        transform = TransformStamped()
        transform.header.stamp = stamp
        transform.header.frame_id = 'odom'
        transform.child_frame_id = 'base_footprint'
        transform.transform.translation.x = self.x
        transform.transform.translation.y = self.y
        transform.transform.rotation.z = quaternion_z
        transform.transform.rotation.w = quaternion_w
        self.tf_broadcaster.sendTransform(transform)

    def publish_odometry(
        self,
        stamp,
        quaternion_z,
        quaternion_w,
        linear_velocity,
        angular_velocity,
    ):
        odometry = Odometry()
        odometry.header.stamp = stamp
        odometry.header.frame_id = 'odom'
        odometry.child_frame_id = 'base_footprint'
        odometry.pose.pose.position.x = self.x
        odometry.pose.pose.position.y = self.y
        odometry.pose.pose.orientation.z = quaternion_z
        odometry.pose.pose.orientation.w = quaternion_w
        odometry.twist.twist.linear.x = linear_velocity
        odometry.twist.twist.angular.z = angular_velocity
        self.odom_publisher.publish(odometry)


def main(args=None):
    rclpy.init(args=args)
    node = UgvMotion()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
