import math

import rclpy
from ackermann_msgs.msg import AckermannDrive
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState


class VehicleStatePublisher(Node):
    def __init__(self):
        super().__init__('vehicle_state_publisher')
        self.declare_parameter('wheel_radius', 0.04)
        self.declare_parameter('steering_ratio', -math.pi / 180.0)
        self.declare_parameter('publish_rate', 30.0)

        self._wheel_radius = float(self.get_parameter('wheel_radius').value)
        self._steering_ratio = float(self.get_parameter('steering_ratio').value)
        publish_rate = float(self.get_parameter('publish_rate').value)

        self._speed_command = 0.0
        self._steering_command = 0.0
        self._wheel_position = 0.0
        self._last_time = self.get_clock().now()

        self._joint_state_publisher = self.create_publisher(
            JointState, '/joint_states', 10
        )
        self.create_subscription(
            AckermannDrive,
            '/yellow_car/cmd_ackermann',
            self._cmd_ackermann_callback,
            10,
        )
        self.create_timer(1.0 / publish_rate, self._publish_joint_states)

    def _cmd_ackermann_callback(self, message: AckermannDrive):
        self._speed_command = float(message.speed)
        self._steering_command = float(message.steering_angle)

    def _publish_joint_states(self):
        now = self.get_clock().now()
        dt = (now - self._last_time).nanoseconds / 1e9
        self._last_time = now

        if self._wheel_radius > 0.0:
            self._wheel_position += (self._speed_command / self._wheel_radius) * dt

        steering_position = self._steering_command * self._steering_ratio

        message = JointState()
        message.header.stamp = now.to_msg()
        message.name = [
            'front_left_steering_joint',
            'front_right_steering_joint',
            'front_left_wheel_joint',
            'front_right_wheel_joint',
            'rear_left_wheel_joint',
            'rear_right_wheel_joint',
        ]
        message.position = [
            steering_position,
            steering_position,
            self._wheel_position,
            self._wheel_position,
            self._wheel_position,
            self._wheel_position,
        ]
        self._joint_state_publisher.publish(message)


def main(args=None):
    rclpy.init(args=args)
    node = VehicleStatePublisher()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
