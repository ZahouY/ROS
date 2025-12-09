#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from ackermann_msgs.msg import AckermannDriveStamped


class JoyTeleop(Node):
    def __init__(self):
        super().__init__('joy_teleop')

        # Paramètres simples (tu pourras les ajuster plus tard)
        self.declare_parameter('axis_steer', 0)      # joystick gauche X
        self.declare_parameter('axis_throttle', 1)  # joystick gauche Y
        self.declare_parameter('max_speed', 2.0)    # m/s
        self.declare_parameter('max_steer', 0.34)   # rad (~20°)

        self.axis_steer = self.get_parameter('axis_steer').get_parameter_value().integer_value
        self.axis_throttle = self.get_parameter('axis_throttle').get_parameter_value().integer_value
        self.max_speed = self.get_parameter('max_speed').get_parameter_value().double_value
        self.max_steer = self.get_parameter('max_steer').get_parameter_value().double_value

        # Abonnement au joystick
        self.sub_joy = self.create_subscription(
            Joy,
            'joy',
            self.joy_callback,
            10
        )

        # Publication de la commande de conduite
        self.pub_cmd = self.create_publisher(
            AckermannDriveStamped,
            'drive',   # topic "commande voiture"
            10
        )

        self.get_logger().info('JoyTeleop Ackermann prêt.')

    def joy_callback(self, msg: Joy):
        if len(msg.axes) <= max(self.axis_steer, self.axis_throttle):
            self.get_logger().warn('Message Joy ne contient pas assez d axes')
            return

        # Attention : souvent vers l avant = axe négatif → on inverse le throttle
        steering = msg.axes[self.axis_steer] * self.max_steer
        speed = -msg.axes[self.axis_throttle] * self.max_speed

        cmd = AckermannDriveStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = 'base_link'

        cmd.drive.speed = float(speed)
        cmd.drive.steering_angle = float(steering)

        self.pub_cmd.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = JoyTeleop()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
