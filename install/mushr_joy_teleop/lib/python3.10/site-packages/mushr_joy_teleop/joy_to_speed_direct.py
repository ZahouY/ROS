#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64

class JoyToSpeed(Node):
    def __init__(self):
        super().__init__('joy_to_speed')

        # paramètres — tu peux ajuster
        self.declare_parameter('axis_speed', 1)     # axe joystick pour avancer/reculer
        self.declare_parameter('max_speed', 20000.0)     # vitesse max (unité selon VESC, ex: m/s ou valeur à adapter)

        self.axis_speed = self.get_parameter('axis_speed').get_parameter_value().integer_value
        self.max_speed = self.get_parameter('max_speed').get_parameter_value().double_value

        self.sub_joy = self.create_subscription(
            Joy,
            'joy',
            self.joy_callback,
            10
        )
        self.pub_speed = self.create_publisher(
            Float64,
            '/commands/motor/speed',
            10
        )

        self.get_logger().info('JoyToSpeed node ready, axis {}, max_speed {}'.format(
            self.axis_speed, self.max_speed
        ))

    def joy_callback(self, msg: Joy):
        if len(msg.axes) <= self.axis_speed:
            self.get_logger().warn('Joy message does not have axis {}'.format(self.axis_speed))
            return

        # e.g. pousser joystick avant donne -1.0 → on inverse pour “avant = +”
        joy_val = - msg.axes[self.axis_speed]

        speed = joy_val * self.max_speed
        msg_out = Float64()
        msg_out.data = float(speed)
        self.pub_speed.publish(msg_out)
        # debug
        self.get_logger().debug(f'Published speed: {msg_out.data}')

def main(args=None):
    rclpy.init(args=args)
    node = JoyToSpeed()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
