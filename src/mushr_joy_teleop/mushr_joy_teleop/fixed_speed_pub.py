#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

class FixedSpeedLoop(Node):
    def __init__(self):
        super().__init__('fixed_speed_loop')
        self.pub = self.create_publisher(Float64, '/commands/motor/speed', 10)
        self.declare_parameter('data', 2000.0)
        self.declare_parameter('rate_hz', 20.0)

        self.speed = float(self.get_parameter('data').value)
        self.rate_hz = float(self.get_parameter('rate_hz').value)
        self.get_logger().info(
            f"Publishing {self.speed} continuously on /commands/motor/speed at {self.rate_hz} Hz"
        )

def main(args=None):
    rclpy.init(args=args)
    node = FixedSpeedLoop()
    rate = node.create_rate(node.rate_hz)

    try:
        msg = Float64()
        msg.data = node.speed
        while rclpy.ok():
            node.pub.publish(msg)
            rate.sleep()
    except KeyboardInterrupt:
        pass
    finally:
        # sécurité : en sortant, on envoie 0 une fois
        stop = Float64()
        stop.data = 0.0
        node.pub.publish(stop)
        node.get_logger().info("Stopped: published 0.0 once on /commands/motor/speed")
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
