import rclpy
from rclpy.node import Node

from urdf_mr999.msg import Control


class ControlPublisher(Node):

    def __init__(self):
        super().__init__("control_publisher")
        self.publisher_ = self.create_publisher(Control, "topic", 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = Control()
        msg.body = self.i
        msg.shoulder = self.i /2
        msg.elbow = self.i / 3
        msg.wrist = self.i / 4
        msg.hand = self.i / 5
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%d"', self.i)
        self.i += 1


def main(args=None):
    rclpy.init(args=args)

    publisher = ControlPublisher()

    rclpy.spin(publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
