"""動作確認用."""
import rclpy
from rclpy.node import Node

from msg_mr999.msg import Control


class ControlSubscriber(Node):

    def __init__(self):
        super().__init__("control_subscriber")
        self.subscription = self.create_subscription(
            Control,
            "control",
            self.listener_callback,
            10)
        self.get_logger().info(f"run: {self.subscription}")

    def listener_callback(self, msg):
        self.get_logger().info(f"body: {msg.body}, shoulder: {msg.shoulder},"
                f"elbow: {msg.elbow}, wrist: {msg.wrist}, hand: {msg.hand}")


def main(args=None):
    rclpy.init(args=args)

    subscriber = ControlSubscriber()

    rclpy.spin(subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
