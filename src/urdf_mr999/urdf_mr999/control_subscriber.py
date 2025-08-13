import rclpy
from rclpy.node import Node

from urdf_mr999.msg import Control


class ControlSubscriber(Node):

    def __init__(self):
        super().__init__("control_subscriber")
        self.subscription = self.create_subscription(
            Control,
            "topic",
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info("body: %f, shoulder: %f, elbow: %f, wrist: %f, hand: %f",
                               msg.body, msg.shoulder, msg.elbow, msg.wrist, msg.hand)


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
