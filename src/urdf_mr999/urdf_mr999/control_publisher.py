"""mr999をキーボードで操作するコントローラ."""
from enum import Enum

import rclpy
from rclpy.node import Node
from sshkeyboard import listen_keyboard

from msg_mr999.msg import Control


class Move(Enum):
    up = 1.0
    down = -1.0
    stop = 0.0

class ControlPublisher(Node):
    """操作用パブリッシャー."""

    def __init__(self)->None:
        """コンストラクタ."""
        super().__init__("control_publisher")
        self.keys = {
            "q": ("body", Move.up),
            "a": ("body", Move.down),
            "w": ("shoulder", Move.up),
            "s": ("shoulder", Move.down),
            "e": ("elbow", Move.up),
            "d": ("elbow", Move.down),
            "r": ("wrist", Move.up),
            "f": ("wrist", Move.down),
            "t": ("hand", Move.up),
            "g": ("hand", Move.down),
        }
        self.publisher_ = self.create_publisher(Control, "control", 10)
        # timer_period = 0.5  # seconds
        # self.timer = self.create_timer(timer_period, self.timer_callback)
        self.msg = Control()
        listen_keyboard(
            on_press=self.press,
            on_release=self.release,
        )

    def press(self, key:str)->None:
        """キーが押されたときの処理."""
        if key in self.keys:
            target, move = self.keys[key]
            setattr(self.msg, target, move.value)
            self.publisher_.publish(self.msg)

    def release(self, key:str)->None:
        """キーが離されたときの処理."""
        self.get_logger().info(f"release: {key}")
        if key in self.keys:
            target, _ = self.keys[key]
            setattr(self.msg, target, Move.stop.value)
            self.publisher_.publish(self.msg)

def main(args:dict|None=None)->None:
    """メイン関数."""
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
