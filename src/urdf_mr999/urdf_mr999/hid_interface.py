"""hidデバイスを通しロボットを操作."""

import hid
import rclpy
from rclpy.node import Node

from msg_mr999.msg import Control


class HidInterface(Node):
    """hidの操作を実施."""

    def __init__(self):
        """コンストラクタ."""
        super().__init__("hid_interface")
        self.subscription = self.create_subscription(
            Control,
            "control",
            self.listener_callback,
            10)
        self.get_logger().info(f"run: {self.subscription}")
        self.hid = hid.device()
        self.hid.open(0x12ed, 0x1003)

    def __del__(self)->None:
        """デストラクタ."""
        self.write_device(bytes.fromhex("ffff"))
        self.hid.close()

    def listener_callback(self, msg:Control)->None:
        """メッセージ受信時の処理."""
        self.get_logger().info(f"body: {msg.body}, shoulder: {msg.shoulder},"
                               f"elbow: {msg.elbow}, wrist: {msg.wrist}, hand: {msg.hand}")
        self.write_device(self.make_status(msg))

    def make_status(self, ctrl:Control)->bytearray:
        """コントロールメッセージからデバイスへ書き込むバイト列を生成."""
        byte = bytearray.fromhex("ffff")
        self.bit_control(byte, ctrl.body, 2)
        self.bit_control(byte, ctrl.shoulder, 3)
        self.bit_control(byte, -ctrl.elbow, 1)
        self.bit_control(byte, -ctrl.wrist, 4)
        self.bit_control(byte, ctrl.hand, 0)
        return byte

    def bit_control(self, bit:bytes, value:float, parts:int)->bytes:
        """ポート操作."""
        bit_count = 4
        index = parts // bit_count
        target = parts % bit_count

        if value > 0:
            bit[index] ^= 1 << (target * 2)
        elif value < 0:
            bit[index] ^= 2 << (target * 2)

    def write_device(self, status:bytes)->None:
        """デバイスへ書き込み."""
        write_size = 9
        for i, b in enumerate(status):
            d = [0]*write_size
            d[1] = i + 1
            d[2] = b
            self.hid.write(d)

def main(args:dict|None=None)->None:
    """メイン関数."""
    rclpy.init(args=args)

    publisher = HidInterface()

    rclpy.spin(publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
