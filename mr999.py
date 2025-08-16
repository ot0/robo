"""hidデバイスを通しロボットを操作."""

import logging
import time
from enum import Enum

import hid

logger = logging.getLogger(__name__)

class Move(Enum):
    """Enum for robot movements."""

    up = 1
    down = -1
    stop = 0

class Control:
    """コントロールメッセージのクラス."""

    def __init__(self, body:int=0, shoulder:int=0,
                 elbow:int=0, wrist:int=0, hand:int=0)->None:
        """コンストラクタ."""
        self.body = body
        self.shoulder = shoulder
        self.elbow = elbow
        self.wrist = wrist
        self.hand = hand
    def __repr__(self)->str:
        """文字列化."""
        return (f"Control(body={self.body}, shoulder={self.shoulder}, "
                f"elbow={self.elbow}, wrist={self.wrist}, hand={self.hand})")

class Mr999:
    """hidの操作を実施."""

    def __init__(self)->None:
        """コンストラクタ."""
        self.hid = hid.device()
        self.hid.open(0x12ed, 0x1003)

    def __del__(self)->None:
        """デストラクタ."""
        self.close()

    def close(self)->None:
        """デバイスを閉じる."""
        if(self.hid):
            self.write_device(bytes.fromhex("ffff"))
            self.hid.close()
            self.hid = None

    def move(self, body:int=0, shoulder:int=0,
                 elbow:int=0, wrist:int=0, hand:int=0)->None:
        """ロボットを動かす."""
        ctrl = Control(body, shoulder, elbow, wrist, hand)
        self.move_control(ctrl)

    def move_control(self, msg:Control)->None:
        """メッセージ受信時の処理."""
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

    def bit_control(self, bit:bytearray, value:int, parts:int)->None:
        """ポート操作."""
        bit_count = 4
        index = parts // bit_count
        target = parts % bit_count

        if value > 0:
            bit[index] ^= 1 << (target * 2)
        elif value < 0:
            bit[index] ^= 2 << (target * 2)

    def write_device(self, status:bytearray)->None:
        """デバイスへ書き込み."""
        write_size = 9
        for i, b in enumerate(status):
            d = [0]*write_size
            d[1] = i + 1
            d[2] = b
            self.hid.write(d)

def main()->None:
    """メイン関数."""
    mr999 = Mr999()
    mr999.move(shoulder=-1)
    time.sleep(1)
    mr999.move(shoulder=0)
    mr999.close()

def find_device()->None:
    """デバイスの検索."""
    for device_dict in hid.enumerate():
        for key, value in device_dict.items():
            logger.info("%s: %s", key,
                        hex(value) if key in ("vendor_id", "product_id") else value)

def _exp_write_device()->None:
    """ポートの書き換え.

    動作テスト。
    1111 1111,1111 1111が無動作
    1111 11xx,1111 1111が指
    1111 xx11,1111 1111が肘
    11xx 1111,1111 1111が胴
    xx11 1111,1111 1111が肩
    1111 1111,1111 11xxが手首
    """
    try:
        h = hid.device()
        h.open(0x12ed, 0x1003)
        d = [0]*9
        d[1] =0x01
        d[2] = 0xff
        h.write(d)
        time.sleep(1)
        logger.debug(h.read(9))
        d[1] =0x02
        d[2] = 0xff
        h.write(d)
        time.sleep(1)
        logger.debug(h.read(9))
        time.sleep(1)


    finally:
        h.close()

if __name__ == "__main__":
    main()
