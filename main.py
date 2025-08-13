"""MR999を操作するプログラム."""
import logging
import time

import hid
import pygame

logger = logging.getLogger(__name__)

def main()->None:
    """メイン."""
    screen_width=640
    screen_height=480
    background_color=(0,0,0)
    font_color = (255,255,255)
    pygame.init()
    h = hid.device()
    h.open(0x12ed, 0x1003)
    try:

        key_setting = {
            pygame.K_q:0,
            pygame.K_a:1,
            pygame.K_w:2,
            pygame.K_s:3,
            pygame.K_e:4,
            pygame.K_d:5,
            pygame.K_r:6,
            pygame.K_f:7,
            pygame.K_t:8,
            pygame.K_g:9,
        }
        screen = pygame.display.set_mode((screen_width, screen_height), pygame.DOUBLEBUF)
        pygame.display.set_caption("MR-999 Controller")
        clock = pygame.time.Clock()
        font = pygame.font.Font(None, 28)

        status = bytes.fromhex("ffff")

        running = True
        while running:
            # 画面構成
            screen.fill(background_color)
            text = font.render(f"{bin(status[0])} {bin(status[1])}",
                                True, font_color)  # noqa: FBT003
            text_rect = text.get_rect(topleft=(1,1))
            screen.blit(text, text_rect)
            pygame.display.flip()

            for event in pygame.event.get():
                if (event.type == pygame.QUIT or
                    (event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE)):
                    running = False
                elif (event.type == pygame.KEYDOWN):
                    key = key_setting.get(event.key, None)
                    if key is not None:
                        status = bit_control(status, key, is_on=True)
                elif (event.type == pygame.KEYUP):
                    key = key_setting.get(event.key, None)
                    if key is not None:
                        status = bit_control(status, key, is_on=False)
            write_device(h, status)
            clock.tick(15)


    finally:
        write_device(h, bytes.fromhex("ffff"))
        h.close()
        pygame.quit()

def bit_control(bit:bytes, key:int, *, is_on:bool=False)->bytes:
    """ポート操作."""
    bit_count = 8
    index = key // bit_count
    target = key % bit_count
    port = target // 2
    new_bit = bytearray(bit)

    shift = 0b11 << port * 2
    new_bit[index] |= shift
    if is_on:
        new_bit[index] ^= 1 << target
    return bytes(new_bit)

def write_device(h:object, status:bytes)->None:
    """デバイスへ書き込み."""
    write_size = 9
    for i, b in enumerate(status):
        d = [0]*write_size
        d[1] = i + 1
        d[2] = b
        h.write(d)

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
    logging.basicConfig(format="%(asctime)s:%(levelname)s:%(name)s:%(message)s", level=logging.DEBUG)
    logger.info("Started: %s", __file__)

    # find_device()
    # _exp_write_device()
    main()
    logger.info("Finished")
