"""MR999を操作するプログラム."""
import logging

import cv2
import numpy as np
import pygame

from location_detect_by_camera import Camera
from mr999 import Control, Move, Mr999

logger = logging.getLogger(__name__)

def change_image(base:np.ndarray)->pygame.Surface:
    """OpenCVの画像をpygameの画像に変換."""
    return pygame.image.frombuffer(cv2.cvtColor(base, cv2.COLOR_BGR2RGB),
                                   base.shape[1::-1], "RGB")

def main()->None:
    """メイン."""
    screen_width=1920
    screen_height=1080
    background_color=(0,0,0)
    font_color = (255,255,255)
    key_setting = {
        pygame.K_q: ("body", Move.up),
        pygame.K_a: ("body", Move.down),
        pygame.K_w: ("shoulder", Move.up),
        pygame.K_s: ("shoulder", Move.down),
        pygame.K_e: ("elbow", Move.up),
        pygame.K_d: ("elbow", Move.down),
        pygame.K_r: ("wrist", Move.up),
        pygame.K_f: ("wrist", Move.down),
        pygame.K_t: ("hand", Move.up),
        pygame.K_g: ("hand", Move.down),
    }
    pygame.init()
    robo = Mr999()
    camera = Camera()
    try:
        screen = pygame.display.set_mode((screen_width, screen_height), pygame.DOUBLEBUF)
        pygame.display.set_caption("MR-999 Controller")
        clock = pygame.time.Clock()
        font = pygame.font.Font(None, 28)

        running = True
        control = Control()
        frame = None
        surface = pygame.Surface((screen_width, screen_height))
        locates = {}

        while running:
            # 画面構成
            if frame is not None:
                surface = change_image(frame)
            screen.blit(surface, (0, 0))
            text = font.render(str(control), True, font_color)  # noqa: FBT003
            text_rect = text.get_rect(topleft=(1,1))
            screen.blit(text, text_rect)
            for i, locate in locates.items():
                t = str(locate["tvec"].flatten())
                r = str(locate["rvec"].flatten())
                text = font.render(f"{i}: t={t}, r={r}", True, font_color) # noqa: FBT003
                text_rect = text.get_rect(topleft=(1, i * 30))
                screen.blit(text, text_rect)

            pygame.display.flip()

            #　操作
            locates, frame = camera.get_locate()

            for event in pygame.event.get():
                if (event.type == pygame.QUIT or
                    (event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE)):
                    running = False
                elif (event.type == pygame.KEYDOWN and event.key in key_setting):
                    target, move = key_setting[event.key]
                    setattr(control, target, move.value)
                elif (event.type == pygame.KEYUP and event.key in key_setting):
                    target, move = key_setting[event.key]
                    setattr(control, target, move.stop.value)
            robo.move_control(control)
            clock.tick(10)


    finally:
        camera.close()
        robo.close()
        pygame.quit()


if __name__ == "__main__":
    logging.basicConfig(format="%(asctime)s:%(levelname)s:%(name)s:%(message)s", level=logging.DEBUG)
    logger.info("Started: %s", __file__)

    # find_device()
    # _exp_write_device()
    main()
    logger.info("Finished")
