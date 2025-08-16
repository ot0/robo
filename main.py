"""MR999を操作するプログラム."""
import logging
import time

import pygame

from location_detect_by_camera import Camera
from mr999 import Control, Mr999
from screen import Screen
from state import State

logger = logging.getLogger(__name__)

def main()->None:
    """メイン."""
    state = State()
    pygame.init()
    robo = Mr999()
    state.robo = robo
    camera = Camera(0, state.width, state.height)
    state.camera = camera
    try:
        clock = pygame.time.Clock()

        screen = Screen(state.width, state.height)
        state.screen = screen

        running = True
        control = Control()
        state.control = control

        while running:
            start = time.perf_counter()

            #　操作
            locates, frame = camera.get_locate()
            screen.update(state, frame, locates)

            for event in pygame.event.get():
                if (event.type == pygame.QUIT or
                    (event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE)):
                    running = False
                elif (event.type == pygame.KEYDOWN and event.key in state.key):
                    target, move = state.key[event.key]
                    setattr(control, target, move.value)
                elif (event.type == pygame.KEYUP and event.key in state.key):
                    target, move = state.key[event.key]
                    setattr(control, target, move.stop.value)
                elif ((event.type == pygame.MOUSEBUTTONDOWN
                       and event.button == pygame.BUTTON_LEFT)
                    or (event.type == pygame.MOUSEMOTION and event.buttons[0])):
                    target = event.pos
                elif (event.type == pygame.MOUSEBUTTONDOWN
                      and event.button == pygame.BUTTON_RIGHT):
                    target = None

            robo.move_control(control)

            screen.add_speed(time.perf_counter() - start)
            clock.tick(state.tick)

    finally:
        camera.close()
        robo.close()
        pygame.quit()


if __name__ == "__main__":
    logging.basicConfig(format="%(asctime)s:%(levelname)s:%(name)s:%(message)s",
                        level=logging.DEBUG)
    logger.info("Started: %s", __file__)

    # find_device()
    # _exp_write_device()
    main()
    logger.info("Finished")
