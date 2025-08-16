"""状態を管理するクラス."""

import pygame

from mr999 import Move


class State:
    """状態のクラス."""

    def __init__(self) -> None:
        """コンストラクタ."""
        # self.width, self.height = 640, 480 # UCAM-DLE300ではうまく動作せず。
        # self.width, self.height = 2048, 1536
        self.width, self.height = 1600, 1200
        self.key = {
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
        self.tick = 10
        self.locates = {}
        self.target = None
        self.robo = None
        self.control = None
        self.camera = None
        self.screen = None
        self.speeds = [0.0]

