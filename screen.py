"""画面作成."""

import cv2
import numpy as np
import pygame

from state import State


def change_image(base:np.ndarray)->pygame.Surface:
    """OpenCVの画像をpygameの画像に変換."""
    return pygame.image.frombuffer(cv2.cvtColor(base, cv2.COLOR_BGR2RGB),
                                   base.shape[1::-1], "RGB")


class Screen:
    """スクリーンのクラス."""

    def __init__(self, width:int, height:int)->None:
        """コンストラクタ."""
        self.width = width
        self.height = height
        self.surface = pygame.Surface((width, height))
        self.background_color=(0,0,0)
        self.font_color = (255,255,255)
        self.caution_color = (255, 0, 0)
        self.screen = pygame.display.set_mode((width, height), pygame.DOUBLEBUF)
        pygame.display.set_caption("MR-999 Controller")
        self.surface = pygame.Surface((self.width, self.height))
        self.frame = None
        self.graph_size = 200
        self.speeds = [0.0]
        self.font = pygame.font.Font(None, 28)

    def write_str(self, msg:str, pos:tuple[int, int])->None:
        """文字列を画面に書き込む."""
        text = self.font.render(msg, True, self.font_color) # noqa: FBT003
        text_rect = text.get_rect(topleft=pos)
        self.screen.blit(text, text_rect)

    def write_speed_graph(self, speeds:list[float])->None:
        """速度のグラフを画面に書き込む."""
        graph = pygame.Surface((self.graph_size, self.graph_size))
        tick = 50
        for i in range(tick, self.graph_size, tick):
            pygame.draw.line(graph, self.font_color, (0, i), (self.graph_size, i), 1)
        for i, speed in enumerate(speeds):
            y = int(self.graph_size - speed * 1000)
            if( y < self.graph_size):
                graph.set_at((i, y), self.font_color)
            else:
                graph.set_at((i, self.graph_size-1), self.caution_color)
        self.screen.blit(graph,
                         (self.width-self.graph_size, self.height-self.graph_size))

    def add_speed(self, speed:float)->None:
        """速度を追加."""
        self.speeds.append(speed)
        if len(self.speeds) > self.graph_size:
            self.speeds.pop(0)

    def update(self, state:State, frame:np.ndarray, locates:dict)->None:
        """フレームを更新."""
        target_size = 10

        ## 画面構成
        ### カメラの画像の設定
        if frame is not None:
            surface = change_image(frame)
        self.screen.blit(surface, (0, 0))

        self.write_speed_graph(self.speeds)

        ### 情報表示
        self.write_str(f"{self.speeds[-1]*1000:.1f}ms, {state.control}",(1,1))

        for i, locate in locates.items():
            t = str(locate["tvec"].flatten())
            r = str(locate["rvec"].flatten())
            self.write_str(f"{i}: t={t}, r={r}", (1, i * 30))

        ### 姿勢描画
        self.write_str(f"rel: {state.location.rel}", (1, 30))
        if len(state.location.rel) >2:
            center = state.location.rel[0]
            to = state.location.rel[1]
            pygame.draw.line(self.screen, self.caution_color, center, to)

        ### マウスカーソルの設定
        if state.target is not None:
            pygame.draw.circle(self.screen, self.caution_color,
                               state.target, target_size, 1)

        pygame.display.flip()


