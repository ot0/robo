"""位置を管理する."""

import logging

import numpy as np

from state import State

logger = logging.getLogger(__name__)

class Location:
    """位置のクラス."""

    def __init__(self)->None:
        """コンストラクタ."""
        self.ids =[4,5,3,6]
        self.names = [
            "base",
            "shoulder",
            "elbow",
            "wrist",
        ]
        self.loc = [None] * len(self.ids)
        self.xy_acc = 0.01
        self.theta_acc = 0.1
        self.rel = np.array([0.0, 0.0])
        self.mpp = 1.0

    def to_pixcel(self, vec:dict, state:State)->list[int,int]:
        """位置からピクセルに変換."""
        return (vec * (1, -1) / self.mpp + (state.width / 2, state.height / 2)).tolist()

    def get_control(self, locations:dict, state:State)->None:
        """位置を設定."""
        has_pos = np.full(len(self.ids), False)
        for key, loc in locations.items():
            if key in self.ids:
                i = self.ids.index(key)
                self.loc[i] = loc
                has_pos[i] = True
                # if self.loc[i] is None:
                #     self.loc[i] = loc
                # else:
                #     self.loc[i]["tvec"] = 0.5 * self.loc[i]["tvec"] + 0.5 * loc["tvec"]
                #     self.loc[i]["rvec"] = 0.5 * self.loc[i]["rvec"] + 0.5 * loc["rvec"]
        if self.loc[0] is None or state.target is None:
            return

        base_pixel = sum([corner[0] for corner in self.loc[0]["corner"]]) - state.width / 2
        self.mpp = self.loc[0]["tvec"][0][0] /base_pixel
        target = (np.array(state.target) - (state.width / 2, state.height / 2))
        target = target * (1,-1) * self.mpp

        center = self.loc[0]["tvec"].flatten()[0:2] - (0.07, 0.03)
        target -= center
        target_r = np.linalg.norm(target)

        ar_id = 2
        if has_pos[ar_id]:
            shoulder = self.loc[ar_id]["tvec"].flatten()[0:2] - center
            #self.rel  = shoulder - target
            shoulder_r = np.linalg.norm(shoulder)

            r = shoulder_r - target_r

            dot = np.dot(shoulder, target)
            cross = shoulder[0] * target[1] - shoulder[1] * target[0]
            theta = np.arccos(dot/(shoulder_r*target_r)) * np.sign(cross)
            self.rel = (self.to_pixcel(center, state), self.to_pixcel(shoulder+center, state), (r, theta))
            if r > self.xy_acc:
                state.control.shoulder = 1
            elif r < -self.xy_acc:
                state.control.shoulder = -1
            else:
                state.control.shoulder = 0

            if theta > self.theta_acc:
                state.control.body =-1
            elif theta < -self.theta_acc:
                state.control.body = 1
            else:
                state.control.body = 0
        else:
            state.control.shoulder = 0
            state.control.body = 0

    def __repr__(self)->str:
        """文字列化."""
        return f"{self.loc}"
