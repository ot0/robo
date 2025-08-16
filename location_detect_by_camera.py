"""カメラによる位置検知."""
import logging
import math

import cv2
import numpy as np
from cv2 import aruco

logger = logging.getLogger(__name__)

class Camera:
    """カメラのクラス."""

    def __init__(self, camera_id:int=0, width:int=1600, height:int=1200)->None:
        """コンストラクタ."""
        self.camera_id = camera_id
        self.cap = cv2.VideoCapture(camera_id)
        if not self.cap.isOpened():
            logger.error("Could not open camera with ID %d", camera_id)
            msg = f"Could not open camera with ID {camera_id}"
            raise RuntimeError(msg)

        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        dic_aruco = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
        detector_params = aruco.DetectorParameters()
        detector_params.useAruco3Detection = True
        self.detector = aruco.ArucoDetector(dic_aruco, detector_params)

        horizontal_fov = math.radians(64.0)  # 水平視野角
        focal_length = width / (2 * math.tan(horizontal_fov / 2))  # 焦点距離
        marker_size = 0.02

        self.object_points = np.array([
            [-marker_size / 2, -marker_size / 2, 0],
            [marker_size / 2, -marker_size / 2, 0],
            [marker_size / 2, marker_size / 2, 0],
            [-marker_size / 2, marker_size / 2, 0],
        ], dtype=np.float32)
        self.camera_matrix = np.array([[focal_length, 0, width / 2],
                                [0, focal_length, height / 2],
                                [0, 0, 1]], dtype=np.float32)
        self.dist_coeffs = np.array([0, 0, 0, 0], dtype=np.float32)  # 歪み係数
        self.marker_size = marker_size

    def __del__(self)->None:
        """デストラクタ."""
        self.close()

    def close(self)-> None:
        """カメラを解放."""
        self.cap.release()

    def get_locate(self)->tuple[dict, np.ndarray|None ]:
        """位置を取得."""
        ret, frame = self.cap.read()
        if not ret:
            logger.error("Failed to capture image")
            return {}, None

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _rejected = self.detector.detectMarkers(gray)
        frame = aruco.drawDetectedMarkers(frame, corners, ids)

        if ids is None:
            return {}, frame

        result = {}
        for corner, index in zip(corners, ids, strict=True):
            success, rvec, tvec = cv2.solvePnP(self.object_points, corner,
                                              self.camera_matrix, self.dist_coeffs)
            if success:
                i = int(index[0])
                result[i] = {
                    "rvec": rvec,
                    "tvec": tvec,
                    "corner": corner,
                    "id": i,
                }
                frame = cv2.drawFrameAxes(frame, self.camera_matrix, self.dist_coeffs,
                                rvec, tvec, self.marker_size, thickness=2)
        return result, frame

def capture_camera()->None:
    """カメラからの画像取得."""
    camera = Camera()

    while True:
        locate, frame = camera.get_locate()
        if frame is None:
            logger.error("Failed to capture image")
            break

        logger.info("Detected markers: %s", locate)

        cv2.imshow("Camera Feed", frame)

        if cv2.waitKey(0) & 0xFF == ord("q"):
            break

    camera.close()
    cv2.destroyAllWindows()

def make_marker()-> None:
    """マーカーの生成."""
    aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_100)
    dpm = 40 / 3
    size = 200
    space = 400
    imag = np.full((int(297*dpm), int(210*dpm)), 255, dtype=np.uint8)
    for i in range(70):
        row = i % 7
        col = i // 7
        marker_image = aruco.generateImageMarker(aruco_dict, i, 200)
        imag[col*space + size//2:col*space + size//2 + size,
             row*space + size//2:row*space + size//2 + size] = marker_image
    cv2.imwrite("marker/markers.png", imag)

def main()->None:
    """メイン関数."""
    capture_camera()
    #make_marker()

if __name__ == "__main__":
    logging.basicConfig(format="%(asctime)s:%(levelname)s:%(name)s:%(message)s",
                        level=logging.DEBUG)
    logger.info("Started: %s", __file__)
    main()
    logger.info("Finished")
