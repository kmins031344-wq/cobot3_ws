"""[색상 분류] 손목 카메라 영상에서 큐브 색을 감지해 /color_id 로 보내는 노드 (PC B)

구독: /rgb       (sensor_msgs/Image)  Isaac Sim 손목 카메라 영상
발행: /color_id  (std_msgs/Int32)     1 = 파랑, 2 = 초록 (감지 못하면 발행하지 않음)
      /color_debug (sensor_msgs/Image)  결과 글자를 그린 영상 (rqt_image_view 로 확인)

방법 (HSV 색 감지)
  1. 영상을 HSV 로 바꾼다 (OpenCV 의 H 범위는 0~179)
  2. 파랑/초록 범위에 드는 픽셀만 남기는 마스크를 만든다
  3. 영상 가운데 영역(ROI)에서 픽셀 수가 더 많은 색을 고른다
     (큐브는 카메라 바로 아래에 있으므로 가운데만 본다)
  4. 그 픽셀 비율이 MIN_RATIO 보다 작으면 아무것도 발행하지 않는다

실행:
  ros2 run m0609 m0609_color_detector
"""

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Int32

# HSV 범위 (H: 0~179, S: 0~255, V: 0~255)
#   순수 파랑 (0, 0, 255) -> H = 120 / 순수 초록 (0, 255, 0) -> H = 60
#   S(채도) 하한을 170 으로 높게: 하늘색 바닥(H 약 103, S 약 130) 같은 옅은 색은 제외하고
#   큐브처럼 진한 색만 잡는다
COLOR_RANGES = {
    1: ("파랑", np.array([108, 170, 50]), np.array([130, 255, 255])),
    2: ("초록", np.array([40, 170, 50]), np.array([80, 255, 255])),
}
ROI_RATIO = 0.6    # 가운데 60% x 60% 영역만 본다
MIN_RATIO = 0.02   # ROI 의 2% 이상이 그 색이어야 "있다"고 본다

# 결과 영상(/color_debug) 왼쪽 위 글자: (영문 이름, 글자색 BGR). cv2.putText 는 한글을 못 쓴다
DEBUG_TEXT = {
    0: ("NONE", (128, 128, 128)),
    1: ("BLUE", (255, 0, 0)),
    2: ("GREEN", (0, 200, 0)),
}


def image_to_bgr(msg):
    """sensor_msgs/Image -> OpenCV BGR 배열 (cv_bridge 없이 numpy 로 변환)"""
    channels = {"rgb8": 3, "bgr8": 3, "rgba8": 4, "bgra8": 4}.get(msg.encoding)
    if channels is None:
        raise ValueError(f"지원하지 않는 encoding: {msg.encoding}")
    data = np.frombuffer(msg.data, dtype=np.uint8)
    # step(한 줄 바이트 수)에 여백이 있을 수 있어 step 기준으로 자른 뒤 필요한 폭만 쓴다
    img = data.reshape(msg.height, msg.step)[:, : msg.width * channels].reshape(msg.height, msg.width, channels)
    conversions = {
        "rgb8": cv2.COLOR_RGB2BGR,
        "rgba8": cv2.COLOR_RGBA2BGR,
        "bgra8": cv2.COLOR_BGRA2BGR,
    }
    if msg.encoding in conversions:
        return cv2.cvtColor(img, conversions[msg.encoding])
    return img.copy()


class ColorDetector(Node):
    def __init__(self):
        super().__init__("color_detector_node")
        self.sub = self.create_subscription(Image, "rgb", self.image_callback, 10)
        self.pub = self.create_publisher(Int32, "color_id", 10)
        self.debug_pub = self.create_publisher(Image, "color_debug", 10)  # rqt_image_view 로 보는 결과 영상
        self.last_id = None  # 바뀔 때만 로그를 찍기 위해
        self.get_logger().info("색 감지 시작: /rgb 구독 -> /color_id 발행 (1=파랑, 2=초록), 결과 영상 /color_debug")

    def detect(self, bgr):
        """BGR 영상 -> (color_id, {color_id: 픽셀 수}, ROI 사각형)"""
        h, w = bgr.shape[:2]
        mh, mw = int(h * (1 - ROI_RATIO) / 2), int(w * (1 - ROI_RATIO) / 2)
        roi = bgr[mh: h - mh, mw: w - mw]
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

        areas = {}
        for cid, (_, lower, upper) in COLOR_RANGES.items():
            mask = cv2.inRange(hsv, lower, upper)
            areas[cid] = cv2.countNonZero(mask)  # 그 색 픽셀 수

        best = max(areas, key=areas.get)
        min_area = MIN_RATIO * roi.shape[0] * roi.shape[1]
        color_id = best if areas[best] >= min_area else 0
        return color_id, areas, (mw, mh, w - mw, h - mh)

    def draw_debug(self, bgr, color_id, roi_box):
        """왼쪽 위에 결과 글자, 가운데에 감지 영역(ROI) 사각형을 그린다"""
        img = bgr.copy()
        x1, y1, x2, y2 = roi_box
        cv2.rectangle(img, (x1, y1), (x2, y2), (200, 200, 200), 1)
        if color_id:
            text = f"color_id = {color_id}  {DEBUG_TEXT[color_id][0]} (id={color_id})"
        else:
            text = "color_id = 0  NONE"
        cv2.putText(img, text, (8, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.55, DEBUG_TEXT[color_id][1], 2, cv2.LINE_AA)
        return img

    def publish_debug(self, img, header):
        msg = Image()
        msg.header = header
        msg.height, msg.width = img.shape[:2]
        msg.encoding = "bgr8"
        msg.step = msg.width * 3
        msg.data = img.tobytes()
        self.debug_pub.publish(msg)

    def image_callback(self, msg):
        try:
            bgr = image_to_bgr(msg)
        except ValueError as e:
            self.get_logger().error(str(e))
            return

        color_id, areas, roi_box = self.detect(bgr)
        if color_id in COLOR_RANGES:  # 파랑(1) / 초록(2) 일 때만 발행. 감지 못하면(0) 보내지 않는다
            self.pub.publish(Int32(data=color_id))
        self.publish_debug(self.draw_debug(bgr, color_id, roi_box), msg.header)

        if color_id != self.last_id:
            if color_id:
                result = f"{COLOR_RANGES[color_id][0]} →Place {color_id}"
            else:
                result = "미감지"
            self.get_logger().info(f"color_id = {color_id} ({result})  blue_area={areas[1]}  green_area={areas[2]}")
            self.last_id = color_id


def main(args=None):
    rclpy.init(args=args)
    node = ColorDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
