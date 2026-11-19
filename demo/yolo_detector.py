"""yolo_detector - нейросетевой детектор объектов как нода ROS 2.

Кадры приходят в /camera/image_raw десять раз в секунду, один инференс
на процессоре занимает доли секунды, поэтому колбэк только запоминает
последний кадр, а инференс выполняет таймер. Пропуск кадров заложен
в устройство ноды: обрабатывается самый свежий из пришедших кадров.

Публикует vision_msgs/Detection2DArray в /my_detections и кадр
с нарисованными рамками в /my_detections/image_compressed.

Запуск без сборки пакета, из той папки, где лежит этот файл:

    python3 yolo_detector.py

При первом запуске библиотека скачает файл весов yolov8n.pt в текущую папку.
"""

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from ultralytics import YOLO
from vision_msgs.msg import (
    BoundingBox2D,
    Detection2D,
    Detection2DArray,
    ObjectHypothesisWithPose,
)


class YoloDetector(Node):
    def __init__(self):
        super().__init__("yolo_detector")

        self.declare_parameter("model_path", "yolov8n.pt")
        self.declare_parameter("conf_min", 0.5)
        self.declare_parameter("period", 1.0)

        model_path = self.get_parameter("model_path").value
        self.conf_min = float(self.get_parameter("conf_min").value)
        period = float(self.get_parameter("period").value)

        self.get_logger().info(f"загружаю модель {model_path}")
        self.model = YOLO(model_path)
        self.get_logger().info(f"модель загружена, период {period} с, жду кадры")

        self.frame = None  # последний принятый кадр
        self.frame_header = None  # его время и система координат

        self.image_sub = self.create_subscription(
            CompressedImage, "/camera/image_raw", self.image_callback, 1
        )
        self.det_pub = self.create_publisher(Detection2DArray, "/my_detections", 5)
        self.img_pub = self.create_publisher(
            CompressedImage, "/my_detections/image_compressed", 2
        )
        self.timer = self.create_timer(period, self.tick)

    def image_callback(self, msg: CompressedImage):
        raw = np.frombuffer(msg.data, dtype=np.uint8)
        frame = cv2.imdecode(raw, cv2.IMREAD_COLOR)
        if frame is None:
            return
        self.frame = frame
        self.frame_header = msg.header

    def tick(self):
        if self.frame is None:
            return
        frame = self.frame
        header = self.frame_header

        results = self.model(frame, conf=self.conf_min, verbose=False)
        found = results[0]

        arr = Detection2DArray()
        arr.header = header

        names = []
        for box in found.boxes:
            conf = float(box.conf[0])
            name = found.names[int(box.cls[0])]
            x1, y1, x2, y2 = [float(v) for v in box.xyxy[0]]

            det = Detection2D()
            det.header = arr.header
            det.id = f"{name}_{len(arr.detections)}"

            bb = BoundingBox2D()
            bb.center.position.x = (x1 + x2) / 2.0
            bb.center.position.y = (y1 + y2) / 2.0
            bb.center.theta = 0.0
            bb.size_x = x2 - x1
            bb.size_y = y2 - y1
            det.bbox = bb

            hyp = ObjectHypothesisWithPose()
            hyp.hypothesis.class_id = name
            hyp.hypothesis.score = conf
            det.results.append(hyp)

            arr.detections.append(det)
            names.append(name)

        self.det_pub.publish(arr)
        self.get_logger().info(
            "обнаружено: " + (", ".join(names) if names else "ничего")
        )

        ok, buf = cv2.imencode(
            ".jpg", found.plot(), [int(cv2.IMWRITE_JPEG_QUALITY), 60]
        )
        if ok:
            out = CompressedImage()
            out.header = arr.header
            out.format = "jpeg"
            out.data = buf.tobytes()
            self.img_pub.publish(out)


def main():
    rclpy.init()
    node = YoloDetector()
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
