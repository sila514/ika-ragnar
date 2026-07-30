#!/usr/bin/env python3
"""
RAGNAR IKA - YOLO Inference Node (Hailo-8L)
Gorev 4: Kameradan goruntu alip Hailo cipinde YOLOv8 calistirir,
sonuclari /detected_targets topic'ine basar.

Rehberdeki iskeletten farki (performans duzeltmesi):
- Hailo VDevice / network group / InferVStreams / activate SADECE __init__'te
  bir kez aciliyor. Callback icinde sadece pipe.infer() cagriliyor.
  Bu, rehberin de belirttigi "Performans notu"nun uygulanmis hali.
- FPS loglama eklendi (25-30 FPS hedefine ulasilip ulasilmadigini gormek icin).
- Sinif isimleri (class_names) parkur.yaml'daki siraya gore kolayca
  degistirilebilir hale getirildi (Gorev 5 bitince).
"""

import time

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose

from hailo_platform import (
    HEF,
    ConfigureParams,
    FormatType,
    HailoStreamInterface,
    InferVStreams,
    InputVStreamParams,
    OutputVStreamParams,
    VDevice,
)

# ---------------------------------------------------------------------------
# AYARLAR - Gorev 5 bitince (kendi modelin gelince) burasi guncellenir
# ---------------------------------------------------------------------------
HEF_PATH = "/home/ika/hailo_models/yolov8s_h8l.hef"
CONF_THRESHOLD = 0.5

# Gorev 5 dataset'i (dataset_egitim/data.yaml) sirasiyla - index 0..14.
# NOT: HEF_PATH hala eski COCO modelini gosteriyorsa bu liste ile eslesmez
# (COCO cikisi 80 sinif, sirasi farkli) - HEF_PATH yeni modele cevrilene kadar
# CLASS_NAMES'i devre disi birakmak icin gecici olarak None yapabilirsin.
CLASS_NAMES = [
    "koni",              # 0
    "sign_1",            # 1
    "sign_2",            # 2
    "sign_3",            # 3
    "sign_4",            # 4
    "sign_5",            # 5
    "sign_6",            # 6
    "sign_7",            # 7
    "sign_8",            # 8
    "sign_9",            # 9
    "sign_10",           # 10
    "sign_11",           # 11
    "sign_11_crossed",   # 12
    "sign_stop",         # 13
    "sign_target",       # 14 - hedef panel
]


class YoloNode(Node):
    def __init__(self):
        super().__init__("yolo_node")

        self.bridge = CvBridge()
        self.sub = self.create_subscription(Image, "/image_raw", self.cb, 1)
        self.pub = self.create_publisher(Detection2DArray, "/detected_targets", 10)

        # --- FPS olcumu icin ---
        self._frame_count = 0
        self._last_fps_log = time.time()

        # --- Hailo kurulumu (SADECE BIR KEZ, __init__'te) ---
        self.get_logger().info(f"Hailo modeli yukleniyor: {HEF_PATH}")
        self.hef = HEF(HEF_PATH)
        self.dev = VDevice()

        params = ConfigureParams.create_from_hef(
            self.hef, interface=HailoStreamInterface.PCIe
        )
        self.network_group = self.dev.configure(self.hef, params)[0]
        self.network_group_params = self.network_group.create_params()

        self.input_vstream_params = InputVStreamParams.make(
            self.network_group, format_type=FormatType.UINT8
        )
        self.output_vstream_params = OutputVStreamParams.make(
            self.network_group, format_type=FormatType.FLOAT32
        )

        info = self.hef.get_input_vstream_infos()[0]
        self.input_name = info.name
        self.in_h, self.in_w = info.shape[0], info.shape[1]

        # Infer pipeline'i ve network group'u SURECLI acik tut (kritik performans duzeltmesi)
        self._infer_pipeline = InferVStreams(
            self.network_group, self.input_vstream_params, self.output_vstream_params
        )
        self._infer_pipeline.__enter__()
        self._activated_network = self.network_group.activate(self.network_group_params)
        self._activated_network.__enter__()

        self.get_logger().info(
            f"Model hazir. Giris boyutu: {self.in_w}x{self.in_h}. "
            f"Sinif isimleri: {'COCO (varsayilan)' if CLASS_NAMES is None else CLASS_NAMES}"
        )

    def cb(self, msg: Image):
        t0 = time.time()

        frame = self.bridge.imgmsg_to_cv2(msg, "rgb8")
        resized = cv2.resize(frame, (self.in_w, self.in_h))
        input_data = {self.input_name: np.expand_dims(resized, 0)}

        # Onceden acilmis pipeline'a sadece infer cagrisi - her karede
        # yeniden context acmiyoruz, bu FPS'i kat kat artirir.
        raw_output = self._infer_pipeline.infer(input_data)

        detections = self._parse_output(raw_output, msg.width, msg.height)

        det_array = Detection2DArray()
        det_array.header = msg.header
        det_array.detections = detections
        self.pub.publish(det_array)

        self._log_fps(t0)

    def _parse_output(self, raw_output, img_width, img_height):
        """
        Hailo YOLOv8 postprocess ciktisini Detection2DArray'e cevirir.
        Beklenen format: her sinif icin [y_min, x_min, y_max, x_max, score]
        (0-1 araliginda normalize), NMS zaten chip uzerinde yapilmis.

        NOT: Ilk gercek calistirmada `print(raw_output)` ile yapiyi dogrula -
        Hailo surum/derleme farkina gore format degisebilir.
        """
        detections = []
        for _output_name, per_class in raw_output.items():
            # per_class[0]: her sinif icin kutu listesi
            for cls_id, boxes in enumerate(per_class[0]):
                for box in boxes:
                    score = float(box[4])
                    if score < CONF_THRESHOLD:
                        continue

                    y_min, x_min, y_max, x_max = box[0], box[1], box[2], box[3]

                    det = Detection2D()
                    det.bbox.center.position.x = float((x_min + x_max) / 2 * img_width)
                    det.bbox.center.position.y = float((y_min + y_max) / 2 * img_height)
                    det.bbox.size_x = float((x_max - x_min) * img_width)
                    det.bbox.size_y = float((y_max - y_min) * img_height)

                    hyp = ObjectHypothesisWithPose()
                    class_label = (
                        CLASS_NAMES[cls_id]
                        if CLASS_NAMES and cls_id < len(CLASS_NAMES)
                        else str(cls_id)
                    )
                    hyp.hypothesis.class_id = class_label
                    hyp.hypothesis.score = score
                    det.results.append(hyp)

                    detections.append(det)

        return detections

    def _log_fps(self, t0):
        self._frame_count += 1
        now = time.time()
        elapsed_since_log = now - self._last_fps_log
        if elapsed_since_log >= 5.0:
            fps = self._frame_count / elapsed_since_log
            infer_ms = (now - t0) * 1000
            self.get_logger().info(
                f"FPS: {fps:.1f}  |  son karenin infer suresi: {infer_ms:.1f}ms"
            )
            self._frame_count = 0
            self._last_fps_log = now

    def destroy_node(self):
        # Hailo kaynaklarini duzgun kapat
        try:
            self._activated_network.__exit__(None, None, None)
            self._infer_pipeline.__exit__(None, None, None)
        except Exception as e:
            self.get_logger().warn(f"Hailo kaynaklari kapatilirken uyari: {e}")
        super().destroy_node()


def main():
    rclpy.init()
    node = YoloNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
