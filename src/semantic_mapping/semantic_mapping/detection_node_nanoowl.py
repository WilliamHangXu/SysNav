#!/usr/bin/env python
# coding: utf-8

# ========== Environment Setup ==========
import os
os.environ["PYTORCH_ENABLE_MPS_FALLBACK"] = "1"
os.environ["TORCH_CUDA_ARCH_LIST"] = "8.9"

# ========== Standard Library ==========
import time
from collections import deque
from pathlib import Path

# ========== Third-party Libraries ==========
import cv2
import numpy as np
import yaml
import PIL.Image

# ========== Computer Vision Libraries ==========
import supervision as sv
from supervision.draw.color import ColorPalette
from ultralytics.utils import LOGGER, IterableSimpleNamespace
from ultralytics.trackers import BOTSORT
LOGGER.setLevel("ERROR")

# ========== NanoOwl ==========
from nanoowl.owl_predictor import OwlPredictor
from nanoowl.owl_drawing import (
    draw_owl_output
)

# ========== ROS2 Core ==========
import rclpy
from rclpy.node import Node
from rclpy.time import Time

# ========== ROS2 Messages ==========
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

# ========== Custom Messages ==========
from tare_planner.msg import DetectionResult


class _BotsortInput:
    """Results-like shim that exposes the attributes BOTSORT.update / init_track read."""

    def __init__(self, xyxy, conf, cls):
        self.xyxy = np.asarray(xyxy, dtype=np.float32).reshape(-1, 4)
        self.conf = np.asarray(conf, dtype=np.float32).reshape(-1)
        self.cls = np.asarray(cls, dtype=np.float32).reshape(-1)
        if len(self.xyxy) > 0:
            x1, y1, x2, y2 = self.xyxy.T
            self.xywh = np.stack([(x1 + x2) / 2.0, (y1 + y2) / 2.0, x2 - x1, y2 - y1], axis=-1)
        else:
            self.xywh = np.empty((0, 4), dtype=np.float32)

    def __len__(self):
        return len(self.conf)

    def __getitem__(self, idx):
        return _BotsortInput(self.xyxy[idx], self.conf[idx], self.cls[idx])


class DetectNode(Node):
    def __init__(self, device='cuda'):
        super().__init__('semantic_mapping_node')
        self.CONFIG_DIR = Path(__file__).resolve().parent

        self.detection_stamps = deque(maxlen=10)
        self.rgb_stack = deque(maxlen=10)

        # parameters
        self.declare_parameter('platform', 'mecanum_sim')
        self.declare_parameter('grounding_score_thresh', 0.1)
        self.declare_parameter('device', device)
        self.declare_parameter('annotate_image', True)
        self.declare_parameter('object_file', str(self.CONFIG_DIR / 'config' / 'objects.yaml'))
        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('nanoowl_model', 'google/owlvit-base-patch32')
        self.declare_parameter(
            'nanoowl_image_encoder_engine',
            str(self.CONFIG_DIR / 'external' / 'nanoowl' / 'data' / 'owl_image_encoder_patch32.engine'),
        )
        self.declare_parameter('tracker_frame_rate', 10)
        # NanoOwl returns sigmoid scores typically in the 0.01–0.15 range on this panoramic
        # input — much smaller than YOLO's classifier scores. These params let the NanoOwl
        # path use its own thresholds without touching the shared botsort.yaml or the
        # platform yaml's `grounding_score_thresh` (which is tuned for YOLO).
        self.declare_parameter('tracker_high_thresh', 0.03)
        self.declare_parameter('tracker_low_thresh', 0.02)
        self.declare_parameter('tracker_new_thresh', 0.03)

        self.platform = self.get_parameter('platform').get_parameter_value().string_value
        self.ANNOTATE = self.get_parameter('annotate_image').get_parameter_value().bool_value
        self.grounding_score_thresh = self.get_parameter('grounding_score_thresh').get_parameter_value().double_value
        object_file_path = self.get_parameter('object_file').get_parameter_value().string_value
        self.image_topic = self.get_parameter('image_topic').get_parameter_value().string_value
        self.nanoowl_model = self.get_parameter('nanoowl_model').get_parameter_value().string_value
        self.nanoowl_image_encoder_engine = self.get_parameter('nanoowl_image_encoder_engine').get_parameter_value().string_value
        self.tracker_frame_rate = self.get_parameter('tracker_frame_rate').get_parameter_value().integer_value
        self.tracker_high_thresh = self.get_parameter('tracker_high_thresh').get_parameter_value().double_value
        self.tracker_low_thresh = self.get_parameter('tracker_low_thresh').get_parameter_value().double_value
        self.tracker_new_thresh = self.get_parameter('tracker_new_thresh').get_parameter_value().double_value

        with open(object_file_path, "r") as file:
            self.object_config = yaml.safe_load(file)
        self.label_template = self.object_config['prompts']
        self.text_prompt_list = []
        for value in self.label_template.values():
            self.text_prompt_list += value['prompts']
        self.text_prompt = " . ".join(self.text_prompt_list) + " ."
        self.text_prompt_list = np.array(self.text_prompt_list)
        print(f"Text prompt: {self.text_prompt}")

        engine_path = self.nanoowl_image_encoder_engine
        if not os.path.isfile(engine_path):
            self.get_logger().warn(
                f"NanoOwl image-encoder engine not found at '{engine_path}'. "
                "Predictor will fall back to the unoptimized image encoder."
            )
            engine_path = None
        self.grounding_model = OwlPredictor(self.nanoowl_model, image_encoder_engine=engine_path)
        self.text_encodings = self.grounding_model.encode_text(list(self.text_prompt_list))

        # Standalone BoT-SORT, configured from the same yaml the YOLO path uses, but with
        # the three confidence thresholds AND fuse_score overridden in-memory to match
        # NanoOwl's sigmoid score range. The on-disk yaml is untouched so the YOLO path
        # keeps its tuning.
        #
        # fuse_score must be False here: BoT-SORT's score fusion computes
        #     cost = 1 - IoU * detection_score
        # and requires cost < match_thresh (0.5–0.8) for a match. With NanoOwl's typical
        # scores of 0.05–0.15, even an IoU of 1.0 yields cost >= 0.85, so no track ever
        # gets re-confirmed across frames and the tracker output stays empty.
        with open(self.CONFIG_DIR / "config" / "botsort.yaml", "r") as f:
            tracker_cfg = IterableSimpleNamespace(**yaml.safe_load(f))
        tracker_cfg.track_high_thresh = self.tracker_high_thresh
        tracker_cfg.track_low_thresh = self.tracker_low_thresh
        tracker_cfg.new_track_thresh = self.tracker_new_thresh
        tracker_cfg.fuse_score = False
        self.tracker = BOTSORT(tracker_cfg, frame_rate=self.tracker_frame_rate)

        self.device = device

        if self.ANNOTATE:
            self.box_annotator = sv.BoxAnnotator(color=ColorPalette.DEFAULT)
            self.label_annotator = sv.LabelAnnotator(
                color=ColorPalette.DEFAULT,
                text_padding=4,
                text_scale=0.5,
                text_position=sv.Position.TOP_LEFT,
                color_lookup=sv.ColorLookup.INDEX,
                smart_position=True,
            )
            self.mask_annotator = sv.MaskAnnotator(color=ColorPalette.DEFAULT)
            self.ANNOTATE_OUT_DIR = os.path.join('output/debug_mapper', 'annotated_3d_in_loop_detection')
            self.IMAGE_DIR = os.path.join('output/debug_mapper', 'detection')
            self.VIEWPOINT_IMAGE_DIR = os.path.join(os.path.dirname(__file__), 'output/viewpoint_images')
            if os.path.exists(self.ANNOTATE_OUT_DIR):
                os.system(f"rm -r {self.ANNOTATE_OUT_DIR}")
            os.makedirs(self.ANNOTATE_OUT_DIR, exist_ok=True)

        self.bridge = CvBridge()

        # ROS2 subscriptions and publishers
        self.rgb_sub = self.create_subscription(
            Image,
            self.image_topic,
            self.image_callback,
            10,
        )

        self.annotated_image_pub = self.create_publisher(Image, '/annotated_image_detection', 10)
        self.detection_result_pub = self.create_publisher(DetectionResult, '/detection_result', 50)

        self.call_back_time_stamp = time.time()

        self.log_info('Detection node (NanoOwl) has been started.')

    def log_info(self, msg):
        self.get_logger().info(msg)

    def inference(self, cv_image):
        """
        Run NanoOwl on the input image, then attach standalone BoT-SORT track IDs.

        cv_image: np.ndarray, shape (H, W, 3), BGR format.
        """
        rgb_image = cv_image[:, :, ::-1]  # BGR -> RGB
        pil_image = PIL.Image.fromarray(np.ascontiguousarray(rgb_image))

        start_time = time.time()
        output = self.grounding_model.predict(
            image=pil_image,
            text=list(self.text_prompt_list),
            text_encodings=self.text_encodings,
            threshold=float(self.grounding_score_thresh),
            pad_square=False,
        )
        time1 = time.time()
        # self.log_info(f"🚨🚨 NanoOwl predict: {(time1-start_time)*1000:.1f} ms.")

        # no_image = draw_owl_output(pil_image, output, text=list(self.text_prompt_list), draw_text=True)
        # no_image.save(os.path.join(self.IMAGE_DIR, f"no_track_{int(time.time()*1000)}.jpg"))


        boxes_t = output.boxes.detach().cpu().numpy() if output.boxes.numel() > 0 else np.empty((0, 4), dtype=np.float32)
        scores_t = output.scores.detach().cpu().numpy() if output.scores.numel() > 0 else np.empty((0,), dtype=np.float32)
        labels_t = output.labels.detach().cpu().numpy() if output.labels.numel() > 0 else np.empty((0,), dtype=np.int64)

        if len(boxes_t) > 0:
            H, W = cv_image.shape[:2]
            boxes_t[:, 0::2] = np.clip(boxes_t[:, 0::2], 0, W - 1)
            boxes_t[:, 1::2] = np.clip(boxes_t[:, 1::2], 0, H - 1)

        det_in = _BotsortInput(boxes_t, scores_t, labels_t.astype(np.float32))
        tracked = self.tracker.update(det_in, cv_image)

        time3 = time.time()
        # self.log_info(f"🚨🚨 BoT-SORT tracking: {(time3-time1)*1000:.1f} ms.")

        if tracked is None or len(tracked) == 0:
            # self.log_info("🚨🚨 No detections to track.")
            return {
                "bboxes": np.empty((0, 4), dtype=float),
                "labels": np.array([], dtype=str),
                "confidences": np.array([], dtype=float),
                "ids": np.array([], dtype=int),
            }

        # self.log_info(f"🚨🚨 Tracked {len(tracked)} objects.")

        # tracked layout: [x1, y1, x2, y2, track_id, conf, cls, det_idx]
        tracked = np.asarray(tracked)
        bboxes = tracked[:, 0:4].astype(float)
        ids = tracked[:, 4].astype(int)
        confidences = tracked[:, 5].astype(float)
        cls_idx = tracked[:, 6].astype(int)
        class_names = self.text_prompt_list[cls_idx]

        det_result = {
            "bboxes": bboxes,
            "labels": class_names,
            "confidences": confidences,
            "ids": ids,
        }
        time2 = time.time()
        time_taken1 = time1 - start_time
        time_taken2 = time2 - time1
        # self.log_info(f"🚨🚨 NanoOwl: {time_taken1*1000:.1f} ms, tracker+pack: {time_taken2*1000:.1f} ms.")

        return det_result

    def image_callback(self, msg):
        start_time = time.time()

        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        det_stamp = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
        # Save the image and timestamp for potential future use (e.g., debugging, visualization).
        self.detection_processing(cv_image, det_stamp)

    def detection_processing(self, image, detection_stamp):
        start_time = time.time()
        self.call_back_time_stamp = start_time

        # ================== Process detection and tracking ==================
        detections = self.inference(image)
        detection_time = time.time()

        image_anno = image.copy()
        if self.ANNOTATE:
            bboxes = detections['bboxes']
            labels = detections['labels']
            obj_ids = detections['ids']

            if len(bboxes) > 0:
                class_ids = np.array(list(range(len(labels))))
                annotation_labels = [
                    f"{class_name} {id_}"
                    for class_name, id_ in zip(labels, obj_ids)
                ]
                detections_ = sv.Detections(xyxy=bboxes, class_id=class_ids)
                self.box_annotator.annotate(scene=image_anno, detections=detections_)
                self.label_annotator.annotate(scene=image_anno, detections=detections_, labels=annotation_labels)

        anotate_time = time.time()
        self.publish_detection_results(detections, detection_stamp, image, image_anno)
        publish_time = time.time()
        # self.log_info(
        #     f"🚨🚨🚨🚨 NanoOwl Detection time: {time.time() - start_time:.2f} seconds, "
        #     f"detection time: {detection_time - start_time:.2f}, "
        #     f"annotate time: {anotate_time - detection_time:.2f}, "
        #     f"publish time: {publish_time - anotate_time:.2f}"
        # )

    def publish_detection_results(self, detections_tracked, detection_stamp, image, image_anno):
        """
        Publish the detection results as a DetectionResult message.
        """
        seconds = int(detection_stamp)
        nanoseconds = int((detection_stamp - seconds) * 1e9)

        detection_result_msg = DetectionResult()
        detection_result_msg.header.stamp = Time(seconds=seconds, nanoseconds=nanoseconds).to_msg()
        detection_result_msg.header.frame_id = 'map'

        for i in range(len(detections_tracked['ids'])):
            detection_result_msg.track_id.append(int(detections_tracked['ids'][i]))
            detection_result_msg.x1.append(float(detections_tracked['bboxes'][i][0]))
            detection_result_msg.y1.append(float(detections_tracked['bboxes'][i][1]))
            detection_result_msg.x2.append(float(detections_tracked['bboxes'][i][2]))
            detection_result_msg.y2.append(float(detections_tracked['bboxes'][i][3]))
            detection_result_msg.label.append(str(detections_tracked['labels'][i]))
            detection_result_msg.confidence.append(float(detections_tracked['confidences'][i]))
        detection_result_msg.image = self.bridge.cv2_to_imgmsg(image, encoding='bgr8')
        self.detection_result_pub.publish(detection_result_msg)
        # self.log_info(f"🚨🚨 NanoOwl Published {len(detections_tracked['ids'])} detected objects.")

        annotated_image_msg = self.bridge.cv2_to_imgmsg(image_anno, encoding='bgr8')
        annotated_image_msg.header.stamp = Time(seconds=seconds, nanoseconds=nanoseconds).to_msg()
        annotated_image_msg.header.frame_id = 'map'
        self.annotated_image_pub.publish(annotated_image_msg)


def main(args=None):
    rclpy.init(args=args)

    detection_node = DetectNode()

    rclpy.spin(detection_node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
