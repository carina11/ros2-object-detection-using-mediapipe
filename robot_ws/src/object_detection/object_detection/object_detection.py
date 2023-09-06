import mediapipe as mp
from ament_index_python.packages import get_package_share_directory
import os
import numpy as np
import cv2
from object_detection_msgs.msg import Box, Detection, DetectionResult

from visualizer import Visualizer

BaseOptions = mp.tasks.BaseOptions
ObjectDetector = mp.tasks.vision.ObjectDetector
ObjectDetectorOptions = mp.tasks.vision.ObjectDetectorOptions
VisionRunningMode = mp.tasks.vision.RunningMode


class ObjectDetection:

    def __init__(self, max_results=1) -> None:
        model_path = os.path.join(
            get_package_share_directory(
                "object_detection"), "model", "efficientdet_lite0.tflite"
        )

        # Options for object detector model
        options = ObjectDetectorOptions(
            base_options=BaseOptions(model_asset_path=model_path),
            max_results=max_results,
            running_mode=VisionRunningMode.IMAGE,
        )

        # Object Detector
        self.model = ObjectDetector.create_from_options(options)

        # Load visualizer
        self.visualizer = Visualizer((0, 255, 0))  # Text color: green

    def detect(self, image: np.ndarray):
        """Runs Object Detection on the input image and returns an image with bounding box and results.

        Args:
            image (np.ndarra): Input image

        Returns:
            Image with bounding box
            Detection results
        """
        mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=image)
        detection_result = self.model.detect(mp_image)
        return self.visualizer.visualize(image, detection_result), self._prepare_result(detection_result)

    @staticmethod
    def _prepare_result(detection_result):
        detection_result_ros = DetectionResult()
        for detection in detection_result.detections:
            bounding_box_ros = Box(
                x=detection.bounding_box.origin_x,
                y=detection.bounding_box.origin_y,
                w=detection.bounding_box.width,
                h=detection.bounding_box.height,
            )

            detection_ros = Detection(
                bounding_box=bounding_box_ros,
                score=detection.categories[0].score,
                name=detection.categories[0].category_name,
            )

            detection_result_ros.detections.append(detection_ros)

        return detection_result_ros
