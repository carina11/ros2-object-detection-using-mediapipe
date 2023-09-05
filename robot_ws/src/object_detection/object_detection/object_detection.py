import mediapipe as mp
from mediapipe.tasks import python
from mediapipe.tasks.python import vision
from ament_index_python.packages import get_package_share_directory
import os
import numpy as np
import cv2

BaseOptions = mp.tasks.BaseOptions
ObjectDetector = mp.tasks.vision.ObjectDetector
ObjectDetectorOptions = mp.tasks.vision.ObjectDetectorOptions
VisionRunningMode = mp.tasks.vision.RunningMode

MARGIN = 10  # pixels
ROW_SIZE = 10  # pixels
FONT_SIZE = 1
FONT_THICKNESS = 1
TEXT_COLOR = (0, 255, 0)  # red

class ObjectDetection:
    
    
    
    def __init__(self) -> None:
        model_path = os.path.join(
            get_package_share_directory("object_detection"), "model", "efficientdet_lite0.tflite"
        )
        
        options = ObjectDetectorOptions(
            base_options=BaseOptions(model_asset_path=model_path),
            max_results=2,
            running_mode=VisionRunningMode.IMAGE,
        )
        
        self.model = ObjectDetector.create_from_options(options)
        
        
    def detect(self, image: np.ndarray):
        mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=image)
        detection_result = self.model.detect(mp_image)
        return self.visualize(image, detection_result)
        


    
    @staticmethod
    def visualize(
        image,
        detection_result
    ) -> np.ndarray:
        """Draws bounding boxes on the input image and return it.
        Args:
            image: The input RGB image.
            detection_result: The list of all "Detection" entities to be visualize.
        Returns:
            Image with bounding boxes.
        """
        for detection in detection_result.detections:
            # Draw bounding_box
            bbox = detection.bounding_box
            start_point = bbox.origin_x, bbox.origin_y
            end_point = bbox.origin_x + bbox.width, bbox.origin_y + bbox.height
            cv2.rectangle(image, start_point, end_point, TEXT_COLOR, 3)

            # Draw label and score
            category = detection.categories[0]
            category_name = category.category_name
            probability = round(category.score, 2)
            result_text = category_name + ' (' + str(probability) + ')'
            text_location = (MARGIN + bbox.origin_x,
                            MARGIN + ROW_SIZE + bbox.origin_y)
            cv2.putText(image, result_text, text_location, cv2.FONT_HERSHEY_PLAIN,
                        FONT_SIZE, TEXT_COLOR, FONT_THICKNESS)

        return image