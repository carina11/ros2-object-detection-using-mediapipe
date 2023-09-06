from typing import Tuple
import numpy as np
import cv2


class Visualizer:
    def __init__(self, text_color: Tuple[int, int, int]) -> None:
        self.margin = 10
        self.row_size = 10
        self.font_size = 1
        self.font_thickness = 1
        self.text_color = text_color

    def visualize(
        self,
        image,
        detection_result
    ) -> np.ndarray:
        """Draws bounding boxes on the input image and returns it.
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
            cv2.rectangle(image, start_point, end_point, self.text_color, 3)

            # Draw label and score
            category = detection.categories[0]
            category_name = category.category_name
            probability = round(category.score, 2)
            result_text = category_name + ' (' + str(probability) + ')'
            text_location = (self.margin + bbox.origin_x,
                             self.margin + self.row_size + bbox.origin_y)
            cv2.putText(image, result_text, text_location, cv2.FONT_HERSHEY_PLAIN,
                        self.font_size, self.text_color, self.font_thickness)

        return image
