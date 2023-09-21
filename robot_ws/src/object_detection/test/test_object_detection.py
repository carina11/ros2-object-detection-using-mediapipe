import os
import unittest

# Thirdparty
import cv2 as cv

# Current package
from object_detection.object_detection_main import ObjectDetection

# Image path
IMAGE_PATH = os.path.realpath(os.path.join(os.getcwd(), os.path.dirname(__file__), "images"))


class GestureDetectionTests(unittest.TestCase):
    """Unit tests for Object Detection."""

    def setUp(self) -> None:
        # Load the package
        self.object_detection = ObjectDetection()
        
        # Load test images
        image_person = cv.imread(os.path.join(IMAGE_PATH, "person.jpeg"))
        self.image_person = cv.cvtColor(image_person, cv.COLOR_BGR2RGB)
        image_phone = cv.imread(os.path.join(IMAGE_PATH, "phone.jpeg"))
        self.image_phone = cv.cvtColor(image_phone, cv.COLOR_BGR2RGB)

    def test_object_detection_person(self):
        """Test object detection."""
        _, detection_result = self.object_detection.detect(self.image_person)
        object_name = detection_result.detections[0].name
        self.assertEqual(object_name, "person")

    def test_object_detection_phone(self):
        """Test object detection."""
        
        _, detection_result = self.object_detection.detect(self.image_phone)
        object_name = detection_result.detections[0].name
        self.assertEqual(object_name, "cell phone")



if __name__ == "__main__":
    unittest.main()