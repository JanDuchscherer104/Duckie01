#!/usr/bin/env python3

import os

import cv2
import rospy
from cv_bridge import CvBridge
from duckietown.dtros import DTROS, NodeType
from sensor_msgs.msg import CompressedImage


class CameraReaderNode(DTROS):
    def __init__(self, node_name: str):
        """
        Initializes the DTROS parent class and sets up the camera reader node.

        Args:
            node_name (str): The name of the node.
        """
        super(CameraReaderNode, self).__init__(
            node_name=node_name, node_type=NodeType.VISUALIZATION
        )

        # Static parameters
        self._vehicle_name = os.environ["VEHICLE_NAME"]

        # Camera topic
        self._camera_topic = f"/{self._vehicle_name}/camera_node/image/compressed"

        # Bridge between OpenCV and ROS
        self._bridge = CvBridge()

        # Create window
        self._window = "camera-reader"
        self._gray_window = "gray-camera-reader"
        self._canny_window = "canny-camera-reader"

        cv2.namedWindow(self._window, cv2.WINDOW_AUTOSIZE)
        cv2.namedWindow(self._gray_window, cv2.WINDOW_AUTOSIZE)
        cv2.namedWindow(self._canny_window, cv2.WINDOW_AUTOSIZE)

        # Construct subscriber
        self.sub = rospy.Subscriber(self._camera_topic, CompressedImage, self.callback)

    def callback(self, msg: CompressedImage):
        """
        Callback function that processes the camera stream.

        Args:
            msg (CompressedImage): The incoming ROS message containing the compressed image.
        """
        # Convert JPEG bytes to CV image
        image = self._bridge.compressed_imgmsg_to_cv2(msg, desired_encoding="bgr8")

        # Convert the image to a gray-scale image
        gray_filtered = cv2.GaussianBlur(
            cv2.cvtColor(image, cv2.COLOR_BGR2GRAY), ksize=(5, 5), sigmaX=0
        )

        # Convert the gray-scale image into a Canny (edge detector)
        canny = cv2.Canny(gray_filtered, threshold1=100, threshold2=200)

        # Show the different views as an output
        cv2.imshow(self._window, image)
        cv2.imshow(self._gray_window, gray_filtered)
        cv2.imshow(self._canny_window, canny)

        cv2.waitKey(1)

    # def callback(self, msg: CompressedImage) -> None:
    #     # Convert JPEG bytes to CV image
    #     image = self._bridge.compressed_imgmsg_to_cv2(msg, desired_encoding="bgr8")

    #     # Display frame
    #     cv2.imshow(self._window, image)

    #     cv2.waitKey(1)


if __name__ == "__main__":
    # Create the node
    node = CameraReaderNode(node_name="camera_reader_node")

    # Keep spinning to keep the program alive
    rospy.spin()
