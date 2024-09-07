# This script defines a ROS2 node that subscribes to a topic to receive image data. Upon
# receiving an image, it uses the cvlib library to detect common objects, draws bounding
# boxes around these detected objects, and publishes the processed image to another topic.
# The node also logs the time taken for the detection process


# rclpy: the ROS2 client library for Python, used for creating and interacting with ROS nodes
import rclpy

# Node: a base class in ROS2 for creating nodes
from rclpy.node import Node

# The OpenCV library for computer vision tasks
import cv2

# CvBridge: a ROS package for converting between ROS image messages and OpenCV images
from cv_bridge import CvBridge

# Image: a message type for publishing image data
from sensor_msgs.msg import Image

# Import the cvlib library and alias it as cv. More information about the cvlib library here:
# https://github.com/arunponnusamy/cvlib
import cvlib as cv
from cvlib.object_detection import draw_bbox

import time


class DetectorNode(Node):


    def __init__(self, node_name = "detector_node", publish_topic = "/object_detection/output", subscribe_topic = "camera/image_raw"):

        # Assign parameters to instance variables
        self.publish_topic = publish_topic
        self.subscribe_topic = subscribe_topic

        # Initialize the node with the passed name
        super().__init__(node_name)

        # Create a subscriber to listen to the messages received on the self.subscribe_topic topic.
        # The callback method is called when a message is received.
        self.sub = self.create_subscription(Image, self.subscribe_topic, self.callback, 1)

        # Create a publisher that publishes the processed image to the self.publish_topic topic.
        self.pub = self.create_publisher(Image, self.publish_topic, 1)

        # Initialize a CvBridge object for converting between ROS image messages and OpenCV images
        self.cv_bridge = CvBridge()


    def callback(self, msg):

        # Record the current time to measure processing duration
        time_now = time.time()

        # Convert the incoming ROS image message to an OpenCV image
        img_opencv = self.cv_bridge.imgmsg_to_cv2(msg)

        # Uses cvlib to detect common objects in the image. bbox contains bounding boxes, label
        # contains labels of detected objects, and conf contains confidence scores
        bbox, label, conf = cv.detect_common_objects(img_opencv, enable_gpu=False)

        # Draw bounding boxes around detected objects on the image
        output_image = draw_bbox(img_opencv, bbox, label, conf)

        output_image = cv2.cvtColor(output_image, cv2.COLOR_BGR2RGB)

        # Convert the processed OpenCV image back to a ROS Image message
        img_msg = self.cv_bridge.cv2_to_imgmsg(output_image)

        # Copy the header from the incoming message to the outgoing message to maintain synchronization
        img_msg.header = msg.header

        # Publish the processed image to the publishing topic
        self.pub.publish(img_msg)

        # Log the time taken for detection
        self.get_logger().info("Detection took {}s ".format(time.time() - time_now))


def main(args=None):
    rclpy.init(args=args)
    detector = DetectorNode()
    rclpy.spin(detector)
    detector.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()