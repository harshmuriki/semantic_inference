#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import CompressedImage, Image
import cv2
import numpy as np
from cv_bridge import CvBridge

class ImageDecompressor:
    def __init__(self):
        # Initialize the node
        #rospy.init_node('image_decompressor', anonymous=True)

        # Create a CvBridge instance
        self.bridge = CvBridge()

        # Subscriber to the compressed image topic
        self.compressed_sub = rospy.Subscriber('/camera/color/image_raw/compressed', CompressedImage, self.callback)

        # Publisher for the decompressed image topic
        self.raw_pub = rospy.Publisher('/camera/color/image_raw_uncompressed_semantic', Image, queue_size=10)

    def callback(self, compressed_msg):
        try:
            # Ensure the compressed data is properly handled as a NumPy array
            np_arr = np.frombuffer(compressed_msg.data, dtype=np.uint8)

            # Decode the compressed image data
            cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

            # Convert OpenCV image to ROS Image message
            ros_image = self.bridge.cv2_to_imgmsg(cv_image, encoding="bgr8")

            # Publish the decompressed image
            self.raw_pub.publish(ros_image)
        except Exception as e:
            rospy.logerr(f"Failed to decompress image: {e}")

if __name__ == "__main__":
    try:
        decompressor = ImageDecompressor()
        rospy.loginfo("Image decompressor node started.")
        # rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Image decompressor node terminated.")
