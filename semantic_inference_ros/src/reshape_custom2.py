#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import time
import numpy as np

use_webcamera = False
image_rate = 1000  # 10 Hz means the callback will process one image every 100ms
last_time = time.time()
seq = None
# global time, seq
# seq = None

# Callback function to process incoming image messages
def image_callback(msg):
    global time_msg, last_time, seq
    time_msg = msg.header.stamp
    seq = msg.header.seq
    # print("seg is:", seq)
    # current_time = time.time()
    # if current_time - last_time < (1.0 / image_rate):
    #     return  # Skip processing if we are not at the right interval
    
    # try:
    #     # Convert the ROS Image message to OpenCV format (BGR)
    #     bridge = CvBridge()
    #     cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")

    #     # flip the image
    #     # flip_image = cv2.flip(cv2.flip(cv_image, 0), -1)

    #     # Resize the image to 720x720
    #     # Doing this to run the model
    #     resized_image = cv2.resize(cv_image, (720, 720))

    #     if not use_webcamera:
    #         rotated_image = cv2.transpose(resized_image)
    #         flip_image = cv2.flip(rotated_image, flipCode=1)
    #         resized_image = flip_image

    #     # Convert the resized image back to a ROS message
    #     resized_msg = bridge.cv2_to_imgmsg(resized_image, "bgr8")

    #     # res raw no rotation
    #     resized_noflip_msg = bridge.cv2_to_imgmsg(resized_image, "bgr8")

    #     resized_msg.header.frame_id = "camera_color_optical_frame"
    #     resized_msg.header.stamp = time_msg #msg.header.stamp
    #     resized_msg.header.seq = seq

    #     resized_noflip_msg.header.frame_id = "camera_color_optical_frame"
    #     resized_noflip_msg.header.stamp = time_msg #msg.header.stamp
    #     resized_noflip_msg.header.seq = seq

    # Publish the resized image to another topic to be sent to semantic seg
    pub.publish(msg)
    # camera_pub.publish(resized_noflip_msg)
    print("Now doing image: ", seq)
    rospy.loginfo("Image resized and published to '/resized_image'")

    # except Exception as e:
    #     rospy.logerr("Error processing image: %s", str(e))


# Callback function to process incoming image messages
def semantic_image_callback(msg):
    global seq, time_msg  # Declare seq as global here to modify it
    try:
        # Convert the ROS Image message to OpenCV format (BGR)
        bridge = CvBridge()
        cv_image = bridge.imgmsg_to_cv2(msg, "16SC1")

        # flip the image
        # flip_image = cv2.flip(cv2.flip(cv_image, 0), -1)

        # Resize the image to 720x1280
        resized_image = cv2.resize(cv_image, (720, 1280))

        if not use_webcamera:
            rotated_image = cv2.transpose(resized_image)
            flip_image = cv2.flip(rotated_image, flipCode=0)
            resized_image = flip_image

        normalized_image = cv2.normalize(resized_image, None, 0, 255, cv2.NORM_MINMAX)

        # Convert to an 8-bit single channel (8UC1) format
        final_image = normalized_image.astype('uint8')

        # print(np.unique(final_image))

        # Convert the final image back to a ROS message
        resized_msg_semantic = bridge.cv2_to_imgmsg(final_image, "8UC1")
        resized_msg_semantic.header.frame_id = "camera_color_optical_frame"
        resized_msg_semantic.header.stamp = time_msg #msg.header.stamp
        
        resized_msg_semantic.header.seq = seq

        # Convert the resized image back to a ROS message
        # resized_msg = bridge.cv2_to_imgmsg(resized_image, "16SC1")

        # Publish the resized image to another topic
        # print("Now doing semantic: ", seq, resized_msg_semantic.header.seq)
        semantic_pub.publish(resized_msg_semantic)
        rospy.loginfo("Image resized and published to '/resized_image'")

    except Exception as e:
        rospy.logerr("Error processing image: %s", str(e))


def image_resizer():
    # Initialize the ROS node
    rospy.init_node('image_resizer_node', anonymous=True)

    # Subscribe to the incoming image topic (adjust topic name as needed)
    rospy.Subscriber('/tesse/seg_cam/converted/image', Image, image_callback)

    # Create a ROS publisher that will publish the resized image
    global pub, camera_pub, semantic_pub
    # camera_pub = rospy.Publisher('/camera/color/resized', Image, queue_size=10)

    # This topic goes into the semantic segmentation pipeline
    pub = rospy.Publisher('/semantic_inference/color/image_raw', Image, queue_size=10)

    # This is the binary semantic topic
    rospy.Subscriber('/semantic_inference/semantic/image_raw', Image, semantic_image_callback)

    # New binary semantic seg topic resized, rotated as image_raw and same format as custom2.bag
    # This topic should go into khronos
    semantic_pub = rospy.Publisher('/semantic_inference/semantic/image_raw_resized', Image, queue_size=10)

    # Spin to keep the node running
    rospy.spin()

if __name__ == '__main__':
    try:
        image_resizer()
    except rospy.ROSInterruptException:
        pass
