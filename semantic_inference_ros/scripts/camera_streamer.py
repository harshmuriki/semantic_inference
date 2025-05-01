#!/usr/bin/env python3

import rospy
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

def main():
    rospy.init_node('webcam_streamer', anonymous=True)

    # Publisher to the required topic
    pub = rospy.Publisher('/camera/color/image_raw/semantic_uncompressed', Image, queue_size=1)

    # Capture from default webcam
    cap = cv2.VideoCapture(1)
    if not cap.isOpened():
        rospy.logerr("Failed to open webcam.")
        return

    bridge = CvBridge()
    rate = rospy.Rate(30)  # 30 FPS

    while not rospy.is_shutdown():
        ret, frame = cap.read()
        if not ret:
            rospy.logwarn("Failed to grab frame")
            continue

        frame = cv2.resize(frame, (640, 480))  # Example resize
        # Draw a black point at the center of the image
        center_x, center_y = frame.shape[1] // 2, frame.shape[0] // 2
        cv2.circle(frame, (center_x, center_y), 10, (0, 0, 0), -1)  # Draw a filled black circle of radius 10 px
        try:
            msg = bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            pub.publish(msg)
        except Exception as e:
            rospy.logerr(f"Error converting or publishing frame: {e}")

        rate.sleep()

    cap.release()

if __name__ == '__main__':
    try:
        print("Starting webcam streamer...")
        main()
    except rospy.ROSInterruptException:
        pass
