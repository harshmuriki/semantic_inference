#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
import csv
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from message_filters import Subscriber, ApproximateTimeSynchronizer

def load_label_map(filepath):
    """
    Loads a CSV label map file formatted as:
      name,red,green,blue,alpha,id
    and returns a dict { id: { 'name': name, 'color': (r,g,b) } }.
    """
    label_map = {}
    with open(filepath, "r") as f:
        reader = csv.reader(f)
        next(reader, None)  # skip header if present
        for row in reader:
            if len(row) < 6:
                continue
            name = row[0]
            color = tuple(map(int, row[1:4]))
            idx = int(row[5])
            label_map[idx] = {'name': name, 'color': color}

    return label_map

def callback(color_msg, label_msg):
    try:
        # Convert the color image and label image messages to OpenCV images
        color_image = bridge.imgmsg_to_cv2(color_msg, desired_encoding="bgr8")
        # Assuming label image is published as mono8 (each pixel is an integer label)
        label_image = bridge.imgmsg_to_cv2(label_msg, desired_encoding="16UC1")
    except Exception as e:
        rospy.logerr("CV Bridge conversion error: %s", e)
        return

    # Prepare an overlay image (same size as the color image)
    # Ensure label_image matches the dimensions of color_image
    label_image_resized = label_image
    uque_labels = np.unique(label_image)
    print("Unique labels in label image:", uque_labels)
    # label_image_resized = cv2.resize(label_image, (color_image.shape[1], color_image.shape[0]), interpolation=cv2.INTER_NEAREST)
    overlay = np.zeros_like(color_image)
    # For each label id in our label map create a colored mask
    center_x, center_y = label_image_resized.shape[1] // 2, label_image_resized.shape[0] // 2
    print("Center of label image:", center_x, center_y, label_image_resized[center_y, center_x])
    for label_id, info in label_map.items():
        mask = label_image_resized == label_id
        overlay[mask] = info['color']

        # Add label_id text to the overlay
        y_coords, x_coords = np.where(mask)
        if len(x_coords) > 0 and len(y_coords) > 0:
            # Calculate the centroid of the mask
            centroid_x = int(np.mean(x_coords))
            centroid_y = int(np.mean(y_coords))
            # Put the label_id text at the centroid
            cv2.putText(overlay, str(info['name']), (centroid_x, centroid_y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)

    # Blend the overlay with the original image
    alpha = 0.5  # transparency factor for overlay
    combined = cv2.addWeighted(color_image, 1.0 - alpha, overlay, alpha, 0)

    cv2.imshow("Image with Label Overlay", combined)
    cv2.waitKey(1)

if __name__ == '__main__':
    rospy.init_node("image_label_overlay", anonymous=True)
    bridge = CvBridge()

    # Change the file path below to your actual label map file.
    label_map = load_label_map("/home/khronos/catkin_ws/src/semantic_inference/semantic_inference/config/distinct_150_colors.csv")

    # Subscribe to both the color image and the label image topics.
    color_sub = Subscriber("/camera/color/image_raw/semantic_uncompressed", Image)
    label_sub = Subscriber("/semantic_inference/semantic/image_raw", Image)
    # Use ApproximateTimeSynchronizer to synchronize the incoming messages
    ats = ApproximateTimeSynchronizer([color_sub, label_sub], queue_size=10, slop=0.1)
    ats.registerCallback(callback)

    rospy.loginfo("Starting overlay subscriber...")
    rospy.spin()
    cv2.destroyAllWindows()