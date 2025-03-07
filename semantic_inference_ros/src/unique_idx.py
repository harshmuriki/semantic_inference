#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Header
import time
import numpy as np

# This prints the unique idxs from segmentation

# Callback function that will be triggered when an image message is received
def image_callback(msg):
    idx = np.unique(msg.data)

    print("Unique idx:", idx)

# Initialize the ROS node
def main():
    # Initialize the ROS node (give it a name)
    rospy.init_node('image_subscriber', anonymous=True)
    
    # Define the image topic to subscribe to
    image_topic = "/semantic_inference/semantic/new_semantic"  # Change this to your desired topic
    
    # Create a subscriber to the image topic
    rospy.Subscriber(image_topic, Image, image_callback)
    
    # Keep the node running
    rospy.spin()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
