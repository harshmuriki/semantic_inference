import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np

class ImageComparer:
    def __init__(self):
        self.bridge = CvBridge()

        # Subscribe to two image topics
        self.image_sub1 = rospy.Subscriber("/semantic_inference/semantic/image_raw", Image, self.image_callback1)
        self.image_sub2 = rospy.Subscriber("/semantic_inference/semantic/image_raw_resized", Image, self.image_callback2)

        self.image_data1 = None
        self.image_data2 = None

    def image_callback1(self, msg):
        # Convert the image message to a numpy array
        self.image_data1 = self.bridge.imgmsg_to_cv2(msg, desired_encoding="16SC1")
        rospy.loginfo("Received first image")

    def image_callback2(self, msg):
        # Convert the image message to a numpy array
        self.image_data2 = self.bridge.imgmsg_to_cv2(msg, desired_encoding="16UC1")
        rospy.loginfo("Received second image")
        
        # Once both images are received, compare them
        if self.image_data1 is not None and self.image_data2 is not None:
            self.compare_images()

    def compare_images(self):
        # Check if the shapes (dimensions) of the two images are the same
        if self.image_data1.shape != self.image_data2.shape:
            rospy.loginfo("The images have different shapes.")
            return
        
        # Compare pixel-wise using NumPy
        difference = np.sum(self.image_data1 != self.image_data2)

        if difference == 0:
            rospy.loginfo("The images are identical.")
        else:
            rospy.loginfo(f"The images are different. Number of different pixels: {difference}")

if __name__ == "__main__":
    rospy.init_node("image_comparer_node")
    image_comparer = ImageComparer()
    
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
