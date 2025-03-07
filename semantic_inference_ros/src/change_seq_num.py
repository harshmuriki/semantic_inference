import rospy
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Header 
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import CameraInfo
from rospy.exceptions import ROSInterruptException
from cv_bridge import CvBridge
import cv2

# Dont need this

class ImageProcessor:
    def __init__(self):
        self.bridge = CvBridge()
        
        # Define subscribers for two topics
        self.image_sub = rospy.Subscriber("/semantic_inference/semantic/image_raw_resized", Image, self.image_callback)
        self.info_sub = rospy.Subscriber("/camera/color/image_raw", Image, self.info_callback)
        self.depth_img = rospy.Subscriber("/camera/depth/image_rect_raw", Image, self.depth_callback)
        self.image_info = rospy.Subscriber("/camera/color/camera_info", CameraInfo, self.camera_info_callback)
        
        self.image_pub = rospy.Publisher("/camera/color/new_image", Image, queue_size=10)
        self.image_info_pub = rospy.Publisher("/camera/color/new_camera_info", CameraInfo, queue_size=10)
        self.depth_pub = rospy.Publisher("/camera/depth/new_depth", Image, queue_size=10)

        # Variable to store timestamp and seq
        self.timestamp = None
        self.seq = None
        
        self.image_data = None
        self.image_info_data = None
        self.depth_data = None

    def camera_info_callback(self, msg):
        self.image_info_data = msg

    def depth_callback(self, msg):
        msg.header.frame_id = "camera_color_optical_frame"  # Change this to your desired frame_id
        self.depth_data = msg

    def info_callback(self, msg):
        # Capture timestamp and sequence from camera_info or another relevant topic
        self.timestamp = msg.header.stamp
        self.seq = msg.header.seq

        self.image_data = msg

    def image_callback(self, msg):
        # Only proceed if we have the timestamp and seq
        if self.timestamp is None or self.seq is None:
            rospy.logwarn("Waiting for timestamp and seq id.")
            return
        
        # Create a new Image message using the timestamp and seq from the info topic
        new_image = Image()
        new_image.header.stamp = self.timestamp
        new_image.header.seq = self.seq
        new_image.header.frame_id = msg.header.frame_id  # Keep the same frame_id
        
        # Copy the image data from the received message
        new_image.height = msg.height
        new_image.width = msg.width
        new_image.encoding = msg.encoding
        new_image.is_bigendian = msg.is_bigendian
        new_image.step = msg.step
        new_image.data = msg.data  # Copy the image pixel data
        
        # Publish the new image
        new_image_pub = rospy.Publisher("/semantic_inference/semantic/new_semantic", Image, queue_size=10)
        new_image_pub.publish(new_image)

        self.image_pub.publish(self.image_data)

        self.image_info_pub.publish(self.image_info_data)

        self.depth_pub.publish(self.depth_data)

        rospy.loginfo("Published new image with timestamp: %s, seq: %d", self.timestamp, self.seq)


if __name__ == "__main__":
    rospy.init_node("image_processor_node")
    processor = ImageProcessor()
    
    try:
        rospy.spin()
    except ROSInterruptException:
        pass
