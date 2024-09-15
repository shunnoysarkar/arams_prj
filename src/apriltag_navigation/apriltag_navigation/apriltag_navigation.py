import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from apriltag_msgs.msg import AprilTagDetectionArray
from geometry_msgs.msg import Twist
import cv2
from cv_bridge import CvBridge

# Making a class
class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('image_subscriber')
        self.subscription = self.create_subscription(
            Image, 
            '/camera/image_raw',
            self.listener_callback,
            10
        )
        self.detection_subscription = self.create_subscription(
            AprilTagDetectionArray,
            '/detections',
            self.detection_callback,
            10
        )
        #self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.bridge = CvBridge()
        self.target_detected = False
        self.target_centre = None
        
    def listener_callback(self, data):
        current_frame = self.bridge.imgmsg_to_cv2(data, desired_encoding='bgr8')
        if self.target_detected and self.target_centre:
            cv2.circle(current_frame, (int(self.target_centre.x), int(self.target_centre.y)), 10,(0, 255, 0), -1)
        cv2.imshow("Camera Feed", current_frame)
        cv2.waitKey(1)
    
    def detection_callback(self, msg):
        if len(msg.detections) > 0:
            self.target_detected = True
            self.target_centre = msg.detections[0].centre
            
            #self.move_towards_tag()
        else:
            self.target_detected = False
    


def main(args=None):
    rclpy.init(args=args)
    image_subscriber = ImageSubscriber()
    rclpy.spin(image_subscriber)
    image_subscriber.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
