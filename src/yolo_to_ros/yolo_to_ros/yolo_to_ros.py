#!/usr/bin/env python

import cv2
import torch
from ultralytics import YOLO
import math
from os import putenv

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped
from image_geometry import PinholeCameraModel
from std_msgs.msg import Bool  # To notify waypoint_explorer to return

from cv_bridge import CvBridge
import time

# For AMD ROCm
# putenv("HSA_OVERRIDE_GFX_VERSION", "10.3.0")
# For NVIDIA CUDA
torch.cuda.set_device(0)

class DetectionNode(Node):
    def __init__(self):
        super().__init__('detection_node')
        self.bridge = CvBridge()
        self.detections = self.create_publisher(Image, '/yolo_detections', 10)
        #self.subscription = self.create_subscription(Image, '/head_front_camera/rgb/image_raw', self.image_callback, 10)
        self.subscription = self.create_subscription(Image, 'camera/image_raw', self.image_callback, 10)
        self.subscription  # prevent unused variable warning
        self.model = YOLO('yolov8n.pt')  # standard YOLOv8 nano model
        
        
        # File to log the detected images
        self.log_file = open("detected_images_log.txt", "a")
        self.check = 0   #Flag to check whether Bool=True published
        
         # Set a confidence threshold to filter out low-confidence detections
        self.confidence_threshold = 0.9  # Change this value to adjust the certainty level
        
        # To keep track of detected distinct classes (images)
        self.detected_classes = set()
        self.max_detections = 3  # Number of distinct images to detect
        
        # Publisher to notify waypoint_explorer when 3 distinct images are detected
        self.image_detection_complete_pub = self.create_publisher(Bool, '/image_detection_complete', 10)
    
    
    def log_detection(self, class_name, confidence):
        """Log the detected image and confidence to a file."""
        timestamp = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime())
        log_entry = f"{timestamp}: Detected class: {class_name}, Confidence: {confidence}, Check: {self.check}\n"
        self.log_file.write(log_entry)
        self.log_file.flush()  # Ensure the data is written immediately



    def image_callback(self, frame):
        frame = self.bridge.imgmsg_to_cv2(frame, "bgr8")
        results = self.model(frame, stream=True)
        for r in results:
            boxes = r.boxes
            for box in boxes:
                # Pixel coordinates
                #x1, y1, x2, y2 = box.xyxy[0]
                #x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)

                # Put boxes in frame
                #cv2.rectangle(frame, (x1, y1), (x2, y2), (100, 0, 255), 1)

                # Confidence
                confidence = math.ceil((box.conf[0] * 100)) / 100
                
                # Only process detections above the confidence threshold
                if confidence < self.confidence_threshold:
                    continue  # Skip detections with low confidence

                # Optional confidence output in console
                print("Confidence --->", confidence)

                # Class name
                cls = int(box.cls[0])

                # Optional class name output in console
                print("Class name -->", r.names[cls])
                
                # Log the detection
                class_name = r.names[cls]
                print(f"Detected class: {class_name}, Confidence: {confidence}")
                self.log_detection(class_name, confidence)
                
                # If this class hasn't been detected yet, add it to the set
                if cls not in self.detected_classes:
                    self.detected_classes.add(cls)
                    print(f"New image detected: {r.names[cls]} (Confidence: {confidence})")
                    print(f"Currently detected distinct images: {self.detected_classes}")

                # If 3 distinct images have been detected, notify waypoint_explorer
                if len(self.detected_classes) >= self.max_detections:
                    print("Three distinct images detected. Notifying waypoint_explorer to return to (0, 0).")
                    self.check = 1
                    self.log_detection(class_name, confidence)
                    self.image_detection_complete_pub.publish(Bool(data=True))
                    self.check = 2
                    self.log_detection(class_name, confidence)


                #org = [x1, y1]
                font = cv2.FONT_HERSHEY_SIMPLEX
                fontScale = 1
                color = (100, 0, 255)
                thickness = 1
                #cv2.putText(frame, f"{r.names[cls]} {confidence}", org, font, fontScale, color, thickness)
        self.detections.publish(self.bridge.cv2_to_imgmsg(frame, 'bgr8'))

def main():
    rclpy.init()
    depth_to_pose_node = DetectionNode()
    try:
        rclpy.spin(depth_to_pose_node)
    except KeyboardInterrupt:
        pass
    depth_to_pose_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()