#!/usr/bin/env python3
import os
import sys

# Ensure Python AI virtual environment site-packages are accessible
venv_site = '/home/soham-darade/CV_Autonomous_Navigation/.venv/lib/python3.14/site-packages'
if venv_site not in sys.path:
    sys.path.insert(0, venv_site)

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ultralytics import YOLO
import torch

from autonomous_robot_interfaces.msg import Detection2D, Detection2DArray

class ObjectDetectionNode(Node):
    def __init__(self):
        super().__init__('object_detection_node')

        self.declare_parameter('model_path', '/home/soham-darade/CV_Autonomous_Navigation/models/yolov8n.pt')
        self.declare_parameter('confidence_threshold', 0.35)
        self.declare_parameter('device', 'cuda:0' if torch.cuda.is_available() else 'cpu')
        self.declare_parameter('publish_annotated_image', True)

        model_path = self.get_parameter('model_path').get_parameter_value().string_value
        self.conf_thresh = self.get_parameter('confidence_threshold').get_parameter_value().double_value
        self.device = self.get_parameter('device').get_parameter_value().string_value
        self.publish_annotated = self.get_parameter('publish_annotated_image').get_parameter_value().bool_value

        self.get_logger().info(f'Loading YOLO model from: {model_path} on {self.device}')
        self.model = YOLO(model_path)
        self.model.to(self.device)

        # Strictly verify that model parameters are actually allocated on CUDA
        param_device = next(self.model.model.parameters()).device
        self.get_logger().info(f'YOLO device: {param_device}')
        if torch.cuda.is_available():
            self.get_logger().info(f'GPU: {torch.cuda.get_device_name(0)}')
        else:
            self.get_logger().warn('CUDA is not available according to PyTorch!')

        assert str(param_device) == 'cuda:0', (
            f"CRITICAL: Model parameters allocated on {param_device} instead of cuda:0!"
        )
        self.get_logger().info('Model parameter verification PASSED: cuda:0 confirmed.')

        self.bridge = CvBridge()

        # Publishers
        self.pub_detections = self.create_publisher(Detection2DArray, '/vision/detections', 10)
        self.pub_annotated = self.create_publisher(Image, '/vision/annotated_image', 10)

        # Subscriber
        self.sub_image = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 1)
        self.get_logger().info('ObjectDetectionNode ready and listening to /camera/image_raw')

    def image_callback(self, msg: Image):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'CvBridge error: {e}')
            return

        # Perform YOLO inference on GPU
        results = self.model(cv_image, conf=self.conf_thresh, device=self.device, verbose=False)[0]

        detection_array = Detection2DArray()
        detection_array.header = msg.header

        for box in results.boxes:
            cls_id = int(box.cls[0].item())
            class_name = self.model.names[cls_id]
            conf = float(box.conf[0].item())
            xyxy = box.xyxy[0].cpu().numpy()

            det = Detection2D()
            det.class_name = class_name
            det.confidence = conf
            det.x_min = float(xyxy[0])
            det.y_min = float(xyxy[1])
            det.x_max = float(xyxy[2])
            det.y_max = float(xyxy[3])
            det.distance = -1.0  # Will be populated by LiDAR fusion node
            det.bearing = 0.0

            detection_array.detections.append(det)

        self.pub_detections.publish(detection_array)

        if self.publish_annotated:
            annotated_frame = results.plot()
            annotated_msg = self.bridge.cv2_to_imgmsg(annotated_frame, encoding='passthrough')
            annotated_msg.encoding = 'bgr8'
            annotated_msg.header = msg.header
            self.pub_annotated.publish(annotated_msg)

def main(args=None):
    rclpy.init(args=args)
    node = ObjectDetectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
