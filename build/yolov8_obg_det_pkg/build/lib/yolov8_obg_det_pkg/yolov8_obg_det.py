import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError
from ultralytics import YOLO
import json
import os
from ament_index_python.packages import get_package_share_directory

class YOLOv8ObjDetNode(Node):
    def __init__(self):
        super().__init__('yolov8_obj_det_node')
        
        try:
            package_share_directory = get_package_share_directory('yolov8_obg_det_pkg')
            model_path = os.path.join(package_share_directory, 'model', 'yolov8n.pt')
        except KeyError:
            self.get_logger().error('Package yolov8_obg_det_pkg not found in share directory.')
            return
        
        # 檢查模型文件是否存在
        if not os.path.exists(model_path):
            self.get_logger().error(f'Model file not found: {model_path}')
            return
        
        self.get_logger().info(f'Loading model from: {model_path}')
        
        # 初始化模型
        self.model = YOLO(model_path)
        
        # 訂閱RGB圖片話題
        self.subscription = self.create_subscription(
            Image,
            '/rgb',
            self.listener_callback,
            10)
        
        self.get_logger().info('Subscribed to /rgb topic')
        
        # 發布檢測結果話題
        self.publisher_ = self.create_publisher(String, 'detection_results', 10)
        
        self.bridge = CvBridge()

    def listener_callback(self, msg):
        self.get_logger().info('Received an image')
        try:
            # 將ROS影像消息轉換為OpenCV格式
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except CvBridgeError as e:
            self.get_logger().error(f'CvBridge Error: {e}')
            return

        # 進行物體檢測
        results = self.model.predict(cv_image)

        detections = []
        for result in results:
            boxes = result.boxes.xyxy  # 檢測框座標
            confidences = result.boxes.conf  # 檢測置信度
            class_ids = result.boxes.cls  # 檢測類別ID

            for box, confidence, class_id in zip(boxes, confidences, class_ids):
                detection = {
                    'box': box.tolist(),
                    'confidence': float(confidence),
                    'class_id': int(class_id),
                    'class_name': self.model.names[int(class_id)]
                }
                detections.append(detection)

        self.get_logger().info(f'Detections: {detections}')

        # 發送檢測結果
        detection_msg = String()
        detection_msg.data = json.dumps(detections)
        self.publisher_.publish(detection_msg)
        self.get_logger().info('Published detection results')

def main(args=None):
    rclpy.init(args=args)
    node = YOLOv8ObjDetNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
