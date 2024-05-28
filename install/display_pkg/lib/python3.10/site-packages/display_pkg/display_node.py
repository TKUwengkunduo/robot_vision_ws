import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError
import json
import cv2

class DisplayNode(Node):
    def __init__(self):
        super().__init__('display_node')
        
        # 訂閱影像話題
        self.image_subscription = self.create_subscription(
            Image,
            '/rgb',
            self.image_callback,
            10)
        
        # 訂閱檢測結果話題
        self.detection_subscription = self.create_subscription(
            String,
            'detection_results',
            self.detection_callback,
            10)
        
        self.bridge = CvBridge()
        self.current_image = None
        self.detections = []

    def image_callback(self, msg):
        self.get_logger().info('Received an image')
        try:
            self.current_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            self.draw_detections()
        except CvBridgeError as e:
            self.get_logger().error(f'CvBridge Error: {e}')

    def detection_callback(self, msg):
        self.get_logger().info('Received detection results')
        self.detections = json.loads(msg.data)
        self.draw_detections()

    def draw_detections(self):
        if self.current_image is None:
            return
        
        image = self.current_image.copy()
        
        for detection in self.detections:
            box = detection['box']
            x1, y1, x2, y2 = map(int, box)
            confidence = detection['confidence']
            class_name = detection['class_name']
            
            label = f"{class_name}: {confidence:.2f}"
            cv2.rectangle(image, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(image, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        
        cv2.imshow('Detections', image)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = DisplayNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
