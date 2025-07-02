import rclpy as rp
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os

class ImageLogger(Node):
    def __init__(self):
        super().__init__('image_logger')
        self.bridge = CvBridge() # ROS 이미지 메시지 <-> OpenCV 간 변환해주는 라이브러리
        self.subscription = self.create_subscription(Image, '/camera/image_raw', self.listener_callback, 10)


        self.output_dir = os.path.expanduser('~/study1_logs/images')
        os.makedirs(self.output_dir, exist_ok=True) # mkdir -> 한 단계 디렉토리만 생성 , makedirs -> 중첩된 경로도 전부 생성
        self.count = 0

    def listener_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        file_name = os.path.join(self.output_dir, f'image_{self.count:05d}.jpg')
        cv2.imwrite(file_name, cv_image)
        self.get_logger().info(f'Saved {file_name}') # ROS2 python 노드에서 로그 출력할 때 쓰는 공식 방법
        self.count += 1

def main(args=None):
    rp.init(args=args)
    image_logger = ImageLogger()
    rp.spin(image_logger)
    image_logger.destroy_node()
    rp.shutdown()

    
if __name__ == "__main__":
    main()