import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge
import cv2
import csv
import os
from datetime import datetime

class ImageOdomSaver(Node):
    def __init__(self):
        super().__init__('image_odom_saver')

        # 설정
        self.image_sub = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.bridge = CvBridge()

        now = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.image_dir = f'images_{now}'
        os.makedirs(self.image_dir, exist_ok=True)
        self.csv_file = open(f'odom_{now}.csv', 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(['timestamp', 'x', 'y', 'z', 'qx', 'qy', 'qz', 'qw'])

        self.image_count = 0

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            image_path = os.path.join(self.image_dir, f'image_{self.image_count:04d}.jpg')
            cv2.imwrite(image_path, cv_image)
            self.get_logger().info(f'Saved {image_path}')
            self.image_count += 1
        except Exception as e:
            self.get_logger().error(f'Image conversion failed: {e}')

    def odom_callback(self, msg):
        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation
        timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.csv_writer.writerow([timestamp, pos.x, pos.y, pos.z, ori.x, ori.y, ori.z, ori.w])
        self.get_logger().info(f'Odom: x={pos.x:.2f}, y={pos.y:.2f}')

    def destroy_node(self):
        self.csv_file.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = ImageOdomSaver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down...')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
