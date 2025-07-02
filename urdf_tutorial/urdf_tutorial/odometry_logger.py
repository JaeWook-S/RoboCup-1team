import rclpy as rp
from rclpy.node import Node
from nav_msgs.msg import Odometry
import csv
import os

class OdomLogger(Node):
    def __init__(self):
        super().__init__('odom_logger')
        self.subscription = self.create_subscription(Odometry, '/odom', self.listener_callback, 10)

        self.output_dir = os.path.expanduser('~/study1_logs/logs')
        os.makedirs(self.output_dir, exist_ok=True) # mkdir -> 한 단계 디렉토리만 생성 , makedirs -> 중첩된 경로도 전부 생성

        log_file_path = os.path.join(self.output_dir, 'odom_log.csv')
        self.file = open(log_file_path, mode='w', newline='')
        self.writer = csv.writer(self.file)
        self.writer.writerow(['time', 'x', 'y', 'theta', 'linear_x', 'angular_z'])
    
    def listener_callback(self, msg):
        t = self.get_clock().now().nanoseconds / 1e9
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        theta = msg.pose.pose.orientation.z
        linear_x = msg.twist.twist.linear.x
        angular_z = msg.twist.twist.angular.z
        self.writer.writerow([t, x, y, theta, linear_x, angular_z])
        self.file.flush()
        self.get_logger().info(f'Logged odom at time {t:.2f}')

def main(args=None):
    rp.init(args=args)
    odom_logger = OdomLogger()
    rp.spin(odom_logger)
    odom_logger.file.close()
    odom_logger.destroy_node()
    rp.shutdown()


if __name__ == "__main__":
    main()
    
