from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rclpy.serialization import deserialize_message
from cv_bridge import CvBridge
import sensor_msgs.msg
import nav_msgs.msg
import cv2
import os
import csv

bag_path = '/home/kjy/my_bag'
reader = SequentialReader()
reader.open(StorageOptions(uri=bag_path, storage_id='sqlite3'),
            ConverterOptions('', ''))

bridge = CvBridge()
os.makedirs('rgb', exist_ok=True)
os.makedirs('depth', exist_ok=True)
csv_file = open('odom.csv', 'w', newline='')
csv_writer = csv.writer(csv_file)
csv_writer.writerow(['timestamp', 'x', 'y', 'vx', 'vy', 'angular_z'])

while reader.has_next():
    topic, data, t = reader.read_next()
    timestamp = f"{t // 1_000_000_000}.{str(t % 1_000_000_000).zfill(9)}"

    if topic == "/camera/image_raw":
        msg = deserialize_message(data, sensor_msgs.msg.Image)
        img = bridge.imgmsg_to_cv2(msg, 'bgr8')
        cv2.imwrite(f"rgb/{timestamp}.png", img)

    elif topic == "/camera/depth/image_raw":
        msg = deserialize_message(data, sensor_msgs.msg.Image)
        img = bridge.imgmsg_to_cv2(msg, 'passthrough')
        cv2.imwrite(f"depth/{timestamp}.png", img)

    elif topic == "/odom":
        msg = deserialize_message(data, nav_msgs.msg.Odometry)
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        wz = msg.twist.twist.angular.z
        csv_writer.writerow([timestamp, x, y, vx, vy, wz])

csv_file.close()
