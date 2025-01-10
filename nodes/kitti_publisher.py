import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge
import cv2
import numpy as np

DATA_PATH = "/home/kaj/ws/data/kitti/dataset/sequences/00"  # Update path
POSES_PATH = "/home/kaj/ws/data/kitti/dataset/poses/00.txt"  # Update path


class KITTIPublisher(Node):
    def __init__(self, data_path, poses_path):
        super().__init__('kitti_publisher')
        self.image_pub0 = self.create_publisher(Image, '/cam0/image_raw', 10)
        self.image_pub1 = self.create_publisher(Image, '/cam1/image_raw', 10)
        self.odom_pub = self.create_publisher(Odometry, '/ground_truth/odom', 10)
        
        self.bridge = CvBridge()
        self.data_path = data_path
        self.poses = self.load_poses(poses_path)
        self.frame_id = 0

        self.timer = self.create_timer(0.1, self.publish_data)  # Publish at 10Hz

    def load_poses(self, poses_path):
        with open(poses_path, 'r') as f:
            lines = f.readlines()
        poses = []
        for line in lines:
            pose = np.array([float(x) for x in line.strip().split()]).reshape(3, 4)
            poses.append(pose)
        return poses

    def publish_data(self):
        image_path0 = f"{self.data_path}/image_0/{self.frame_id:06d}.png"
        image_path1 = f"{self.data_path}/image_1/{self.frame_id:06d}.png"
        image0 = cv2.imread(image_path0, cv2.IMREAD_GRAYSCALE)
        image1 = cv2.imread(image_path1, cv2.IMREAD_GRAYSCALE)
        if image0 is None or image1 is None:
            self.get_logger().info("All frames published.")
            rclpy.shutdown()
            return

        # Publish image
        msg0 = self.bridge.cv2_to_imgmsg(image0, encoding="mono8")
        msg0.header.stamp = self.get_clock().now().to_msg()
        self.image_pub0.publish(msg0)
        msg1 = self.bridge.cv2_to_imgmsg(image1, encoding="mono8")
        msg1.header.stamp = self.get_clock().now().to_msg()
        self.image_pub1.publish(msg1)
        # Publish ground truth odometry
        if self.frame_id < len(self.poses):
            pose = self.poses[self.frame_id]
            odom = Odometry()
            odom.header.stamp = msg0.header.stamp
            odom.header.frame_id = 'world'
            odom.pose.pose.position.x = pose[0, 3]
            odom.pose.pose.position.y = pose[1, 3]
            odom.pose.pose.position.z = pose[2, 3]
            self.odom_pub.publish(odom)

        self.frame_id += 1

def main(args=None):
    rclpy.init(args=args)
    data_path = DATA_PATH
    poses_path = POSES_PATH
    node = KITTIPublisher(data_path, poses_path)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()