import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Vector3
from nav_msgs.msg import Odometry
import numpy as np
import os

class SLAMEvaluationNode(Node):
    def __init__(self):
        super().__init__('slam_evaluation_node')

        self.subscription_vo = self.create_subscription(
            PoseStamped,
            '/vo_pose',
            self.vo_callback,
            10
        )
        self.subscription_gt = self.create_subscription(
            Odometry,
            '/ground_truth/odom',
            self.gt_callback,
            10
        )

        self.evaluation_publisher = self.create_publisher(Vector3, '/evaluation', 10)

        self.gt_pose = None
        self.vo_pose = None
        self.errors = []

    def vo_callback(self, msg):
        self.get_logger().info("Received vo_pose")
        self.vo_pose = msg.pose.position
        self.evaluate()

    def gt_callback(self, msg):
        self.get_logger().info("Received gt_pose")
        self.gt_pose = msg.pose.pose.position

    def evaluate(self):
        if self.vo_pose is None or self.gt_pose is None:
            return

        error_x = abs(self.gt_pose.x - self.vo_pose.x)
        error_y = abs(self.gt_pose.y - self.vo_pose.y)
        error_z = abs(self.gt_pose.z - self.vo_pose.z)

        self.errors.append((error_x, error_y, error_z))

        # Publikacja błędu
        eval_msg = Vector3()
        eval_msg.x = error_x
        eval_msg.y = error_y
        eval_msg.z = error_z
        self.evaluation_publisher.publish(eval_msg)

        self.get_logger().info(f"Published evaluation: x={error_x:.4f}, y={error_y:.4f}, z={error_z:.4f}")

    def save_results(self):
        if not self.errors:
            self.get_logger().info("No data to save.")
            return

        errors_array = np.array(self.errors)
        rmse = np.sqrt(np.mean(errors_array ** 2, axis=0))
        mean_error = np.mean(errors_array, axis=0)
        std_dev = np.std(errors_array, axis=0)

        file_path = os.path.expanduser('~/ws/evaluation/evaluation.txt')
        os.makedirs(os.path.dirname(file_path), exist_ok=True)

        with open(file_path, 'w') as f:
            f.write("Evaluation Summary\n")
            f.write(f"RMSE: x={rmse[0]:.4f}, y={rmse[1]:.4f}, z={rmse[2]:.4f}\n")
            f.write(f"Mean Error: x={mean_error[0]:.4f}, y={mean_error[1]:.4f}, z={mean_error[2]:.4f}\n")
            f.write(f"Standard Deviation: x={std_dev[0]:.4f}, y={std_dev[1]:.4f}, z={std_dev[2]:.4f}\n\n")
            f.write("Frame Errors (x, y, z):\n")
            for error in self.errors:
                f.write(f"{error[0]:.4f} {error[1]:.4f} {error[2]:.4f}\n")

        self.get_logger().info(f"Results saved to {file_path}")


def main(args=None):
    rclpy.init(args=args)
    node = SLAMEvaluationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.save_results()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
