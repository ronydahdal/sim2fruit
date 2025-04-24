import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

import random

# this will the policy taht sends joint information and commands
class DummyPolicyNode(Node):
    def __init__(self):
        super().__init__('dummy_policy_node')
        self.subscription = self.create_subscription(
            String,
            '/pointnet/predictions',
            self.callback,
            10
        )
        self.publisher = self.create_publisher(
            JointTrajectory,
            '/mycobot/joint_trajectory_controller/joint_trajectory',
            10
        )
        self.joint_names = [
            'joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6'
        ]

    # this function will send randomized joint positions from -0.5 to 5 
    def callback(self, msg):
        self.get_logger().info(f"Received prediction: {msg.data}")

        jt = JointTrajectory()
        jt.joint_names = self.joint_names
        point = JointTrajectoryPoint()
        point.positions = [random.uniform(-0.5, 0.5) for _ in self.joint_names]
        point.time_from_start.sec = 1
        jt.points.append(point)

        self.publisher.publish(jt)
        self.get_logger().info("dummy action to joint controller")

# initialize node and run randomized 'policy'
def main(args=None):
    rclpy.init(args=args)
    node = DummyPolicyNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
