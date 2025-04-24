import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import torch
import numpy as np
from std_msgs.msg import String

# many placeholders and deps in this file such as PyTorch, and pointnet_model which 
# is from the pretrained .pth file
from pointnet_model import PointNetCls  
from sensor_msgs_py import point_cloud2

# this is where we initialize our pre-trained pointnet model
class PointNetNode(Node):
    def __init__(self):
        super().__init__('pointnet_node')
        self.subscription = self.create_subscription(
            PointCloud2,
            # from simulated depth camera in gazebo -- yet to be implemented
            '/camera/depth/points',
            self.listener_callback,
            10)
        
        self.publisher = self.create_publisher(String, '/pointnet/predictions', 10)

        # example on how we'd use saved model weights from a pytorch model and load 
        self.device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
        self.model = PointNetCls(k=5) 
        self.model.load_state_dict(torch.load('/path/to/model.pth', map_location=self.device))
        self.model.eval()

    # call the model by feeding in point clouds
    def listener_callback(self, msg):
        cloud_points = list(point_cloud2.read_points(msg, skip_nans=True, field_names=("x", "y", "z")))
        cloud_array = np.array(cloud_points, dtype=np.float32)

        # incorporate random sampling as PointNet requires equal number of points for input samples
        if cloud_array.shape[0] > 1024:
            idx = np.random.choice(cloud_array.shape[0], 1024, replace=False)
            cloud_array = cloud_array[idx]

        # prepare tensor to feed into pointnet and take the highest predicted label
        cloud_tensor = torch.from_numpy(cloud_array).unsqueeze(0).transpose(2, 1).to(self.device)
        pred = self.model(cloud_tensor)
        pred_class = pred.argmax(dim=1).item()

        self.publisher.publish(String(data=f"Predicted class: {pred_class}"))

# initialize node
def main(args=None):
    rclpy.init(args=args)
    node = PointNetNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
