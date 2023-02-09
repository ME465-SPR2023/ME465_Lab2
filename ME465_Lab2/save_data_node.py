import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import numpy as np
from scipy.io import savemat


class SaveDataNode(Node):
    def __init__(self):
        super().__init__("save_data_node")
        self.create_subscription(
            Float32MultiArray,
            '/pdf',
            self.data_callback,
            5,
        )
        self.pdf = None
        self.x = None
        
    def data_callback(self, msg):
        data = np.array(msg.data).reshape(2, -1)
        if self.x is None:
            self.x = data[0, :]
            self.pdf = data[1, :]
        else:
            self.pdf = np.vstack((self.pdf, data[1, :]))
            
    def save(self):
        savemat("lab2.mat", {"pdf": self.pdf, "x": self.x})


def main(args=None):
    rclpy.init(args=args)
    node = SaveDataNode()
    try:
        rclpy.spin(node)
    finally:
        node.save()


if __name__ == '__main__':
    main()
