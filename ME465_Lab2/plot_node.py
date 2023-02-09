import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import numpy as np
import matplotlib.pyplot as plt


class PlotNode(Node):
    def __init__(self):
        super().__init__("plot_node")
        self.create_subscription(
            Float32MultiArray,
            '/pdf',
            self.data_callback,
            5,
        )
        plt.figure()
        self.line = plt.plot([], [])[0]
        plt.show(block=False)
        
    def data_callback(self, msg):
        data = np.array(msg.data).reshape(2, -1)
        self.line.set_xdata(data[0, :])
        self.line.set_ydata(data[1, :])
        plt.xlim(data[0, :].min(), data[0, :].max())
        # plt.ylim(0, 1.1 * data[1, :].max())
        plt.ylim(0, 0.1)


def main(args=None):
    rclpy.init(args=args)
    node = PlotNode()
    while True:
        rclpy.spin_once(node)
        plt.pause(0.01)


if __name__ == '__main__':
    main()
