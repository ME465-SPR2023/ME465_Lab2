import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, Float32MultiArray
import numpy as np
from scipy.signal import fftconvolve


class Lab2(Node):
    """The node for lab 2 implementing Markov Localization,
    
    This class is the starting point for lab 2. So far it has multiple supporting
    functions and attributes included to help with Markov Localization.
    
    Attributes:
        dt (float): The timestep used for localization in seconds.
        pdf (numpy.ndarray): The probability density function.
        x (numpy.ndarray): The locations for each element of the PDF.
        detected (bool): Represents whether or not the fiducial is currently detected.
        vel (float): The current linear velocity of the robot.
        map (list of float): The locations of the fiducials.
    """
    
    dt = 0.05
    
    def __init__(self):
        super().__init__("lab2_node")
        
        self.create_timer(self.dt, self.timer_callback)
        self.vel_publisher = self.create_publisher(
            Twist,
            '/ctrl_vel',
            5,
        )
        self.pdf_publisher = self.create_publisher(
            Float32MultiArray,
            '/pdf',
            5,
        )
        self.create_subscription(
            Twist,
            '/commands/velocity',
            self.vel_callback,
            5,
        )
        self.create_subscription(
            Bool,
            '/detected',
            self.detected_callback,
            5,
        )
        self.declare_parameter("map")
        self.map = self.get_parameter("map").value
        assert self.map is not None
        self.detected = False
        self.vel = 0
        self.x = np.linspace(-5, 5, 801)
        self.pdf = np.ones_like(self.x) / self.x.size
        
    def vel_callback(self, msg):
        self.vel = msg.linear.x
        
    def detected_callback(self, msg):
        self.detected = msg.data
        
    def gaussian(self, x, mean, std):
        return np.exp(-(x - mean)**2 / (2 * std**2)) / (std * np.sqrt(2 * np.pi))
        
    def detected_pdf(self):
        pdf = np.zeros_like(self.pdf)
        for loc in self.map:
            pdf += self.gaussian(self.x, loc, 0.2)
        return pdf / pdf.sum()
        
    def publish_pdf(self):
        """Publishes current probability density function for plotting."""
        msg = Float32MultiArray()
        msg.data = np.vstack((self.x, self.pdf)).flatten().tolist()
        self.pdf_publisher.publish(msg)
        
    def timer_callback(self):
        msg = Twist()
        msg.linear.x = 0.2
        self.vel_publisher.publish(msg)
        dx = self.vel * self.dt
        self.pdf = np.convolve(self.gaussian(self.x, 2 * dx, 0.02), self.pdf, mode="same")
        if self.detected:
            self.pdf *= self.detected_pdf()
        self.pdf /= self.pdf.sum()
        self.publish_pdf()


def main(args=None):
    rclpy.init(args=args)
    node = Lab2()
    rclpy.spin(node)


if __name__ == '__main__':
    main()
