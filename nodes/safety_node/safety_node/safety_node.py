#!/usr/bin/env python3

import numpy as np

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped


class SafetyNode(Node):
    """
    ROS2 node for emergency braking.

    Subscribes to laser scan data and odometry info to detect obstacles and calculate time to collision (TTC).
    If we calculate a TTC to some object that is under the specified time before breaking (TBB), the node will publish
    a command to stop the ego car to prevent collision.

    Subscribing to:
        scan (type sensor_msgs.msg.LaserScan)
        ego_racecar/odom (type nav_msgs.msg.Odometry)

    Publishing to:
        drive (type ackermann_msgs.msg.AckermannDriveStamped)
    """

    def __init__(self):
        """
        Called at initialization of the class.

        Calls init from parent class to initialize node with name 'safety_node', and initializes the velocity,
        minimum TTC before breaking, subscriptions, and publisher members.
        """
        super().__init__('safety_node')
        # Initialize velocity of ego car
        self.velocity = 0.0
        # Minimum Time to Collision before braking
        self.tbb = 0.6
        # Create subscriptions and publisher
        # qos_profile=1 (last parameter)Use a history depth of 1, which means only the latest LaserScan message will be kept.
        self.create_subscription(LaserScan, 'scan', self.scan_callback, 1)
        self.create_subscription(Odometry, 'ego_racecar/odom', self.odom_callback, 1)
        self.pub_drive = self.create_publisher(AckermannDriveStamped, 'drive', 1)
        # Initialize the car to move forward
        ack = AckermannDriveStamped()
        ack.drive.speed = 1.4
        self.pub_drive.publish(ack)

    def odom_callback(self, odom_msg):
        """
        Called every time a new message is received from ego_racecar/odom topic.

        Extracts velocity of car in forward (positive) and backward (negative) direction.

        From odom_msg:
            odom_msg.twist.twist.linear.x (float) (varies)
        """
        # Velocity recieveed from odometry message (float)
        self.velocity = odom_msg.twist.twist.linear.x 

    def scan_callback(self, scan_msg):
        """
        Called every time a new message is received from scan topic.

        Calculates TTC between ego car and the nearby areas in the environment. When theta = 0 rad, then
        the corresponding area is directly in front of the car. LaserScan gives a total view of about 6pi/4 radians
        around the car (about 3pi/4 to one side and 3pi/4 to the other).

        First, the valid distances are extracted and the angles for each are calculated.
        Next, the range rates are calculated (the max range rate is always at np.cos(0), as that is directly in
        front of the car). Finally, the time to collision for each valid nearby area is calculated (ttcs), and
        if the smallest value in the ttcs array is less than self.tbb, then signal the ego car to stop.

        From scan_msg:
            scan_msg.ranges (array) (varies)
            scan_msg.range_min (float) = 0.0
            scan_msg.range_max (float) = 30.0
            scan_msg.angle_min (float) ~= -2.35 ~= -3pi/4
            scan_msg.angle_max (float) ~= 2.35 ~= 3pi/4
            scan_msg.angle_increment (float) ~= 0.00435

        range_rate_min: The minimum value any range_rate can be to calculate ttcs. Removes problems due to dividing
                        by numbers very close to zero.
        """
        ranges_temp = np.array(scan_msg.ranges) # Convert ranges to a NumPy array (1D array)
        range_min = scan_msg.range_min # Minimum range (float)
        range_max = scan_msg.range_max # Maximum range (float)
        angle_min = scan_msg.angle_min # Minimum angle (float)
        angle_inc = scan_msg.angle_increment # Angle increment (float)
        range_rate_min = 1e-4 # Small threshold to avoid division by zero (float)

        # Filter out invalid ranges and replace them with -1 (1D array)
        ranges_temp = np.where((ranges_temp < range_min) | (ranges_temp > range_max) | np.isnan(ranges_temp) | np.isinf(ranges_temp), -1, ranges_temp)
        # Extract valid ranges (1D array)
        ranges = ranges_temp[ranges_temp != -1]
        # Extract thetas for valid ranges (1D array)
        theta_idxs = np.arange(len(ranges_temp))[ranges_temp != -1]
        # Calculate corresponding angles (1D array)
        thetas = angle_min + angle_inc * theta_idxs
        # Calculate range rates (1D array)
        range_rates = self.velocity * np.cos(thetas)
        # Calculate Time to Collision (TTC) for valid ranges (1D array)
        ttcs = ranges[range_rates > range_rate_min] / range_rates[range_rates > range_rate_min]

        # Check for emergency braking condition
        if ttcs.size > 0 and ttcs.min() < self.tbb:
            ack = AckermannDriveStamped()
            ack.drive.speed = 0.0
            self.pub_drive.publish(ack)
            print(f'AEB Activated (due to TTC = {ttcs.min():.4f} secs)')


def main(args=None):
    """
    Called when script is ran.

    Initializes Ros2 client library, creates instance of SafetyNode, keeps the node alive,
    and when the node is no longer in use, destroys the node and shuts down the Ros2 client library.
    """
    rclpy.init(args=args)
    safety_node = SafetyNode()
    print(f'Safety Node Initialized')
    rclpy.spin(safety_node)
    safety_node.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    try:
        main()
    except rclpy.ROSInterruptException:
        pass
