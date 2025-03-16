import rclpy
from rclpy.node import Node

import numpy as np
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive

class ReactiveFollowGap(Node):
    """
    Implement Wall Following on the car
    This is a template implementation of the Follow Gap algorithm.
    """
    def __init__(self):
        super().__init__('reactive_node')
        
        # Topics & Subs, Pubs
        lidarscan_topic = '/scan'
        drive_topic = '/drive'
        # Subscribe to LIDAR
        self.lidar_sub = self.create_subscription(LaserScan, lidarscan_topic, self.lidar_callback, 1)
        # Publish to drive
        self.pub_drive = self.create_publisher(AckermannDriveStamped, drive_topic, 1)
        
        self.bubble_radius = 16  # Safety bubble radius
        self.preprocess_window_size = 3 # Convolution window size for preprocessing
        self.best_point_window_size = 80  # Convolution window size for find_best_point
        self.max_distance = 3.0  # Distance for rejecting high values
        self.straight_speed = 1.0 # Speed when going straight
        self.corner_speed = 0.5 # Speed when turning
        self.straight_steering_angle = np.pi / 18 
        self.angle_viewport = 90  # Only the front and sides of the car
        self.angles = None # Store lidar angles
        self.angle_index = None # Store angle indices

    def preprocess_lidar(self, scan_data):
        """ 
        Preprocess the LiDAR scan array. Expert implementation includes:
        1.Setting each value to the mean over some window
        2.Rejecting high values (eg. > 3m)
        """
        # Convert to np array
        proc_ranges = np.array(scan_data.ranges)

        # Initialize angles array
        if self.angles is None:
            min_angle = scan_data.angle_min
            max_angle = scan_data.angle_max
            angle_increment = scan_data.angle_increment
            num_ranges = len(scan_data.ranges)

            # Generate angles array using angle_min and angle_increment
            self.angles = min_angle + np.arange(num_ranges) * angle_increment
            # Get indices where angles are within the viewport
            self.angle_index = np.where(
                np.logical_and(
                    self.angles > np.radians(-self.angle_viewport),
                    self.angles < np.radians(self.angle_viewport)
                )
            )
            # Update the angles array to only include angles in the viewport
            self.angles = self.angles[self.angle_index]
            
        # Replace invalid values
        proc_ranges = np.where(np.isnan(proc_ranges), 0, proc_ranges) 
        proc_ranges = np.where(np.isinf(proc_ranges), self.max_distance, proc_ranges)
        # Apply convolution to smooth the range data
        kernel = np.ones(self.preprocess_window_size) / self.preprocess_window_size
        proc_ranges = np.convolve(proc_ranges, kernel, mode='same')
        # Reject high values
        proc_ranges = np.clip(proc_ranges, 0, self.max_distance)

        # Filter ranges to include only those within our viewport
        proc_ranges = proc_ranges[self.angle_index]

        return proc_ranges

    def find_max_gap(self, free_space_ranges):
        """ 
        Return the start index & end index of the max gap in free_space_ranges 
        """
        # Mask ranges with 0 values
        masked = np.ma.masked_where(free_space_ranges == 0, free_space_ranges)
        # Get slices of non-masked values which are consecutive non-zeros
        slices = np.ma.notmasked_contiguous(masked)
        # Find the longest slice, (maximum length sequence of consecutive non-zeros)
        max_len = slices[0].stop - slices[0].start 
        chosen_slice = slices[0]
        for slice in slices[1:]:
            slice_len = slice.stop - slice.start
            if slice_len > max_len:
                max_len = slice_len
                chosen_slice = slice
        # Return start and end indices of the longest slice
        return chosen_slice.start, chosen_slice.stop

    def find_best_point(self, start_i, end_i, ranges):
        """ Start_i & end_i are start and end indices of the max-gap range, respectively
        Return index of best point in ranges
            Naive: Choose the furthest point within ranges and go there
        """
        # Subset arrays for the ranges and angles in the gap
        gap_ranges = ranges[start_i:end_i]
        gap_angles = self.angles[start_i:end_i]

        # Apply convolution to smooth the ranges in the gap
        kernel = np.ones(self.best_point_window_size) / self.best_point_window_size
        smoothed_gap_ranges = np.convolve(gap_ranges, kernel, mode='same')
        """
            This is an attempt to help avoid clipping corners and obstacles
            by penalizing indeces in the gap_ranges array that have a high angle
            an index with a high angle should be a corner or an obstacle.
            Penalizing these indeces make them unlikely to be chosen as the best point
        """
        smoothed_gap_ranges = smoothed_gap_ranges[:len(gap_angles)]
        # Penalty size based on the angle
        penalties = np.abs(gap_angles)
        scores = smoothed_gap_ranges - penalties
        # Best point based on the highest 'score'
        best_point_index = np.argmax(scores)  
        
        # Return index of best point mapped to the original ranges array
        return best_point_index + start_i
        # return np.argmax(smoothed_gap_ranges) + start_i

    def get_steering_angle(self, best_point_index):
        """ Get the steering angle for the best point index """
        return self.angles[best_point_index]

    def lidar_callback(self, data):
        """ Process each LiDAR scan as per the Follow Gap algorithm & publish an AckermannDriveStamped Message
        """
        proc_ranges = self.preprocess_lidar(data)
        
        # TODO:
        # Find closest point to LiDAR
        closest_point = proc_ranges.argmin()
        
        # Eliminate all points inside 'bubble' (set them to zero)
        safety_radius = int(self.bubble_radius / 0.1) 
        min_index = max(0, closest_point - safety_radius)
        max_index = min(len(proc_ranges) - 1, closest_point + safety_radius + 1)
        proc_ranges[min_index:max_index] = 0

        # Find max length gap
        gap_start, gap_end = self.find_max_gap(proc_ranges)
        print(f"Gap Start: {gap_start}, Gap End: {gap_end}")
        
        # Find the best point in the gap
        best_point = self.find_best_point(gap_start, gap_end, proc_ranges)
        print(f"Best Point: {best_point}")
        
        # Get steering angle
        steering_angle = self.get_steering_angle(best_point)
        
        # Set speed based on steering angle
        if abs(steering_angle) > self.straight_steering_angle:
            speed = self.corner_speed
        else:
            speed = self.straight_speed

        # Publish Drive message
        drive_msg = AckermannDriveStamped()
        drive_msg.drive.steering_angle_velocity = 0.0
        drive_msg.drive.steering_angle = steering_angle
        drive_msg.drive.speed = speed
        self.pub_drive.publish(drive_msg)
        print(f"Steering Angle: {steering_angle}")
        
def main(args=None):
    rclpy.init(args=args)
    print("Reactive Follow Gap Initialized")
    reactive_node = ReactiveFollowGap()
    rclpy.spin(reactive_node)

    reactive_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()