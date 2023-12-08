import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped, Pose
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from math import sin, cos, pi
import numpy as np
import random
import math

class Simulator(Node):
    """
    A ROS2 node for simulating the motion of a differential drive robot.
    """

    def __init__(self):
        """
        Initializes the Simulator node.
        """

        super().__init__('simulator')

        # Declare parameters for robot
        self.declare_parameter('radius')
        self.declare_parameter('wheel_distance')
        self.declare_parameter('error_variance_left')
        self.declare_parameter('error_variance_right')
        self.declare_parameter('error_update_rate')
        
        #Declare parameters for world
        self.declare_parameter('resolution')
        self.declare_parameter('map_data')
        self.declare_parameter('initial_pose')

        #Declare parameters for laser
        self.declare_parameter('laser_rate')
        self.declare_parameter('laser_count')
        self.declare_parameter('angle_min')
        self.declare_parameter('angle_max')
        self.declare_parameter('range_min')
        self.declare_parameter('range_max')
        self.declare_parameter('laser_error_variance')
        self.declare_parameter('laser_fail_probability')
        self.declare_parameter('laser_publish_rate')

        # Get parameters from the parameter server for robot
        self.radius = self.get_parameter('radius').value 
        self.L = self.get_parameter('wheel_distance').value 
        self.error_variance_left = self.get_parameter('error_variance_left').value 
        self.error_variance_right = self.get_parameter('error_variance_right').value 
        self.error_update_rate = self.get_parameter('error_update_rate').value 
        
        #Get Parameters for world file
        self.map_data = self.get_parameter('map_data').value 
        self.resolution = self.get_parameter('resolution').value 
        self.initial_pose = self.get_parameter('initial_pose').value 

        #Get Parameters for laser
        self.laser_rate = self.get_parameter('laser_rate').value
        self.laser_count = self.get_parameter('laser_count').value
        self.range_min = self.get_parameter('range_min').value
        self.range_max = self.get_parameter('range_max').value
        self.angle_min = self.get_parameter('angle_min').value
        self.angle_max = self.get_parameter('angle_max').value
        self.laser_error_variance = self.get_parameter('laser_error_variance').value
        self.laser_fail_probability = self.get_parameter('laser_fail_probability').value
        self.laser_publish_rate = self.get_parameter('laser_publish_rate').value

        # self.get_logger().info(f'{self.resolution = }')
        # self.get_logger().info(f'{self.map_data = }')
        # Initialize robot state and timers
        self.init_robot_state()

        #publisher for ocuupancy grid
        self.map_publisher = self.create_publisher(OccupancyGrid, '/map', 10)
        
        self.map_msg = self.get_grid()

        # Created a timer to periodically publish the map (every two seconds)
        self.map_timer = self.create_timer(2.0, self.publish_map)

        self.laser_scan_publisher = self.create_publisher(LaserScan, '/scan', 10)

        self.laser_scan_timer = self.create_timer(1.0 / self.laser_publish_rate, self.publish_laser_scan)

        # Create subscribers for left and right wheel velocities
        self.vl_subscriber = self.create_subscription(Float64, '/vl', self.vl_callback, 10)
        self.vr_subscriber = self.create_subscription(Float64, '/vr', self.vr_callback, 10)

        # Create a TransformBroadcaster for publishing TF transforms
        self.tf_broadcaster = TransformBroadcaster(self)

        # Create timers for updating pose and error
        self.pose_update_timer = self.create_timer(0.1, self.update_pose)
        self.error_update_timer = self.create_timer(self.error_update_rate, self.update_error)

        self.current_pos_publisher = self.create_publisher(Pose, '/current_pos', 10)
        self.create_timer(0.1, self.publish_current_position)

    def publish_current_position(self):
        """
        Publishes the current position of the robot.
        """
        pose_msg = Pose()
        pose_msg.position.x = self.x
        pose_msg.position.y = self.y
        pose_msg.position.z = 0.0
        
        q = quaternion_from_euler(0, 0, self.theta)
        pose_msg.orientation.x = q[0]
        pose_msg.orientation.y = q[1]
        pose_msg.orientation.z = q[2]
        pose_msg.orientation.w = q[3]

        self.current_pos_publisher.publish(pose_msg)

    def ray_cast(self, angle):

        if random.random() < self.laser_fail_probability:
            return float('nan')  # Return NaN for a failed measurement
        # Start at the robot's location
        x = self.x
        y = self.y

        # Calculate the ray's direction in the world frame
        world_angle = self.theta + angle
        dx = math.cos(world_angle)
        dy = math.sin(world_angle)

        # Step size for the ray increments, depending on the map resolution
        step_size = self.map_msg.info.resolution / 2.0
        max_distance = self.range_max
        distance = 0

        # Incrementally step along the ray
        while distance < max_distance:
            # Check the occupancy grid cell at the current ray position
            map_x = int((x - self.map_msg.info.origin.position.x) / self.map_msg.info.resolution)
            map_y = int((y - self.map_msg.info.origin.position.y) / self.map_msg.info.resolution)

            # Check if out of bounds
            if map_x < 0 or map_y < 0 or map_x >= self.map_msg.info.width or map_y >= self.map_msg.info.height:
                return max_distance  # Return max range if out of bounds

            # Calculate the index in the occupancy grid data array
            index = map_y * self.map_msg.info.width + map_x

            # Check if the occupancy grid cell is occupied
            if self.map_msg.data[index] == 100:
                # Add some noise to the measurement
                noise = np.random.normal(0, math.sqrt(self.laser_error_variance))
                measured_distance = distance + noise
                return min(max(measured_distance, self.range_min), max_distance)

            # Move to the next position along the ray
            x += dx * step_size
            y += dy * step_size
            distance += step_size

        # If no obstacle was hit, return the max range
        return max_distance
    
    
    def publish_laser_scan(self):
        scan = LaserScan()
        scan.header.stamp = self.get_clock().now().to_msg()
        scan.header.frame_id = 'laser'
        scan.angle_min = self.angle_min
        scan.angle_max = self.angle_max
        scan.angle_increment = (scan.angle_max - scan.angle_min) / self.laser_count
        scan.time_increment = 0.0
        scan.range_min = self.range_min
        scan.range_max = self.range_max

        scan.ranges = [self.ray_cast(angle) for angle in np.linspace(scan.angle_min, scan.angle_max, self.laser_count)]

        self.laser_scan_publisher.publish(scan) 


    def publish_map(self):
        # This function will be called periodically by the timer
        # self.get_logger().info(f'MAP IS BEING PUBLISHED')
        self.map_publisher.publish(self.map_msg)

    def get_grid(self):

        lines = self.map_data.strip().split('\n')
        height = len(lines)
        width = len(lines[0])

        grid = grid = np.zeros((height, width), dtype=np.int8)

        for y, line in enumerate(reversed(lines)):
            for x, char in enumerate(line):
                if char == '#':  # '#' represents an obstacle
                    grid[y, x] = 100  # Occupied cell
                elif char == '.':  # '.' represents free space
                    grid[y, x] = 0  # Free cell


        
        
        # Create the OccupancyGrid message
        occupancy_grid = OccupancyGrid()
        occupancy_grid.info.width = width
        occupancy_grid.info.height = height
        occupancy_grid.info.resolution = self.resolution
        occupancy_grid.info.origin.position.x = 0.0
        occupancy_grid.info.origin.position.y = 0.0
        occupancy_grid.info.origin.position.z = 0.0
        occupancy_grid.info.origin.orientation.x = 0.0
        occupancy_grid.info.origin.orientation.y = 0.0
        occupancy_grid.info.origin.orientation.z = 0.0
        occupancy_grid.info.origin.orientation.w = 1.0
        occupancy_grid.header.frame_id = 'world'
        
        occupancy_grid.data = grid.flatten().tolist()

        return occupancy_grid

    def update_error(self):
        """
        Update left and right wheel errors based on Gaussian noise.
        """
        now = self.get_clock().now()
        dt = (now - self.last_error_update_time).nanoseconds /1e9

        # self.get_logger().info(f'{dt = }')

        # Update errors using Gaussian noise
        self.error_left = np.random.normal(1.0, np.sqrt(self.error_variance_left))
        self.error_right = np.random.normal(1.0, np.sqrt(self.error_variance_right))

        self.last_error_update_time = self.get_clock().now()

    

    def vl_callback(self, vl: Float64):
        """
        Callback for left wheel velocity subscriber.
        """
        # self.get_logger().info(f'{vl.data = }')

        # Update left wheel velocity and introduce error
        self.vl = vl.data * self.error_left
        # self.get_logger().info(f'{self.vl = }')
        # self.get_logger().info(f'{self.error_left = }')
        self.last_update_time = self.get_clock().now()

    def vr_callback(self, vr: Float64):
        """
        Callback for right wheel velocity subscriber.
        """
        # self.get_logger().info(f'{vr.data = }')

        # Update right wheel velocity and introduce error
        self.vr = vr.data * self.error_right
        # self.get_logger().info(f'{self.vr = }')
        # self.get_logger().info(f'{self.error_right = }')
        self.last_update_time = self.get_clock().now()

    def init_robot_state(self):
        """
        Initialize the state variables for the robot.
        """
        self.x = self.initial_pose[0]
        self.y = self.initial_pose[1]
        self.theta = self.initial_pose[2]
        
        self.vl = 0.0
        self.vr = 0.0
        self.last_update_time = self.get_clock().now()
        
        self.error_left = 1.0
        self.error_right = 1.0
        self.last_error_update_time = self.get_clock().now()

    def check_collision(self, next_x, next_y):
        map_resolution = self.map_msg.info.resolution
        map_origin = self.map_msg.info.origin.position

        # Calculate the bounds of the robot considering its radius
        bounds = [
            (next_x + math.cos(angle) * self.radius, next_y + math.sin(angle) * self.radius)
            for angle in np.linspace(0, 2 * math.pi, num=60)  # Check around the circumference
        ]

        for bx, by in bounds:
            # Convert each bound point to map coordinates
            map_x = int((bx - map_origin.x) / map_resolution)
            map_y = int((by - map_origin.y) / map_resolution)

            # Check if the cell is within map bounds
            if (map_x >= self.map_msg.info.width or map_x < 0 or
                map_y >= self.map_msg.info.height or map_y < 0):
                return True  # Collision with boundary

            # Check if the cell is occupied
            index = map_y * self.map_msg.info.width + map_x
            if index < len(self.map_msg.data) and self.map_msg.data[index] == 100:
                return True  # Collision with obstacle

        return False  # No collision

        
    def update_pose(self):
        """
        Update the robot's pose based on wheel velocities.
        """
        now = self.get_clock().now()
        dt = (now - self.last_update_time).nanoseconds / 1e9

        if dt > 1.0:
            self.vl = 0.0
            self.vr = 0.0

    
        v = (self.vr + self.vl)/2
        w = (self.vr - self.vl)/self.L

        delta_x =  v * cos(self.theta) * dt 
        delta_y =  v * sin(self.theta) * dt

        next_x = self.x + delta_x
        next_y = self.y + delta_y

        if not self.check_collision(next_x, next_y):
            self.x = next_x
            self.y = next_y
        else:
            # Stop the robot at the obstacle boundary
            self.vl = 0.0
            self.vr = 0.0 
        
        self.theta += w *dt
    

        if self.theta > pi:
            self.theta -= 2*pi
        elif self.theta < -pi:
            self.theta += 2*pi

        self.broadcast_tf() 

    def broadcast_tf(self):
        """
        Broadcast the current robot pose as a TF transform.
        """
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'world'
        t.child_frame_id = 'base_link'
        
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        
        q = quaternion_from_euler(0, 0, self.theta)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        
        self.tf_broadcaster.sendTransform(t)


def quaternion_from_euler(ai, aj, ak):
    """
    Convert Euler angles to quaternion representation.
    """
    ai /= 2.0
    aj /= 2.0
    ak /= 2.0
    ci = cos(ai)
    si = sin(ai)
    cj = cos(aj)
    sj = sin(aj)
    ck = cos(ak)
    sk = sin(ak)
    cc = ci*ck
    cs = ci*sk
    sc = si*ck
    ss = si*sk

    q = np.empty((4, ))
    q[0] = cj*sc - sj*cs
    q[1] = cj*ss + sj*cc
    q[2] = cj*cs - sj*sc
    q[3] = cj*cc + sj*ss

    return q


def main(args = None):
    """
    Main function to initialize the ROS2 node and spin it.
    """
    rclpy.init(args = args)
    node = Simulator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
