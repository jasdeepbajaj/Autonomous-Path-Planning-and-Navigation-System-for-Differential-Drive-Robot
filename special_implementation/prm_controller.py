import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped, Twist, Pose
import numpy as np
import math
from special_implementation.prm_planner import PRM_Planner
import time

KP_LINEAR = 3.7
KP_ANGULAR = 4.0

DISTANCE_TOLERANCE = 0.001
ANGLE_TOLERANCE = 0.001


def euler_from_quaternion(quaternion):
    # Convert quaternion to Euler angles
    x, y, z, w = quaternion
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2 * (w * y - z * x)
    pitch = math.asin(sinp)

    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw

class PRM_Controller(Node):
    def __init__(self):
        super().__init__('prm_controller')

        self.occupancy_grid = None
        self.start_coords = None
        
        self.idx = 0
        self.goal_coords = None
        self.current_pose = None
        self.path_coords = []

        # Get parameters from the parameter server for robot
        self.declare_parameter('world_path')
        self.world_path = self.get_parameter('world_path').value 
        
        
        self.grid_subscriber = self.create_subscription(OccupancyGrid, '/map', self.grid_callback, 10)        
        self.goal_pose_subscription = self.create_subscription(PoseStamped, '/goal_pose', self.goal_pose_callback, 10)        
        self.current_pose_subscriber = self.create_subscription(Pose, '/current_pos', self.current_pos_callback, 10)

        self.path_pub = self.create_publisher(Path, '/path', 10)
        self.cmd_vel_sub = self.create_publisher(Twist, '/cmd_vel', 10)      


    def current_pos_callback(self, msg: Pose):
        """
        Callback function for handling the current position information.

        This method processes the received current position message and updates the
        internal current_pose variable. It extracts the x and y coordinates from the
        received Pose message and sets them as the start_coords.

        Args:
            msg (Pose): Current pose message containing position information.
        """
        self.current_pose = msg

        # Extract x and y coordinates from the received Pose message
        x_curr = self.current_pose.position.x
        y_curr = self.current_pose.position.y
        
        # quater = (self.pose.orientation.x, self.pose.orientation.y, self.pose.orientation.z, self.pose.orientation.w)
        # _,_,yaw = euler_from_quaternion(quater)
        # yaw_curr = yaw

        # Set the current position coordinates as start_coords
        self.start_coords = (x_curr, y_curr)

    def grid_callback(self, occupancy_grid: OccupancyGrid):
        """
        Callback function for handling occupancy grid data.

        This method processes the received occupancy grid message and stores relevant
        information in the internal occupancy_grid variable.

        Args:
            occupancy_grid (OccupancyGrid): Occupancy grid message containing grid data.
        """

        # Extract necessary information from the received occupancy grid message
        height = occupancy_grid.info.height
        width = occupancy_grid.info.width
        resolution = occupancy_grid.info.resolution
        data = np.array(occupancy_grid.data).reshape((height, width)).tolist()

        # Store occupancy grid parameters in the internal occupancy_grid variable            
        self.occupancy_grid = {
            'height': height,
            'width': width,
            'resolution': resolution,
            'data': data
        }

    def goal_pose_callback(self, goal_pose: PoseStamped):
        """
        Callback function for processing goal pose information.

        This method receives the goal pose message, extracts goal coordinates, and initiates
        path planning using PRM_Planner based on the received goal position and the current
        robot's start position. It then triggers movement along the planned path after a delay.

        Args:
            goal_pose (PoseStamped): Goal pose message.
        """

        # Reset path coordinates and index
        self.path_coords = []
        self.idx = 0

        # Extract x and y coordinates from the received goal pose message
        goal_x = goal_pose.pose.position.x
        goal_y = goal_pose.pose.position.y
        
        # goal_quaternion = (
        #     goal_pose.pose.orientation.x,
        #     goal_pose.pose.orientation.y,
        #     goal_pose.pose.orientation.z,
        #     goal_pose.pose.orientation.w
        # )
        # _, _, goal_heading = euler_from_quaternion(goal_quaternion)
        
        # Set goal coordinates based on received goal position
        self.goal_coords = (goal_x, goal_y)

        # Retrieve resolution and height from occupancy grid
        res = self.occupancy_grid['resolution']
        height = self.occupancy_grid['height']

        # Convert start and goal coordinates to match grid resolution and height and to match the args requirement for PRM_Planner
        self.start_coords = (self.start_coords[0]/res, height - self.start_coords[1]/res)
        self.goal_coords = (self.goal_coords[0]/res, height - self.goal_coords[1]/res)

        # Initialize PRM_Planner with world path, start, goal coordinates, num_points and min_distance
        planner = PRM_Planner(self.world_path, self.start_coords, self.goal_coords, num_points= 200, min_distance=3.0)

        # Delay to allow the planner to compute the path
        # time.sleep(3)

        # Get computed path coordinates from the planner
        path = planner.path_coords
        
        #converts coordinates of path obtained from planner w.r.t 'world' frame i.e. origin at bottom left corner
        self.get_path_coords(path=path)

        # Start a timer to trigger robot movement along the planned path
        self.timer = self.create_timer(0.1, self.move_robot)

    def move_robot(self):
        """
        Move the robot along the planned path towards the goal.

        This method calculates and controls the robot's movement towards the next goal
        along the planned path. It computes the required linear and angular velocities
        based on the distance and angle error to the goal. The robot is directed towards
        the goal position by adjusting its velocity commands.

        The robot's movement continues until all goals are reached or the path is completed.

        Note:
        - The method relies on predefined constants 'KP_LINEAR', 'KP_ANGULAR',
        'DISTANCE_TOLERANCE', and 'ANGLE_TOLERANCE' for control.

        Returns:
            None
        """

        if self.idx >= len(self.path_coords):
            # Inform when all goals are reached and reset variables  
            self.get_logger().info("All goals reached")
            self.goal_coords = None
            self.idx = 0
            self.current_pose = None
            self.path_coords = []
            self.timer.cancel()
            
            return

        # Retrieve the coordinates of the next goal along the path
        goal = self.path_coords[self.idx]

        # Initialize Twist message for velocity commands    
        new_vel = Twist()

        # Calculate distance and angle to the next goal
        distance_to_goal = math.sqrt((goal[0] - self.current_pose.position.x)**2 + (goal[1] - self.current_pose.position.y)**2)
        angle_to_goal = math.atan2(goal[1] - self.current_pose.position.y, goal[0] - self.current_pose.position.x)

        
        # Calculate angle error to steer towards the goal
        quater = (self.current_pose.orientation.x, self.current_pose.orientation.y, self.current_pose.orientation.z, self.current_pose.orientation.w)
        _,_,yaw = euler_from_quaternion(quater)

        angle_error = angle_to_goal - yaw

        # Normalize angle error between -pi to pi range
        while angle_error > math.pi:
            angle_error -= 2*math.pi
        while angle_error < -math.pi:
            angle_error += 2*math.pi

        # Calculate control efforts for linear and angular velocities
        linear_control_effort = KP_LINEAR * distance_to_goal
        angular_control_effort = KP_ANGULAR * angle_error

        # Adjust velocity commands based on angle error and distance to goal
        if abs(angle_error) > ANGLE_TOLERANCE:
            new_vel.angular.z = angular_control_effort 
        else:
            if distance_to_goal > DISTANCE_TOLERANCE:
                new_vel.linear.x = linear_control_effort
                new_vel.angular.z = 0.0
            else:
                # Stop the robot when the distance to the goal is within tolerance
                new_vel.linear.x = 0.0
                self.get_logger().info(f"Goal reached: {goal}")
                self.idx += 1
        
        # Publish the velocity commands
        self.cmd_vel_sub.publish(new_vel)

    def get_path_coords(self, path):
        """
        Process and publish path coordinates as a Path message.

        This method takes a list of path coordinates and converts them to coordinates
        compatible with the occupancy grid resolution and height. It then creates a Path
        message with PoseStamped elements containing these converted coordinates and
        publishes the Path message.

        Args:
            path (list): List of path coordinates.

        Returns:
            None
        """
        # Extract resolution and height from the occupancy grid
        res = self.occupancy_grid['resolution']
        height = self.occupancy_grid['height']

        # Convert the received path coordinates w.r.t 'world' frame i.e. origin at bottom left corner
        for i in range(0, len(path)):
            x_coord = (path[i][0])*res
            y_coord = (height - path[i][1]) * res 
            self.path_coords.append((x_coord, y_coord))

        # Create a Path message and populate it with converted PoseStamped elements for path visualization
        path_msg = Path()
        path_msg.header.frame_id = 'world'  # Change the frame_id if needed

        for i in range(len(self.path_coords)):
            pose = PoseStamped()
            pose.header.frame_id = 'world'
            pose.pose.position.x = self.path_coords[i][0]
            pose.pose.position.y = self.path_coords[i][1]
            pose.pose.position.z = 0.0 

            path_msg.poses.append(pose)

        # Publish the Path message containing the converted path coordinates
        self.path_pub.publish(path_msg)
        
        

def main(args=None):
    """
    Main function to initialize the ROS2 node and spin it.
    """
    rclpy.init(args=args)
    node = PRM_Controller()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()