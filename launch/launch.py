from launch import LaunchDescription 
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler
from launch.substitutions import LaunchConfiguration, Command
from special_implementation.disc_robot import load_disc_robot, load_world
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
import sys

def generate_launch_description():
    """
    Generates a LaunchDescription for launching the robot simulation.

    Returns:
        LaunchDescription: The launch description for the robot simulation.
    """

    # Declare launch arguments
    arg1 = DeclareLaunchArgument(name='robot_name', default_value='normal.robot', description='Name of the robot')
    robot_name = [arg for arg in sys.argv if arg.startswith("robot_name:=")][0].split(':=')[1]

    # Load robot description from disc_robot module
    robot_path = f"/home/jasdeep/ros2_ws/src/special_implementation/robot/{robot_name}" 
    robot = load_disc_robot(robot_path)
    
    urdf = robot['urdf']
    robot_description = ParameterValue(urdf, value_type=str)

    # Extract wheel-related parameters from the robot configuration
    radius = robot['body']['radius']
    wheel_distance = robot['wheels']['distance']
    error_variance_left = robot['wheels']['error_variance_left']
    error_variance_right = robot['wheels']['error_variance_right']
    error_update_rate = robot['wheels']['error_update_rate']

    #Extract laser related paramters from the robot file 
    laser_rate = robot['laser']['rate']
    laser_count = robot['laser']['count']
    angle_min = robot['laser']['angle_min']
    angle_max = robot['laser']['angle_max']
    range_min = robot['laser']['range_min']
    range_max = robot['laser']['range_max']
    laser_error_variance = robot['laser']['error_variance']
    laser_fail_probability = robot['laser']['fail_probability']
    laser_publish_rate = 20

    arg4 = DeclareLaunchArgument(name='world_name', default_value='brick.world', description='Name of the world')
    world_name = [arg for arg in sys.argv if arg.startswith("world_name:=")][0].split(':=')[1]

    world_path = f"/home/jasdeep/ros2_ws/src/special_implementation/world/{world_name}"
    world = load_world(world_path)

    #Extract World Related Parameters
    resolution = world['resolution']
    map_data = world['map']
    initial_pose = world['initial_pose']

    # Declare launch argument for RViz configuration file
    default_rviz_config_path = "/home/jasdeep/ros2_ws/src/special_implementation/config/saved_rviz_config.rviz" 
    arg2 = DeclareLaunchArgument(name='rviz_config', default_value=default_rviz_config_path, description='Path to the RViz configuration file')


    # Define VelocityTranslator node with wheel_distance parameter
    velcoity_translator_node = Node(package='special_implementation', executable='velocity_translator', parameters=[{'wheel_distance': wheel_distance}])

    # Define Simulator node with wheel-related parameters
    simulator_node = Node(package='special_implementation', executable='simulator', parameters=[{'wheel_distance': wheel_distance},
                                                                                   {'error_variance_left': error_variance_left},
                                                                                   {'error_variance_right': error_variance_right},
                                                                                   {'error_update_rate': error_update_rate},
                                                                                   {'radius': radius},
                                                                                   {'resolution': resolution},
                                                                                   {'map_data': map_data},
                                                                                   {'initial_pose': initial_pose},
                                                                                   {'laser_rate': laser_rate},
                                                                                   {'laser_count': laser_count},
                                                                                   {'angle_min': angle_min},
                                                                                   {'angle_max': angle_max},
                                                                                   {'range_min': range_min},
                                                                                   {'range_max': range_max},
                                                                                   {'laser_error_variance':laser_error_variance},
                                                                                   {'laser_fail_probability':laser_fail_probability},
                                                                                   {'laser_publish_rate':laser_publish_rate}])
    
    controller_node = Node(package='special_implementation', executable='prm_controller', parameters=[{'world_path': world_path}
                                                                                                      ])

    # Define Robot State Publisher node with robot_description parameter
    robot_publisher_node = Node(package="robot_state_publisher", executable="robot_state_publisher", parameters=[{'robot_description': robot_description}])

    # Define RViz node with the specified RViz configuration file
    rviz_node = Node(package="rviz2", executable="rviz2", arguments=['-d', LaunchConfiguration('rviz_config')])

    

    #Register an event handler to terminate the launch on process exit
    # event_handler = OnProcessExit(target_action=ep1, on_exit=[EmitEvent(event=Shutdown())])
    # terminate_at_end = RegisterEventHandler(event_handler)

    # Define the LaunchDescription
    ld = LaunchDescription([arg1,
                            arg2,
                            velcoity_translator_node,
                            simulator_node,
                            robot_publisher_node,
                            rviz_node, 
                            arg4,
                            controller_node,
                            ])

    return ld  

#ros2 launch special_implementation robot_name:=ideal.robot world_name:=bricks.world
