import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64

class VelocityTranslator(Node):
    """
    Node for translating linear and angular velocities to left and right wheel velocities.
    """
    def __init__(self):
        """
        Initializes the VelocityTranslator node.
        """
        super().__init__('velocity_translator')

        # Get the wheel_distance parameter from the parameter server
        self.declare_parameter('wheel_distance')
        self.L = self.get_parameter('wheel_distance').value  # Wheel distance between the left and right wheels
           
        # Create a subscriber for the cmd_vel topic
        self.subscription = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        
        # Create publishers for left and right wheel velocities
        self.left_publisher = self.create_publisher(Float64, '/vl', 10)
        self.right_publisher = self.create_publisher(Float64, '/vr', 10)

    def cmd_vel_callback(self, msg: Twist):
        """
        Callback function for /cmd_vel topic subscriber.

        Args:
            msg (Twist): The Twist message containing linear and angular velocities.
        """
        vel = msg.linear.x
        omega = msg.angular.z

        # Calculate left and right wheel velocities
        vel_l, vel_r = self.get_wheel_velocities(vel, omega)

        # self.get_logger().info(f"Left Wheel Velocity: {vel_l.data}")
        # self.get_logger().info(f"Right Wheel Velocity: {vel_r.data}")

        # Publish left and right wheel velocities
        self.left_publisher.publish(vel_l)
        self.right_publisher.publish(vel_r)

    def get_wheel_velocities(self, vel, omega):
        """
        Calculates left and right wheel velocities based on linear and angular velocities.

        Args:
            vel (float): Linear velocity.
            omega (float): Angular velocity.

        Returns:
            Float64: Left wheel velocity.
            Float64: Right wheel velocity.
        """
        vel_l = (vel - omega * self.L / 2)
        vel_r = (vel + omega * self.L / 2)

        # Create Float64 messages for left and right wheel velocities
        vel_l = Float64(data=vel_l)
        vel_r = Float64(data=vel_r)

        return vel_l, vel_r


def main(args=None):
    """
    Main function to run the VelocityTranslator node.

    Args:
        args: Command-line arguments (not used in this example).
    """
    rclpy.init(args=args)
    node = VelocityTranslator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
