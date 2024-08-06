import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

class VelocityAdjuster(Node):
    def __init__(self):
        super().__init__('velocity_adjuster')
        self.subscription = self.create_subscription(
            Float32MultiArray,
            '/arucu_filter',
            self.listener_callback,
            10
        )
        self.publisher_ = self.create_publisher(Float32MultiArray, 'adjusted_velocity', 10)
        self.subscription  # prevent unused variable warning

        self.max_speed = 2

    def listener_callback(self, msg):
        x_position, y_position, z_position = msg.data

        # Adjust velocity ratios based on z_position
        x_velocity_ratio, y_velocity_ratio, z_velocity_ratio = self.adjust_velocity_ratios(z_position)

        # Calculate velocities
        x_velocity = self.calculate_velocity(x_position, x_velocity_ratio)
        y_velocity = self.calculate_velocity(y_position, y_velocity_ratio)
        z_velocity = self.calculate_velocity(z_position, z_velocity_ratio)

        # Ensure velocities do not exceed max_speed
        x_velocity, y_velocity, z_velocity = self.limit_velocities(x_velocity, y_velocity, z_velocity)

        # Publish the adjusted velocities
        adjusted_velocity = Float32MultiArray()
        adjusted_velocity.data = [-x_velocity, -y_velocity, -z_velocity]
        self.publisher_.publish(adjusted_velocity)

    def adjust_velocity_ratios(self, z_position):
        if z_position < 1:
            x_velocity_ratio = 0.25*10
            y_velocity_ratio = 0.25*10
            z_velocity_ratio = 0.25*10
        elif 1 <= z_position < 2.0:
            x_velocity_ratio = 0.25*10
            y_velocity_ratio = 0.25*10
            z_velocity_ratio = 0.25*10
        else:
            x_velocity_ratio = 0.25*10
            y_velocity_ratio = 0.25*10
            z_velocity_ratio = 0.25*10
        return x_velocity_ratio, y_velocity_ratio, z_velocity_ratio

    def calculate_velocity(self, position, velocity_ratio):
        return position * velocity_ratio

    def limit_velocities(self, x_velocity, y_velocity, z_velocity):
        max_velocity = max(abs(x_velocity), abs(y_velocity), abs(z_velocity))
        if max_velocity > self.max_speed:
            scale = self.max_speed / max_velocity
            x_velocity *= scale
            y_velocity *= scale
            z_velocity *= scale
        return x_velocity, y_velocity, z_velocity

def main(args=None):
    rclpy.init(args=args)

    velocity_adjuster = VelocityAdjuster()

    rclpy.spin(velocity_adjuster)

    velocity_adjuster.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
