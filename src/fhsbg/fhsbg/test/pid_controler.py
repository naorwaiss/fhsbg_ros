import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist  # Assuming adjusted_velocity is of type Twist
from nav_msgs.msg import Odometry  # Assuming /mavros/odometry/in is of type Odometry
from std_msgs.msg import Float64  # Assuming the control effort will be published as a Float64 message


class PIDController:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.previous_error = 0
        self.integral = 0

    def compute(self, target, actual, dt):
        error = target - actual
        self.integral += error * dt
        derivative = (error - self.previous_error) / dt if dt > 0 else 0
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        self.previous_error = error
        return output


class DualListenerNode(Node):
    def __init__(self):
        super().__init__('dual_listener_node')
        self.subscription1 = self.create_subscription(
            Twist,
            'adjusted_velocity',
            self.adjusted_velocity_callback,
            10)
        self.subscription1  # prevent unused variable warning

        self.subscription2 = self.create_subscription(
            Odometry,
            '/mavros/odometry/in',
            self.odometry_callback,
            10)
        self.subscription2  # prevent unused variable warning

        self.publisher_ = self.create_publisher(Float64, 'pid_velo', 10)

        self.adjusted_velocity_data = None
        self.odometry_data = None
        self.previous_time = None
        self.dt = None

        self.pid_controller = PIDController(kp=1.0, ki=0.1, kd=0.01)  # Adjust PID coefficients as needed

    def adjusted_velocity_callback(self, msg):
        current_time = self.get_clock().now()

        if self.previous_time is not None:
            self.dt = (current_time - self.previous_time).nanoseconds / 1e9  # Convert nanoseconds to seconds

        self.previous_time = current_time
        self.adjusted_velocity_data = msg

        self.get_logger().info(f'Adjusted Velocity: {msg}')
        self.get_logger().info(f'Delta Time (dt): {self.dt}')

        self.process_data()

    def odometry_callback(self, msg):
        self.odometry_data = msg
        self.get_logger().info(f'Odometry Data: {msg}')
        self.process_data()

    def process_data(self):
        if self.adjusted_velocity_data is not None and self.odometry_data is not None:
            target_velocity = self.adjusted_velocity_data.linear.x  # Assuming linear.x is the relevant component
            actual_velocity = self.odometry_data.twist.twist.linear.x  # Assuming linear.x is the relevant component

            control_effort = self.pid_controller.compute(target_velocity, actual_velocity, self.dt)

            self.get_logger().info(f'Control Effort: {control_effort}')

            # Publish the control effort to the 'pid_velo' topic
            msg = Float64()
            msg.data = control_effort
            self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = DualListenerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
