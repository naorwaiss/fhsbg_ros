import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

class LowPassFilter:
    def __init__(self, alpha=0.2):
        self.alpha = alpha
        self.last_value = None

    def apply(self, value):
        if self.last_value is None:
            self.last_value = value
        else:
            self.last_value = self.alpha * value + (1 - self.alpha) * self.last_value
        return self.last_value

class ArucoFilter(Node):
    def __init__(self):
        super().__init__('aruco_filter')
        self.subscription = self.create_subscription(
            Float32MultiArray,
            'aruco_data',
            self.listener_callback,
            10
        )
        self.publisher_ = self.create_publisher(Float32MultiArray, 'arucu_filter', 10)
        self.x_filter = LowPassFilter(alpha=0.2)
        self.y_filter = LowPassFilter(alpha=0.2)
        self.depth_filter = LowPassFilter(alpha=0.2)

    def listener_callback(self, msg):
        x_meters, y_meters, fused_distance = msg.data

        dx = -y_meters
        dy = -x_meters
        dz = fused_distance


        x_meters_filtered = self.x_filter.apply(dx)
        y_meters_filtered = self.y_filter.apply(dy)
        fused_distance_filtered = self.depth_filter.apply(dz)

        filtered_data = Float32MultiArray()
        filtered_data.data = [float(x_meters_filtered), float(y_meters_filtered), float(fused_distance_filtered)]
        self.publisher_.publish(filtered_data)

        self.get_logger().info(f"Filtered data - x_meters: {x_meters_filtered:.4f}, y_meters_filtered: {y_meters_filtered:.4f}, fused_distance: {fused_distance_filtered:.4f}")

def main(args=None):
    rclpy.init(args=args)
    aruco_filter = ArucoFilter()
    rclpy.spin(aruco_filter)
    aruco_filter.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
