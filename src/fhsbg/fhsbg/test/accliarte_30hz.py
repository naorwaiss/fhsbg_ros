import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3
from collections import deque
import numpy as np


class VelocityPredictionNode(Node):
    def __init__(self):
        super().__init__('velocity_prediction_node')

        self.subscription = self.create_subscription(
            Vector3,
            '/adjusted_velocity',
            self.velocity_callback,
            10)

        self.publisher_predicted = self.create_publisher(Vector3, '/predicted_velocity', 10)
        self.publisher_additional = self.create_publisher(Vector3, '/additional_topic', 10)

        self.velocity_samples = deque(maxlen=10)
        self.timer = self.create_timer(1 / 30.0, self.publish_prediction)

    def velocity_callback(self, msg):
        velocity = np.array([msg.x, msg.y, msg.z])
        self.velocity_samples.append(velocity)

    def publish_prediction(self):
        if len(self.velocity_samples) < 10:
            return

        # Here we can make a prediction. For simplicity, let's use the mean of the last 10 samples.
        prediction = np.mean(self.velocity_samples, axis=0)

        predicted_msg = Vector3()
        predicted_msg.x = float(prediction[0])
        predicted_msg.y = float(prediction[1])
        predicted_msg.z = float(prediction[2])

        additional_msg = Vector3()
        additional_msg.x = float(prediction[0] * 1.1)  # Example modification
        additional_msg.y = float(prediction[1] * 1.1)
        additional_msg.z = float(prediction[2] * 1.1)

        self.publisher_predicted.publish(predicted_msg)
        self.publisher_additional.publish(additional_msg)


def main(args=None):
    rclpy.init(args=args)
    node = VelocityPredictionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
