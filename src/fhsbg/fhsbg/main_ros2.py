import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import threading
from mavsdk import System
import asyncio
from drone_function import connect_drone, takeoff_velocity, control_velocity, enable_offboard_mode, set_initial_setpoint


class VelocitySubscriber(Node):
    def __init__(self):
        super().__init__('velocity_subscriber')
        self.subscription = self.create_subscription(
            Float32MultiArray,
            '/adjusted_velocity',
            self.listener_callback,
            10
        )
        self.subscription  # prevent unused variable warning
        self.V_x = 0.0
        self.V_y = 0.0
        self.V_z = 0.0

    def listener_callback(self, msg):
        # Assuming the Float32MultiArray contains [V_x, V_y, V_z]
        if len(msg.data) >= 3:
            self.V_x = msg.data[0]
            self.V_y = msg.data[1]
            self.V_z = msg.data[2]
            # self.get_logger().info(f'Received velocity data: V_x={msg.data[0]}, V_y={msg.data[1]}, V_z={msg.data[2]}')


class MainClass:
    #call the velocity ' takeoff and move from the camera'
    def __init__(self):
        rclpy.init()
        self.subscriber = VelocitySubscriber()
        self.drone = System()

    async def takeoff(self):
        self.get_logger().info("Connecting to drone...")
        await connect_drone(self.drone)
        self.get_logger().info("Drone connected, taking off...")
        await takeoff_velocity(self.drone)
        self.get_logger().info("Takeoff complete")

    async def move(self):
        V_x = self.subscriber.V_x
        V_y = self.subscriber.V_y
        V_z = self.subscriber.V_z
        self.get_logger().info(f"Moving with velocities - V_x: {V_x }, V_y: {V_y }, V_z: {V_z}")

        # Configure the move direction
        await control_velocity(self.drone, V_y , V_x , 0)  # Adjusting velocity as per your

    async def run(self):
        # Start the subscriber node in a separate thread
        subscriber_thread = threading.Thread(target=rclpy.spin, args=(self.subscriber,))
        subscriber_thread.start()

        await self.takeoff()
        await enable_offboard_mode(self.drone)
        await set_initial_setpoint(self.drone)

        try:
            while rclpy.ok():
                await self.move()
                rclpy.spin_once(self.subscriber, timeout_sec=0.01)
        except KeyboardInterrupt:
            pass
        finally:
            self.subscriber.destroy_node()
            rclpy.shutdown()
            subscriber_thread.join()

    def get_logger(self):
        return self.subscriber.get_logger()


if __name__ == '__main__':
    main = MainClass()
    asyncio.run(main.run())
