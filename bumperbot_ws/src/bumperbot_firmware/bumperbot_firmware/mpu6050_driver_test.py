#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from mpu6050 import mpu6050
from geometry_msgs.msg import Quaternion
import math

class MPU6050Node(Node):
    def __init__(self):
        super().__init__('mpu6050_node')

        # Initialize MPU6050 sensor
        self.sensor = mpu6050(0x68)

        # Create a publisher for the /imu/out topic
        self.imu_publisher = self.create_publisher(Imu, '/imu/out', 10)

        # Timer for publishing IMU data
        self.timer = self.create_timer(0.1, self.publish_imu_data)

        self.get_logger().info('MPU6050 IMU Node has started.')

    def publish_imu_data(self):
        try:
            # Read sensor data
            accel_data = self.sensor.get_accel_data()
            gyro_data = self.sensor.get_gyro_data()

            # Apply filtering (basic example; use advanced filters like Kalman for better results)
            accel_data = {k: max(-2.0, min(2.0, v)) for k, v in accel_data.items()}  # Clamp acceleration to [-2, 2] g
            gyro_data = {k: max(-250.0, min(250.0, v)) for k, v in gyro_data.items()}  # Clamp gyro to [-250, 250] dps

            # Create and populate the IMU message
            imu_msg = Imu()
            imu_msg.header.stamp = self.get_clock().now().to_msg()
            imu_msg.header.frame_id = 'imu_link'

            # Populate acceleration (convert from g to m/s^2 by multiplying with 9.81)
            imu_msg.linear_acceleration.x = accel_data['x'] * 9.81
            imu_msg.linear_acceleration.y = accel_data['y'] * 9.81
            imu_msg.linear_acceleration.z = accel_data['z'] * 9.81

            # Populate angular velocity (rad/s)
            imu_msg.angular_velocity.x = math.radians(gyro_data['x'])
            imu_msg.angular_velocity.y = math.radians(gyro_data['y'])
            imu_msg.angular_velocity.z = math.radians(gyro_data['z'])

            # Set orientation (keep constant if no estimation is implemented)
            imu_msg.orientation = Quaternion()
            imu_msg.orientation.x = 0.0
            imu_msg.orientation.y = 0.0
            imu_msg.orientation.z = 0.0
            imu_msg.orientation.w = 1.0

            # Add covariance for sensor noise (tweak these values as needed)
            imu_msg.orientation_covariance = [0.01, 0, 0, 0, 0.01, 0, 0, 0, 0.01]
            imu_msg.angular_velocity_covariance = [0.01, 0, 0, 0, 0.01, 0, 0, 0, 0.01]
            imu_msg.linear_acceleration_covariance = [0.1, 0, 0, 0, 0.1, 0, 0, 0, 0.1]

            # Publish the message
            self.imu_publisher.publish(imu_msg)
            self.get_logger().info('Published IMU data.')
        except Exception as e:
            self.get_logger().error(f'Error reading IMU data: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = MPU6050Node()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
