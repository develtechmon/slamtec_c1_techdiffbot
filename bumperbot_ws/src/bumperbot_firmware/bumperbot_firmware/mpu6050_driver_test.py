import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from mpu6050 import mpu6050
from geometry_msgs.msg import Quaternion
import math

class MPU6050Driver(Node):
    def __init__(self):
        super().__init__('mpu6050_driver')

        # Initialize the MPU6050 sensor
        self.sensor = mpu6050(0x68)

        # Publisher for the IMU data
        self.imu_publisher = self.create_publisher(Imu, '/imu/out', 10)

        # Timer to publish data at 50 Hz
        self.timer = self.create_timer(0.02, self.publish_imu_data)

        self.get_logger().info("MPU6050 IMU Driver initialized.")

    def publish_imu_data(self):
        try:
            # Read sensor data
            accel = self.sensor.get_accel_data()
            gyro = self.sensor.get_gyro_data()

            # Create an IMU message
            imu_msg = Imu()
            imu_msg.header.stamp = self.get_clock().now().to_msg()
            imu_msg.header.frame_id = 'imu_link'

            # Populate linear acceleration (convert from g to m/s^2)
            imu_msg.linear_acceleration.x = accel['x'] * 9.81
            imu_msg.linear_acceleration.y = accel['y'] * 9.81
            imu_msg.linear_acceleration.z = accel['z'] * 9.81

            # Populate angular velocity (convert from deg/s to rad/s)
            imu_msg.angular_velocity.x = math.radians(gyro['x'])
            imu_msg.angular_velocity.y = math.radians(gyro['y'])
            imu_msg.angular_velocity.z = math.radians(gyro['z'])

            # Orientation is not provided by the MPU6050 directly
            # It will be estimated using an EKF or other filtering algorithm
            imu_msg.orientation = Quaternion()
            imu_msg.orientation.x = 0.0
            imu_msg.orientation.y = 0.0
            imu_msg.orientation.z = 0.0
            imu_msg.orientation.w = 1.0

            # Set covariances (adjust these based on your setup and sensor accuracy)
            imu_msg.orientation_covariance = [1e-3, 0, 0, 0, 1e-3, 0, 0, 0, 1e-3]
            imu_msg.angular_velocity_covariance = [1e-3, 0, 0, 0, 1e-3, 0, 0, 0, 1e-3]
            imu_msg.linear_acceleration_covariance = [1e-3, 0, 0, 0, 1e-3, 0, 0, 0, 1e-3]

            # Publish the IMU data
            self.imu_publisher.publish(imu_msg)
        except Exception as e:
            self.get_logger().error(f"Error reading MPU6050 data: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = MPU6050Driver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
