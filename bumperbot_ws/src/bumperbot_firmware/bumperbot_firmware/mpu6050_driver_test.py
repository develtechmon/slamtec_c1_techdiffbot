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

            # Set orientation (dummy values; update if using orientation estimation)
            imu_msg.orientation = Quaternion()
            imu_msg.orientation.x = 0.0
            imu_msg.orientation.y = 0.0
            imu_msg.orientation.z = 0.0
            imu_msg.orientation.w = 1.0

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
