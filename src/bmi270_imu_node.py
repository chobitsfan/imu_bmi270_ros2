#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import smbus2
import struct
import time

I2C_ADDR = 0x68  # Default BMI270 I2C address
bus = smbus2.SMBus(1)

# BMI270 registers for accelerometer data
REG_ACC_DATA = 0x0C  # starting register for accel (X, Y, Z)

class BMI270Node(Node):
    def __init__(self):
        super().__init__('bmi270_imu_node')
        self.publisher_ = self.create_publisher(Imu, 'imu/data_raw', 10)
        self.timer = self.create_timer(0.02, self.timer_callback)  # 50Hz
        self.get_logger().info('BMI270 IMU node started.')

    def timer_callback(self):
        try:
            data = bus.read_i2c_block_data(I2C_ADDR, REG_ACC_DATA, 6)
            # Convert to signed 16-bit integers
            ax, ay, az = struct.unpack('<hhh', bytes(data))
            imu_msg = Imu()
            imu_msg.linear_acceleration.x = ax * 0.0098  # scale factor depends on config
            imu_msg.linear_acceleration.y = ay * 0.0098
            imu_msg.linear_acceleration.z = az * 0.0098
            imu_msg.header.stamp = self.get_clock().now().to_msg()
            imu_msg.header.frame_id = 'imu_link'
            self.publisher_.publish(imu_msg)
        except Exception as e:
            self.get_logger().error(f"Failed to read sensor: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = BMI270Node()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

