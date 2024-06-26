#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, MagneticField
import iio
import struct
import time

class IOProcessor(Node):

    def __init__(self):
        super().__init__('imu_driver')
        self.imu_publisher = self.create_publisher(Imu, 'kit/imu/data_raw', 10)
        self.mag_publisher = self.create_publisher(MagneticField, 'kit/imu/mag', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)
        
        self._mpu = None
        self._buff = None
        
        localiioctx = iio.Context()
        for dev in localiioctx.devices:
            if dev.name == 'mpu9250':
                self._mpu = dev
        if self._mpu is None:
            self.get_logger().error("MPU9250 not found!")
            rclpy.shutdown()
        
        channelmap = {}
        for c in self._mpu.channels:
            channelmap[c.id] = c
            
        channelmap['accel_x'].enabled = True
        channelmap['accel_y'].enabled = True
        channelmap['accel_z'].enabled = True
        channelmap['anglvel_x'].enabled = True
        channelmap['anglvel_y'].enabled = True
        channelmap['anglvel_z'].enabled = True
        channelmap['magn_x'].enabled = True
        channelmap['magn_y'].enabled = True
        channelmap['magn_z'].enabled = True
        channelmap['temp'].enabled = True
        channelmap['timestamp'].enabled = True
        
        self._buff = iio.Buffer(self._mpu, 1)

    def timer_callback(self):
        self._buff.refill()
        data = self._buff.read()
        imu_msg, mag_msg = self.process_data(data)
        self.imu_publisher.publish(imu_msg)
        self.mag_publisher.publish(mag_msg)
        self.get_logger().info('Publishing IMU and magnetic field data...')

    def process_data(self, data):
        # Document the constants used in detail. These are all available in the datasheet
        imu_msg = Imu()
        mag_msg = MagneticField()
        
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = 'imu_link'
        
        mag_msg.header.stamp = self.get_clock().now().to_msg()
        mag_msg.header.frame_id = 'imu_link'

        ax, ay, az = struct.unpack('>hhh', data[0:6])
        wx, wy, wz = struct.unpack('>hhh', data[6:12])
        mx, my, mz = struct.unpack('>hhh', data[12:18])
        temp = struct.unpack('>h', data[18:20])[0]
        timestamp = struct.unpack('<Q', data[24:32])[0]

        imu_msg.linear_acceleration.x = ax * 9.81 / 16384.0
        imu_msg.linear_acceleration.y = ay * 9.81 / 16384.0
        imu_msg.linear_acceleration.z = az * 9.81 / 16384.0

        imu_msg.angular_velocity.x = wx * (250.0 / 32768.0) * (3.14159 / 180.0)
        imu_msg.angular_velocity.y = wy * (250.0 / 32768.0) * (3.14159 / 180.0)
        imu_msg.angular_velocity.z = wz * (250.0 / 32768.0) * (3.14159 / 180.0)

        mag_msg.magnetic_field.x = mx * 0.6 / 32768.0
        mag_msg.magnetic_field.y = my * 0.6 / 32768.0
        mag_msg.magnetic_field.z = mz * 0.6 / 32768.0

        return imu_msg, mag_msg


def main(args=None):
    rclpy.init(args=args)
    imu_driver = IOProcessor()

    try:
        rclpy.spin(imu_driver)
    except KeyboardInterrupt:
        imu_driver.get_logger().info('Shutting down IMU driver...')
    finally:
        imu_driver.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
