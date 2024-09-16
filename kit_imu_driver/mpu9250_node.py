import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, MagneticField, Temperature
from std_msgs.msg import Header
from iio import Context, Buffer

class MPU9250Node(Node):
    def __init__(self):
        super().__init__('mpu9250_driver')

        self._mpu = self.find_device('mpu9250')
        if self._mpu is None:
            self.get_logger().error("MPU9250 not found!")
            rclpy.shutdown()
            return
        
        self.accel_x = self._mpu.find_channel("accel_x", False)
        self.accel_y = self._mpu.find_channel("accel_y", False)
        self.accel_z = self._mpu.find_channel("accel_z", False)
        self.anglvel_x = self._mpu.find_channel("anglvel_x", False)
        self.anglvel_y = self._mpu.find_channel("anglvel_y", False)
        self.anglvel_z = self._mpu.find_channel("anglvel_z", False)
        self.gyro = self._mpu.find_channel("gyro", False)
        self.magn_x = self._mpu.find_channel("magn_x", False)
        self.magn_y = self._mpu.find_channel("magn_y", False)
        self.magn_z = self._mpu.find_channel("magn_z", False)
        self.temp = self._mpu.find_channel("temp", False)
        self.timestamp = self._mpu.find_channel("timestamp", False)

        channels = [
            self.accel_x, self.accel_y, self.accel_z,
            self.anglvel_x, self.anglvel_y, self.anglvel_z,
            self.gyro, self.magn_x, self.magn_y, self.magn_z,
            self.temp, self.timestamp
        ]
        if any(channel is None for channel in channels):
            self.get_logger().error("MPU9250 channels not found!")
            rclpy.shutdown()
            return

        self.imu_pub = self.create_publisher(Imu, 'kit/imu/data_raw', 10)
        self.magn_pub = self.create_publisher(MagneticField, 'kit/imu/mag', 10)
        self.temp_pub = self.create_publisher(Temperature, 'kit/imu/temp', 10)

        self.declare_parameter('mpu9250.scale.accel_scale', 0.000598)
        self.declare_parameter('mpu9250.scale.anglvel_scale', 0.000133090)
        self.declare_parameter('mpu9250.sampling_frequency', 50)
        self.declare_parameter('imu_node.update_frequency', 10)

        desired_accel_scale = self.get_parameter('mpu9250.scale.accel_scale').value
        desired_anglvel_scale = self.get_parameter('mpu9250.scale.anglvel_scale').value
        desired_sampling_frequency = self.get_parameter('mpu9250.sampling_frequency').value

        available_accel_scales = [float(scale) for scale in self.accel_x.attrs['scale_available'].value.split(' ')]
        available_anglvel_scales = [float(scale) for scale in self.anglvel_x.attrs['scale_available'].value.split(' ')]
        available_sampling_frequencies = [float(freq) for freq in self._mpu.attrs['sampling_frequency_available'].value.split(' ')]
        if desired_accel_scale in available_accel_scales:
            try:
                self.accel_x.attrs['scale'].value = str(desired_accel_scale)
                self.accel_y.attrs['scale'].value = str(desired_accel_scale)
                self.accel_z.attrs['scale'].value = str(desired_accel_scale)
            except OSError as e:
                if e.errno == 22:
                    self.get_logger().error("Unavailable accel scale configuration, ignoring...")
                elif e.errno == 16:
                    self.get_logger().error("Device busy while trying to change accel configuration, ignoring...")
                else:
                    self.get_logger().error("Unexpected error while configuring accel scale, ignoring...")
                    print(e)

        if desired_anglvel_scale in available_anglvel_scales:
            try:
                self.anglvel_x.attrs['scale'].value = str(desired_anglvel_scale)
                self.anglvel_y.attrs['scale'].value = str(desired_anglvel_scale)
                self.anglvel_z.attrs['scale'].value = str(desired_anglvel_scale)
            except OSError as e:
                if e.errno == 22:
                    self.get_logger().error("Unavailable anglvel configuration, ignoring...")
                elif e.errno == 16:
                    self.get_logger().error("Device busy while trying to change angvel configuration, ignoring...")
                else:
                    self.get_logger().error("Unexpected error while configuring anglvel scale, ignoring...")

        if desired_sampling_frequency in available_sampling_frequencies:
            try:
                self._mpu.attrs['sampling_frequency_available'].value = str(desired_sampling_frequency)
            except OSError as e:
                if e.errno == 22:
                    self.get_logger().error("Unavailable sampling frequency configuration, ignoring...")
                elif e.errno == 16:
                    self.get_logger().error("Device busy while trying to change sampling frequency configuration, ignoring...")
                else:
                    self.get_logger().error("Unexpected error while configuring sampling frequency, ignoring...")

        imu_node_update_frequency = self.get_parameter('imu_node.update_frequency').value


        self.timer = self.create_timer(1/imu_node_update_frequency, self.publish_imu_data)

        self.accel_x.enabled = True
        self.accel_y.enabled = True
        self.accel_z.enabled = True
        self.anglvel_x.enabled = True
        self.anglvel_y.enabled = True
        self.anglvel_z.enabled = True
        self.gyro.enabled = True
        self.magn_x.enabled = True
        self.magn_y.enabled = True
        self.magn_z.enabled = True
        self.temp.enabled = True
        self.timestamp.enabled = True

        self.buffer = Buffer(self._mpu, 1)

    def __del__(self):
        if(self.buffer):
            self.buffer.cancel()

    def find_device(self, name):
        context = Context()
        for dev in context.devices:
            if dev.name == name:
                return dev
        return None


    def publish_imu_data(self):
        self.buffer.refill()

        data = self.buffer.read()

        accel_x = int.from_bytes(data[0:2], byteorder='big', signed=True)
        accel_y = int.from_bytes(data[2:4], byteorder='big', signed=True)
        accel_z = int.from_bytes(data[4:6], byteorder='big', signed=True)
        
        anglvel_x = int.from_bytes(data[8:10], byteorder='big', signed=True)
        anglvel_y = int.from_bytes(data[10:12], byteorder='big', signed=True)
        anglvel_z = int.from_bytes(data[12:14], byteorder='big', signed=True)
        
        magn_x = int.from_bytes(data[14:16], byteorder='big', signed=True)
        magn_y = int.from_bytes(data[16:18], byteorder='big', signed=True)
        magn_z = int.from_bytes(data[18:20], byteorder='big', signed=True)
        
        temp = int.from_bytes(data[6:8], byteorder='big', signed=True)
        
        timestamp = int.from_bytes(data[-8:], byteorder='little', signed=False)

        accel_scale = float(self.accel_x.attrs['scale'].value)
        gyro_scale = float(self.anglvel_x.attrs['scale'].value)
        magn_scale = float(self.magn_x.attrs['scale'].value)
        temp_scale = float(self.temp.attrs['scale'].value)
        temp_offset = float(self.temp.attrs['offset'].value)


        header = Header()
        header.stamp.sec = timestamp
        header.frame_id = "imu_link"

        imu_msg = Imu()
        imu_msg.header = header

        imu_msg.linear_acceleration.x = accel_x * accel_scale
        imu_msg.linear_acceleration.y = accel_y * accel_scale
        imu_msg.linear_acceleration.z = accel_z * accel_scale
        imu_msg.angular_velocity.x = anglvel_x * gyro_scale
        imu_msg.angular_velocity.y = anglvel_y * gyro_scale
        imu_msg.angular_velocity.z = anglvel_z * gyro_scale


        magn_msg = MagneticField()
        magn_msg.header = imu_msg.header
        magn_msg.magnetic_field.x = magn_x * magn_scale
        magn_msg.magnetic_field.y = magn_y * magn_scale
        magn_msg.magnetic_field.z = magn_z * magn_scale


        temp_msg = Temperature()
        temp_msg.header = imu_msg.header
        temp_msg.temperature = (temp - temp_offset) * temp_scale

        self.imu_pub.publish(imu_msg)
        self.magn_pub.publish(magn_msg)
        self.temp_pub.publish(temp_msg)

def main(args=None):
    rclpy.init(args=args)  # Inicializa o cliente ROS 2
    imu_node = MPU9250Node()  # Cria o node do encoder

    try:
        # Mantém o node ativo para continuar capturando e publicando ticks
        rclpy.spin(imu_node)
    except KeyboardInterrupt:
        # Loga uma mensagem ao encerrar o node
        imu_node.get_logger().info('Shutting down imu driver...')
    finally:
        # Destrói o node e finaliza o cliente ROS 2
        imu_node.destroy_node() 
        rclpy.shutdown()

# Ponto de entrada do script
if __name__ == '__main__':
    main()