import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, MagneticField, Temperature
from std_msgs.msg import Header
from iio import Context, Buffer

class MPU9250Node(Node):
    def __init__(self):
        # Inicializa o node ROS 2 chamado 'mpu9250_driver'
        super().__init__('mpu9250_driver')

        # Encontra o dispositivo MPU9250 conectado
        self._mpu = self.find_device('mpu9250')
        if self._mpu is None:
            # Se o dispositivo não for encontrado, loga um erro e encerra o cliente ROS 2
            self.get_logger().error("MPU9250 not found!")
            rclpy.shutdown()
            return
        
        # Encontra e configura os canais de leitura do MPU9250
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

        # Verifica se todos os canais foram encontrados
        channels = [
            self.accel_x, self.accel_y, self.accel_z,
            self.anglvel_x, self.anglvel_y, self.anglvel_z,
            self.gyro, self.magn_x, self.magn_y, self.magn_z,
            self.temp, self.timestamp
        ]
        if any(channel is None for channel in channels):
            # Se algum canal não for encontrado, loga um erro e encerra o cliente ROS 2
            self.get_logger().error("MPU9250 channels not found!")
            rclpy.shutdown()
            return

        # Cria os publicadores para os dados de IMU, campo magnético e temperatura
        self.imu_pub = self.create_publisher(Imu, 'kit/imu/data_raw', 10)
        self.magn_pub = self.create_publisher(MagneticField, 'kit/imu/mag', 10)
        self.temp_pub = self.create_publisher(Temperature, 'kit/imu/temp', 10)

        # Declara parâmetros ROS 2 com valores padrão
        self.declare_parameter('mpu9250.scale.accel_scale', 0.000598)
        self.declare_parameter('mpu9250.scale.anglvel_scale', 0.000133090)
        self.declare_parameter('mpu9250.sampling_frequency', 50)
        self.declare_parameter('imu_node.update_frequency', 10)

        # Obtém valores dos parâmetros, que podem ser personalizados
        desired_accel_scale = self.get_parameter('mpu9250.scale.accel_scale').value
        desired_anglvel_scale = self.get_parameter('mpu9250.scale.anglvel_scale').value
        desired_sampling_frequency = self.get_parameter('mpu9250.sampling_frequency').value

        # Configura a escala dos canais de aceleração, se o valor desejado estiver disponível
        available_accel_scales = [float(scale) for scale in self.accel_x.attrs['scale_available'].value.split(' ')]
        if desired_accel_scale in available_accel_scales:
            try:
                self.accel_x.attrs['scale'].value = str(desired_accel_scale)
                self.accel_y.attrs['scale'].value = str(desired_accel_scale)
                self.accel_z.attrs['scale'].value = str(desired_accel_scale)
            except OSError as e:
                self.handle_iio_write_error(e, "accel scale")

        # Configura a escala dos canais de velocidade angular, se o valor desejado estiver disponível
        available_anglvel_scales = [float(scale) for scale in self.anglvel_x.attrs['scale_available'].value.split(' ')]
        if desired_anglvel_scale in available_anglvel_scales:
            try:
                self.anglvel_x.attrs['scale'].value = str(desired_anglvel_scale)
                self.anglvel_y.attrs['scale'].value = str(desired_anglvel_scale)
                self.anglvel_z.attrs['scale'].value = str(desired_anglvel_scale)
            except OSError as e:
                self.handle_iio_write_error(e, "anglvel scale")

        # Configura a frequência de amostragem, se o valor desejado estiver disponível
        available_sampling_frequencies = [float(freq) for freq in self._mpu.attrs['sampling_frequency_available'].value.split(' ')]
        if desired_sampling_frequency in available_sampling_frequencies:
            try:
                self._mpu.attrs['sampling_frequency_available'].value = str(desired_sampling_frequency)
            except OSError as e:
                self.handle_iio_write_error(e, "sampling frequency")

        # Configura a frequência de atualização do nó
        imu_node_update_frequency = self.get_parameter('imu_node.update_frequency').value
        self.timer = self.create_timer(1/imu_node_update_frequency, self.publish_imu_data)

        # Ativa todos os canais de leitura do MPU9250
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

        # Cria um buffer para leitura de dados do dispositivo
        self.buffer = Buffer(self._mpu, 1)

    def __del__(self):
        # Libera recursos associados ao buffer quando o objeto é destruído
        if self.buffer:
            self.buffer.cancel()

    def find_device(self, name):
        # Encontra e retorna o dispositivo com o nome especificado
        context = Context()
        for dev in context.devices:
            if dev.name == name:
                return dev
        return None

    def handle_iio_write_error(self, error, configuration_type):
        # Trata erros específicos de configuração de escala do dispositivo
        if error.errno == 22:
            self.get_logger().error(f"Unavailable {configuration_type} configuration, ignoring...")
        elif error.errno == 16:
            self.get_logger().error(f"Device busy while trying to change {configuration_type} configuration, ignoring...")
        else:
            self.get_logger().error(f"Unexpected error while configuring {configuration_type}: {error}, ignoring...")

    def publish_imu_data(self):
        # Preenche o buffer e lê os dados brutos do MPU9250
        self.buffer.refill()
        data = self.buffer.read()

        # Converte os dados brutos para valores inteiros
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

        # Obtém as escalas e offsets dos canais
        accel_scale = float(self.accel_x.attrs['scale'].value)
        gyro_scale = float(self.anglvel_x.attrs['scale'].value)
        magn_scale = float(self.magn_x.attrs['scale'].value)
        temp_scale = float(self.temp.attrs['scale'].value)
        temp_offset = float(self.temp.attrs['offset'].value)

        # Cria o cabeçalho para as mensagens
        header = Header()
        header.stamp.sec = timestamp // 10**9
        header.stamp.nanosec = timestamp % 10**9
        header.frame_id = "imu_link"

        # Cria e publica a mensagem IMU
        imu_msg = Imu()
        imu_msg.header = header
        imu_msg.linear_acceleration.x = accel_x * accel_scale
        imu_msg.linear_acceleration.y = accel_y * accel_scale
        imu_msg.linear_acceleration.z = accel_z * accel_scale
        imu_msg.angular_velocity.x = anglvel_x * gyro_scale
        imu_msg.angular_velocity.y = anglvel_y * gyro_scale
        imu_msg.angular_velocity.z = anglvel_z * gyro_scale

        # Cria e publica a mensagem de campo magnético
        magn_msg = MagneticField()
        magn_msg.header = imu_msg.header
        magn_msg.magnetic_field.x = magn_x * magn_scale
        magn_msg.magnetic_field.y = magn_y * magn_scale
        magn_msg.magnetic_field.z = magn_z * magn_scale

        # Cria e publica a mensagem de temperatura
        temp_msg = Temperature()
        temp_msg.header = imu_msg.header
        temp_msg.temperature = (temp + temp_offset) * temp_scale / 1000.0

        self.imu_pub.publish(imu_msg)
        self.magn_pub.publish(magn_msg)
        self.temp_pub.publish(temp_msg)

def main(args=None):
    # Inicializa o cliente ROS 2 e cria o node do MPU9250
    rclpy.init(args=args)
    imu_node = MPU9250Node()

    try:
        # Mantém o node ativo para capturar e publicar dados
        rclpy.spin(imu_node)
    except KeyboardInterrupt:
        # Loga uma mensagem ao encerrar o node com Ctrl+C
        imu_node.get_logger().info('Shutting down imu driver...')
    finally:
        # Destrói o node e finaliza o cliente ROS 2
        imu_node.destroy_node()
        rclpy.shutdown()

# Ponto de entrada do script
if __name__ == '__main__':
    main()
