# kit_imu_driver

Esse é o pacote ROS do Kit de Robótica dedicado à publicar informações de aceleração, velocidade angular, campo magnético e temperatura, através de uma IMU.

## Dependencias

Clone esse pacote dentro do seu ROS workspace, dentro da pasta src. Para isso, substitua o endereço e nome do seu ROS workspace e copie os comandos abaixo:
```bash
cd ~/ros2_ws/src/
git clone git@github.com:pedro-fuoco/kit_encoder_driver.git
```

### Rosdep
WORK IN PROGRESS

### Instalação manual de bibliotecas
Infelizmente, uma das bibliotecas utilizadas nesse pacote não está nos repositorios de indice do rosdep. Por conta disso, precisamos instala-la manualmente seguindo o seguinte comando:

```bash
pip install pylibiio --break-system-packages
```

### Colcon
Inicialize o seu ROS workspace e compile ele utilizando a ferramenta `colcon`. Mais uma vez, é necessario substituir o endereço e nome do seu próprio ROS workspace abaixo:
```bash
cd ~/ros2_ws/
source install/setup.sh
colcon build --packages-select kit_imu_driver
```
Fique de olho em possiveis erros nesse processo e realize debug se necessario.

### Hardware
Esse pacote assume que a IMU utilizada é a MPU9250. Caso esse hardware seja trocado, é necessário que o driver seja alterado de acordo com a mudança. O restante do pacote, como o `launch` e o `node` são agnósticos ao modelo da IMU, sendo chamados genericamente de `imu_node`.

### Permissões
Para que o node ROS tenha acesso à interface de IIO, utilizado para ler e escrever no MPU9250, precisamos dar algumas permissões. Para isso, basta realizar o seguinte:

```bash
sudo groupadd iio
sudo usermod -a -G iio $USER
touch /etc/udev/rules.d/99-iio-mpu9250.rules
```

Agora adicione o seguinte texto no arquivo `99-iio-mpu9250.rules`:

```bash
KERNEL=="iio:device[0-9]*", SUBSYSTEM=="iio", ATTR{name}=="mpu9250", ACTION=="add", GROUP="iio", PROGRAM="/bin/sh -c 'chgrp -R iio /sys%p; chmod -R g=u /sys%p'"
```

Agora vamos reiniciar o driver do dispositivo:
```bash
sudo rmmod inv-mpu6050-i2c
sudo modprobe inv-mpu6050-i2c
```


## Configurações
As configurações desse pacote são feitas através de ROS `params`. Os arquivos que descrevem as configurações, que podem ser editados manualmente, se encontram na pasta `config`.
Lembre-se sempre de recompilar o pacote depois de fazer mudanças na configuração, com:
```bash
cd ~/ros2_ws/
source install/setup.sh
colcon build --packages-select kit_imu_driver
```

## Launch
Para iniciar o programas `imu_node`, responsável por publicar os topicos de IMU, MagneticField e Temperature, basta utilizar o seguinte comando:
```bash
ros2 launch kit_imu_driver imu_launch.py
```
