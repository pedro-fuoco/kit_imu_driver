# kit_imu_driver

Esse é o pacote ROS do Kit de Robótica dedicado à publicar informações de aceleração, velocidade angular, campo magnético e temperatura, através de uma IMU.

## Dependencias

Clone esse pacote dentro do seu ROS workspace, dentro da pasta src. Para isso, substitua o endereço e nome do seu ROS workspace e copie os comandos abaixo:
```bash
cd ~/ros2_ws/src/
git clone git@github.com:pedro-fuoco/kit_imu_driver.git
```

### Rosdep
Esse pacote foi desenvolvido para a versão ROS 2 Jazzy. Uma vez que o ROS estiver devidamente instalado e inicializado, as dependencias especificas desse pacote podem ser instaladas através do rosdep.

Rosdep é um meta-package manager, utilizado para facilitar a instalação de dependencias. Apesar do nome com base histórica, essa ferramenta se tornou independente do ROS e deve ser instalada separadamente:

```bash
sudo apt install python3-rosdep
```

Uma vez instalado, ele deve ser inicializado e atualizado:

```bash
sudo rosdep init
rosdep update
```

Por fim, para esse repositorio, vamos utilizar o rosdep para instalar as dependencias listadas no arquivo `package.xml`. Para isso, substitua o endereço e nome do seu ROS workspace e copie os comandos abaixo:
```bash
cd ~/ros2_ws/src/kit_imu_driver
rosdep install --from-paths . -y --ignore-src
```

### Instalação manual de bibliotecas
Infelizmente, uma das bibliotecas utilizadas nesse pacote não está nos repositorios de indice do rosdep. Por conta disso, precisamos instala-la manualmente seguindo o seguinte comando:

```bash
pip install pylibiio
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

## Transformadas e eixos de coordenadas
Esse pacote publica os dados da IMU no eixo de coordenadas "imu_link". Porém, os demais nodes ROS do Kit de robótica fazem calculos com o eixo de coordenadas da base do robô, conhecido como "base_link". Por conta disso, precisamos estabelecer uma transformação estática entre "imu_link" e "base_link", para que os demais nodes ROS possam utilizar os dados do acelerômetro no referencial da base do robô.

## Launch
Para iniciar os programas `imu_node`, `imu_complementary_filter` e `static_transform_publisher`, responsáveis por publicar os topicos de IMU, MagneticField e Temperature, calcular a orientação do robô através dos dados crûs da IMU, e publicar a transformação estática entre os eixos do robô, respectivamente, basta utilizar o seguinte comando:
```bash
ros2 launch kit_imu_driver imu_launch.py
```

Para facilitar o teste desse pacote, foi criado um modo de `debug`, no qual a transformação entre os eixos `imu_link` e `odom` é publicada diretamente pelo node `imu_complementary_filter`. Isso permite que a orientação publicada seja visualizada em ferramentas como o `rviz2`. Dito isso, esse modo deve ser desabilitado sempre que outro pacote for responsável por publicar a transformação dos eixos do robô. Para habilita-lo, base rodar o seguinte comando:
```bash
ros2 launch kit_imu_driver imu_launch.py debug:=true
```
