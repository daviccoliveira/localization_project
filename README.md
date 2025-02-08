# Uso de SLAM baseado em Marcos Fiduciais e Fusão de Sensores para Localização de Robô UGV

## Objetivo

Melhorar a precisão das informações da odometria do UGV utilizando um sistema de SLAM baseado em marcadores fiduciais mais a função de sensores.

## Objetivos Específicos

- Detectar marcadores fiduciais (Apriltag) utilizando uma câmera monocular acoplada ao UGV.
- Testar e avaliar o sistema de SLAM em um ambiente controlado (simulado), ajustando parâmetros para otimizar a precisão do mapeamento e da localização.
- Realizar a fusão de dados entre a odometria dos encoders e SLAM a partir das medições da câmera.

## Instalação

O projeto foi testado em um ambiente **Docker** com o **ROS Noetic**.

### Dependências

Instale os pacotes necessários executando os seguintes comandos:

```bash
sudo apt-get install ros-noetic-husky-navigation ros-noetic-husky-gazebo ros-noetic-husky-viz
sudo apt-get install ros-noetic-husky-simulator
sudo apt-get install ros-noetic-realsense2-camera ros-noetic-realsense2-description ros-noetic-gazebo-plugins
sudo apt install ros-noetic-rtabmap ros-noetic-rtabmap-ros
```

### Clonagem de Pacotes

Clone os seguintes pacotes no seu workspace do ROS:

```bash
git clone https://github.com/lar-deeufba/lar_gazebo.git
git clone https://github.com/AprilRobotics/apriltag_ros.git
```

### Execução

Para iniciar os testes, execute os seguintes comandos:

1. **Configurar a URDF do Husky para ativar camera nativa**  
   ```bash
   cd /catkin_ws/src/lar_gazebo
   sh husky_accessories.sh 
   ```

2. **Iniciar o Simulador do Husky com o Gazebo**
    ```bash
    roslaunch lar_gazebo lar_husky.launch
    ```

3. **Iniciar a Detecção Contínua de Apriltags**
    ```bash
    roslaunch apriltag_ros continuous_detection.launch
    ```

4. **Visualizar o Robô no Rviz**
    ```bash
    roslaunch husky_viz view_robot.launch
    ```

5. **Iniciar a Navegação Baseada em Mapless**
    ```bash
    roslaunch husky_navigation move_base_mapless_demo.launch
    ```

6. **Executar o RTAB-Map para SLAM**
    ```bash
    roslaunch rtabmap_ros rtabmap.launch \
        rtabmap_args:="--delete_db_on_start" \
        depth_topic:=/realsense/depth/image_rect_raw \
        rgb_topic:=/realsense/color/image_raw \
        camera_info_topic:=/realsense/color/camera_info \
        odom_topic:=/odom \
        imu_topic:=/imu/data \
        frame_id:=base_link
    ```

7. **Visualizar a Estrutura da Rede de Tópicos no ROS**
    ```bash
    rqt_graph
    ```

### Referências

- [ROS Noetic](http://wiki.ros.org/noetic)
- [Husky Robot](https://clearpathrobotics.com/husky-unmanned-ground-vehicle-robot/)
- [Apriltag ROS](https://github.com/AprilRobotics/apriltag_ros)
- [RTAB-Map ROS](http://wiki.ros.org/rtabmap_ros)
