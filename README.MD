# Object Detection Package

## Deskripsi

Comming soon...
Under construction...

## Penggunaan

### Prasyarat

Sebelum menginstal Object Detection Package, pastikan Anda telah menginstal ROS2 dan semua dependensi yang diperlukan, termasuk:

- rclcpp
- sensor_msgs
- cv_bridge
- image_transport
- OpenCV

### Instalasi

1. Clone repository ini ke dalam workspace ROS2 Anda:

   ```bash
   cd ~/ros2_ws/src
   git clone https://github.com/username/object_detection.git
   ```

2. Masuk ke folder workspace:

   ```bash
   cd ~/ros2_ws
   ```

3. Bangun package:

   ```bash
   colcon build --packages-select object_detection
   ```

4. Sumberkan workspace:

   ```bash
   source install/setup.bash
   ```

5. Jalankan node:

   ```bash
   ros2 run object_detection camera_node
   ```
