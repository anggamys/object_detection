# Object Detection

## Overview

Sebuah package ROS2 yang digunakan untuk mendeteksi objek menggunakan model YOLo. Package ini dirancang untuk digunakan dalam sistem robotika berbasis ROS2, dan dapat diintegrasikan dengan berbagai sensor dan perangkat keras lainnya.

## Installation

1. Clone repository ini ke dalam workspace ROS2 Anda.

   ```bash
   cd ros2_ws/src
   git clone https://github.com/anggamys/object_detection.git
   ```

2. build package ini dengan menggunakan colcon.

   ```bash
   cd ..
   colcon build --package-select object_detection
   source install/setup.bash
   ```
