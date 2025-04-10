# Object Detection Package

## Deskripsi

**Object Detection Package** adalah paket ROS 2 untuk deteksi objek secara real-time menggunakan OpenCV dan model YOLO. Paket ini sedang **dalam tahap pengembangan aktif**, dan dirancang untuk bekerja secara modular, sehingga memudahkan integrasi dan pengembangan lanjutan, seperti integrasi ke sistem robotik atau kendaraan otonom.

Fitur yang saat ini tersedia:

- Pengambilan gambar dari kamera
- Deteksi objek menggunakan OpenCV DNN + YOLO
- Visualisasi hasil deteksi menggunakan bounding box dan label

> **Catatan**: Fitur tambahan seperti pengaturan model via parameter server, output dalam bentuk custom message, atau integrasi ke sistem kontrol robot akan ditambahkan secara bertahap.

## Penggunaan

### Prasyarat

Pastikan Anda telah menginstal:

- ROS 2 Humble (atau versi lain yang kompatibel)
- OpenCV (dengan dukungan DNN)
- `cv_bridge`
- `image_transport`
- `rclcpp`
- `sensor_msgs`

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

5. Jalankan node deteksi:

   ```bash
   ros2 run object_detection detection_node
   ```

## Output

Node menampilkan frame kamera dengan hasil deteksi (bounding box dan label). Ke depan, hasil juga akan tersedia dalam format pesan ROS 2 untuk digunakan oleh node lain.

## Kontribusi

Kontribusi sangat terbuka! Silakan buka issue untuk laporan bug atau saran fitur, dan pull request untuk kontribusi langsung ke kode.
