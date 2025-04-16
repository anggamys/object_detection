# Usage Guide

## Available Commands

This package provides two main ROS 2 nodes: `camera_node` and `detection_node`. Below are the instructions for running and configuring them.

### 1. Running `camera_node`

The `camera_node` is responsible for capturing images from a camera or media file and publishing them to the `/image_raw` topic.

Run the node with the following command:

```bash
ros2 run object_detection camera_node
```

#### Available Parameters

- `source`: Specifies the image source. The default is `0` (the main webcam of your device). You can also provide a path to an image or video file.

To specify a custom source:

```bash
ros2 run object_detection camera_node --ros-args -p source:=/path/to/image_or_video
```

---

### 2. Running `detection_node`

The `detection_node` performs object detection on images received from the `/image_raw` topic.

Run the node with the following command:

```bash
ros2 run object_detection detection_node
```

#### Available Parameters

- `model_path`: Specifies the path to the object detection model (ONNX format). The default is `model/yolo_model.onnx`.

To specify a custom model path:

```bash
ros2 run object_detection detection_node --ros-args -p model_path:=/path/to/model.onnx
```

---

### 3. Using Launch File

Alternatively, you can use the provided launch file to start both nodes:

```bash
ros2 launch object_detection object_detection.launch.py
```
