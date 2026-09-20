# Computer Vision Pipeline on NVIDIA RTX 3050

## Objective
Detect and semantically classify environmental obstacles in real-time using deep learning, accelerating inference via the dedicated NVIDIA RTX 3050 Laptop GPU.

## YOLO Architecture & Model Choice
* **Model**: YOLOv8 Nano (`yolov8n.pt`, 3.2M parameters)
* **Input Resolution**: $640 \times 480 \times 3$ (RGB)
* **Inference Runtime**: PyTorch 2.14.0+cu130 with NVIDIA Tensor / CUDA 13.0 execution
* **Latency**: ~5.6 ms per frame (over 170 FPS)
* **VRAM Footprint**: ~36 MB allocated (comfortably within 4 GB VRAM limit)

## Semantic Classes Tracked
1. `person` (Pedestrians / dynamic human actors)
2. `chair` (Office / hospital seating)
3. `dining table` (Tables, desks, lab benches)
4. `suitcase` / `box` (Warehouse packages and containers)
5. `traffic cone` (Hazard and warning markers)

## ROS 2 Topics
* Subscribes: `/camera/image_raw` (`sensor_msgs/msg/Image`)
* Publishes:
  * `/vision/detections` (`autonomous_robot_interfaces/msg/Detection2DArray`)
  * `/vision/annotated_image` (`sensor_msgs/msg/Image`)
