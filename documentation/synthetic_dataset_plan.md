# Synthetic Dataset Generation & YOLO Fine-Tuning Plan

## 1. Ground Truth Problem in Gazebo Simulation
Pretrained COCO models (such as `yolov8n.pt`) are trained on 80 real-world photographic categories from the MS COCO dataset. When deployed in simulation environments:
1. **Untextured Primitives vs. Real Objects**: Synthetic geometric primitives (cubes, cylinders) lack the micro-textures, photorealistic shadows, and material gradients of real photography.
2. **Missing Domain Classes**: Key industrial and robotic domain classes are **absent from COCO 80**:
   * `traffic cone` (absent)
   * `warehouse shelf / rack` (absent)
   * `pallet` (absent)
   * `cardboard box / container` (COCO only has `suitcase` and `backpack`)

## 2. Solution: Gazebo Synthetic Data Collection Pipeline

```
+---------------------------+       +----------------------------+
|  Gazebo Camera Stream     |       |  Ground Truth Link Poses   |
|  (/camera/image_raw)      |       |  (/world/<name>/pose/info) |
+-------------+-------------+       +--------------+-------------+
              |                                    |
              +-----------------+------------------+
                                |
                                v
                +--------------------------------+
                | Synthetic Dataset Generator    |
                | - Projects 3D Box to 2D BBox   |
                | - P = K * [R | t]              |
                | - Formats YOLO annotation      |
                +---------------+----------------+
                                |
                                v
                +--------------------------------+
                | ~/CV_Autonomous_Navigation/    |
                |   datasets/gazebo_objects/     |
                |   ├── images/ (train, val)     |
                |   └── labels/ (train, val)     |
                +---------------+----------------+
                                |
                                v
                +--------------------------------+
                | Ultralytics YOLOv8 Fine-tuning |
                | (device="cuda:0", epochs=30)   |
                +---------------+----------------+
                                |
                                v
                +--------------------------------+
                | models/gazebo_yolov8n.pt       |
                +--------------------------------+
```

## 3. Dataset Format & Classes
The custom dataset targets 5 distinct simulation classes:
* `0: person`
* `1: chair`
* `2: table`
* `3: box`
* `4: traffic_cone`

## 4. Fine-Tuning Command
```bash
~/CV_Autonomous_Navigation/.venv/bin/yolo detect train \
  data=~/CV_Autonomous_Navigation/datasets/gazebo_data.yaml \
  model=~/CV_Autonomous_Navigation/models/yolov8n.pt \
  epochs=30 \
  imgsz=640 \
  device=0 \
  batch=16 \
  project=~/CV_Autonomous_Navigation/models \
  name=gazebo_finetune
```
