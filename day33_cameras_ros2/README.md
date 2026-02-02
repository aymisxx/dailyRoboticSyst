# Day 33: Cameras in ROS2 - **Data, Noise, Rates**

This day focuses on understanding **cameras as system components**, not as computer-vision magic.

The goal is to treat a camera as what it actually is in a robotics stack:

> a **rate-limited, delayed, noisy measurement source** that publishes data into ROS2.

No feature extraction.  
No OpenCV pipelines.  
No Gazebo yet.

Just **signals, timing, and system behavior**.

---

## Objective

- Understand how camera data appears in ROS2.
- Inspect camera topics and message types.
- Measure **effective publish rate**.
- Observe **non-ideal behavior** (latency, jitter, mismatch between configuration and reality).
- Build intuition for why perception ≠ truth.

This aligns exactly with **Day 33: Cameras in ROS2 (data, noise, rates)** in the project curriculum.

## Environment

- **OS:** Ubuntu 22.04  
- **ROS2:** Humble Hawksbill  
- **Packages used:**  
  - `image_publisher`  
  - `image_tools`

## Installation (one-time)

```bash
sudo apt update
sudo apt install -y \
  ros-humble-image-publisher \
  ros-humble-image-tools
```

Verify packages exist:
```bash
ros2 pkg list | grep image
```

## Execution: Step-by-Step (Exactly as Run)

### Terminal 1: Start the “camera” publisher

A static image is published as a ROS2 image stream.  
This acts as a **camera-like data source** without requiring hardware or Gazebo.

```bash
source /opt/ros/humble/setup.bash

ros2 run image_publisher image_publisher_node \
  --ros-args \
  -p filename:=/home/twilightpriest/Downloads/turtlesim_smoke_test.png \
  -p publish_rate:=5.0
```

This node publishes:
- Image data to `/image_raw`
- Camera metadata to `/camera_info`

The node runs silently once initialized, this is normal.

### Terminal 2: Inspect ROS2 topics

List active topics:

```bash
source /opt/ros/humble/setup.bash
ros2 topic list
```

Observed:

```
/image_raw
/camera_info
/parameter_events
/rosout
```

### Terminal 2: Inspect message type and publishers

```bash
ros2 topic info /image_raw
```

Result:
- **Type:** `sensor_msgs/msg/Image`.
- **Publisher count:** 1.
- **Subscriber count:** ≥ 1.

This confirms a valid camera data stream.

### Terminal 2: Measure effective publish rate

```bash
ros2 topic hz /image_raw
```

Observed behavior:
- Effective rate ≈ **0.13 - 0.20 Hz**.
- Large variance in min / max intervals.
- Significant jitter.

This is **intentional and instructive**.

Although the node was configured for **5 Hz**, the observed rate is much lower and irregular, highlighting that:

> **Configured rate ≠ delivered rate**

### Terminal 3: Visualize the image stream

```bash
source /opt/ros/humble/setup.bash
ros2 run image_tools showimage --ros-args -r image:=/image_raw
```

Results:
- Image window titled `/image_raw`.
- Continuous frame reception.
- Terminal logs confirm repeated message receipt:

  ```
  [showimage]: Received image #camera
  ```

## Evidence Collected

All runtime evidence is stored under `results/`:


```
results/
├── image_raw_stream.png
├── runtime_video.webm
├── showimage_received.png
├── topic_hz_image_raw.png
├── topic_info_image_raw.png
└── topic_list.png
```

This includes:

- Static proof (screenshots).
- Dynamic proof (runtime video).
- Quantitative proof (rate measurements).

## Systems-Level Observations (Key Learning)

### 1. Cameras publish **measurements** and not state
Control and estimation never see reality, they see **messages**.

### 2. Publish configuration does not guarantee delivery rate

Even with:

```
publish_rate := 5.0
```

The observed rate was:

```
~0.13–0.20 Hz with high jitter
```

This mirrors real systems:
- scheduling delays.
- buffering.
- processing overhead.
- toolchain limitations.

### 3. Perception pipelines must tolerate non-ideal timing
Consumers of camera data must assume:
- latency.
- dropped frames.
- jitter.
- stale information.

This is why **time synchronization, buffering, and estimation exist**.

### 4. This behavior is not a bug but a lesson

Day 33 is not about making a “perfect camera”.

It is about **seeing how imperfect data enters the system**.

## Why This Day Is Minimal by Design

- No Gazebo (scheduled for Day 39).
- No CV pipelines (out of scope).
- No feature extraction.

This day builds **intuition**, not capability.

Depth comes later.  
Judgment comes first.

## Summary

- A camera stream was created and validated in ROS2.
- Data flow was inspected at the topic level.
- Effective rate and jitter were measured.
- Non-ideal system behavior was observed and documented.
- Evidence was captured honestly and reproducibly.

**Day 33 complete.**

---