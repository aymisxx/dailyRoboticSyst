# Day 36: Time, Latency, Synchronization (ROS2 Humble)

This day is about **timing truth** in robotics.

Perception isn’t just “what did the sensor see?”  
It’s “**when** did it see it, and **when** did the system use it?”

---

This experiment makes time bugs visible via two measurable quantities:
- **Message age** (latency + jitter)
- **Timestamp mismatch** (sync quality)

## Folder structure

```
day36_time_latency_sync/
├── README.md
├── time_latency_demo.py
└── results/
    ├── skew.csv
    ├── skew_age.png
    └── skew_stamp_mismatch.png
```

## System overview

### Nodes / topics

**FakeCamPublisher**

- Topic: `/cam`
- Rate: 30 Hz
- Timestamp = measurement time
- Artificial delay injected before publish

**FakeImuPublisher**

- Topic: `/imu`
- Rate: 200 Hz
- Timestamp = measurement time
- No artificial delay

**SkewLogger**

- Subscribes to `/cam` and `/imu`
- Pairs camera with closest IMU timestamp
- Logs message age and timestamp mismatch

All timing uses the same ROS clock type.

## Mathematical Model

Message age:

$$a_{cam} = t_{now} - t_{cam}$$  
$$a_{imu} = t_{now} - t_{imu}$$

Timestamp mismatch:

$$m = t_{cam} - t_{imu*}$$

These quantify **latency** and **synchronization** independently.

## Commands executed

ROS2 sanity check:

```
source /opt/ros/humble/setup.bash
ros2 run demo_nodes_cpp talker
ros2 run demo_nodes_py listener
```

Create folders:

```
mkdir -p day36_time_latency_sync/results
```

Install dependency:

```
sudo apt install -y python3-matplotlib
```

Run experiment:

```
source /opt/ros/humble/setup.bash
python3 day36_time_latency_sync/time_latency_demo.py
```

## Results

- Message age shows system-level latency and jitter.
- Timestamp mismatch stays within milliseconds, showing reasonable sync.
- Demonstrates that freshness and synchronization are independent.

## Key takeaway

Timing is a **system design problem**, not an algorithm detail.

---