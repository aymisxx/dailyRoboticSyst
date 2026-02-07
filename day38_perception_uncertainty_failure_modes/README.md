# Day 38: Perception Uncertainty & Failure Modes
### ROS2 Case Study: QoS Mismatch → Silent Data Starvation

## Goal
Demonstrate that perception failure is not always caused by noisy sensors or bad models.
Sometimes, the **system contract itself breaks**, resulting in *zero data* despite active publishers.

This day focuses on a **real ROS2 failure mode**: **QoS incompatibility**.

---

## Experimental Setup
- ROS2 distro: **Humble**
- Two terminals used:
  - **Terminal 1**: Publisher / system under test
  - **Terminal 2**: Inspection & subscriber tools
- Topic under study: `/chatter` (`std_msgs/msg/String`)

All raw command outputs are saved under `results/`.

## Evidence Collected

### Active topics
See `results/topic_list.txt`

### Baseline publish rate
- `/chatter` publishes at ~1 Hz
- Low jitter (~1–2 ms)

Evidence: `results/chatter_hz.txt`

This establishes a **healthy signal reference**.

### Baseline QoS snapshot
- Publisher (`talker`): **RELIABLE**
- CLI subscriber: **BEST_EFFORT**

Evidence: `results/chatter_qos_baseline.txt`

Despite mixed QoS, communication succeeds in this baseline case.

## Failure Mode Demonstrated
### QoS mismatch ⇒ silent perception failure

### Step 1: Induce mismatch
A new publisher was started using **BEST_EFFORT** reliability.

Evidence: `results/pub_best_effort.log`

A subscriber was then started requesting **RELIABLE** reliability:

```
ros2 topic echo /chatter --qos-reliability reliable
```

### Observed behavior
- **No messages received**
- ROS2 warning indicating incompatible QoS (RELIABILITY)

This is a **silent failure**:
- Topic exists
- Publisher is active
- Subscriber is running
- Yet **no data flows**

### QoS mismatch confirmation
A live QoS snapshot shows:
- Publisher: **BEST_EFFORT**
- Subscriber: **RELIABLE**

Evidence: `results/chatter_qos_mismatch.txt`

## Fix Demonstrated
The subscriber QoS was changed to match the publisher:

```
ros2 topic echo /chatter --qos-reliability best_effort
```

### Result
- Messages received immediately
- System recovers without changing the publisher

Evidence: `results/echo_best_effort_against_best_effort.txt`

## Why This Is a Perception Failure (Systems View)

This failure mode is dangerous because it is **non-obvious**:
- No crashes
- No exceptions
- Nodes appear healthy
- Graph is alive

Downstream impact:
- **Estimation**: filters stop updating → stale state
- **Control**: controller acts on old data → instability
- **Planning**: decisions made without fresh perception

## Mitigation Strategies
- Define **explicit QoS contracts** per topic
- Validate QoS compatibility at startup
- Log QoS profiles alongside sensor data
- Treat QoS as part of the perception model

## ROS2 Terminal Commands Used

### Topic discovery
```
ros2 topic list
ros2 topic list | tee results/topic_list.txt
```

### Baseline publisher
```
ros2 run demo_nodes_cpp talker
```

### Rate measurement
```
ros2 topic hz /chatter
ros2 topic hz /chatter | tee results/chatter_hz.txt
```

### QoS inspection
```
ros2 topic info -v /chatter
ros2 topic info -v /chatter | tee results/chatter_qos_baseline.txt
```

### Baseline RELIABLE subscriber
```
ros2 topic echo /chatter --qos-reliability reliable
ros2 topic echo /chatter --qos-reliability reliable | tee results/echo_reliable.txt
```

### BEST_EFFORT publisher (failure induction)
```
ros2 topic pub /chatter std_msgs/msg/String "{data: 'QoS mismatch test'}" -r 5 --qos-reliability best_effort
ros2 topic pub /chatter std_msgs/msg/String "{data: 'QoS mismatch test'}" -r 5 --qos-reliability best_effort | tee results/pub_best_effort.log
```

### Failure observation
```
ros2 topic echo /chatter --qos-reliability reliable
```

### QoS mismatch snapshot
```
ros2 topic info -v /chatter | tee results/chatter_qos_mismatch.txt
```

### Fix (QoS match)
```
ros2 topic echo /chatter --qos-reliability best_effort
ros2 topic echo /chatter --qos-reliability best_effort | tee results/echo_best_effort_against_best_effort.txt
```

### Shutdown
```
Ctrl + C
```

## Key Takeaway
Perception is not truth.
It is a pipeline of **assumptions, timing, and contracts**.

Sometimes the failure is not noise or bias,
it is **zero measurement caused by incompatible system assumptions**.

---