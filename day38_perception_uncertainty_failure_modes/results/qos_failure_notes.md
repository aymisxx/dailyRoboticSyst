# QoS mismatch failure mode (Day 38)

## Baseline
- talker publishes `/chatter` at ~1 Hz with low jitter. (see `results/chatter_hz.txt`) :contentReference[oaicite:0]{index=0}
- Publisher QoS: RELIABLE; CLI subscriber often shows BEST_EFFORT. (see `results/chatter_qos_baseline.txt`) :contentReference[oaicite:1]{index=1}

## Failure induced
- Started a publisher with BEST_EFFORT reliability (see `results/pub_best_effort.log`).
- Ran subscriber demanding RELIABLE:
  - Result: no messages received + warning: "incompatible QoS ... RELIABILITY"
- Verified mismatch via `ros2 topic info -v /chatter` (publisher BEST_EFFORT; subscriber RELIABLE present). (see `results/chatter_qos_mismatch.txt`) :contentReference[oaicite:2]{index=2}

## Fix
- Ran subscriber with BEST_EFFORT:
  - Result: messages immediately received. (see `results/echo_best_effort_against_best_effort.txt`) :contentReference[oaicite:3]{index=3}

## Why this matters (systems view)
This is a "silent perception failure": the sensor/topic exists and is publishing, but the consumer receives *nothing* due to QoS contract mismatch. Downstream, filters/controllers may operate on stale data, freeze, or fail in non-obvious ways.