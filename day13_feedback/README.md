# Day 13: Feedback

## What this day is about
This day introduces the idea of **feedback**, the core concept behind all control systems.

Feedback means the system:
- observes its own output,
- compares it with what is desired,
- and reacts based on the **error**.

Nothing more. Nothing less.

---

## Open-loop vs Closed-loop

**Open-loop system**
- Acts without observing the result.
- No correction.
- Assumes the world is perfect.

**Closed-loop system (with feedback)**
- Measures output.
- Computes error.
- Adjusts input continuously.

Most real systems must be closed-loop to survive.

## Error as the key signal

Let:
- `r` = desired value (reference)
- `y` = actual output

Error is defined as:
```
e = r − y
```

The controller does not act on the system directly.
It reacts only to the **error**.

## Minimal feedback law

The simplest possible feedback rule:
```
u = K · e
```

Where:
- `u` is the control input,
- `K` is a gain deciding how aggressively the system reacts.

This single equation is the foundation of:
- P control
- PID control
- State feedback
- LQR
- MPC

Everything builds on this idea.

## Key insights

- Feedback is a **concept**, not a controller.
- Feedback does **not guarantee stability**.
- Increasing gain increases response, but also risk.
- Control is fundamentally about **reacting to mistakes**, not eliminating them.

---