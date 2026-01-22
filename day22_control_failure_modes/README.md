# Day 22: Control Failure Modes  

## **Integrator Windup and Anti-Windup**

This experiment demonstrates **one of the most common real-world control failures: integrator windup**, and shows how **anti-windup compensation** fixes it.

---

## Failure Mode Studied: Integrator Windup

### What is integrator windup?
When a controller includes an integral term and the actuator **saturates**, the integrator continues accumulating error even though the actuator cannot respond.  
Once the system leaves saturation, the stored integral causes **large overshoot, instability, or complete loss of regulation**.

This is not a tuning issue, it is a **structural failure**.

## System Setup

- **Plant**: 1D double integrator  
  - State: position, velocity  
  - Control: bounded acceleration

- **Controller**: PID on position error

- **Actuator constraint**:
  $-1 ≤ u ≤ 1$

Two cases are compared:

1. **PID + saturation (windup)**  
2. **PID + saturation + anti-windup (back-calculation)**

## Anti-Windup Method

Back-calculation is used to prevent integrator divergence:

$$İ = e + (u_{sat} - u_{unsat}) / K_{aw}$$

When the actuator saturates, this term actively **unwinds** the integrator so it remains consistent with what the actuator can actually deliver.

## Results

### Saturated Control

- Windup case remains pinned at actuator limits.
- Anti-windup backs off smoothly once recovery is possible.

### Integrator State (Key Evidence)

- Windup integrator diverges to very large values.
- Anti-windup integrator remains bounded and converges.

This plot alone explains the entire failure.

### Position Response

- Windup causes runaway overshoot and loss of regulation.
- Anti-windup yields controlled overshoot and convergence.

### Velocity Response

- Windup stores excessive kinetic energy.
- Anti-windup limits energy buildup and stabilizes the system.

## Key Takeaways

- Integral action **must** be paired with saturation awareness.
- Clipping the control signal alone does not prevent failure.
- Integrator windup is a structural problem, not a tuning problem.
- Anti-windup is mandatory in real actuated systems.

> **Controllers fail when assumptions about actuation are violated.**

## Files

day22_control_failure_modes/  
├── day22_control_failure_modes.py  
├── results/  
│   ├── day22_control.png  
│   ├── day22_integrator.png  
│   ├── day22_position.png  
│   └── day22_velocity.png  
└── README.md

Control phase complete.  
Failure mode understood, not memorized.  

---