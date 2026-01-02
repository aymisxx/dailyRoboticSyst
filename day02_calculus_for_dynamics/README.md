# Day 02: Calculus for Dynamics (Numerical)

---

**Theme:** Change is the language of dynamical systems.

The goal of this day is to build **intuition-first understanding** of calculus as it appears in control and robotics, not as symbolic math, but as **numerical operations that drive simulations**.

## Core Ideas

### 1. Derivative = Rate of Change
A derivative answers a simple system-level question:

> “How fast is the state changing right now?”

In physical and control systems:
- Position → velocity
- Velocity → acceleration
- State → state-rate

Derivatives react **instantly** to changes and amplify fast variations.  
This is why derivative terms are powerful but sensitive to noise.

### 2. Integral = Accumulation / Memory
An integral answers a different question:

> “What happens when small changes keep adding up over time?”

In systems:
- Velocity integrated over time gives position
- Biases and offsets accumulate
- Integrals never forget the past

This is why integrals eliminate steady-state error, but also why they can cause drift or windup.

## What I Implemented

A single Python script that demonstrates both concepts numerically:

1. **Numerical differentiation**
   - Uses `numpy.gradient` to compute `dx/dt`
   - Shows how derivatives respond immediately to signal changes

2. **Numerical integration (Euler method)**
   - Integrates a velocity signal to recover position
   - Includes a constant bias to explicitly show drift

All computations are done numerically, reflecting how real simulators and control systems operate.

## Mathematical Form (Minimal)

Numerical derivative (conceptual): $dx/dt ≈ (x[k+1] − x[k]) / Δt$

Euler integration: $x[k+1] = x[k] + x_dot[k] · Δt$

## Files

```
day02/
├── numerical_calculus.py
├── README.md
└── results/
    ├── 01_signal_and_derivative.png
    └── 02_velocity_and_position_integral.png
```

## Results & Observations

### Derivative plot
- The derivative is a cosine-like signal shifted upward due to the linear trend in `x(t)`
- Confirms that derivatives amplify change and react instantly

### Integral plot
- The integrated position shows oscillations plus a steady upward drift
- The drift is caused by a constant bias in velocity
- Demonstrates that integrals act as **memory** and accumulate even small offsets

## Key Takeaways

- Derivatives describe **how fast** a system is changing
- Integrals describe **how much history** a system carries
- Numerical calculus is how real-world simulations actually work
- Bias + integration = drift (always)

## How to Run

```bash
python numerical_calculus.py
```

Plots are saved automatically to the `results/` directory.

---