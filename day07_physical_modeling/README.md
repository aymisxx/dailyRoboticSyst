# Day 07: Physical Modeling (Mass-Spring-Damper)

## Objective

The goal of this day is to understand **how physical systems are modeled**, not just simulated.

Instead of starting from abstract equations, we begin from **physics**:
- forces
- energy
- inertia
- dissipation

and derive the system dynamics from first principles.

This day focuses on the **mass-spring-damper system**, a canonical model that appears across:
- robotics
- vehicle dynamics
- actuators
- compliant mechanisms
- control theory examples

---

## Why Physical Modeling Matters

Control and estimation algorithms assume a model.

If the model:
- has wrong signs,
- ignores energy flow,
- or misrepresents system memory,

then **no controller can save it**.

Physical modeling answers:
- What are the system states?
- Where does memory come from?
- Why does the system oscillate?
- Why does it settle (or not)?

## System Description

We consider a 1D mass-spring-damper system subjected to an external force input.

### Components
- **Mass (m):** inertia, resists acceleration
- **Spring (k):** stores potential energy, provides restoring force
- **Damper (c):** dissipates energy, models friction / losses
- **Input (u):** external force applied to the mass

## Governing Equation (Newton’s Law)

Applying Newton’s second law:

$$
m\ddot{x} + c\dot{x} + kx = u(t)
$$

## State-Space Formulation

Define states based on physical meaning:

$$
x_1 = x, \quad x_2 = \dot{x}
$$

State equations:

$$
\dot{x}_1 = x_2
$$

$$
\dot{x}_2 = \frac{1}{m}(u - c x_2 - k x_1)
$$

## Simulation Setup

- m = 1.0  
- c = 0.5  
- k = 4.0  

Initial condition:

$$
x(0) = 1.0, \quad \dot{x}(0) = 0.0
$$

Time horizon: 0-10 seconds  
Integration: `scipy.integrate.solve_ivp`

## Experiments

### Free Response (u = 0)
- Decaying oscillations
- Stable equilibrium at origin

### Forced Response (u = 1)
- Transient oscillations
- Steady-state at $x_{ss} = u/k = 0.25$

### Phase Plot
- Inward spiral
- Energy dissipation visible

## Results

```
results/
├── free_response.png
├── forced_response.png
└── phase_plot.png
```

## Folder Structure

```
day07_physical_modeling/
├── src/
│   ├── mass_spring_damper.py
│   └── simulate.py
├── results/
└── README.md
```

## Key Takeaways

- Physical modeling precedes control.
- States encode memory.
- Damping governs stability.
- Inputs shift equilibria.

Day 07 builds intuition for everything that follows.

---