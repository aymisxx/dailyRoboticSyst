# Day 09: Linearization of a Nonlinear Pendulum

This experiment studies **local linearization of a nonlinear pendulum** about multiple operating points and quantitatively compares nonlinear and linearized dynamics.

The goal is to **demonstrate when linearization works, when it fails, and why**, using simulation evidence, not theory alone.

---

## System Description

We consider a damped pendulum with control torque input:

$$
\dot{\theta} = \omega
$$
$$
\dot{\omega} = -\frac{g}{l}\sin(\theta) - b\,\omega + u
$$

where:

| Parameter | Value |
|--------|------|
| Gravity \(g\) | 9.81 m/s² |
| Length \(l\) | 1.0 m |
| Damping \(b\) | 0.05 |
| Time step | 0.002 s |
| Simulation time | 8.0 s |

State:
$$
x = \begin{bmatrix}\theta \\ \omega\end{bmatrix}
$$

Input:
$$
u = \text{torque}
$$

## Linearization Method

The system is linearized about an equilibrium point \((x_{eq}, u_{eq})\) using a first-order Taylor expansion:

$$
\delta\dot{x} = A\,\delta x + B\,\delta u
$$

where:

$$
A = \left.\frac{\partial f}{\partial x}\right|_{x_{eq},u_{eq}}, \quad
B = \left.\frac{\partial f}{\partial u}\right|_{x_{eq},u_{eq}}
$$

Both **analytic Jacobians** and **numerical finite-difference Jacobians** are computed and verified to match ($‖error‖ ≈ 1e-9$).

All simulations compare:
- Nonlinear dynamics
- Linearized dynamics
- **Deviation variables** $\delta\theta, \delta\omega$
- Angle wrapping is applied to avoid artificial discontinuities.

## Operating Points Studied

### 1. Upright (Small-Angle)
$$
\theta_{eq} = 0, \quad \omega_{eq} = 0, \quad u_{eq} = 0
$$

**Properties**
- Stable equilibrium
- Complex conjugate eigenvalues with negative real part
- Classic small-angle pendulum behavior

**Observed Results**
- Linear and nonlinear trajectories almost overlap
- $\delta\theta$ error remains below ~0.008 rad over 8 seconds
- Confirms excellent local validity of linearization

### 2. Tilted Equilibrium
$$
\theta_{eq} = 0.6 \text{ rad}, \quad \omega_{eq} = 0
$$
$$
u_{eq} = \frac{g}{l}\sin(\theta_{eq})
$$

**Properties**
- Stable equilibrium at non-zero angle
- Requires constant equilibrium torque
- Eigenvalues remain stable

**Observed Results**
- Linear model accurately tracks nonlinear dynamics
- $\delta\theta$ error remains bounded within ~±0.025 rad
- Demonstrates linearization validity **away from zero angle**

### 3. Near-Inverted Equilibrium
$$
\theta_{eq} = \pi, \quad \omega_{eq} = 0, \quad u_{eq} = 0
$$

**Properties**
- Unstable equilibrium
- One positive real eigenvalue
- Exponentially diverging linear modes

**Observed Results**
- Linear deviations grow rapidly and leave local region
- Linear model is **explicitly clipped** once local validity is lost
- Nonlinear system naturally falls and wraps angle
- Large jumps in $\delta\theta$ arise from angle periodicity, not numerical error

**Key Insight**
> Linearization is only valid locally, unstable equilibria destroy linear models almost immediately.

## Key Takeaways

- Linearization is **local**, not global
- Stability depends entirely on the chosen operating point
- Non-zero equilibrium inputs must be included correctly
- Angle wrapping is essential for meaningful comparison
- Unstable equilibria must be handled carefully and honestly

This experiment provides a **clear, simulation-based demonstration** of all the above principles.

## Output Plots

For each operating point:
- $\delta\theta$: nonlinear vs linearized
- $\delta\omega$: nonlinear vs linearized
- Linearization error in $\delta\theta$
- Applied input $u(t)$

All plots are saved in the `results/` directory.

## Conclusion

This study confirms classical control theory results using direct numerical evidence:
- Linearization works extremely well near stable equilibria
- Accuracy degrades gracefully due to higher-order nonlinearities
- Linear models fail rapidly near unstable equilibria, as they should

This forms the foundation for future topics such as:
- Stability analysis
- LQR design
- Gain scheduling
- Nonlinear control and swing-up strategies

---