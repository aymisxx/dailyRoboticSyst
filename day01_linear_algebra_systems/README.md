# Day 1: Linear Algebra for Systems  
### **(State, Dynamics Matrix, Eigenvalues)**

---

## Today's Goal

Building **Systems-first Intuition of linear algebra in context of Contorl and Robotics**.

Key ideas:
- **Vector = State** (system's instantaneous memory)
- **Matrix = Dynamics** (rules of evolution)
- **Eigenvalues = Behavior indicators** (stable / unstable / oscillatory)

## 1) Core Concepts

### 1.1 State as a Vector

A system's **state** = minimum information by which future prediction is possible.

$Example:$ 

1D motion (mass-like system)

$$
x(t) =
\begin{bmatrix}
p(t) \\
v(t)
\end{bmatrix}
$$

- $p(t)$: position  
- $v(t)$: velocity  

**Interpretation:**

The state vector is not a collection of random numbers.
It represents the system’s complete instantaneous memory snapshot.

Once the state is known at a given time, the past is no longer required, 
the system dynamics uniquely determine how the state will evolve in the future.

---

### 1.2 Dynamics as a Matrix

Linear time-invariant (LTI) continuous system:

$$
\dot{x}(t) = A x(t)
$$

Where:
- $$x(t)$$: state vector  
- $$A$$: system matrix  

For a 2-state system:

$$
A =
\begin{bmatrix}
a_{11} & a_{12} \\
a_{21} & a_{22}
\end{bmatrix}
$$

Each element ka physical meaning hota hai:
- $a_{12}$: contribution of velocity to the rate of change of position  
- $a_{21}$: contribution of position to acceleration  
- diagonal terms: self-influence of each state (damping / growth)

**Key idea:**  
Matrix $A$ is a compact **physics rulebook** describing how states interact.

---

### 1.3 Eigenvalues = Behavior Indicators

Eigenvalue relation:

$$
A v = \lambda v
$$

- $v$: eigenvector (special direction in state space)  
- $\lambda$: eigenvalue (growth / decay rate along that direction)

For continuous-time systems $\dot{x} = A x$:

- **Re($\lambda$) < 0** → trajectories decay → **stable**
- **Re($\lambda$) > 0** → trajectories blow up → **unstable**
- **Complex eigenvalues** $a \pm jb$:
  - $b$: oscillation frequency
  - $a$: decay / growth envelope

**Important line (system intuition):**
> A system’s natural tendencies are encoded in the eigenvalues of its system matrix.

## 2) Math-to-Physics Example Used

A 2-state second-order style system was used:

$$
\dot{x} = A x,\quad
A =
\begin{bmatrix}
0 & 1 \\
-\omega^2 & -2\zeta\omega
\end{bmatrix}
$$

with:
$$
x =
\begin{bmatrix}
p \\
v
\end{bmatrix}
$$

This resembles a **mass-spring-damper** like structure (scaled form), where:
- $\omega$: natural frequency  
- $\zeta$: damping ratio  

This form is common in control and robotics.

## 3) Implementation (Python)

In `demo.py`, the following was done:

1. Defined system matrix $A$
2. Computed eigenvalues and eigenvectors
3. Simulated system using forward Euler integration:
   $$
   x_{k+1} = x_k + \Delta t (A x_k)
   $$
4. Generated plots:
   - **State vs time**: $p(t)$, $v(t)$
   - **Phase portrait**: velocity vs position
   - **Trajectory + eigenvector directions**

Plots are saved in the `results/` directory.

## 4) Results & Discussion

### 4.1 Eigenvalues and Stability

The computed eigenvalues were **complex with negative real parts**.

Interpretation:
- Complex part → oscillatory behavior
- Negative real part → energy decay

This directly predicts a **stable, underdamped system**.

### 4.2 State vs Time Plot

Observed behavior:
- Position and velocity oscillate
- Amplitude decays over time
- Position and velocity are phase-shifted

This matches exactly what is expected from an underdamped second-order system.

### 4.3 Phase Portrait

The phase portrait shows:
- A **spiral trajectory**
- Spiraling inward toward the origin $(p=0, v=0)$

Interpretation:
- Origin is a **stable equilibrium**
- Each loop corresponds to one oscillation
- Shrinking radius indicates damping

This visually confirms eigenvalue-based stability analysis.

### 4.4 Eigenvector Overlay

Eigenvectors were overlaid for intuition.

Observation:
- Trajectory does not align perfectly with a single eigenvector

Reason:
- For oscillatory systems, eigenvalues and eigenvectors are complex
- No single real direction dominates the motion

This is expected and does not indicate an error.

## 5) What Was Tricky / Learned

- Forward Euler integration is simple but sensitive to step size
- Large `dt` can introduce numerical instability even for stable systems
- Eigenvectors represent **directions**, not magnitudes (scaling is irrelevant)

---