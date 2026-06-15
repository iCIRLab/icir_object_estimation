# icir_object_estimation

Online estimation of unknown object parameters (mass, center of mass, inertia tensor) using an Extended Kalman Filter (EKF), based on Newton-Euler equations of the held object.

Used in [icir_phri_panda_husky](https://github.com/iCIRLab/icir_phri_panda_husky) to enable object-aware impedance control during human-robot collaborative transportation.

---

## Method

The object parameters are estimated by treating them as a constant state vector and using the measured end-effector force/torque as observations.

**State vector** (10 parameters):

```
x = [m, cx, cy, cz, Ixx, Ixy, Ixz, Iyy, Iyz, Izz]
```

| Symbol | Description |
|--------|-------------|
| `m` | Object mass [kg] |
| `cx, cy, cz` | Center of mass in EE frame [m] |
| `Ixx, ..., Izz` | Inertia tensor elements [kg·m²] |

**Observation model** (Newton-Euler equations):

```
F = m(ar + α×r_com + ω×(ω×r_com) - g)
T = Ir·α + ω×(Ir·ω) + m·r_com×(ar - g)
```

where `ar`, `α`, `ω` are the EE linear acceleration, angular acceleration, and angular velocity, and `Ir = I + m·[r_com]×[r_com]` is the inertia tensor about the EE frame origin.

The Jacobian `H` of the observation model is computed via numerical differentiation (step size = 1e-8).

---

## Package Structure

```
icir_object_estimation/
├── include/
│   └── icir_object_estimation/
│       ├── main/
│       │   ├── extendedkalman.hpp   # EKF class
│       │   └── kalman.hpp           # Linear Kalman filter class
│       └── objdyn/
│           └── object_dynamics.hpp  # Object dynamics model (h, H)
└── src/
    ├── main/
    │   ├── extendedkalman.cpp
    │   └── kalman.cpp
    ├── objdyn/
    │   └── object_dynamics.cpp
    └── test/
        ├── ekf-test.cpp
        └── kalman-test.cpp
```

---

## API

### EKF

```cpp
#include "icir_object_estimation/main/extendedkalman.hpp"

EKF ekf(dt, A, H, Q, R, P, h);
ekf.init(t0, x0);

// Update at each timestep with new observation y (measured F/T)
ekf.update(y, dt, A, H, h);

Eigen::VectorXd x = ekf.state(); // [m, cx, cy, cz, Ixx, Ixy, Ixz, Iyy, Iyz, Izz]
```

### Object Dynamics

```cpp
#include "icir_object_estimation/objdyn/object_dynamics.hpp"

Objdyn obj;
VectorXd h_vec = obj.h(param, vel, acc, g);   // predicted F/T (6x1)
MatrixXd H_mat = obj.H(param, vel, acc, g);   // Jacobian (6x10)
```

---

## Output

Estimated parameters are published as [`icir_phri_msgs/ObjectParameter`](https://github.com/iCIRLab/icir_phri_msgs):

| Field | Description |
|-------|-------------|
| `mass` | Estimated object mass [kg] |
| `com` | Estimated center of mass [x, y, z] [m] |
| `inertia` | Estimated inertia tensor (6 elements) [kg·m²] |
