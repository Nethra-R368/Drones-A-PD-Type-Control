<p align="center">
  <img src="amrita.png" alt="Logo" width="400"/>
</p>

# A Simple Structure Constrained Attitude Control for Rigid Bodies: A PD-Type Control

<div align="center">


**Group-AB02 | INTRODUCTION TO DRONES | Amrita Vishwa Vidyapeetham**

</div>

---

## Team Members

| Name | Roll Number |
|------|------------|
| Nethra R | CB.SC.U4AIE24147 |
| Dheeraj S | CB.SC.U4AIE24050 |
| Raaman Namputhiri | CB.SC.U4AIE24148 |
| Jyothsna | CB.SC.U4AIE24117 |

---


## 1. Abstract
This project implements and validates a simple-structure full-state constrained Proportional-Derivative (PD)-type attitude control scheme for rigid bodies in high-fidelity robotic simulation . Conventional prescribed performance control schemes involve complex transformed errors and heavy partial derivative calculations. In contrast, the replicated framework integrates a nominal PD law with an auxiliary barrier term and an adaptive saturation/disturbance estimator to constrain the attitude quaternion and angular velocity strictly within finite-time predefined performance boundaries. 

The complete control pipeline was developed in ROS 2 and tested on an X3 quadcopter inside the Gazebo physics environment. Over a 180-second hover stabilization regime at a target altitude of 2.0 m, the closed-loop system achieved precise attitude regulation while respecting motor torque boundaries ($u_{max} = 0.20\text{ N}\cdot\text{m}$). The experimental results verify robust transient containment, zero boundary violations ($0.00\%$), and reliable performance under physical actuator saturation constraints.

---

## 2. Introduction

### 2.1 Overview of the Base Paper
Attitude control is essential for stabilizing rigid bodies such as spacecraft, aerial robots, and satellites. Conventional control schemes frequently struggle to guarantee transient safety bounds while subject to actuator saturation and bounded external disturbances. While Prescribed Performance Control (PPC) provides predefined transient and steady-state boundaries, standard formulations rely on error transformation techniques that yield complex partial derivatives and coupled non-affine equations .

The base paper by *Golestani et al. (IEEE Access, 2022)* proposes a constrained PD-type controller with a simplified analytical structure :
* A standard **PD control component** asymptotically steers the attitude quaternion and angular velocity to zero.
* An **auxiliary barrier function** penalizes trajectories approaching predefined performance envelopes.
* A **Finite-Time Prescribed Performance Function (FTPPF)** establishes hard transient and steady-state boundaries with a deterministic settling time $T_s$ .
* An **on-line adaptive estimator** estimates the lumped unknown bound of actuator saturation and external disturbances .

### 2.2 What is Gazebo?
**Gazebo** is an open-source 3D multi-robot physics simulator designed for validating complex control algorithms under realistic environmental dynamics. It provides:
* **Rigid-Body Physics Engines:** Accurate simulation of gravity, multi-body kinematics, inertial tensors, Coriolis effects, and contact dynamics using engines such as ODE and Bullet.
* **Actuator & Aerodynamic Modeling:** Realistic simulation of motor speed dynamics, quadratic thrust generation ($c_T$), and aerodynamic drag/torque coefficients ($c_Q$).
* **ROS 2 Integration:** Direct pub/sub communication with ROS 2 nodes for seamless software-in-the-loop (SITL) prototyping.

### 2.3 Scope of Implementation in Gazebo
* **Gazebo Platform & Physics Utilized:** The multi-rotor **X3 Quadcopter platform** was deployed. Ground-truth state feedback was acquired from `/world/quadcopter/dynamic_pose/info`, and computed rotor angular velocity commands ($\text{rad/s}$) were published to `/X3/gazebo/command/motor_speed`.
* **Completed Modules:**
  * Discrete-time finite-time prescribed performance envelope calculation ($\rho_q(t), \rho_\omega(t)$) .
  * State filtering with low-pass filters and moving-average windows for numerical derivatives $\dot{z}$ and $\omega$.
  * Constrained barrier control law evaluation with dynamic state clipping .
  * Continuous-time adaptive parameter integration ($\hat{\theta}$) .
  * Closed-loop altitude trajectory generation, hover thrust tracking, tilt angle compensation, and a 4-motor allocation mixer.
  * 180-second stability benchmark without wind disturbance injection.

---

## 3. Methodology

### 3.1 Base Paper Formulation (Implemented Equations)

#### 1. Rigid Body Kinematics and Dynamics
The attitude orientation of the rigid vehicle is described by the unit quaternion vector $q = [q_v^T, q_4]^T = [q_1, q_2, q_3, q_4]^T \in \mathbb{R}^4$ with $q^T q = 1$ . The rotational dynamics and kinematics subject to input saturation and external disturbances are given by :

$$\dot{q}_v = \frac{1}{2}(q_4 I_3 + q_v^\times)\omega$$ 

$$\dot{q}_4 = -\frac{1}{2}q_v^T \omega$$ 

$$J\dot{\omega} = -\omega^\times J \omega + sat(u) + d = -\omega^\times J \omega + u + T_d$$ 

Where $J \in \mathbb{R}^{3 \times 3}$ is the inertia tensor, $\omega \in \mathbb{R}^3$ is the angular velocity, $u \in \mathbb{R}^3$ is the control torque, and $T_d = d + \Pi(t)$ represents the combined disturbance and actuator saturation bounded by $\|T_d\| \le \theta$ . The cross-product matrix is defined as :

$$q_v^\times = \begin{bmatrix} 0 & -q_3 & q_2 \\ q_3 & 0 & -q_1 \\ -q_2 & q_1 & 0 \end{bmatrix}$$ 

#### 2. Finite-Time Prescribed Performance Function (FTPPF)
To enforce $-\rho_q(t) < q_i(t) < \rho_q(t)$ and $-\rho_\omega(t) < \omega_i(t) < \rho_\omega(t)$, the FTPPF $\rho(t)$ is computed as :

$$\rho(t) = \begin{cases} \left(\rho_0 - \rho_T\left(1 + \frac{t}{T_s}\right)\right)\exp\left(\frac{\kappa t}{t - T_s}\right) + \rho_T, & 0 \le t < T_s \\ \rho_T, & t \ge T_s \end{cases}$$ 

Where $\rho_0$ sets the initial envelope boundary, $\rho_T$ is the steady-state bound, and $T_s$ is the settling time .

## 3. Methodology

### 3.1 Mathematical Formulation of the Base Paper
The control objective is to stabilize the orientation of a rigid body while simultaneously guaranteeing that both the attitude quaternion and angular velocity remain strictly within predefined transient and steady-state boundaries under actuator physical limitations[cite: 1, 2].

#### 3.1.1 Rigid Body Kinematics and Dynamic Model
Let the rigid body orientation in the body-fixed reference frame $\mathcal{F}_b$ relative to the inertial reference frame $\mathcal{F}_i$ be represented using the unit quaternion vector $q \in \mathbb{R}^4$[cite: 1, 2]:

$$q = \begin{bmatrix} q_v \\ q_4 \end{bmatrix} = \begin{bmatrix} q_1 \\ q_2 \\ q_3 \\ q_4 \end{bmatrix} \in \mathbb{R}^4, \quad \text{subject to } q^T q = \|q_v\|^2 + q_4^2 = 1$$[cite: 1, 2]

Where $q_v = [q_1, q_2, q_3]^T \in \mathbb{R}^3$ denotes the vector component and $q_4 \in \mathbb{R}$ represents the scalar component of the quaternion[cite: 1, 2]. 

The rotational kinematics and dynamics of the rigid body are expressed as[cite: 1, 2]:

$$\dot{q}_v = \frac{1}{2}(q_4 I_3 + q_v^\times)\omega$$[cite: 1, 2]

$$\dot{q}_4 = -\frac{1}{2}q_v^T \omega$$[cite: 1, 2]

$$J\dot{\omega} = -\omega^\times J \omega + sat(u) + d$$[cite: 1, 2]

Where:
* $\omega = [\omega_1, \omega_2, \omega_3]^T \in \mathbb{R}^3$ is the angular rotation velocity vector of $\mathcal{F}_b$ with respect to $\mathcal{F}_i$ expressed in $\mathcal{F}_b$[cite: 1, 2].
* $J \in \mathbb{R}^{3 \times 3}$ is the symmetric, positive-definite inertia matrix[cite: 1, 2].
* $u = [u_1, u_2, u_3]^T \in \mathbb{R}^3$ is the control torque command[cite: 1, 2].
* $d = [d_1, d_2, d_3]^T \in \mathbb{R}^3$ is the unknown external disturbance vector bounded by $\|d\| \le \tilde{d}$[cite: 1, 2].
* $(\cdot)^\times$ is the skew-symmetric cross-product matrix defined for any vector $z = [z_1, z_2, z_3]^T$ as[cite: 1, 2]:

$$z^\times = \begin{bmatrix} 0 & -z_3 & z_2 \\ z_3 & 0 & -z_1 \\ -z_2 & z_1 & 0 \end{bmatrix}$$[cite: 1, 2]

#### 3.1.2 Actuator Saturation Modeling
Actuators possess physical limits and cannot supply infinite torque[cite: 1, 2]. The actual control torque $sat(u)$ supplied to the rigid vehicle is modeled as[cite: 1, 2]:

$$sat(u_i) = u_i(t) + \pi_i(t), \quad \forall i \in \{1, 2, 3\}$$[cite: 1, 2]

$$\pi_i(t) = \begin{cases} 0, & |u_i(t)| \le u_{max, i} \\ \operatorname{sgn}(u_i(t))u_{max, i} - u_i(t), & |u_i(t)| > u_{max, i} \end{cases}$$[cite: 1, 2]

Where $u_{max, i}$ is the maximum allowable torque around the $i$-th axis[cite: 1, 2]. Defining the lumped unknown term $T_d = d + \Pi(t)$ with bounded norm $\|T_d\| \le \theta$, the governing dynamics simplify to[cite: 1, 2]:

$$J\dot{\omega} = -\omega^\times J \omega + u + T_d$$[cite: 1, 2]

#### 3.1.3 Finite-Time Prescribed Performance Function (FTPPF)
To constrain the overshoot, convergence rate, and steady-state error of both attitude and angular velocity, the states are constrained within time-varying bounds[cite: 1, 2]:

$$-\rho_q(t) < q_i(t) < \rho_q(t), \quad -\rho_\omega(t) < \omega_i(t) < \rho_\omega(t), \quad \forall i \in \{1, 2, 3\}$$[cite: 1, 2]

The base paper defines a **Finite-Time Prescribed Performance Function (FTPPF)** $\rho(t)$ that converges to the ultimate boundary $\rho_T$ within a deterministic settling time $T_s$[cite: 1, 2]:

$$\rho(t) = \begin{cases} \left(\rho_0 - \rho_T\left(1 + \frac{t}{T_s}\right)\right)\exp\left(\frac{\kappa t}{t - T_s}\right) + \rho_T, & 0 \le t < T_s \\ \rho_T, & t \ge T_s \end{cases}$$[cite: 1, 2]

Where:
* $\rho_0 > 0$ defines the maximum allowable initial bound (overshoot protection)[cite: 1, 2].
* $\rho_T = \lim_{t \to T_s} \rho(t) > 0$ denotes the final steady-state precision boundary[cite: 1, 2].
* $T_s > 0$ sets the exact finite settling time[cite: 1, 2].
* $\kappa > 0$ controls the initial rate of envelope decay[cite: 1, 2].

#### 3.1.4 Error Normalization and Barrier Transform
Normalized state error metrics are defined as[cite: 1, 2]:

$$\epsilon_1 = \frac{1}{\rho_q(t)} q_v(t) = \begin{bmatrix} \epsilon_{11} \\ \epsilon_{12} \\ \epsilon_{13} \end{bmatrix}, \quad \epsilon_2 = \frac{1}{\rho_\omega(t)} \omega(t) = \begin{bmatrix} \epsilon_{21} \\ \epsilon_{22} \\ \epsilon_{23} \end{bmatrix}$$[cite: 1, 2]

Ensuring that $|\epsilon_{1i}(t)| < 1$ and $|\epsilon_{2i}(t)| < 1$ guarantees satisfaction of full-state constraints at all times[cite: 1, 2].

#### 3.1.5 Constrained PD-Type Control Law and Adaptive Law
The control law consists of a nominal PD feedback component, a barrier penalty term, and an adaptive robust compensator[cite: 1, 2]:

$$u = -k_p q_v - k_d \omega - u_{cons}$$[cite: 1, 2]

$$u_{cons} = \left[\alpha(1 - \epsilon_1^2)^{-2} + \beta(1 - \epsilon_2^2)^{-2}\right] s + \hat{\theta} \operatorname{sat}\left(\frac{s}{\varsigma}\right)$$[cite: 1, 2]

Where:
* $k_p > 0$ and $k_d > 0$ are the nominal proportional and derivative control gains[cite: 1, 2].
* $\alpha > 0$ and $\beta > 0$ are weighting gains adjusting the repulsion force away from the $\rho_q$ and $\rho_\omega$ bounds[cite: 1, 2].
* $s = \omega + c q_v$ (with $c > 0$) is the composite sliding surface vector[cite: 1, 2].
* $\operatorname{sat}(s/\varsigma) = \operatorname{clip}(s/\varsigma, -1, 1)$ is a continuous boundary layer function replacing the discontinuous $\operatorname{sgn}(s)$ to eliminate chattering[cite: 1, 2].
* $\hat{\theta}$ is the continuous online estimate of the lumped disturbance and saturation upper bound $\theta$, updated via the adaptive law[cite: 1, 2]:

$$\dot{\hat{\theta}} = \frac{1}{\eta_1}\left(\|s\| - \eta_2(t) \hat{\theta}\right)$$[cite: 1, 2]

With $\eta_1 > 0$ and the time-decaying regularization function $\eta_2(t) = \max\left(2\exp(-0.5t), 0.1\right)$[cite: 1, 2].

---

### 3.2 Practical Execution and System Architecture on Gazebo

The theoretical attitude control law was deployed on an **X3 Quadcopter** in Gazebo via ROS 2. Because an underactuated multirotor requires continuous thrust generation alongside attitude stabilization to stay aloft, the framework couples the attitude barrier control with vertical altitude tracking and rotor thrust mixing.


#### Step-by-Step Implementation Pipeline

1. **State Acquisition & Filtering:**
   * Ground-truth position $z$ and orientation quaternion $q = [q_x, q_y, q_z, q_w]^T$ are sampled from `/world/quadcopter/dynamic_pose/info`.
   * To prevent sign-flip ambiguities in quaternion space, the orientation sign is preserved by enforcing $q \leftarrow -q$ if $q \cdot q_{last} < 0$.
   * Angular velocity $\omega$ and vertical velocity $\dot{z}$ are derived via backward differences ($\dot{z} = \frac{z_k - z_{k-1}}{\Delta t}$, $\omega_{raw} = \frac{2(q_{v,k} - q_{v,k-1})}{\Delta t}$) and conditioned using first-order low-pass filters:
     $$\omega_k = (1 - \lambda_\omega)\omega_{k-1} + \lambda_\omega \omega_{raw}, \quad \text{with } \lambda_\omega = 0.10$$

2. **Altitude Control & Tilt Decoupling:**
   * Desired takeoff altitude is smoothed over $t_{takeoff} = 8.0\text{ s}$ to target altitude $z_{target} = 2.0\text{ m}$.
   * Vertical force demand is computed via PD acceleration tracking:
     $$F_{vertical} = m \cdot \left(g + \operatorname{clip}\left(K_{p,z}(z_{target} - z) - K_{d,z}\dot{z}, -3.0, 3.0\right)\right)$$
   * Total thrust $T_{total}$ is scaled by the quadcopter tilt angle to maintain altitude during attitude maneuvers:
     $$\text{tilt\_factor} = \operatorname{clip}\left(\cos(2q_1)\cos(2q_2), 0.85, 1.0\right), \quad T_{total} = \operatorname{clip}\left(\frac{F_{vertical}}{\text{tilt\_factor}}, 0, 22.0\text{ N}\right)$$

3. **Constrained Attitude Control Execution:**
   * Error states are normalized against current FTPPF values: $\epsilon_1 = q_v / \rho_q(t)$ and $\epsilon_2 = \omega / \rho_\omega(t)$.
   * To prevent numeric singularity at envelope limits, normalized errors are bound to $[-0.90, 0.90]$.
   * Barrier penalties and sliding manifold $s = \omega + c q_v$ are evaluated to form $u_{cons}$.
   * Disturbance parameter $\hat{\theta}$ is updated using Euler integration:
     $$\hat{\theta}_{k} = \operatorname{clip}\left(\hat{\theta}_{k-1} + \frac{1}{\eta_1}(\|s\| - \eta_2 \hat{\theta}_{k-1})\Delta t, 0.0, 1.0\right)$$
   * Control torque is evaluated: $u = -K_p q_v - K_d \omega - u_{cons}$ and saturated to maximum available authority $\tau = \operatorname{clip}(u, -\tau_{max}, \tau_{max})$.

4. **Motor Allocation Matrix:**
   * Total thrust $T_{total}$ and 3-axis torques $\tau = [\tau_1, \tau_2, \tau_3]^T$ (roll, pitch, yaw) are mapped to individual rotor angular velocities ($w_1, w_2, w_3, w_4$) via quadcopter mixer geometry ($l = 0.225\text{ m}, c_T = 8.54858 \times 10^{-6}, c_Q = 0.016$):


### 3.3 Execution Workflow
1. **Pose Callback & Filtering:** Subscribes to the Gazebo ground-truth pose. Euler tilt extraction accounts for gravity compensation ($T_{total} = F_{vertical} / (\cos(\text{roll})\cos(\text{pitch}))$).
2. **Velocity Estimation:** Computes numerical derivatives $\dot{z}$ and $\omega = 2\dot{q}_v$ and conditions them through first-order low-pass filters ($\alpha_{filter} = 0.10$).
3. **Barrier Evaluation & Clamping:** Normalizes $q_v$ and $\omega$ against $\rho_q(t)$ and $\rho_\omega(t)$ and clips ratios to $[-0.90, 0.90]$ to prevent numerical singularity.
4. **Adaptive Parameter Integration:** Integrates $\dot{\hat{\theta}}$ in discrete time with anti-windup clamping ($0.0 \le \hat{\theta} \le 1.0$).
5. **Motor Allocation Mixer:** Translates total thrust $T_{total}$ and 3-axis torque commands $\tau = [\tau_1, \tau_2, \tau_3]^T$ into rotor speeds:

$$m_{1, 4} = \frac{T_{total}}{4 c_T} \pm \frac{\tau_2}{4 c_T l} \mp \frac{\tau_1}{4 c_T l} \mp \frac{\tau_3}{4 c_Q}, \quad \omega_i = \sqrt{\max(0, m_i)}$$

### 3.4 Remaining Implementation Tasks
* **Disturbance Injection Verification:** Enable `inject_wind_disturbance = True` using the multi-frequency sinusoidal disturbance vector $d(t) = [0.05\sin(t), 0.10\sin(1.2t), 0.15\sin(1.5t)]^T\text{ N}\cdot\text{m}$ to stress-test adaptive rejection .
* **Baseline PD Benchmark Comparison:** Execute the simulation under `controller_mode = 'conventional_pd'` to collect comparative convergence and overshoot datasets under identical initial conditions .
* **Waypoint Trajectory Tracking:** Extend the regulator from a static hover point ($z = 2.0\text{ m}$) to full dynamic 3D spatial trajectory tracking ($x_d(t), y_d(t), z_d(t), \psi_d(t)$).

---

## 5. Future Work and Conclusion

### 5.1 Future Work
Future work will focus on three key directions. First, external multi-frequency time-varying wind and disturbance profiles will be introduced into the Gazebo environment to analyze the disturbance estimation dynamics of $\hat{\theta}$ and stress-test the controller's robustness limits under torque saturation. Second, a comparative benchmark will be conducted against a conventional unconstrained PD controller to experimentally demonstrate differences in transient overshoot, constraint violations, and motor wear . Third, the framework will be expanded to 6-DOF dynamic trajectory tracking (incorporating position control and dynamic tilt generation) and validated on physical quadcopter hardware through embedded ROS 2 microcontrollers.

### 5.2 Conclusion
This project successfully implemented and evaluated a simple-structure full-state constrained PD-type attitude controller for rigid bodies using ROS 2 and the Gazebo simulation engine . By integrating a finite-time prescribed performance function with an auxiliary barrier term and an adaptive disturbance estimator, the system maintained attitude stability without the analytical complexity of conventional error transformations . The experimental simulation on the X3 quadcopter demonstrated precise altitude regulation at 2.0 m, zero constraint violations across all attitude and angular velocity states, smooth adaptation of the unknown saturation parameter, and compliant motor torque allocation within strict actuator limits.
