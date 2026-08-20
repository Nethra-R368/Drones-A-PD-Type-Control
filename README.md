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
|------|-------------|
| Nethra R | CB.SC.U4AIE24147 |
| Dheeraj S | CB.SC.U4AIE24050 |
| Raaman Namputhiri | CB.SC.U4AIE24148 |
| Jyothsna | CB.SC.U4AIE24117 |

---

## 1. Abstract

This project implements and validates a simple-structure full-state constrained Proportional-Derivative (PD)-type attitude control scheme for rigid bodies in high-fidelity robotic simulation. Conventional prescribed performance control schemes involve complex transformed errors and heavy partial derivative calculations. In contrast, the replicated framework integrates a nominal PD law with an auxiliary barrier term and an adaptive saturation/disturbance estimator to constrain the attitude quaternion and angular velocity strictly within finite-time predefined performance boundaries.

The complete control pipeline was developed in ROS 2 and tested on an X3 quadcopter inside the Gazebo physics environment. Over a 180-second hover stabilization regime at a target altitude of 2.0 m, the closed-loop system achieved precise attitude regulation while respecting motor torque boundaries ($u_{max} = 0.20\,\mathrm{N}\cdot\mathrm{m}$). The experimental results verify robust transient containment, zero boundary violations ($0.00\%$), and reliable performance under physical actuator saturation constraints.

---

## 2. Introduction

### 2.1 Overview of the Base Paper

Attitude control is essential for stabilizing rigid bodies such as spacecraft, aerial robots, and satellites. Conventional control schemes frequently struggle to guarantee transient safety bounds while subject to actuator saturation and bounded external disturbances. While Prescribed Performance Control (PPC) provides predefined transient and steady-state boundaries, standard formulations rely on error transformation techniques that yield complex partial derivatives and coupled non-affine equations.

The base paper by *Golestani et al. (IEEE Access, 2022)* proposes a constrained PD-type controller with a simplified analytical structure:

- A standard **PD control component** asymptotically steers the attitude quaternion and angular velocity to zero.
- An **auxiliary barrier function** penalizes trajectories approaching predefined performance envelopes.
- A **Finite-Time Prescribed Performance Function (FTPPF)** establishes hard transient and steady-state boundaries with a deterministic settling time $T_s$.
- An **on-line adaptive estimator** estimates the lumped unknown bound of actuator saturation and external disturbances.

### 2.2 What is Gazebo?

**Gazebo** is an open-source 3D multi-robot physics simulator designed for validating complex control algorithms under realistic environmental dynamics. It provides:

- **Rigid-Body Physics Engines:** Accurate simulation of gravity, multi-body kinematics, inertial tensors, Coriolis effects, and contact dynamics using engines such as ODE and Bullet.
- **Actuator & Aerodynamic Modeling:** Realistic simulation of motor speed dynamics, quadratic thrust generation ($c_T$), and aerodynamic drag/torque coefficients ($c_Q$).
- **ROS 2 Integration:** Direct pub/sub communication with ROS 2 nodes for seamless software-in-the-loop (SITL) prototyping.

### 2.3 Scope of Implementation in Gazebo

- **Gazebo Platform & Physics Utilized:** The multi-rotor **X3 Quadcopter platform** was deployed. Ground-truth state feedback was acquired from `/world/quadcopter/dynamic_pose/info`, and computed rotor angular velocity commands ($\mathrm{rad/s}$) were published to `/X3/gazebo/command/motor_speed`.
- **Completed Modules:**
  - Discrete-time finite-time prescribed performance envelope calculation ($\rho_q(t), \rho_\omega(t)$).
  - State filtering with low-pass filters and moving-average windows for numerical derivatives $\dot{z}$ and $\omega$.
  - Constrained barrier control law evaluation with dynamic state clipping.
  - Continuous-time adaptive parameter integration ($\hat{\theta}$).
  - Closed-loop altitude trajectory generation, hover thrust tracking, tilt angle compensation, and a 4-motor allocation mixer.
  - 180-second stability benchmark without wind disturbance injection.

---

## 3. Methodology

### 3.1 Base Paper Formulation (Implemented Equations)

#### 1. Rigid Body Kinematics and Dynamics

The attitude orientation of the rigid vehicle is described by the unit quaternion vector:

$$
q =
\begin{bmatrix}
q_v \\
q_4
\end{bmatrix}
=
\begin{bmatrix}
q_1 \\
q_2 \\
q_3 \\
q_4
\end{bmatrix}
\in \mathbb{R}^4
$$

with:

$$
q^Tq = \|q_v\|^2 + q_4^2 = 1
$$

The rotational kinematics are:

$$
\dot{q}_v =
\frac{1}{2}
(q_4 I_3 + q_v^\times)\omega
$$

$$
\dot{q}_4 =
-\frac{1}{2}q_v^T\omega
$$

The rotational dynamics are:

$$
J\dot{\omega}
=
-\omega^\times J\omega
+
\mathrm{sat}(u)
+
d
$$

where:

- $J \in \mathbb{R}^{3\times3}$ is the inertia tensor.
- $\omega \in \mathbb{R}^3$ is the angular velocity.
- $u \in \mathbb{R}^3$ is the control torque.
- $d \in \mathbb{R}^3$ is the unknown external disturbance.

For a vector:

$$
z =
\begin{bmatrix}
z_1 \\
z_2 \\
z_3
\end{bmatrix}
$$

the cross-product matrix is:

$$
z^\times =
\begin{bmatrix}
0 & -z_3 & z_2 \\
z_3 & 0 & -z_1 \\
-z_2 & z_1 & 0
\end{bmatrix}
$$

---

#### 2. Finite-Time Prescribed Performance Function (FTPPF)

To enforce:

$$
-\rho_q(t) < q_i(t) < \rho_q(t)
$$

and

$$
-\rho_\omega(t) < \omega_i(t) < \rho_\omega(t)
$$

the FTPPF is computed as:

$$
\rho(t)=
\begin{cases}
\left(
\rho_0
-
\rho_T
\left(1+\frac{t}{T_s}\right)
\right)
\exp\left(
\frac{\kappa t}{t-T_s}
\right)
+
\rho_T,
& 0\leq t<T_s
\\[6pt]
\rho_T,
& t\geq T_s
\end{cases}
$$

where:

- $\rho_0$ sets the initial envelope boundary.
- $\rho_T$ is the steady-state bound.
- $T_s$ is the settling time.
- $\kappa$ controls the rate of envelope decay.

---

## 3.1 Mathematical Formulation of the Base Paper

The control objective is to stabilize the orientation of a rigid body while simultaneously guaranteeing that both the attitude quaternion and angular velocity remain strictly within predefined transient and steady-state boundaries under actuator physical limitations.

### 3.1.1 Rigid Body Kinematics and Dynamic Model

Let the rigid body orientation in the body-fixed reference frame $\mathcal{F}_b$ relative to the inertial reference frame $\mathcal{F}_i$ be represented using the unit quaternion vector:

$$
q =
\begin{bmatrix}
q_v \\
q_4
\end{bmatrix}
=
\begin{bmatrix}
q_1 \\
q_2 \\
q_3 \\
q_4
\end{bmatrix}
\in \mathbb{R}^4
$$

subject to:

$$
q^Tq = \|q_v\|^2 + q_4^2 = 1
$$

where:

$$
q_v =
\begin{bmatrix}
q_1 \\
q_2 \\
q_3
\end{bmatrix}
$$

denotes the vector component and $q_4$ represents the scalar component.

The rotational kinematics and dynamics are:

$$
\dot{q}_v =
\frac{1}{2}
(q_4I_3+q_v^\times)\omega
$$

$$
\dot{q}_4 =
-\frac{1}{2}q_v^T\omega
$$

$$
J\dot{\omega}
=
-\omega^\times J\omega
+
\mathrm{sat}(u)
+
d
$$

where:

- $\omega=[\omega_1,\omega_2,\omega_3]^T$ is the angular velocity vector.
- $J\in\mathbb{R}^{3\times3}$ is the symmetric positive-definite inertia matrix.
- $u=[u_1,u_2,u_3]^T$ is the control torque command.
- $d=[d_1,d_2,d_3]^T$ is the unknown external disturbance vector.

For any vector:

$$
z=[z_1,z_2,z_3]^T
$$

the skew-symmetric matrix is:

$$
z^\times =
\begin{bmatrix}
0 & -z_3 & z_2 \\
z_3 & 0 & -z_1 \\
-z_2 & z_1 & 0
\end{bmatrix}
$$

---

### 3.1.2 Actuator Saturation Modeling

Actuators possess physical limits and cannot supply infinite torque.

The actual control torque is modeled as:

$$
\mathrm{sat}(u_i)
=
u_i(t)+\pi_i(t)
$$

for:

$$
i\in\{1,2,3\}
$$

The saturation error is:

$$
\pi_i(t)=
\begin{cases}
0,
& |u_i(t)|\leq u_{max,i}
\\[6pt]
\mathrm{sgn}(u_i(t))u_{max,i}-u_i(t),
& |u_i(t)|>u_{max,i}
\end{cases}
$$

where $u_{max,i}$ is the maximum allowable torque around the $i$-th axis.

Defining the lumped unknown term:

$$
T_d=d+\Pi(t)
$$

with:

$$
\|T_d\|\leq\theta
$$

the governing dynamics become:

$$
J\dot{\omega}
=
-\omega^\times J\omega
+
u
+
T_d
$$

---

### 3.1.3 Finite-Time Prescribed Performance Function (FTPPF)

To constrain overshoot, convergence rate, and steady-state error of both attitude and angular velocity, the states are constrained within time-varying bounds:

$$
-\rho_q(t)<q_i(t)<\rho_q(t)
$$

$$
-\rho_\omega(t)<\omega_i(t)<\rho_\omega(t)
$$

for:

$$
i\in\{1,2,3\}
$$

The FTPPF is:

$$
\rho(t)=
\begin{cases}
\left(
\rho_0
-
\rho_T
\left(1+\frac{t}{T_s}\right)
\right)
\exp\left(
\frac{\kappa t}{t-T_s}
\right)
+
\rho_T,
& 0\leq t<T_s
\\[6pt]
\rho_T,
& t\geq T_s
\end{cases}
$$

where:

- $\rho_0>0$ defines the maximum allowable initial bound.
- $\rho_T>0$ denotes the final steady-state precision boundary.
- $T_s>0$ sets the finite settling time.
- $\kappa>0$ controls the initial rate of envelope decay.

---

### 3.1.4 Error Normalization and Barrier Transform

The normalized error vectors are:

$$
\epsilon_1(t)
=
\frac{q_v(t)}{\rho_q(t)}
=
\begin{bmatrix}
\epsilon_{11}(t)\\
\epsilon_{12}(t)\\
\epsilon_{13}(t)
\end{bmatrix}
$$

and:

$$
\epsilon_2(t)
=
\frac{\omega(t)}{\rho_\omega(t)}
=
\begin{bmatrix}
\epsilon_{21}(t)\\
\epsilon_{22}(t)\\
\epsilon_{23}(t)
\end{bmatrix}
$$

where:

- $q_v(t)$ is the vector component of the attitude quaternion.
- $\omega(t)$ is the body angular velocity vector.
- $\rho_q(t)$ and $\rho_\omega(t)$ are the dynamic performance boundaries.

Strictly satisfying:

$$
|\epsilon_{1i}(t)|<1
$$

and:

$$
|\epsilon_{2i}(t)|<1
$$

guarantees that the quaternion and angular velocity components remain within the predefined performance boundaries.

---

### 3.1.5 Constrained PD-Type Control Law and Adaptive Law

The control torque command is:

$$
u=-k_pq_v-k_d\omega-u_{cons}
$$

where the first two terms form the nominal PD controller and $u_{cons}$ provides constraint and adaptive robustness.

The auxiliary barrier control component is:

$$
u_{cons,i}
=
\left[
\frac{\alpha}{(1-\epsilon_{1i}^2)^2}
+
\frac{\beta}{(1-\epsilon_{2i}^2)^2}
\right]s_i
+
\hat{\theta}\,
\mathrm{sat}
\left(
\frac{s_i}{\varsigma}
\right)
$$

where:

- $k_p>0$ and $k_d>0$ are the proportional and derivative gains.
- $\alpha>0$ and $\beta>0$ are barrier weighting parameters.
- $s=\omega+cq_v$ is the composite sliding surface.
- $c>0$ is a design constant.
- $\hat{\theta}$ is the adaptive estimate of the unknown disturbance-saturation bound.
- $\varsigma>0$ is the boundary-layer parameter.

The continuous saturation function is:

$$
\mathrm{sat}
\left(
\frac{s_i}{\varsigma}
\right)
=
\begin{cases}
1,
& s_i>\varsigma
\\[6pt]
\frac{s_i}{\varsigma},
& |s_i|\leq\varsigma
\\[6pt]
-1,
& s_i<-\varsigma
\end{cases}
$$

The adaptive parameter is updated according to:

$$
\dot{\hat{\theta}}
=
\frac{1}{\eta_1}
\left(
\|s\|-\eta_2(t)\hat{\theta}
\right)
$$

where $\eta_1>0$ is the adaptation gain.

The time-varying leakage rate is:

$$
\eta_2(t)
=
\max
\left(
2\exp(-0.5t),
0.1
\right)
$$

---

## 3.2 Practical Execution and System Architecture on Gazebo

The theoretical attitude control law was deployed on an **X3 Quadcopter** in Gazebo via ROS 2. Because an underactuated multirotor requires continuous thrust generation alongside attitude stabilization to stay aloft, the framework couples the attitude barrier control with vertical altitude tracking and rotor thrust mixing.

### Step-by-Step Implementation Pipeline

#### 1. State Acquisition & Filtering

- Ground-truth position $z$ and orientation quaternion $q=[q_x,q_y,q_z,q_w]^T$ are sampled from `/world/quadcopter/dynamic_pose/info`.
- To prevent sign-flip ambiguities in quaternion space, the orientation sign is preserved by enforcing $q\leftarrow-q$ if $q\cdot q_{last}<0$.
- Angular velocity $\omega$ and vertical velocity $\dot{z}$ are derived using backward differences:

$$
\dot{z}
=
\frac{z_k-z_{k-1}}{\Delta t}
$$

$$
\omega_{raw}
=
\frac{2(q_{v,k}-q_{v,k-1})}{\Delta t}
$$

The angular velocity is conditioned using a first-order low-pass filter:

$$
\omega_k
=
(1-\lambda_\omega)\omega_{k-1}
+
\lambda_\omega\omega_{raw}
$$

with:

$$
\lambda_\omega=0.10
$$

---

#### 2. Altitude Control & Tilt Decoupling

The desired takeoff altitude is smoothed over:

$$
t_{takeoff}=8.0\,\mathrm{s}
$$

to a target altitude:

$$
z_{target}=2.0\,\mathrm{m}
$$

The vertical force demand is:

$$
F_{vertical}
=
m
\left(
g+
\mathrm{clip}
\left(
K_{p,z}(z_{target}-z)
-
K_{d,z}\dot{z},
-3.0,
3.0
\right)
\right)
$$

The tilt compensation factor is:

$$
\mathrm{tilt\_factor}
=
\mathrm{clip}
\left(
\cos(2q_1)\cos(2q_2),
0.85,
1.0
\right)
$$

The total thrust is:

$$
T_{total}
=
\mathrm{clip}
\left(
\frac{F_{vertical}}{\mathrm{tilt\_factor}},
0,
22.0\,\mathrm{N}
\right)
$$

---

#### 3. Constrained Attitude Control Execution

The normalized errors are:

$$
\epsilon_1=\frac{q_v}{\rho_q(t)}
$$

and:

$$
\epsilon_2=\frac{\omega}{\rho_\omega(t)}
$$

To prevent numerical singularity near the envelope limits, the normalized errors are clipped to:

$$
[-0.90,0.90]
$$

The sliding surface is:

$$
s=\omega+cq_v
$$

The adaptive parameter is updated using Euler integration:

$$
\hat{\theta}_k
=
\mathrm{clip}
\left(
\hat{\theta}_{k-1}
+
\frac{1}{\eta_1}
\left(
\|s\|
-
\eta_2\hat{\theta}_{k-1}
\right)
\Delta t,
0.0,
1.0
\right)
$$

The control torque is evaluated as:

$$
u=-K_pq_v-K_d\omega-u_{cons}
$$

and saturated to the available authority:

$$
\tau
=
\mathrm{clip}
\left(
u,
-\tau_{max},
\tau_{max}
\right)
$$

---

#### 4. Motor Allocation Matrix

The total thrust $T_{total}$ and three-axis torques:

$$
\tau=
\begin{bmatrix}
\tau_1\\
\tau_2\\
\tau_3
\end{bmatrix}
$$

are mapped to individual rotor angular velocities.

The quadcopter mixer uses:

$$
l=0.225\,\mathrm{m}
$$

$$
c_T=8.54858\times10^{-6}
$$

and:

$$
c_Q=0.016
$$

The rotor allocation is represented by:

$$
m_{1,4}
=
\frac{T_{total}}{4c_T}
\pm
\frac{\tau_2}{4c_Tl}
\mp
\frac{\tau_1}{4c_Tl}
\mp
\frac{\tau_3}{4c_Q}
$$

with rotor speed:

$$
\omega_i
=
\sqrt{
\max(0,m_i)
}
$$

---

## 3.3 Execution Workflow

1. **Pose Callback & Filtering:** Subscribes to the Gazebo ground-truth pose. Euler tilt extraction accounts for gravity compensation:

$$
T_{total}
=
\frac{F_{vertical}}
{\cos(\mathrm{roll})\cos(\mathrm{pitch})}
$$

2. **Velocity Estimation:** Computes numerical derivatives $\dot{z}$ and $\omega=2\dot{q}_v$ and conditions them through first-order low-pass filters with:

$$
\alpha_{filter}=0.10
$$

3. **Barrier Evaluation & Clamping:** Normalizes $q_v$ and $\omega$ against $\rho_q(t)$ and $\rho_\omega(t)$ and clips the ratios to $[-0.90,0.90]$ to prevent numerical singularity.

4. **Adaptive Parameter Integration:** Integrates $\dot{\hat{\theta}}$ in discrete time with anti-windup clamping:

$$
0.0\leq\hat{\theta}\leq1.0
$$

5. **Motor Allocation Mixer:** Translates total thrust $T_{total}$ and three-axis torque commands into rotor speeds.

---

## 3.4 Remaining Implementation Tasks

- **Disturbance Injection Verification:** Enable `inject_wind_disturbance = True` using the multi-frequency sinusoidal disturbance vector:

$$
d(t)=
\begin{bmatrix}
0.05\sin(t)\\
0.10\sin(1.2t)\\
0.15\sin(1.5t)
\end{bmatrix}
\,\mathrm{N}\cdot\mathrm{m}
$$

to stress-test adaptive rejection.

- **Baseline PD Benchmark Comparison:** Execute the simulation under `controller_mode = 'conventional_pd'` to collect comparative convergence and overshoot datasets under identical initial conditions.

- **Waypoint Trajectory Tracking:** Extend the regulator from a static hover point:

$$
z=2.0\,\mathrm{m}
$$

to full dynamic 3D spatial trajectory tracking:

$$
(x_d(t),y_d(t),z_d(t),\psi_d(t))
$$

---

## 5. Future Work and Conclusion

### 5.1 Future Work

Future work will focus on three key directions.

First, external multi-frequency time-varying wind and disturbance profiles will be introduced into the Gazebo environment to analyze the disturbance estimation dynamics of $\hat{\theta}$ and stress-test the controller's robustness limits under torque saturation.

Second, a comparative benchmark will be conducted against a conventional unconstrained PD controller to experimentally demonstrate differences in transient overshoot, constraint violations, and motor wear.

Third, the framework will be expanded to 6-DOF dynamic trajectory tracking, incorporating position control and dynamic tilt generation, and validated on physical quadcopter hardware through embedded ROS 2 microcontrollers.

### 5.2 Conclusion

This project successfully implemented and evaluated a simple-structure full-state constrained PD-type attitude controller for rigid bodies using ROS 2 and the Gazebo simulation engine.

By integrating a finite-time prescribed performance function with an auxiliary barrier term and an adaptive disturbance estimator, the system maintained attitude stability without the analytical complexity of conventional error transformations.

The experimental simulation on the X3 quadcopter demonstrated precise altitude regulation at 2.0 m, zero constraint violations across all attitude and angular velocity states, smooth adaptation of the unknown saturation parameter, and compliant motor torque allocation within strict actuator limits.
