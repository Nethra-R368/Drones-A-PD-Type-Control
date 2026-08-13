/**
 * Spacecraft Simulation Mathematical Engine
 * Implements rigid body attitude dynamics, FTPPF bounds,
 * proposed constrained controller, conventional controller, and RK4 solver.
 */

class SpacecraftSimulation {
  constructor(config = {}) {
    // Inertia Matrix J
    this.J = config.J || [
      [20.0, 1.2, 0.9],
      [1.2, 17.0, 1.4],
      [0.9, 1.4, 15.0]
    ];
    this.invJ = this.invert3x3(this.J);

    // Initial Conditions
    // q = [q1, q2, q3, q4] where qv = [q1, q2, q3] and q4 is scalar
    this.q0 = config.q0 || [0.2, -0.15, -0.25, 0.9354]; 
    this.omega0 = config.omega0 || [0.03, 0.02, -0.01];
    this.thetaHat0 = config.thetaHat0 || 0.0;

    // Controller Parameters
    this.Kp = config.Kp !== undefined ? config.Kp : 40.0;
    this.Kd = config.Kd !== undefined ? config.Kd : 20.0;
    this.alpha = config.alpha !== undefined ? config.alpha : 1.5;
    this.beta = config.beta !== undefined ? config.beta : 1.2;
    this.c = config.c !== undefined ? config.c : 1.0; // Sliding surface coeff (s = w + c*qv)
    this.eta1 = config.eta1 !== undefined ? config.eta1 : 0.1;
    this.varsigma = config.varsigma !== undefined ? config.varsigma : 0.01; // Chattering reduction
    this.uMax = config.uMax !== undefined ? config.uMax : 2.5; // Actuator torque limit (Nm)

    // FTPPF Parameters
    this.rho_q0 = config.rho_q0 !== undefined ? config.rho_q0 : 0.3;
    this.rho_qT = config.rho_qT !== undefined ? config.rho_qT : 0.008;
    this.kappa_q = config.kappa_q !== undefined ? config.kappa_q : 2.5;

    this.rho_w0 = config.rho_w0 !== undefined ? config.rho_w0 : 0.4;
    this.rho_wT = config.rho_wT !== undefined ? config.rho_wT : 0.005;
    this.kappa_w = config.kappa_w !== undefined ? config.kappa_w : 2.5;

    this.Ts = config.Ts !== undefined ? config.Ts : 15.0; // Settling time (seconds)

    // Controller Mode: 'constrained' or 'conventional'
    this.mode = config.mode || 'constrained';

    // State: [q1, q2, q3, q4, w1, w2, w3, thetaHat]
    this.state = [
      this.q0[0], this.q0[1], this.q0[2], this.q0[3],
      this.omega0[0], this.omega0[1], this.omega0[2],
      this.thetaHat0
    ];
    this.time = 0.0;

    // History for plotting
    this.history = [];
    this.recordHistory();
  }

  // Finite-Time Prescribed Performance Function
  ftppf(t, rho0, rhoT, kappa, Ts) {
    if (t >= Ts - 1e-9) {
      return rhoT;
    }
    const exponent = (kappa * t) / (t - Ts);
    if (exponent < -50) return rhoT; // Guard against underflow
    return (rho0 - rhoT * (1.0 + t / Ts)) * Math.exp(exponent) + rhoT;
  }

  // Cross product matrix for a 3-element vector
  crossProductMatrix(v) {
    return [
      [0, -v[2], v[1]],
      [v[2], 0, -v[0]],
      [-v[1], v[0], 0]
    ];
  }

  // Matrix multiplication: 3x3 matrix * 3x1 vector
  matMul3x1(M, v) {
    return [
      M[0][0]*v[0] + M[0][1]*v[1] + M[0][2]*v[2],
      M[1][0]*v[0] + M[1][1]*v[1] + M[1][2]*v[2],
      M[2][0]*v[0] + M[2][1]*v[1] + M[2][2]*v[2]
    ];
  }

  // Vector additions/subtractions/scaling
  vecAdd(v1, v2) { return [v1[0] + v2[0], v1[1] + v2[1], v1[2] + v2[2]]; }
  vecSub(v1, v2) { return [v1[0] - v2[0], v1[1] - v2[1], v1[2] - v2[2]]; }
  vecScale(v, s) { return [v[0] * s, v[1] * s, v[2] * s]; }
  vecNorm(v) { return Math.sqrt(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]); }
  vecDot(v1, v2) { return v1[0]*v2[0] + v1[1]*v2[1] + v1[2]*v2[2]; }

  // 3x3 matrix inversion
  invert3x3(M) {
    const d = M[0][0] * (M[1][1] * M[2][2] - M[1][2] * M[2][1]) -
              M[0][1] * (M[1][0] * M[2][2] - M[1][2] * M[2][0]) +
              M[0][2] * (M[1][0] * M[2][1] - M[1][1] * M[2][0]);
    if (Math.abs(d) < 1e-12) return [[1,0,0],[0,1,0],[0,0,1]];
    const invd = 1.0 / d;
    return [
      [
        (M[1][1]*M[2][2] - M[1][2]*M[2][1]) * invd,
        (M[0][2]*M[2][1] - M[0][1]*M[2][2]) * invd,
        (M[0][1]*M[1][2] - M[0][2]*M[1][1]) * invd
      ],
      [
        (M[1][2]*M[2][0] - M[1][0]*M[2][2]) * invd,
        (M[0][0]*M[2][2] - M[0][2]*M[2][0]) * invd,
        (M[0][2]*M[1][0] - M[0][0]*M[1][2]) * invd
      ],
      [
        (M[1][0]*M[2][1] - M[1][1]*M[2][0]) * invd,
        (M[0][1]*M[2][0] - M[0][0]*M[2][1]) * invd,
        (M[0][0]*M[1][1] - M[0][1]*M[1][0]) * invd
      ]
    ];
  }

  // Saturation function for a scalar or vector
  sat(x, delta) {
    return Math.max(-1.0, Math.min(1.0, x / delta));
  }

  // Compute disturbance torque d(t) = [0.05*sin(t), 0.1*sin(1.2*t), 0.15*sin(1.5*t)]
  getDisturbance(t) {
    return [
      0.05 * Math.sin(t),
      0.1 * Math.sin(1.2 * t),
      0.15 * Math.sin(1.5 * t)
    ];
  }

  // Derivatives function for RK4 solver
  derivatives(t, state) {
    // Unpack state
    const q1 = state[0];
    const q2 = state[1];
    const q3 = state[2];
    const q4 = state[3];
    const w1 = state[4];
    const w2 = state[5];
    const w3 = state[6];
    const thetaHat = state[7];

    const qv = [q1, q2, q3];
    const omega = [w1, w2, w3];

    // Normalized boundaries
    const rho_q = this.ftppf(t, this.rho_q0, this.rho_qT, this.kappa_q, this.Ts);
    const rho_w = this.ftppf(t, this.rho_w0, this.rho_wT, this.kappa_w, this.Ts);

    // Compute s = w + c * qv
    const s = this.vecAdd(omega, this.vecScale(qv, this.c));

    let u = [0, 0, 0];
    let dThetaHat = 0.0;

    if (this.mode === 'constrained') {
      // Proposed constrained PD-type control
      // Compute epsilon_1 and epsilon_2
      const eps1 = [qv[0] / rho_q, qv[1] / rho_q, qv[2] / rho_q];
      const eps2 = [omega[0] / rho_w, omega[1] / rho_w, omega[2] / rho_w];

      // Calculate u_cons
      const uCons = [0, 0, 0];
      for (let i = 0; i < 3; i++) {
        // Clamp epsilon to prevent numerical division by zero if it goes out of bounds
        const e1_clamped = Math.max(-0.9999, Math.min(0.9999, eps1[i]));
        const e2_clamped = Math.max(-0.9999, Math.min(0.9999, eps2[i]));

        const term1 = this.alpha / Math.pow(1.0 - e1_clamped * e1_clamped, 2);
        const term2 = this.beta / Math.pow(1.0 - e2_clamped * e2_clamped, 2);

        uCons[i] = (term1 + term2) * s[i] + thetaHat * this.sat(s[i], this.varsigma);
      }

      // Control input: u = -Kp*qv - Kd*w - uCons
      u = this.vecSub(this.vecSub(this.vecScale(qv, -this.Kp), this.vecScale(omega, this.Kd)), uCons);

      // Adaptive parameter derivative: dThetaHat = 1/eta1 * (||s|| - eta2 * thetaHat)
      const eta2 = 2.0 * Math.exp(-0.5 * t);
      const sNorm = this.vecNorm(s);
      dThetaHat = (1.0 / this.eta1) * (sNorm - eta2 * thetaHat);

      // Guard against negative thetaHat
      if (thetaHat <= 0.0 && dThetaHat < 0.0) {
        dThetaHat = 0.0;
      }
    } else {
      // Conventional PD control: u = -Kp*qv - Kd*w
      u = this.vecAdd(this.vecScale(qv, -this.Kp), this.vecScale(omega, -this.Kd));
      dThetaHat = 0.0;
    }

    // Actuator Saturation: clamp torque to [-uMax, uMax]
    const uSat = [
      Math.max(-this.uMax, Math.min(this.uMax, u[0])),
      Math.max(-this.uMax, Math.min(this.uMax, u[1])),
      Math.max(-this.uMax, Math.min(this.uMax, u[2]))
    ];

    // Rigid body attitude dynamics
    // 1) Quaternion derivative: dq = 0.5 * [ q4*I + qv^x ] * w, dq4 = -0.5 * qv^T * w
    const qCross = this.crossProductMatrix(qv);
    const qMat = [
      [q4, -qv[2], qv[1]],
      [qv[2], q4, -qv[0]],
      [-qv[1], qv[0], q4]
    ];
    const dq_v = this.vecScale(this.matMul3x1(qMat, omega), 0.5);
    const dq_4 = -0.5 * this.vecDot(qv, omega);

    // 2) Angular velocity derivative: J * dw = -w^x * J * w + uSat + d(t)
    const Jw = this.matMul3x1(this.J, omega);
    const wCross = this.crossProductMatrix(omega);
    const negative_wCrossJw = this.vecScale(this.matMul3x1(wCross, Jw), -1.0);
    const extDist = this.getDisturbance(t);
    const torqueSum = this.vecAdd(this.vecAdd(negative_wCrossJw, uSat), extDist);
    const dOmega = this.matMul3x1(this.invJ, torqueSum);

    return [
      dq_v[0], dq_v[1], dq_v[2], dq_4,
      dOmega[0], dOmega[1], dOmega[2],
      dThetaHat
    ];
  }

  // Perform a single step of simulation using RK4 method
  step(dt) {
    const t = this.time;
    const y = [...this.state];

    // k1
    const k1 = this.derivatives(t, y);

    // k2
    const y2 = y.map((val, idx) => val + 0.5 * dt * k1[idx]);
    const k2 = this.derivatives(t + 0.5 * dt, y2);

    // k3
    const y3 = y.map((val, idx) => val + 0.5 * dt * k2[idx]);
    const k3 = this.derivatives(t + 0.5 * dt, y3);

    // k4
    const y4 = y.map((val, idx) => val + dt * k3[idx]);
    const k4 = this.derivatives(t + dt, y4);

    // Update state: y_new = y + dt/6 * (k1 + 2*k2 + 2*k3 + k4)
    for (let i = 0; i < this.state.length; i++) {
      this.state[i] += (dt / 6.0) * (k1[i] + 2.0 * k2[i] + 2.0 * k3[i] + k4[i]);
    }

    // Normalize quaternion to prevent numerical drift
    const qNorm = Math.sqrt(
      this.state[0]*this.state[0] +
      this.state[1]*this.state[1] +
      this.state[2]*this.state[2] +
      this.state[3]*this.state[3]
    );
    if (qNorm > 1e-6) {
      this.state[0] /= qNorm;
      this.state[1] /= qNorm;
      this.state[2] /= qNorm;
      this.state[3] /= qNorm;
    }

    // Advance time
    this.time += dt;

    // Record data history
    this.recordHistory();
  }

  // Record the current state and computed control torques for graphing
  recordHistory() {
    const t = this.time;
    const qv = [this.state[0], this.state[1], this.state[2]];
    const q4 = this.state[3];
    const omega = [this.state[4], this.state[5], this.state[6]];
    const thetaHat = this.state[7];

    const rho_q = this.ftppf(t, this.rho_q0, this.rho_qT, this.kappa_q, this.Ts);
    const rho_w = this.ftppf(t, this.rho_w0, this.rho_wT, this.kappa_w, this.Ts);

    const s = this.vecAdd(omega, this.vecScale(qv, this.c));

    let u = [0, 0, 0];
    if (this.mode === 'constrained') {
      const eps1 = [qv[0] / rho_q, qv[1] / rho_q, qv[2] / rho_q];
      const eps2 = [omega[0] / rho_w, omega[1] / rho_w, omega[2] / rho_w];

      const uCons = [0, 0, 0];
      for (let i = 0; i < 3; i++) {
        const e1_clamped = Math.max(-0.9999, Math.min(0.9999, eps1[i]));
        const e2_clamped = Math.max(-0.9999, Math.min(0.9999, eps2[i]));
        const term1 = this.alpha / Math.pow(1.0 - e1_clamped * e1_clamped, 2);
        const term2 = this.beta / Math.pow(1.0 - e2_clamped * e2_clamped, 2);
        uCons[i] = (term1 + term2) * s[i] + thetaHat * this.sat(s[i], this.varsigma);
      }
      u = this.vecSub(this.vecSub(this.vecScale(qv, -this.Kp), this.vecScale(omega, this.Kd)), uCons);
    } else {
      u = this.vecAdd(this.vecScale(qv, -this.Kp), this.vecScale(omega, -this.Kd));
    }

    const uSat = [
      Math.max(-this.uMax, Math.min(this.uMax, u[0])),
      Math.max(-this.uMax, Math.min(this.uMax, u[1])),
      Math.max(-this.uMax, Math.min(this.uMax, u[2]))
    ];

    this.history.push({
      t,
      q: [qv[0], qv[1], qv[2], q4],
      omega: [...omega],
      thetaHat,
      u: [...u],
      uSat: [...uSat],
      rho_q,
      rho_w
    });
  }

  // Reset simulation to initial conditions
  reset() {
    this.state = [
      this.q0[0], this.q0[1], this.q0[2], this.q0[3],
      this.omega0[0], this.omega0[1], this.omega0[2],
      this.thetaHat0
    ];
    this.time = 0.0;
    this.history = [];
    this.recordHistory();
  }

  // Pre-run the entire simulation up to tMax with step size dt
  runTo(tMax, dt = 0.01) {
    this.reset();
    while (this.time < tMax) {
      this.step(dt);
    }
  }
}

// Export for node or browser environment
if (typeof module !== 'undefined' && module.exports) {
  module.exports = SpacecraftSimulation;
} else {
  window.SpacecraftSimulation = SpacecraftSimulation;
}
