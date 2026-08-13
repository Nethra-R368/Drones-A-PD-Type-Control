/**
 * Spacecraft ACS Simulation UI & Visualizer Application
 * Orchestrates Three.js rendering, Chart.js graphs, telemetry HUD, and scenario configuration.
 */

document.addEventListener('DOMContentLoaded', () => {
  // Global simulation instance
  let sim = null;
  let chart = null;
  
  // Animation state
  let isPlaying = false;
  let animationFrameId = null;
  let lastTime = 0;
  let playbackSpeed = 1.0;
  
  // Three.js Variables
  let scene, camera, renderer, spacecraft, orbitControls;
  let inertialGrid, arrowHelpers = [];
  
  // Active chart tab
  let activeTab = 'q'; // 'q', 'w', 'u', 'theta'
  let lastChartUpdateTime = 0;
  
  // Initialize MathJax rendering check
  function triggerMathJaxUpdate() {
    if (window.MathJax && window.MathJax.typeset) {
      window.MathJax.typeset();
    }
  }

  // --- 3D Scene Initialization ---
  function init3D() {
    const container = document.getElementById('three-container');
    const width = container.clientWidth;
    const height = container.clientHeight;

    // Scene
    scene = new THREE.Scene();
    scene.background = new THREE.Color(0x03050b);

    // Camera
    camera = new THREE.PerspectiveCamera(45, width / height, 0.1, 100);
    camera.position.set(5, 4, 8);

    // Renderer
    renderer = new THREE.WebGLRenderer({ antialias: true });
    renderer.setSize(width, height);
    renderer.shadowMap.enabled = true;
    container.appendChild(renderer.domElement);

    // OrbitControls
    orbitControls = new THREE.OrbitControls(camera, renderer.domElement);
    orbitControls.enableDamping = true;
    orbitControls.dampingFactor = 0.05;
    orbitControls.maxDistance = 25;
    orbitControls.minDistance = 3;

    // Starfield Background
    const starGeometry = new THREE.BufferGeometry();
    const starCount = 800;
    const starPositions = new Float32Array(starCount * 3);
    for (let i = 0; i < starCount * 3; i++) {
      starPositions[i] = (Math.random() - 0.5) * 60;
    }
    starGeometry.setAttribute('position', new THREE.BufferAttribute(starPositions, 3));
    const starMaterial = new THREE.PointsMaterial({
      color: 0xffffff,
      size: 0.1,
      transparent: true,
      opacity: 0.8
    });
    const stars = new THREE.Points(starGeometry, starMaterial);
    scene.add(stars);

    // Lights
    const ambientLight = new THREE.AmbientLight(0xffffff, 0.25);
    scene.add(ambientLight);

    const dirLight1 = new THREE.DirectionalLight(0xffffff, 0.8);
    dirLight1.position.set(10, 10, 10);
    scene.add(dirLight1);

    const dirLight2 = new THREE.DirectionalLight(0x38bdf8, 0.4); // Cyber blue accent light
    dirLight2.position.set(-10, -5, -10);
    scene.add(dirLight2);

    // Inertial Reference Grid (faint)
    inertialGrid = new THREE.GridHelper(20, 20, 0x1e293b, 0x0f172a);
    inertialGrid.position.y = -2.5;
    scene.add(inertialGrid);

    // Inertial Frame Axes
    const inertialAxes = new THREE.AxesHelper(4);
    inertialAxes.position.set(0, -2.49, 0);
    // Make axes faint
    inertialAxes.material.opacity = 0.25;
    inertialAxes.material.transparent = true;
    scene.add(inertialAxes);

    // Build the Spacecraft Model
    spacecraft = new THREE.Group();

    // Central cylindrical hull (Metallic Grey)
    const hullGeom = new THREE.CylinderGeometry(0.5, 0.5, 2.4, 16);
    const hullMat = new THREE.MeshStandardMaterial({
      color: 0x475569,
      metalness: 0.85,
      roughness: 0.15
    });
    const hull = new THREE.Mesh(hullGeom, hullMat);
    hull.rotation.x = Math.PI / 2; // Align cylinder along body Z-axis
    spacecraft.add(hull);

    // Golden Front Nose Cone
    const coneGeom = new THREE.ConeGeometry(0.5, 0.6, 16);
    const coneMat = new THREE.MeshStandardMaterial({
      color: 0xeab308, // Gold
      metalness: 0.9,
      roughness: 0.1
    });
    const nose = new THREE.Mesh(coneGeom, coneMat);
    nose.position.z = 1.5;
    nose.rotation.x = Math.PI / 2;
    spacecraft.add(nose);

    // Back Thruster nozzle
    const nozzleGeom = new THREE.CylinderGeometry(0.4, 0.25, 0.4, 16);
    const nozzleMat = new THREE.MeshStandardMaterial({
      color: 0x1e293b,
      metalness: 0.9,
      roughness: 0.4
    });
    const nozzle = new THREE.Mesh(nozzleGeom, nozzleMat);
    nozzle.position.z = -1.4;
    nozzle.rotation.x = Math.PI / 2;
    spacecraft.add(nozzle);

    // Blue Thruster Plume (Flame)
    const plumeGeom = new THREE.ConeGeometry(0.18, 0.5, 16);
    const plumeMat = new THREE.MeshBasicMaterial({
      color: 0x06b6d4, // Cyan glow
      transparent: true,
      opacity: 0.7
    });
    const plume = new THREE.Mesh(plumeGeom, plumeMat);
    plume.position.z = -1.75;
    plume.rotation.x = -Math.PI / 2;
    plume.name = "plume";
    spacecraft.add(plume);

    // Solar Panel Arrays (Wings)
    const panelGeom = new THREE.BoxGeometry(3.6, 0.05, 1.0);
    const panelMat = new THREE.MeshStandardMaterial({
      color: 0x1e1b4b, // Deep blue solar cell color
      metalness: 0.5,
      roughness: 0.1
    });
    
    // Left Wing
    const leftPanel = new THREE.Mesh(panelGeom, panelMat);
    leftPanel.position.set(-2.0, 0, 0);
    spacecraft.add(leftPanel);

    // Right Wing
    const rightPanel = new THREE.Mesh(panelGeom, panelMat);
    rightPanel.position.set(2.0, 0, 0);
    spacecraft.add(rightPanel);

    // Panel support rods (connecting cylinders)
    const rodGeom = new THREE.CylinderGeometry(0.06, 0.06, 4.0, 8);
    const rodMat = new THREE.MeshStandardMaterial({ color: 0x94a3b8, metalness: 0.9 });
    const rod = new THREE.Mesh(rodGeom, rodMat);
    rod.rotation.z = Math.PI / 2;
    spacecraft.add(rod);

    // Body-Fixed Frame Coordinate Axes (Fb)
    // X - Red, Y - Green, Z - Blue
    const axisLength = 1.8;
    const origin = new THREE.Vector3(0, 0, 0);
    
    const arrowX = new THREE.ArrowHelper(new THREE.Vector3(1, 0, 0), origin, axisLength, 0xef4444, 0.3, 0.15);
    const arrowY = new THREE.ArrowHelper(new THREE.Vector3(0, 1, 0), origin, axisLength, 0x10b981, 0.3, 0.15);
    const arrowZ = new THREE.ArrowHelper(new THREE.Vector3(0, 0, 1), origin, axisLength, 0x38bdf8, 0.3, 0.15);
    
    spacecraft.add(arrowX);
    spacecraft.add(arrowY);
    spacecraft.add(arrowZ);

    scene.add(spacecraft);

    // Build the Desired Reference Spacecraft Model (Faint Wireframe)
    const desiredSpacecraft = new THREE.Group();
    
    // Cylindrical hull
    const desiredHull = new THREE.Mesh(
      new THREE.CylinderGeometry(0.5, 0.5, 2.4, 16),
      new THREE.MeshStandardMaterial({
        color: 0x94a3b8,
        wireframe: true,
        transparent: true,
        opacity: 0.12
      })
    );
    desiredHull.rotation.x = Math.PI / 2;
    desiredSpacecraft.add(desiredHull);

    // Nose
    const desiredNose = new THREE.Mesh(
      new THREE.ConeGeometry(0.5, 0.6, 16),
      new THREE.MeshStandardMaterial({
        color: 0xeab308,
        wireframe: true,
        transparent: true,
        opacity: 0.12
      })
    );
    desiredNose.position.z = 1.5;
    desiredNose.rotation.x = Math.PI / 2;
    desiredSpacecraft.add(desiredNose);

    // Wings (Solar panels)
    const desiredPanelGeom = new THREE.BoxGeometry(3.6, 0.05, 1.0);
    const desiredPanelMat = new THREE.MeshStandardMaterial({
      color: 0x818cf8,
      wireframe: true,
      transparent: true,
      opacity: 0.1
    });
    
    const desiredLeftPanel = new THREE.Mesh(desiredPanelGeom, desiredPanelMat);
    desiredLeftPanel.position.set(-2.0, 0, 0);
    desiredSpacecraft.add(desiredLeftPanel);

    const desiredRightPanel = new THREE.Mesh(desiredPanelGeom, desiredPanelMat);
    desiredRightPanel.position.set(2.0, 0, 0);
    desiredSpacecraft.add(desiredRightPanel);

    const desiredRod = new THREE.Mesh(
      new THREE.CylinderGeometry(0.06, 0.06, 4.0, 8),
      new THREE.MeshStandardMaterial({
        color: 0x94a3b8,
        wireframe: true,
        transparent: true,
        opacity: 0.1
      })
    );
    desiredRod.rotation.z = Math.PI / 2;
    desiredSpacecraft.add(desiredRod);

    // Desired reference axes (dashed/faint)
    const desiredArrowX = new THREE.ArrowHelper(new THREE.Vector3(1, 0, 0), origin, 2.0, 0xef4444, 0.2, 0.1);
    const desiredArrowY = new THREE.ArrowHelper(new THREE.Vector3(0, 1, 0), origin, 2.0, 0x10b981, 0.2, 0.1);
    const desiredArrowZ = new THREE.ArrowHelper(new THREE.Vector3(0, 0, 1), origin, 2.0, 0x38bdf8, 0.2, 0.1);
    
    // Faint transparency
    desiredArrowX.line.material.transparent = true;
    desiredArrowX.line.material.opacity = 0.35;
    desiredArrowY.line.material.transparent = true;
    desiredArrowY.line.material.opacity = 0.35;
    desiredArrowZ.line.material.transparent = true;
    desiredArrowZ.line.material.opacity = 0.35;

    desiredSpacecraft.add(desiredArrowX);
    desiredSpacecraft.add(desiredArrowY);
    desiredSpacecraft.add(desiredArrowZ);

    scene.add(desiredSpacecraft);

    // Resize handler
    window.addEventListener('resize', onWindowResize);
  }

  function onWindowResize() {
    const container = document.getElementById('three-container');
    camera.aspect = container.clientWidth / container.clientHeight;
    camera.updateProjectionMatrix();
    renderer.setSize(container.clientWidth, container.clientHeight);
  }

  // --- Chart.js Graph Setup ---

  // Custom plugin to draw vertical cursor line at window.currentSimTime
  const verticalCursorPlugin = {
    id: 'verticalCursor',
    afterDraw: (chartInstance) => {
      if (typeof window.currentSimTime === 'undefined') return;
      
      const ctx = chartInstance.ctx;
      const xAxis = chartInstance.scales.x;
      const yAxis = chartInstance.scales.y;
      
      const xPixel = xAxis.getPixelForValue(window.currentSimTime);
      if (xPixel >= xAxis.left && xPixel <= xAxis.right) {
        ctx.save();
        ctx.beginPath();
        ctx.moveTo(xPixel, yAxis.top);
        ctx.lineTo(xPixel, yAxis.bottom);
        ctx.lineWidth = 1.5;
        ctx.strokeStyle = '#38bdf8'; // Glowing Cyber Blue
        ctx.setLineDash([6, 4]);
        ctx.stroke();
        ctx.restore();
      }
    }
  };

  function initChart() {
    const ctx = document.getElementById('performance-chart').getContext('2d');
    
    Chart.register(verticalCursorPlugin);

    chart = new Chart(ctx, {
      type: 'line',
      data: {
        labels: [],
        datasets: []
      },
      options: {
        responsive: true,
        maintainAspectRatio: false,
        animation: { duration: 0 }, // Disable animations for performance during real-time updates
        scales: {
          x: {
            type: 'linear',
            title: {
              display: true,
              text: 'Time (seconds)',
              color: '#94a3b8',
              font: { family: 'Outfit', size: 11 }
            },
            ticks: { color: '#64748b' },
            grid: { color: 'rgba(255,255,255,0.03)' }
          },
          y: {
            title: {
              display: true,
              color: '#94a3b8',
              font: { family: 'Outfit', size: 11 }
            },
            ticks: { color: '#64748b' },
            grid: { color: 'rgba(255,255,255,0.03)' }
          }
        },
        plugins: {
          legend: {
            labels: {
              color: '#f8fafc',
              font: { family: 'Outfit', size: 11 }
            }
          },
          tooltip: {
            mode: 'index',
            intersect: false
          }
        }
      }
    });

    // Handle scrubber click on graph to update simulation state
    const canvas = document.getElementById('performance-chart');
    canvas.addEventListener('click', (evt) => {
      const points = chart.getElementsAtEventForMode(evt, 'nearest', { intersect: false }, true);
      if (points.length) {
        const xAxis = chart.scales.x;
        const clickX = evt.clientX - canvas.getBoundingClientRect().left;
        const timeVal = xAxis.getValueForPixel(clickX);
        if (timeVal >= 0 && timeVal <= sim.Ts) {
          updatePlaybackToTime(timeVal);
        }
      }
    });
  }

  // Update chart data based on active tab
  function updateChart() {
    if (!sim || !chart) return;
    
    const history = sim.history;
    const times = history.map(h => h.t);

    let datasets = [];
    let yTitle = '';

    if (activeTab === 'q') {
      yTitle = 'Quaternion Component / Boundaries';
      
      const q1 = history.map(h => h.q[0]);
      const q2 = history.map(h => h.q[1]);
      const q3 = history.map(h => h.q[2]);
      const rho_q = history.map(h => h.rho_q);
      const neg_rho_q = history.map(h => -h.rho_q);

      datasets = [
        { label: 'q1', data: times.map((t, idx) => ({x: t, y: q1[idx]})), borderColor: '#ef4444', borderWidth: 2, pointRadius: 0 },
        { label: 'q2', data: times.map((t, idx) => ({x: t, y: q2[idx]})), borderColor: '#10b981', borderWidth: 2, pointRadius: 0 },
        { label: 'q3', data: times.map((t, idx) => ({x: t, y: q3[idx]})), borderColor: '#38bdf8', borderWidth: 2, pointRadius: 0 },
        { label: '+rho_q', data: times.map((t, idx) => ({x: t, y: rho_q[idx]})), borderColor: '#64748b', borderDash: [5, 5], borderWidth: 1.5, pointRadius: 0 },
        { label: '-rho_q', data: times.map((t, idx) => ({x: t, y: neg_rho_q[idx]})), borderColor: '#64748b', borderDash: [5, 5], borderWidth: 1.5, pointRadius: 0 }
      ];
    } else if (activeTab === 'w') {
      yTitle = 'Angular Velocity (rad/s)';
      
      const w1 = history.map(h => h.omega[0]);
      const w2 = history.map(h => h.omega[1]);
      const w3 = history.map(h => h.omega[2]);
      const rho_w = history.map(h => h.rho_w);
      const neg_rho_w = history.map(h => -h.rho_w);

      datasets = [
        { label: 'w1', data: times.map((t, idx) => ({x: t, y: w1[idx]})), borderColor: '#ef4444', borderWidth: 2, pointRadius: 0 },
        { label: 'w2', data: times.map((t, idx) => ({x: t, y: w2[idx]})), borderColor: '#10b981', borderWidth: 2, pointRadius: 0 },
        { label: 'w3', data: times.map((t, idx) => ({x: t, y: w3[idx]})), borderColor: '#38bdf8', borderWidth: 2, pointRadius: 0 },
        { label: '+rho_w', data: times.map((t, idx) => ({x: t, y: rho_w[idx]})), borderColor: '#64748b', borderDash: [5, 5], borderWidth: 1.5, pointRadius: 0 },
        { label: '-rho_w', data: times.map((t, idx) => ({x: t, y: neg_rho_w[idx]})), borderColor: '#64748b', borderDash: [5, 5], borderWidth: 1.5, pointRadius: 0 }
      ];
    } else if (activeTab === 'u') {
      yTitle = 'Control Torque (Nm)';
      
      const u1 = history.map(h => h.uSat[0]);
      const u2 = history.map(h => h.uSat[1]);
      const u3 = history.map(h => h.uSat[2]);
      const uLim = times.map(() => sim.uMax);
      const neg_uLim = times.map(() => -sim.uMax);

      datasets = [
        { label: 'u1 (sat)', data: times.map((t, idx) => ({x: t, y: u1[idx]})), borderColor: '#ef4444', borderWidth: 2, pointRadius: 0 },
        { label: 'u2 (sat)', data: times.map((t, idx) => ({x: t, y: u2[idx]})), borderColor: '#10b981', borderWidth: 2, pointRadius: 0 },
        { label: 'u3 (sat)', data: times.map((t, idx) => ({x: t, y: u3[idx]})), borderColor: '#38bdf8', borderWidth: 2, pointRadius: 0 },
        { label: 'uMax', data: times.map((t, idx) => ({x: t, y: uLim[idx]})), borderColor: 'rgba(239, 68, 68, 0.4)', borderDash: [4, 4], borderWidth: 1.5, pointRadius: 0 },
        { label: '-uMax', data: times.map((t, idx) => ({x: t, y: neg_uLim[idx]})), borderColor: 'rgba(239, 68, 68, 0.4)', borderDash: [4, 4], borderWidth: 1.5, pointRadius: 0 }
      ];
    } else if (activeTab === 'theta') {
      yTitle = 'Estimate (thetaHat)';
      const th = history.map(h => h.thetaHat);

      datasets = [
        { label: 'thetaHat', data: times.map((t, idx) => ({x: t, y: th[idx]})), borderColor: '#c084fc', borderWidth: 2.5, pointRadius: 0 }
      ];
    }

    chart.data.datasets = datasets;
    chart.options.scales.y.title.text = yTitle;
    chart.options.scales.x.max = sim.Ts; // Match the X-axis limit to settling time
    chart.update('none'); // Update without animation for responsiveness
  }


  // --- Simulation Integration & Scenarios ---

  function applySimulationConfig() {
    const Ts = parseFloat(document.getElementById('input-Ts').value);
    const Kp = parseFloat(document.getElementById('input-Kp').value);
    const Kd = parseFloat(document.getElementById('input-Kd').value);
    const alpha = parseFloat(document.getElementById('input-alpha').value);
    const beta = parseFloat(document.getElementById('input-beta').value);
    const c = parseFloat(document.getElementById('input-c').value);
    const eta1 = parseFloat(document.getElementById('input-eta1').value);
    const uMax = parseFloat(document.getElementById('input-umax').value);

    // Initial states
    const q1 = parseFloat(document.getElementById('init-q1').value);
    const q2 = parseFloat(document.getElementById('init-q2').value);
    const q3 = parseFloat(document.getElementById('init-q3').value);
    // Reconstruct unit quaternion scalar part: q4 = sqrt(1 - qv^2)
    const qvSq = q1*q1 + q2*q2 + q3*q3;
    let q4 = 0;
    if (qvSq < 1.0) {
      q4 = Math.sqrt(1.0 - qvSq);
    } else {
      // Normalize
      const norm = Math.sqrt(qvSq);
      alert(`Warning: qv norm (${norm.toFixed(3)}) is greater than 1. Normalizing qv and setting q4 = 0.`);
      document.getElementById('init-q1').value = (q1 / norm).toFixed(3);
      document.getElementById('init-q2').value = (q2 / norm).toFixed(3);
      document.getElementById('init-q3').value = (q3 / norm).toFixed(3);
      return applySimulationConfig(); // Retry with normalized values
    }

    const w1 = parseFloat(document.getElementById('init-w1').value);
    const w2 = parseFloat(document.getElementById('init-w2').value);
    const w3 = parseFloat(document.getElementById('init-w3').value);

    const mode = document.getElementById('mode-constrained').classList.contains('active') ? 'constrained' : 'conventional';

    // Build configuration
    const config = {
      Ts, Kp, Kd, alpha, beta, c, eta1, uMax, mode,
      q0: [q1, q2, q3, q4],
      omega0: [w1, w2, w3]
    };

    // Instantiate or re-initialize simulation
    sim = new SpacecraftSimulation(config);

    // Pre-calculate full simulation run (20 seconds max)
    const simMaxTime = Math.max(20.0, Ts);
    sim.runTo(simMaxTime, 0.01); // 10ms step size

    // Set UI scrub limit
    const scrubber = document.getElementById('sim-scrubber');
    scrubber.max = simMaxTime;
    document.getElementById('max-time').textContent = `${simMaxTime.toFixed(2)}s`;

    // Set initial playback point
    updatePlaybackToTime(0.0);

    // Update performance charts
    updateChart();
  }

  // Telemetry update helper
  function updateTelemetryUI(data) {
    document.getElementById('hud-q').textContent = `[${data.q[0].toFixed(3)}, ${data.q[1].toFixed(3)}, ${data.q[2].toFixed(3)}, ${data.q[3].toFixed(3)}]`;
    document.getElementById('hud-w').textContent = `[${data.omega[0].toFixed(3)}, ${data.omega[1].toFixed(3)}, ${data.omega[2].toFixed(3)}] rad/s`;
    document.getElementById('hud-u').textContent = `[${data.uSat[0].toFixed(2)}, ${data.uSat[1].toFixed(2)}, ${data.uSat[2].toFixed(2)}] Nm`;
    document.getElementById('hud-theta').textContent = data.thetaHat.toFixed(4);

    // Update scrubber text and slider position
    document.getElementById('sim-time').textContent = `${data.t.toFixed(2)}s`;
    document.getElementById('sim-scrubber').value = data.t;

    // Set global time for Chart.js vertical line marker
    window.currentSimTime = data.t;
    
    // Throttle chart updates during active playback to prevent UI thread lockup
    const now = performance.now();
    if (!isPlaying || (now - lastChartUpdateTime > 50)) { // ~20fps throttle
      chart.update('none'); // Re-draw vertical cursor line
      lastChartUpdateTime = now;
    }
  }

  // Spacecraft 3D position update helper
  function update3DModel(q, uSat) {
    if (!spacecraft) return;
    
    // Set 3D rotation from simulation quaternion
    // q = [q1, q2, q3, q4] where q4 is scalar
    const threeQuat = new THREE.Quaternion(q[0], q[1], q[2], q[3]);
    spacecraft.setRotationFromQuaternion(threeQuat);

    // Animate nozzle fire based on control torque activity
    const plume = spacecraft.getObjectByName("plume");
    if (plume) {
      const uNorm = Math.sqrt(uSat[0]*uSat[0] + uSat[1]*uSat[1] + uSat[2]*uSat[2]);
      // Scale plume length with applied torque
      plume.scale.set(1, 1 + uNorm * 0.8, 1);
      plume.visible = uNorm > 0.05;
    }
  }

  // Playback state updater
  function updatePlaybackToTime(t) {
    if (!sim || !sim.history.length) return;

    // Find nearest recorded state in pre-calculated history
    // step size is 0.01, so index is t / 0.01
    let idx = Math.round(t / 0.01);
    idx = Math.max(0, Math.min(sim.history.length - 1, idx));

    const stateRecord = sim.history[idx];

    // Update UI components
    updateTelemetryUI(stateRecord);
    update3DModel(stateRecord.q, stateRecord.uSat);
  }


  // --- Playback Loop ---

  function animationLoop(timestamp) {
    if (!isPlaying) return;

    if (!lastTime) lastTime = timestamp;
    const elapsedSec = (timestamp - lastTime) / 1000.0;
    lastTime = timestamp;

    // Compute next time
    let nextTime = parseFloat(document.getElementById('sim-scrubber').value) + elapsedSec * playbackSpeed;
    const maxTime = parseFloat(document.getElementById('sim-scrubber').max);

    if (nextTime >= maxTime) {
      nextTime = maxTime;
      pauseSimulation();
    }

    updatePlaybackToTime(nextTime);

    // Render 3D Scene
    orbitControls.update();
    renderer.render(scene, camera);

    animationFrameId = requestAnimationFrame(animationLoop);
  }

  function playSimulation() {
    if (isPlaying) return;
    isPlaying = true;
    lastTime = 0;
    
    // Toggle play button icon
    document.getElementById('svg-play').classList.add('hidden');
    document.getElementById('svg-pause').classList.remove('hidden');

    // If we are at the end, wrap around to start
    const scrubber = document.getElementById('sim-scrubber');
    if (parseFloat(scrubber.value) >= parseFloat(scrubber.max) - 0.02) {
      scrubber.value = 0;
    }

    animationFrameId = requestAnimationFrame(animationLoop);
  }

  function pauseSimulation() {
    if (!isPlaying) return;
    isPlaying = false;
    if (animationFrameId) {
      cancelAnimationFrame(animationFrameId);
    }
    
    // Toggle play button icon
    document.getElementById('svg-play').classList.remove('hidden');
    document.getElementById('svg-pause').classList.add('hidden');
  }

  // --- Scenarios / Presets Configuration ---

  const scenarios = {
    1: { // Part 1 - 15s settling time
      mode: 'constrained',
      Ts: 15.0,
      Kp: 40.0,
      Kd: 20.0,
      alpha: 1.5,
      beta: 1.2,
      c: 1.0,
      eta1: 0.1,
      umax: 2.5,
      initQ: [0.2, -0.15, -0.25],
      initW: [0.03, 0.02, -0.01]
    },
    2: { // Part 1 - 10s settling time
      mode: 'constrained',
      Ts: 10.0,
      Kp: 40.0,
      Kd: 20.0,
      alpha: 1.5,
      beta: 1.2,
      c: 1.0,
      eta1: 0.1,
      umax: 2.5,
      initQ: [0.2, -0.15, -0.25],
      initW: [0.03, 0.02, -0.01]
    },
    3: { // Part 2 - Proposed constrained (initial conditions diff)
      mode: 'constrained',
      Ts: 15.0,
      Kp: 40.0,
      Kd: 20.0,
      alpha: 1.5,
      beta: 1.2,
      c: 1.0,
      eta1: 0.1,
      umax: 2.5,
      initQ: [0.3, -0.2, -0.3],
      initW: [-0.02, -0.02, 0.025]
    },
    4: { // Part 2 - Conventional PD
      mode: 'conventional',
      Ts: 15.0,
      Kp: 40.0,
      Kd: 20.0,
      alpha: 1.5,
      beta: 1.2,
      c: 1.0,
      eta1: 0.1,
      umax: 2.5,
      initQ: [0.3, -0.2, -0.3],
      initW: [-0.02, -0.02, 0.025]
    }
  };

  function loadScenario(id) {
    const sc = scenarios[id];
    if (!sc) return;

    // Update Scenario Active Buttons
    document.querySelectorAll('.scenario-btn').forEach((btn, idx) => {
      btn.classList.toggle('active', idx === (id - 1));
    });

    // Update toggle mode buttons
    if (sc.mode === 'constrained') {
      document.getElementById('mode-constrained').classList.add('active');
      document.getElementById('mode-conventional').classList.remove('active');
    } else {
      document.getElementById('mode-constrained').classList.remove('active');
      document.getElementById('mode-conventional').classList.add('active');
    }

    // Update Slider inputs
    document.getElementById('input-Ts').value = sc.Ts;
    document.getElementById('val-Ts').textContent = `${sc.Ts.toFixed(1)} s`;

    document.getElementById('input-Kp').value = sc.Kp;
    document.getElementById('val-Kp').textContent = sc.Kp.toFixed(1);

    document.getElementById('input-Kd').value = sc.Kd;
    document.getElementById('val-Kd').textContent = sc.Kd.toFixed(1);

    document.getElementById('input-alpha').value = sc.alpha;
    document.getElementById('val-alpha').textContent = sc.alpha.toFixed(1);

    document.getElementById('input-beta').value = sc.beta;
    document.getElementById('val-beta').textContent = sc.beta.toFixed(1);

    document.getElementById('input-c').value = sc.c;
    document.getElementById('val-c').textContent = sc.c.toFixed(1);

    document.getElementById('input-eta1').value = sc.eta1;
    document.getElementById('val-eta1').textContent = sc.eta1.toFixed(2);

    document.getElementById('input-umax').value = sc.umax;
    document.getElementById('val-umax').textContent = `${sc.umax.toFixed(1)} Nm`;

    // Numerical state inputs
    document.getElementById('init-q1').value = sc.initQ[0];
    document.getElementById('init-q2').value = sc.initQ[1];
    document.getElementById('init-q3').value = sc.initQ[2];

    document.getElementById('init-w1').value = sc.initW[0];
    document.getElementById('init-w2').value = sc.initW[1];
    document.getElementById('init-w3').value = sc.initW[2];

    pauseSimulation();
    applySimulationConfig();
    triggerMathJaxUpdate();
  }


  // --- Event Bindings ---

  // Connect slider change listeners
  const sliderMap = {
    'input-Ts': 'val-Ts',
    'input-Kp': 'val-Kp',
    'input-Kd': 'val-Kd',
    'input-alpha': 'val-alpha',
    'input-beta': 'val-beta',
    'input-c': 'val-c',
    'input-eta1': 'val-eta1',
    'input-umax': 'val-umax'
  };

  Object.entries(sliderMap).forEach(([sliderId, labelId]) => {
    const slider = document.getElementById(sliderId);
    const label = document.getElementById(labelId);
    slider.addEventListener('input', (e) => {
      let suffix = '';
      if (sliderId === 'input-Ts') suffix = ' s';
      if (sliderId === 'input-umax') suffix = ' Nm';
      
      const val = parseFloat(e.target.value);
      label.textContent = `${val.toFixed(sliderId === 'input-eta1' ? 2 : 1)}${suffix}`;
      
      // Auto-recalculate when parameters change
      pauseSimulation();
      applySimulationConfig();
    });
  });

  // Toggles for Controller Mode
  document.getElementById('mode-constrained').addEventListener('click', () => {
    document.getElementById('mode-constrained').classList.add('active');
    document.getElementById('mode-conventional').classList.remove('active');
    pauseSimulation();
    applySimulationConfig();
  });
  
  document.getElementById('mode-conventional').addEventListener('click', () => {
    document.getElementById('mode-conventional').classList.add('active');
    document.getElementById('mode-constrained').classList.remove('active');
    pauseSimulation();
    applySimulationConfig();
  });

  // Scenario Presets
  document.getElementById('btn-preset-1').addEventListener('click', () => loadScenario(1));
  document.getElementById('btn-preset-2').addEventListener('click', () => loadScenario(2));
  document.getElementById('btn-preset-3').addEventListener('click', () => loadScenario(3));
  document.getElementById('btn-preset-4').addEventListener('click', () => loadScenario(4));

  // Playback Control Button Bindings
  document.getElementById('btn-play-pause').addEventListener('click', () => {
    if (isPlaying) {
      pauseSimulation();
    } else {
      playSimulation();
    }
  });

  document.getElementById('btn-reset').addEventListener('click', () => {
    pauseSimulation();
    updatePlaybackToTime(0.0);
  });

  document.getElementById('sim-scrubber').addEventListener('input', (e) => {
    pauseSimulation();
    updatePlaybackToTime(parseFloat(e.target.value));
  });

  document.getElementById('sim-speed').addEventListener('change', (e) => {
    playbackSpeed = parseFloat(e.target.value);
  });

  // Telemetry HUD numeric input triggers
  document.getElementById('btn-apply-states').addEventListener('click', () => {
    pauseSimulation();
    applySimulationConfig();
  });

  // Scene view control helpers
  let gridVisible = true;
  document.getElementById('btn-toggle-grid').addEventListener('click', () => {
    gridVisible = !gridVisible;
    inertialGrid.visible = gridVisible;
  });

  document.getElementById('btn-reset-camera').addEventListener('click', () => {
    camera.position.set(5, 4, 8);
    orbitControls.target.set(0, 0, 0);
    orbitControls.update();
  });

  // Chart Tab Buttons
  const tabDescriptions = {
    'q': {
      title: 'Attitude Quaternion vector part tracking error q<sub>v</sub> vs FTPPF bounds',
      body: 'The vector components of the quaternion q<sub>v</sub> are kept strictly within the time-varying finite-time boundary &plusmn;&rho;<sub>q</sub>(t) (shown as dashed lines). As t &rarr; T<sub>s</sub>, q<sub>v</sub> &rarr; 0 (meaning the attitude quaternion q converges to the identity desired orientation q<sub>d</sub> = [0,0,0,1]<sup>T</sup>).'
    },
    'w': {
      title: 'Angular Velocity tracking error vs FTPPF bounds',
      body: 'The three components of the angular velocity &omega; are constrained inside the pre-defined finite-time boundaries &plusmn;&rho;<sub>&omega;</sub>(t). Notice that the conventional PD controller violates these boundaries, while the proposed control stays compliant.'
    },
    'u': {
      title: 'Control inputs with physical saturation bounds',
      body: 'Displays the actual torque command u applied to each axis. The red dashed lines show the actuator saturation limit (&plusmn;2.5 Nm). Notice the initial peaks when the system works hardest to arrest pointing rates.'
    },
    'theta': {
      title: 'Adaptive Estimation of Disturbance bounds',
      body: 'The adaptive parameter &theta;&#770;(t) dynamically estimates the upper bound of the external disturbances d(t) and actuator saturation mismatch, converging to a steady value without overshoot.'
    }
  };

  const tabs = ['q', 'w', 'u', 'theta'];
  tabs.forEach(tabId => {
    document.getElementById(`tab-${tabId}`).addEventListener('click', () => {
      tabs.forEach(t => document.getElementById(`tab-${t}`).classList.remove('active'));
      document.getElementById(`tab-${tabId}`).classList.add('active');
      activeTab = tabId;
      
      // Update chart desc
      document.getElementById('desc-title').innerHTML = tabDescriptions[tabId].title;
      document.getElementById('desc-body').innerHTML = tabDescriptions[tabId].body;
      
      updateChart();
    });
  });

  // --- Start Up App ---

  init3D();
  initChart();
  
  // Load scenario 1 by default
  loadScenario(1);

  // Static 3D Render Loop (for when the simulation is paused but camera moves)
  function staticRenderLoop() {
    if (!isPlaying) {
      orbitControls.update();
      renderer.render(scene, camera);
    }
    requestAnimationFrame(staticRenderLoop);
  }
  staticRenderLoop();
});
