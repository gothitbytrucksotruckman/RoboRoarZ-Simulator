/**
 * Interactive Code Injection & Script Execution Engine
 * Platform: Single-Block Smorphi Base Unit with Front Cargo Mesh Scoop
 * Compiles and executes user JavaScript navigation scripts in real-time.
 */

class CodeEngine {
  constructor() {
    this.compiledFunction = null;
    this.memory = {}; // Preserved state between ticks
    this.isRunning = false;
    this.hasError = false;
    this.lastErrorMessage = null;
    this.logCallbacks = [];
    this.lastLogTime = 0;

    // Default Preset Scripts
    this.presets = {
      // 1. Autonomous AprilTag-Guided Exploration & Coordinate Navigation (Primary Default)
      apriltag_nav: `/**
 * SMORPHI AUTONOMOUS APRILTAG NAVIGATION LOOP
 * ----------------------------------------------------
 * Platform: Single-Block Smorphi with LiDAR + Camera
 * Fog of War: True sensor reach (max 2.0m radius, occluded by walls)
 *
 * Sequence:
 * 1. Autonomously search for AprilTag
 * 2. Decode designated coordinate directly from tag payload
 * 3. Navigate to designated coordinate with LiDAR obstacle avoidance
 * 4. Arrive at coordinate -> scan AprilTag at location for next coordinate
 * 5. Repeat until all AprilTags scanned, then proceed to FINISH
 */

// Initialize state machine
if (!memory.init) {
  memory.state = "SEARCH_TAG";
  memory.spinTimer = 0;
  memory.lastScannedId = null;
  memory.init = true;
  robot.log("=== Smorphi Autonomous AprilTag Navigation Initialized ===");
  robot.log("Sensor constraints: 2.0m LiDAR + Forward Vision Camera (True Fog of War)");
}

const frontDist = sensors.lidar.getFront(30);
const leftDist  = sensors.lidar.getLeft(40);
const rightDist = sensors.lidar.getRight(40);
const tag = sensors.apriltag;
const target = sensors.target;

// Check if entire mission is complete
if (sensors.mission && sensors.mission.completed) {
  robot.setVelocity(0, 0, 0);
  return;
}

// Log whenever a new tag payload is decoded
if (tag && tag.lastDecoded && tag.lastDecoded.tagId !== memory.lastScannedId) {
  memory.lastScannedId = tag.lastDecoded.tagId;
  const nextX = tag.lastDecoded.coordinate.x.toFixed(2);
  const nextY = tag.lastDecoded.coordinate.y.toFixed(2);
  robot.log(\`[DECODE] Tag #\${tag.lastDecoded.tagId} scanned! Next: \${tag.lastDecoded.nextLabel} @ (\${nextX}, \${nextY})\`);
  memory.state = "NAVIGATE_TO_COORD";
}

// STATE 1: SEARCH_TAG (Look around for AprilTag within vision range)
if (memory.state === "SEARCH_TAG") {
  if (target && target.active) {
    memory.state = "NAVIGATE_TO_COORD";
    return;
  }

  memory.spinTimer = (memory.spinTimer || 0) + dt;

  if (frontDist < 0.35) {
    robot.setVelocity(-0.15, 0, 0.9);
  } else if (memory.spinTimer < 6.0) {
    // 360-degree rotation sweep to detect tag
    robot.setVelocity(0, 0, 0.85);
  } else {
    // If spin completed without tag, explore forward into clearest opening
    const steer = leftDist > rightDist ? 0.35 : -0.35;
    robot.setVelocity(0.20, 0, steer);
    if (memory.spinTimer > 10.0) memory.spinTimer = 0;
  }
  return;
}

// STATE 2: NAVIGATE_TO_COORD (Travel to the designated coordinate)
if (memory.state === "NAVIGATE_TO_COORD") {
  if (!target || !target.active) {
    memory.state = "SEARCH_TAG";
    return;
  }

  // Check if reached destination coordinate
  if (target.reached || target.distance < 0.35) {
    robot.setVelocity(0, 0, 0);
    if (target.isFinish) {
      robot.log("=== Destination FINISH zone reached! Mission complete! ===");
      return;
    }
    robot.log(\`[ARRIVED] Reached \${target.label}! Scanning AprilTag for next coordinate...\`);
    memory.state = "SCAN_AT_LOCATION";
    memory.scanTimer = 0;
    return;
  }

  // Obstacle avoidance with 2.0m LiDAR
  const targetAngle = target.angle;
  let steer = Math.max(-1.5, Math.min(1.5, targetAngle * 1.8));
  let fwdSpeed = Math.min(0.35, Math.max(0.12, target.distance * 0.5));
  let strafeSpeed = 0.0;

  if (frontDist < 0.50) {
    // Obstacle ahead: evade laterally toward clearer corridor
    const dodge = leftDist > rightDist ? 0.28 : -0.28;
    strafeSpeed = dodge;
    fwdSpeed = 0.05;
    steer = dodge > 0 ? 0.7 : -0.7;
  } else if (leftDist < 0.28) {
    strafeSpeed = -0.18;
  } else if (rightDist < 0.28) {
    strafeSpeed = 0.18;
  }

  robot.setVelocity(fwdSpeed, strafeSpeed, steer);
  return;
}

// STATE 3: SCAN_AT_LOCATION (Scan AprilTag at current location for next waypoint)
if (memory.state === "SCAN_AT_LOCATION") {
  memory.scanTimer = (memory.scanTimer || 0) + dt;

  // Gentle rotate in place so tag enters camera FOV
  robot.setVelocity(0, 0, 0.75);

  // Once new target is decoded, resume navigation
  if (target && target.active && !target.reached) {
    robot.log(\`[LOCKED] Next coordinate locked: \${target.label} (\${target.distance.toFixed(2)}m)\`);
    memory.state = "NAVIGATE_TO_COORD";
  }
}
`,

      // 2. Autonomous Cargo Retrieval & Transport
      default_avoidance: `/**
 * SMORPHI AUTONOMOUS CARGO RETRIEVAL & TRANSPORT
 * ----------------------------------------------------
 * Platform: Single-Block Smorphi with Front Cargo Mesh Scoop
 * Inputs:
 *   - sensors.cargo: { count, totalMassKg, hasContact, frontClearance, cubes: [...] }
 *   - sensors.lidar: 360-deg laser array (.getFront(), .getLeft(), .getRight())
 *   - sensors.imu:   { heading, yaw_rate }
 *   - sensors.pose:  { x, y, theta, comOffsetX, totalMass }
 *   - sensors.target:{ distance, angle, reached }
 *
 * Outputs:
 *   - robot.setVelocity(vx, vy, omega): Set 3-DOF holonomic speed (m/s, rad/s)
 *   - robot.getLoadedMass(): Total mass in kg
 *   - robot.log(message): Output text to simulator console
 */

// Initialize state machine
if (!memory.initialized) {
  memory.state = "SEARCH_CARGO";
  memory.initialized = true;
  robot.log("=== Smorphi Cargo Retrieval Mission Started ===");
}

const frontDist = sensors.lidar.getFront(25);
const leftDist  = sensors.lidar.getLeft(35);
const rightDist = sensors.lidar.getRight(35);
const cargo     = sensors.cargo;

// Cek jika finish point sudah tercapai
if ((sensors.mission && sensors.mission.completed) || (sensors.target.active && sensors.target.reached)) {
  robot.setVelocity(0, 0, 0);
  return;
}

// JIKA SEMUA 3 KUBUS SUDAH TERKUMPUL, MENUJU ZONA FINIS (PASSIVE GOAL)
if (cargo.count >= 3) {
  if (Math.random() < 0.01) {
    robot.log(\`Muatan Penuh (3/3) [\${cargo.totalMassKg.toFixed(2)} kg]. Menuju zona akhir...\`);
  }

  // Hindari rintangan sambil bergerak ke arah target finis
  if (frontDist < 0.55) {
    const dodge = leftDist > rightDist ? 0.28 : -0.28;
    robot.setVelocity(-0.05, dodge, 0.6);
  } else if (sensors.target.active) {
    // Arahkan ke target dari decoded AprilTag
    const targetAngle = sensors.target.angle;
    const steer = Math.max(-1.0, Math.min(1.0, targetAngle * 1.5));
    robot.setVelocity(0.30, 0.0, steer);
  } else {
    // No target decoded yet, explore forward
    robot.setVelocity(0.25, 0.0, 0.0);
  }
  return;
}

// STATE 1: PENCARIAN & PENDEKATAN KUBUS (SEARCH & APPROACH)
if (memory.state === "SEARCH_CARGO") {
  // Cari kubus terdekat yang belum terjaring
  const target = cargo.cubes.find(c => c.state === "UNTOUCHED");

  if (target && target.dist < 2.0) {
    // Hadapkan robot langsung ke arah kubus
    const headingError = target.relAngle;
    const turnSpeed = Math.max(-1.5, Math.min(1.5, headingError * 2.2));

    // Bergerak mendekat dengan kecepatan proporsional
    const fwdSpeed = target.dist > 0.40 ? 0.30 : 0.15;
    robot.setVelocity(fwdSpeed, 0.0, turnSpeed);

    if (target.dist < 0.25) {
      memory.state = "SCOOP_ENGAGE";
      robot.log(\`Mendekati kubus #\${target.id} (\${target.color}), menyapukan sekat...\`);
    }
  } else {
    // Navigasi jelajah lorong arena
    if (frontDist < 0.60) {
      const dodge = leftDist > rightDist ? 0.25 : -0.25;
      robot.setVelocity(0.0, dodge, 0.8 * Math.sign(dodge));
    } else {
      robot.setVelocity(0.35, 0.0, 0.0);
    }
  }
}

// STATE 2: MENYAPU & MENGUNCI KUBUS KE DALAM SEKAT JARING (SCOOP ENGAGE)
else if (memory.state === "SCOOP_ENGAGE") {
  // Dorong lurus ke depan agar kubus melewati bibir penahan bawah sekat
  robot.setVelocity(0.20, 0.0, 0.0);

  if (cargo.hasContact || cargo.frontClearance < 0.05) {
    robot.log(\`Kubus berhasil ditampung! Beban saat ini: \${cargo.totalMassKg.toFixed(2)} kg (\${cargo.count}/3)\`);
    memory.state = "SEARCH_CARGO";
  }
}
`,

      // 2. Goal Seeking with Artificial Potential Field
      goal_seeker: `/**
 * ROBO-ROARZ POTENTIAL FIELD GOAL SEEKER
 * Platform: Single-Block Smorphi
 */

if (!memory.init) {
  memory.init = true;
  robot.log("Target Seeking Navigator Started!");
}

const target = sensors.target;
const frontDist = sensors.lidar.getFront(35);
const leftDist = sensors.lidar.getLeft(45);
const rightDist = sensors.lidar.getRight(45);

if (target.reached || !target.active) {
  robot.setVelocity(0, 0, 0);
  if (target.reached) robot.log("Zona Target Tercapai!");
  return;
}

// 1. Attractive force towards target
let targetAngle = target.angle;
let attractiveVx = Math.cos(targetAngle) * 0.32;
let attractiveVy = Math.sin(targetAngle) * 0.32;

// 2. Repulsive force from obstacles
let repulseVx = 0;
let repulseVy = 0;

if (frontDist < 0.65) {
  const urgency = (0.65 - frontDist) / 0.65;
  repulseVx -= urgency * 0.45;
  if (leftDist > rightDist) {
    repulseVy += urgency * 0.35;
  } else {
    repulseVy -= urgency * 0.35;
  }
}

let cmdVx = attractiveVx + repulseVx;
let cmdVy = attractiveVy + repulseVy;
let cmdOmega = targetAngle * 0.8;

robot.setVelocity(cmdVx, cmdVy, cmdOmega);
`,

      // 3. Mecanum Holonomic Orbit (No-Turn)
      holonomic_drift: `/**
 * MECANUM HOLONOMIC ORBIT DEMO
 * Demonstrates 3-DOF lateral strafing without altering robot heading.
 */

if (!memory.timer) memory.timer = 0;
memory.timer += dt;

const phase = (memory.timer % 4.0) / 4.0;
const angle = phase * Math.PI * 2;

// Move along a circle in local frame without rotating
const vx = Math.cos(angle) * 0.30;
const vy = Math.sin(angle) * 0.30;

robot.setVelocity(vx, vy, 0.0);
`,

      // 4. PID Wall Follower
      wall_follower: `/**
 * PID WALL FOLLOWER (Single-Block Smorphi)
 * Maintains constant 0.35m distance from the right wall.
 */

const targetDist = 0.35;
const currentDist = sensors.lidar.getRight(30);

if (!memory.prevError) memory.prevError = 0;

const error = targetDist - currentDist;
const derivative = (error - memory.prevError) / dt;
memory.prevError = error;

const Kp = 1.8;
const Kd = 0.4;
const steer = Kp * error + Kd * derivative;

robot.setVelocity(0.28, 0.0, -steer);
`,
    };
  }

  /**
   * Compile user script text into an executable function
   * @param {string} codeText
   */
  compileScript(codeText) {
    this.hasError = false;
    this.lastErrorMessage = null;

    try {
      this.compiledFunction = new Function("sensors", "robot", "memory", "dt", codeText);
      this.memory = {};
      this.log("Code compiled successfully.");
      return { success: true };
    } catch (err) {
      this.hasError = true;
      this.lastErrorMessage = `Syntax Error: ${err.message}`;
      this.log(`[SYNTAX ERROR] ${err.message}`, "error");
      return { success: false, error: err.message };
    }
  }

  /**
   * Execute one simulation tick of user code
   * @param {Object} sensorsInput - Read-only sensor snapshot
   * @param {SmorphiRobot} robot - Robot model instance
   * @param {number} dt - Delta time
   */
  executeTick(sensorsInput, robot, dt) {
    if (!this.isRunning || !this.compiledFunction || this.hasError) {
      return;
    }

    const robotAPI = {
      setVelocity: (vx, vy, omega) => {
        robot.setVelocity(vx, vy, omega);
      },
      setShape: (shape) => {
        return true;
      },
      getLoadedMass: () => {
        return robot.getLoadedMass();
      },
      stop: () => {
        robot.stop();
      },
      log: (msg) => {
        this.log(String(msg), "user");
      },
    };

    try {
      this.compiledFunction(sensorsInput, robotAPI, this.memory, dt);
    } catch (err) {
      this.hasError = true;
      this.lastErrorMessage = `Runtime Error: ${err.message}`;
      this.log(`[RUNTIME ERROR] ${err.message}`, "error");
      robot.stop();
    }
  }

  /**
   * Logging facility for simulator console
   */
  log(message, type = "info") {
    const now = performance.now();
    if (type === "user" && now - this.lastLogTime < 50) return;
    this.lastLogTime = now;

    const entry = {
      time: new Date().toLocaleTimeString(),
      type,
      text: message,
    };

    for (const cb of this.logCallbacks) {
      cb(entry);
    }
  }

  onLog(callback) {
    this.logCallbacks.push(callback);
  }

  start() {
    this.isRunning = true;
    this.hasError = false;
  }

  stop() {
    this.isRunning = false;
  }

  resetMemory() {
    this.memory = {};
  }
}

if (typeof module !== "undefined" && module.exports) {
  module.exports = CodeEngine;
}
