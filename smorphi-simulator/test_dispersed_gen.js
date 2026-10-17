const assert = require("assert");

// Expose globals for simulator scripts
global.CONFIG = require("./js/config.js");
global.RobotGeometry = require("./js/geometry.js");
const { ArenaMap } = require("./js/map.js");
const { CargoManager } = require("./js/cargo.js");
const { FogOfWar } = require("./js/fog.js");
const { CheckpointManager } = require("./js/checkpoints.js");

console.log("Running self-check tests for dispersed obstacle map generation & dynamic cargo spawning...");

// 1. Verify DENSITY_PRESETS
assert.strictEqual(CONFIG.DENSITY_PRESETS.LOW.minObstacles, 8);
assert.strictEqual(CONFIG.DENSITY_PRESETS.LOW.maxObstacles, 12);
assert.strictEqual(CONFIG.DENSITY_PRESETS.MEDIUM.minObstacles, 14);
assert.strictEqual(CONFIG.DENSITY_PRESETS.MEDIUM.maxObstacles, 18);
assert.strictEqual(CONFIG.DENSITY_PRESETS.HIGH.minObstacles, 18);
assert.strictEqual(CONFIG.DENSITY_PRESETS.HIGH.maxObstacles, 22);

const map = new ArenaMap(5.0, 5.0);
const cargo = new CargoManager();

// Test multiple seeds for MEDIUM preset
const routeMidpoints = [];

for (let seed = 101; seed <= 120; seed++) {
  map.generateMap("MEDIUM", seed);
  const result = map.lastGenerationResult;

  assert(result.valid, `Map generation failed for seed ${seed}`);
  assert(map.obstacles.length >= 14 && map.obstacles.length <= 18,
    `Obstacle count ${map.obstacles.length} out of range [14, 18] for seed ${seed}`);

  // Test inter-obstacle center distance >= 0.45m
  for (let i = 0; i < map.obstacles.length; i++) {
    const o1 = map.obstacles[i];
    const c1x = o1.x + o1.w / 2;
    const c1y = o1.y + o1.h / 2;

    for (let j = i + 1; j < map.obstacles.length; j++) {
      const o2 = map.obstacles[j];
      const c2x = o2.x + o2.w / 2;
      const c2y = o2.y + o2.h / 2;
      const dist = Math.hypot(c1x - c2x, c1y - c2y);
      assert(dist >= 0.44, `Obstacles ${i} and ${j} too close: ${dist.toFixed(3)}m < 0.45m`);
    }
  }

  // Record max deviation from diagonal line along path
  assert(map.reachablePath.length > 5, `Reachable path too short`);
  let maxDev = 0;
  for (const pt of map.reachablePath) {
    const d = Math.abs(pt.x - pt.y) / Math.SQRT2;
    if (d > maxDev) maxDev = d;
  }
  routeMidpoints.push(maxDev);

  // Test Cargo Spawning
  cargo.spawnCubes(map);
  assert.strictEqual(cargo.cubes.length, 3, "Expected 3 cargo cubes");

  for (let i = 0; i < 3; i++) {
    const cube = cargo.cubes[i];
    // Bounds margin
    assert(cube.x >= 0.59 && cube.x <= 4.41, `Cube ${i} x out of bounds: ${cube.x}`);
    assert(cube.y >= 0.59 && cube.y <= 4.41, `Cube ${i} y out of bounds: ${cube.y}`);

    // Distance to spawn & goal
    const distSpawn = Math.hypot(cube.x - map.spawn.x, cube.y - map.spawn.y);
    const distGoal = Math.hypot(cube.x - map.goal.x, cube.y - map.goal.y);
    assert(distSpawn >= 0.89, `Cube ${i} too close to spawn: ${distSpawn.toFixed(3)}m < 0.9m`);
    assert(distGoal >= 0.79, `Cube ${i} too close to goal: ${distGoal.toFixed(3)}m < 0.8m`);

    // Obstacle perimeter clearance >= 0.22m
    for (const obs of map.obstacles) {
      const dx = Math.max(obs.x - cube.x, 0, cube.x - (obs.x + obs.w));
      const dy = Math.max(obs.y - cube.y, 0, cube.y - (obs.y + obs.h));
      const distObs = Math.hypot(dx, dy);
      assert(distObs >= 0.21, `Cube ${i} too close to obstacle: ${distObs.toFixed(3)}m < 0.22m`);
    }

    // Inter-cube distance >= 1.0m
    for (let j = i + 1; j < 3; j++) {
      const other = cargo.cubes[j];
      const distCubes = Math.hypot(cube.x - other.x, cube.y - other.y);
      assert(distCubes >= 0.99, `Cubes ${i} and ${j} too close: ${distCubes.toFixed(3)}m < 1.0m`);
    }

    // Reachability
    assert(map.isPointReachable(cube.x, cube.y), `Cube ${i} at (${cube.x.toFixed(2)}, ${cube.y.toFixed(2)}) is not reachable`);
  }
}

// Check route diversity: paths should branch away from diagonal
let diverseBranches = 0;
for (const dev of routeMidpoints) {
  if (dev > 0.3) {
    diverseBranches++;
  }
}
console.log(`Max path deviations: ${routeMidpoints.map(d => d.toFixed(2)).join(", ")}`);
console.log(`Diverse branches: ${diverseBranches}/${routeMidpoints.length}`);
assert(diverseBranches > 0, "Routes lacked branching diversity across seeds");

// Test LOW and HIGH density presets
map.generateMap("LOW", 42);
assert(map.lastGenerationResult.valid);
assert(map.obstacles.length >= 8 && map.obstacles.length <= 12);

map.generateMap("HIGH", 42);
assert(map.lastGenerationResult.valid);
assert(map.obstacles.length >= 18 && map.obstacles.length <= 22);

console.log("All assertions passed successfully! (20 maps verified)");

// ============================================================
// Fog of War & Checkpoint self-checks
// ============================================================
console.log("\nRunning fog-of-war & checkpoint tests...");

// --- Fog: reveal mask ---
const fog = new FogOfWar(5.0, 5.0, 100);
assert.strictEqual(fog.isExplored(0.5, 0.5), false, "Fog should start fully unexplored");
fog.revealDisk(0.5, 0.5, 0.3);
assert(fog.isExplored(0.5, 0.5), "Center of reveal disc must be explored");
assert(!fog.isExplored(4.0, 4.0), "Far corner must stay fogged");
assert(fog.dirty, "Revealing must raise the dirty flag for texture repaint");

// --- Fog: line-of-sight along a ray stops at the hit point ---
fog.reset();
assert.strictEqual(fog.revealedCount, 0, "Reset must clear the mask");
fog.revealFromRays(0.5, 0.5, [{ x: 1.5, y: 0.5 }]);
assert(fog.isExplored(1.0, 0.5), "Cells along the ray up to the hit must be explored");
assert(!fog.isExplored(3.0, 0.5), "Cells beyond the LiDAR hit must remain fogged");

// --- Checkpoints: placed at cube spawn origins, chained reveal ---
cargo.spawnCubes(map);
const cpMgr = new CheckpointManager();
cpMgr.syncToCargo(cargo);
assert.strictEqual(cpMgr.checkpoints.length, 3, "Expected one checkpoint per cargo cube");
for (let i = 0; i < 3; i++) {
  const cp = cpMgr.checkpoints[i];
  assert.strictEqual(cp.x, cargo.cubes[i].spawnX, `CP ${cp.id} x must match cube spawn x`);
  assert.strictEqual(cp.y, cargo.cubes[i].spawnY, `CP ${cp.id} y must match cube spawn y`);
  assert.strictEqual(cp.discovered, false, `CP ${cp.id} must start undiscovered`);
  assert.strictEqual(cp.isNext, i === 0, `Only the first checkpoint starts flagged as next`);
}

const probe = { x: 0, y: 0, theta: 0 };
probe.x = cpMgr.checkpoints[0].x; probe.y = cpMgr.checkpoints[0].y;
let discoveredEvents = 0;
cpMgr.onDiscover = () => { discoveredEvents++; };
cpMgr.update(1 / 60, probe, map);
assert(cpMgr.checkpoints[0].discovered, "CP-1 must be discovered on approach");
assert.strictEqual(cpMgr.revealedTarget.x, cpMgr.checkpoints[1].x, "CP-1 must reveal CP-2 coordinate");

probe.x = cpMgr.checkpoints[1].x; probe.y = cpMgr.checkpoints[1].y;
cpMgr.update(1 / 60, probe, map);
assert(cpMgr.checkpoints[1].discovered, "CP-2 must be discovered on approach");
assert.strictEqual(cpMgr.revealedTarget.x, cpMgr.checkpoints[2].x, "CP-2 must reveal CP-3 coordinate");

probe.x = cpMgr.checkpoints[2].x; probe.y = cpMgr.checkpoints[2].y;
cpMgr.update(1 / 60, probe, map);
assert(cpMgr.checkpoints[2].discovered, "CP-3 must be discovered on approach");
assert.strictEqual(cpMgr.revealedTarget.label, "FINISH", "Last checkpoint must reveal the finish point");
assert.strictEqual(cpMgr.revealedTarget.x, map.goal.x, "Finish target x must equal arena goal");
assert.strictEqual(discoveredEvents, 3, "Exactly three discovery events expected");

// --- Checkpoints: re-sync on respawn clears discovered state ---
cpMgr.syncToCargo(cargo, map);
assert.strictEqual(cpMgr.revealedTarget, null, "Re-sync must clear the revealed target");
assert(cpMgr.checkpoints.every(c => !c.discovered), "Re-sync must reset all discovered flags");

// --- Checkpoint data exposed to scripts ---
const cpData = cpMgr.getScriptData();
assert.strictEqual(cpData.list.length, 3);
assert.strictEqual(cpData.discoveredCount, 0);
assert.strictEqual(cpData.navTarget, null);

// --- Limited Sensor Range & AprilTag Payload Assertions ---
assert.strictEqual(CONFIG.LIDAR.MAX_RANGE, 2.0, "LiDAR max range must be bounded to 2.0m for true fog");
assert.strictEqual(CONFIG.CHECKPOINT.SCAN_RADIUS, 0.65, "AprilTag scan proximity must be 0.65m");
assert.strictEqual(CONFIG.CHECKPOINT.DETECTION_RANGE, 2.0, "AprilTag max visual detection must be 2.0m");

for (const cp of cpMgr.checkpoints) {
  assert(cp.payload, `CP ${cp.id} must have encoded payload`);
  assert(typeof cp.payload.nextX === "number", `CP ${cp.id} payload must contain nextX coordinate`);
  assert(typeof cp.payload.nextY === "number", `CP ${cp.id} payload must contain nextY coordinate`);
}

// Discover CP-1 and test payload decoding
probe.x = cpMgr.checkpoints[0].x;
probe.y = cpMgr.checkpoints[0].y;
cpMgr.update(1 / 60, probe, map);
assert(cpMgr.lastDecodedTag, "Discovering a tag must populate lastDecodedTag");
assert.strictEqual(cpMgr.lastDecodedTag.tagId, 1);
assert.strictEqual(cpMgr.lastDecodedTag.coordinate.x, cpMgr.checkpoints[1].x);

console.log("Fog-of-war & checkpoint assertions passed successfully!");

// =============================================================
// Camera FOV & Detection Range tests
// =============================================================
console.log("\nRunning camera FOV & detection range tests...");

cargo.spawnCubes(map);
const fovMgr = new CheckpointManager();
fovMgr.syncToCargo(cargo, map);

const cp0 = fovMgr.checkpoints[0];

// Robot 0.5m away from tag, facing AWAY (theta = opposite direction)
const awayAngle = Math.atan2(cp0.y - 2.5, cp0.x - 2.5); // angle FROM center TO tag
const fovProbe = { x: cp0.x - Math.cos(awayAngle) * 0.5, y: cp0.y - Math.sin(awayAngle) * 0.5, theta: awayAngle + Math.PI };
fovMgr.update(1 / 60, fovProbe, map);
assert(!fovMgr.checkpoints[0].discovered, "Tag must NOT be discovered when robot faces away (outside FOV)");

// Same position, now facing TOWARD the tag
fovProbe.theta = awayAngle;
fovMgr.update(1 / 60, fovProbe, map);
assert(fovMgr.checkpoints[0].discovered, "Tag must be discovered when robot faces toward it (inside FOV)");

// Detection range: robot 3.0m away (beyond 0.65m scan radius) should NOT detect
const fovMgr2 = new CheckpointManager();
fovMgr2.syncToCargo(cargo, map);
const cp0b = fovMgr2.checkpoints[0];
const toTag = Math.atan2(cp0b.y - 1.0, cp0b.x - 1.0);
const farProbe = { x: cp0b.x - Math.cos(toTag) * 3.0, y: cp0b.y - Math.sin(toTag) * 3.0, theta: toTag };
fovMgr2.update(1 / 60, farProbe, map);
assert(!fovMgr2.checkpoints[0].discovered, "Tag must NOT be discovered beyond SCAN_RADIUS (0.65m)");

// getVisibleTags should use DETECTION_RANGE (2.0m), not SCAN_RADIUS
const fovMgr3 = new CheckpointManager();
fovMgr3.syncToCargo(cargo, map);
const cp0c = fovMgr3.checkpoints[0];
const toTag3 = Math.atan2(cp0c.y - 1.0, cp0c.x - 1.0);
const midProbe = { x: cp0c.x - Math.cos(toTag3) * 1.5, y: cp0c.y - Math.sin(toTag3) * 1.5, theta: toTag3 };
const visTags = fovMgr3.getVisibleTags(midProbe, map);
// Tag should be visible at 1.5m (within 2.0m detection range) if LOS is clear
if (fovMgr3._hasLineOfSight(midProbe.x, midProbe.y, cp0c.x, cp0c.y, map)) {
  assert(visTags.length > 0, "getVisibleTags must detect tag at 1.5m (within DETECTION_RANGE 2.0m)");
}

console.log("Camera FOV & detection range assertions passed successfully!");

