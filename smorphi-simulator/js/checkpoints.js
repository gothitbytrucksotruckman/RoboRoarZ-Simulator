/**
 * Checkpoint System (AprilTag pads)
 * Every cargo cube spawns on top of a floor AprilTag "checkpoint".
 * Physical tags encode coordinate payloads directly:
 * When the robot's camera detects a tag within range & FOV with clear LOS,
 * the tag is decoded, revealing the designated coordinate of the NEXT waypoint (or FINISH).
 */

class CheckpointManager {
  constructor() {
    this.checkpoints = [];
    this.revealedTarget = null; // { x, y, label, isFinish } | null
    this.lastDecodedTag = null; // Decoded AprilTag payload
    this.onDiscover = null;     // (checkpoint, target) => void
  }

  /**
   * (Re)build checkpoints from the cargo cubes' *spawn origins*.
   * The pad stays on the floor even after the box is dragged away.
   * @param {CargoManager} cargo
   * @param {ArenaMap} [map]
   */
  syncToCargo(cargo, map = null) {
    this.checkpoints = [];
    this.revealedTarget = null;
    this.lastDecodedTag = null;
    const colors = CONFIG.CARGO.COLORS;
    const goalX = map && map.goal ? map.goal.x : CONFIG.ARENA.DEFAULT_GOAL.x;
    const goalY = map && map.goal ? map.goal.y : CONFIG.ARENA.DEFAULT_GOAL.y;

    for (let i = 0; i < cargo.cubes.length; i++) {
      const cube = cargo.cubes[i];
      const c = colors[i % colors.length];
      const nextCube = i + 1 < cargo.cubes.length ? cargo.cubes[i + 1] : null;

      const targetCoord = nextCube
        ? { x: nextCube.spawnX, y: nextCube.spawnY, label: `CP-${i + 2}`, isFinish: false }
        : { x: goalX, y: goalY, label: "FINISH", isFinish: true };

      this.checkpoints.push({
        id: i + 1,
        x: cube.spawnX,
        y: cube.spawnY,
        colorHex: c.hex,
        colorCss: c.css,
        label: `CP-${i + 1}`,
        discovered: false,
        isNext: i === 0,
        payload: {
          tagId: i + 1,
          nextX: targetCoord.x,
          nextY: targetCoord.y,
          nextLabel: targetCoord.label,
          isFinish: targetCoord.isFinish,
          dataString: `TAG_${i + 1}:X=${targetCoord.x.toFixed(2)},Y=${targetCoord.y.toFixed(2)},NEXT=${targetCoord.label}`,
        },
      });
    }
  }

  /**
   * Scan for undiscovered checkpoints within camera sight.
   * Enforces vision radius (max 2m), forward FOV cone, and line-of-sight occlusion.
   * @param {number} dt
   * @param {SmorphiRobot} robot
   * @param {ArenaMap} map
   */
  update(dt, robot, map) {
    const scanRadius = CONFIG.CHECKPOINT.SCAN_RADIUS || 0.65;
    const fovDeg = CONFIG.CHECKPOINT.FOV_DEG || 120;
    const halfFovRad = (fovDeg * Math.PI) / 360;

    for (let i = 0; i < this.checkpoints.length; i++) {
      const cp = this.checkpoints[i];
      if (cp.discovered) continue;
      if (!cp.isNext) continue; // Sequentially scan tags: CP-1 -> CP-2 -> CP-3 -> FINISH

      const dx = cp.x - robot.x;
      const dy = cp.y - robot.y;
      const dist = Math.hypot(dx, dy);

      // 1. Tag decode range limit (floor AprilTag decode proximity)
      if (dist > scanRadius) continue;

      // 2. Camera Field-of-View (must face tag unless within immediate contact)
      if (typeof robot.theta === "number" && dist > 0.35) {
        let relAngle = Math.atan2(dy, dx) - robot.theta;
        while (relAngle > Math.PI) relAngle -= 2 * Math.PI;
        while (relAngle < -Math.PI) relAngle += 2 * Math.PI;
        if (Math.abs(relAngle) > halfFovRad) continue;
      }

      // 3. Line-of-sight test (obstacles occlude view)
      if (!this._hasLineOfSight(robot.x, robot.y, cp.x, cp.y, map)) continue;

      this._discover(i, map);
      break;
    }
  }

  _discover(i, map) {
    const cp = this.checkpoints[i];
    cp.discovered = true;

    let target;
    if (cp.payload) {
      target = { x: cp.payload.nextX, y: cp.payload.nextY, label: cp.payload.nextLabel, isFinish: cp.payload.isFinish };
    } else if (i + 1 < this.checkpoints.length) {
      const next = this.checkpoints[i + 1];
      target = { x: next.x, y: next.y, label: next.label, isFinish: false };
    } else {
      const gx = map && map.goal ? map.goal.x : CONFIG.ARENA.DEFAULT_GOAL.x;
      const gy = map && map.goal ? map.goal.y : CONFIG.ARENA.DEFAULT_GOAL.y;
      target = { x: gx, y: gy, label: "FINISH", isFinish: true };
    }

    if (i + 1 < this.checkpoints.length) {
      for (let k = 0; k < this.checkpoints.length; k++) {
        this.checkpoints[k].isNext = k === i + 1;
      }
    } else {
      for (const c of this.checkpoints) c.isNext = false;
    }

    this.lastDecodedTag = {
      tagId: cp.id,
      label: cp.label,
      coordinate: { x: target.x, y: target.y },
      nextLabel: target.label,
      isFinish: !!target.isFinish,
      rawPayload: cp.payload ? cp.payload.dataString : `TAG_${cp.id}`,
    };

    this.revealedTarget = target;
    if (this.onDiscover) this.onDiscover(cp, target);
  }

  /**
   * Return all AprilTags currently visible in camera frame (within range, FOV, and LOS)
   */
  getVisibleTags(robot, map) {
    const detectionRange = CONFIG.CHECKPOINT.DETECTION_RANGE || 2.0;
    const fovDeg = CONFIG.CHECKPOINT.FOV_DEG || 120;
    const halfFovRad = (fovDeg * Math.PI) / 360;
    const visible = [];

    for (const cp of this.checkpoints) {
      const dx = cp.x - robot.x;
      const dy = cp.y - robot.y;
      const dist = Math.hypot(dx, dy);
      if (dist > detectionRange) continue;

      let relAngle = Math.atan2(dy, dx) - (robot.theta || 0);
      while (relAngle > Math.PI) relAngle -= 2 * Math.PI;
      while (relAngle < -Math.PI) relAngle += 2 * Math.PI;

      if (typeof robot.theta === "number" && dist > 0.25 && Math.abs(relAngle) > halfFovRad) continue;
      if (!this._hasLineOfSight(robot.x, robot.y, cp.x, cp.y, map)) continue;

      visible.push({
        id: cp.id,
        label: cp.label,
        distance: dist,
        relAngle: relAngle,
        relAngleDeg: (relAngle * 180) / Math.PI,
        discovered: cp.discovered,
        x: cp.x,
        y: cp.y,
      });
    }

    return visible;
  }

  _hasLineOfSight(ax, ay, bx, by, map) {
    if (!map || !map.segments) return true;
    for (const s of map.segments) {
      if (this._segmentsIntersect(ax, ay, bx, by, s.p1.x, s.p1.y, s.p2.x, s.p2.y)) {
        return false;
      }
    }
    return true;
  }

  _crossPt(ox, oy, ax, ay, bx, by) {
    return (ax - ox) * (by - oy) - (ay - oy) * (bx - ox);
  }

  _segmentsIntersect(x1, y1, x2, y2, x3, y3, x4, y4) {
    const d1 = this._crossPt(x3, y3, x4, y4, x1, y1);
    const d2 = this._crossPt(x3, y3, x4, y4, x2, y2);
    const d3 = this._crossPt(x1, y1, x2, y2, x3, y3);
    const d4 = this._crossPt(x1, y1, x2, y2, x4, y4);
    return ((d1 > 0 && d2 < 0) || (d1 < 0 && d2 > 0)) &&
           ((d3 > 0 && d4 < 0) || (d3 < 0 && d4 > 0));
  }

  /** Data exposed to the injected navigation script via sensors.checkpoints / sensors.navTarget */
  getScriptData() {
    return {
      list: this.checkpoints.map(c => ({
        id: c.id,
        x: c.x,
        y: c.y,
        colorHex: c.colorHex,
        colorCss: c.colorCss,
        label: c.label,
        discovered: c.discovered,
        isNext: c.isNext,
      })),
      navTarget: this.revealedTarget ? { ...this.revealedTarget } : null,
      lastDecodedTag: this.lastDecodedTag ? { ...this.lastDecodedTag } : null,
      discoveredCount: this.checkpoints.filter(c => c.discovered).length,
      total: this.checkpoints.length,
    };
  }
}

if (typeof module !== "undefined" && module.exports) {
  module.exports = { CheckpointManager };
}
