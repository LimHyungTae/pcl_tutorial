// Smoke test: synthetic indoor scene → Patchwork should classify floor as
// ground and reject ceiling/walls. Run with:
//   npx tsx scripts/smoke-patchwork.mjs

const { runPatchwork } = await import("../src/lib/filters/patchwork.ts");
const { SENSOR_VLP16, SENSOR_HDL64 } = await import("../src/lib/sensorConfig.ts");
const { cloudFromPositions } = await import("../src/lib/types.ts");

function makeIndoorScene() {
  // VLP-16 sensor at origin, mounted 0.6 m above floor (so floor z = -0.6).
  // 8 m × 8 m × 3 m room, ceiling at z = +2.4. Add some object boxes.
  const pts = [];
  const rand = mulberry32(42);
  // Floor: scatter 3000 points across the room.
  for (let i = 0; i < 3000; i++) {
    pts.push((rand() - 0.5) * 6, (rand() - 0.5) * 6, -0.6 + (rand() - 0.5) * 0.02);
  }
  // Ceiling: 1500 points.
  for (let i = 0; i < 1500; i++) {
    pts.push((rand() - 0.5) * 6, (rand() - 0.5) * 6, 2.4 + (rand() - 0.5) * 0.02);
  }
  // 4 walls, 800 points each.
  for (let w = 0; w < 4; w++) {
    for (let i = 0; i < 800; i++) {
      const along = (rand() - 0.5) * 6;
      const up = -0.6 + rand() * 3;
      const wallX = w === 0 ? 3 : w === 1 ? -3 : along;
      const wallY = w === 0 ? along : w === 1 ? along : w === 2 ? 3 : -3;
      pts.push(wallX, wallY, up);
    }
  }
  // 3 box-shaped obstacles on the floor.
  const objs = [
    [1.2, 0.5, 0.4],
    [-1.5, 1.8, 0.6],
    [0.5, -1.6, 0.5],
  ];
  for (const [cx, cy, height] of objs) {
    for (let i = 0; i < 600; i++) {
      pts.push(cx + (rand() - 0.5) * 0.6, cy + (rand() - 0.5) * 0.6, -0.6 + rand() * height);
    }
  }
  return cloudFromPositions(new Float32Array(pts));
}

function makeOutdoorKittiLike() {
  // HDL-64 sensor at origin, mounted 1.723 m above ground (so ground z ≈ -1.7).
  // Flat ground out to 80 m, plus a few "vehicle" boxes.
  const pts = [];
  const rand = mulberry32(7);
  for (let i = 0; i < 8000; i++) {
    const r = 2 + rand() * 70;
    const a = rand() * 2 * Math.PI;
    pts.push(r * Math.cos(a), r * Math.sin(a), -1.723 + (rand() - 0.5) * 0.05);
  }
  const cars = [
    [10, 0, 1.5, 4, 1.6],
    [-15, 5, 1.5, 4, 1.7],
    [3, -8, 1.6, 4, 1.7],
  ];
  for (const [cx, cy, w, l, h] of cars) {
    for (let i = 0; i < 600; i++) {
      pts.push(
        cx + (rand() - 0.5) * w,
        cy + (rand() - 0.5) * l,
        -1.723 + rand() * h,
      );
    }
  }
  return cloudFromPositions(new Float32Array(pts));
}

function mulberry32(seed) {
  let a = seed;
  return () => {
    a |= 0;
    a = (a + 0x6d2b79f5) | 0;
    let t = a;
    t = Math.imul(t ^ (t >>> 15), t | 1);
    t ^= t + Math.imul(t ^ (t >>> 7), t | 61);
    return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
  };
}

function classify(cloud, isGround, zMin, zMax) {
  let groundIn = 0,
    nonGroundIn = 0,
    total = 0;
  for (let i = 0; i < cloud.count; i++) {
    const z = cloud.positions[3 * i + 2];
    if (z < zMin || z > zMax) continue;
    total++;
    if (isGround[i]) groundIn++;
    else nonGroundIn++;
  }
  return { groundIn, nonGroundIn, total };
}

console.log("=== VLP-16 indoor synthetic ===");
const indoor = makeIndoorScene();
const ind = runPatchwork(indoor, SENSOR_VLP16);
const floorBand = classify(indoor, ind.isGround, -0.7, -0.5);
const ceilingBand = classify(indoor, ind.isGround, 2.3, 2.5);
const wallBand = classify(indoor, ind.isGround, 0.0, 1.5);
console.log(
  `  floor   z∈[-0.7, -0.5]: ground=${floorBand.groundIn}/${floorBand.total} (${pct(floorBand)})`,
);
console.log(
  `  ceiling z∈[ 2.3,  2.5]: ground=${ceilingBand.groundIn}/${ceilingBand.total} (${pct(ceilingBand)})`,
);
console.log(
  `  walls   z∈[ 0.0,  1.5]: ground=${wallBand.groundIn}/${wallBand.total} (${pct(wallBand)})`,
);
console.log(
  `  TOTAL: ${ind.groundCloud.count}/${indoor.count} marked ground; ${ind.uniqueRegions.length} regions`,
);

console.log("");
console.log("=== HDL-64 outdoor synthetic ===");
const outdoor = makeOutdoorKittiLike();
const out = runPatchwork(outdoor, SENSOR_HDL64);
const groundBand = classify(outdoor, out.isGround, -1.8, -1.65);
const carBand = classify(outdoor, out.isGround, -0.8, 0.3);
console.log(
  `  ground z∈[-1.80, -1.65]: ground=${groundBand.groundIn}/${groundBand.total} (${pct(groundBand)})`,
);
console.log(
  `  car    z∈[-0.80,  0.30]: ground=${carBand.groundIn}/${carBand.total} (${pct(carBand)})`,
);
console.log(
  `  TOTAL: ${out.groundCloud.count}/${outdoor.count} marked ground; ${out.uniqueRegions.length} regions`,
);

console.log("");
const indoorOk = floorBand.groundIn / floorBand.total > 0.7
  && ceilingBand.groundIn / Math.max(1, ceilingBand.total) < 0.2
  && wallBand.groundIn / Math.max(1, wallBand.total) < 0.2;
const outdoorOk = groundBand.groundIn / groundBand.total > 0.7
  && carBand.groundIn / Math.max(1, carBand.total) < 0.2;
console.log(`indoor OK: ${indoorOk}`);
console.log(`outdoor OK: ${outdoorOk}`);
process.exit(indoorOk && outdoorOk ? 0 : 1);

function pct(b) {
  if (b.total === 0) return "—";
  return `${((100 * b.groundIn) / b.total).toFixed(1)}%`;
}
