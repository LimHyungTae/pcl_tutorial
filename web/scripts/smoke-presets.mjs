// Smoke test on the actual bundled preset PCDs — confirms that after the
// per-preset ensureZUp transform, Patchwork finds reasonable ground and
// produces many distinct patches.
import { readFileSync } from "node:fs";
import { resolve } from "node:path";

const { parseFromBuffer } = await import("../src/lib/pcd.ts");
const { runPatchwork, unpackPatch } = await import("../src/lib/filters/patchwork.ts");
const { ensureZUp } = await import("../src/lib/axisTransform.ts");
const { SENSOR_VLP16, SENSOR_HDL64 } = await import("../src/lib/sensorConfig.ts");

function loadLocal(relPath) {
  const buf = readFileSync(resolve(relPath));
  const ab = buf.buffer.slice(buf.byteOffset, buf.byteOffset + buf.byteLength);
  return parseFromBuffer(ab, relPath);
}

function summarize(label, result) {
  const groundN = result.groundCloud.count;
  const ngN = result.nonGroundCloud.count;
  const total = groundN + ngN;
  const ratio = total === 0 ? 0 : groundN / total;
  const sectorsPresent = new Set();
  for (let i = 0; i < result.groundRegionIds.length; i++) {
    sectorsPresent.add(result.groundRegionIds[i]);
  }
  // How many distinct sectors per (zone, ring)?
  const ringKeys = new Set();
  for (const rid of sectorsPresent) {
    const { zone, ring } = unpackPatch(rid);
    ringKeys.add(`${zone}-${ring}`);
  }

  console.log(`\n=== ${label} ===`);
  console.log(`  ground / total: ${groundN.toLocaleString()} / ${total.toLocaleString()} (${(100 * ratio).toFixed(1)}%)`);
  console.log(`  patches with ground: ${sectorsPresent.size} (across ${ringKeys.size} distinct rings)`);
  // Show ground z-band distribution.
  const zs = [];
  for (let i = 0; i < groundN; i++) {
    zs.push(result.groundCloud.positions[3 * i + 2]);
  }
  zs.sort((a, b) => a - b);
  if (zs.length > 0) {
    const p = (q) => zs[Math.min(zs.length - 1, Math.max(0, Math.floor(q * zs.length)))];
    console.log(
      `  ground z: p1=${p(0.01).toFixed(2)}  p50=${p(0.5).toFixed(2)}  p99=${p(0.99).toFixed(2)}`,
    );
  }
}

const naverRaw = loadLocal("public/data/naverlabs_vel16.pcd");
console.log(`NaverLabs raw z range: [${naverRaw.bbox.min[2].toFixed(2)}, ${naverRaw.bbox.max[2].toFixed(2)}]`);
const naverFlipped = ensureZUp(naverRaw, "naverlabs");
console.log(`NaverLabs flipped z range: [${naverFlipped.bbox.min[2].toFixed(2)}, ${naverFlipped.bbox.max[2].toFixed(2)}]`);
summarize("NaverLabs VLP-16 (Z flipped)", runPatchwork(naverFlipped, SENSOR_VLP16));

const kittiRaw = loadLocal("public/data/kitti00_000000.bin");
console.log(`KITTI raw z range: [${kittiRaw.bbox.min[2].toFixed(2)}, ${kittiRaw.bbox.max[2].toFixed(2)}]`);
const kittiZup = ensureZUp(kittiRaw, "kitti");
summarize("KITTI HDL-64 (no flip)", runPatchwork(kittiZup, SENSOR_HDL64));
