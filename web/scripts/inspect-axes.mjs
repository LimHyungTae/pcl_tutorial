// Quick axis-orientation check on the bundled presets — prints per-axis
// percentile ranges so we can decide which axis is "up" for each sensor.
import { readFileSync } from "node:fs";
import { resolve } from "node:path";
const { parseFromBuffer } = await import("../src/lib/pcd.ts");

async function dump(label, relPath) {
  const buf = readFileSync(resolve(relPath));
  const ab = buf.buffer.slice(buf.byteOffset, buf.byteOffset + buf.byteLength);
  const c = parseFromBuffer(ab, relPath);
  const N = c.count;
  const xs = new Float32Array(N), ys = new Float32Array(N), zs = new Float32Array(N);
  for (let i = 0; i < N; i++) {
    xs[i] = c.positions[3 * i];
    ys[i] = c.positions[3 * i + 1];
    zs[i] = c.positions[3 * i + 2];
  }
  const sortedX = xs.slice().sort();
  const sortedY = ys.slice().sort();
  const sortedZ = zs.slice().sort();
  const pct = (a, p) => a[Math.min(a.length - 1, Math.max(0, Math.floor(p * a.length)))];
  console.log(`\n=== ${label} (${N} pts) ===`);
  console.log(
    `  X: p1=${pct(sortedX, 0.01).toFixed(2)}  p50=${pct(sortedX, 0.5).toFixed(2)}  p99=${pct(sortedX, 0.99).toFixed(2)}`,
  );
  console.log(
    `  Y: p1=${pct(sortedY, 0.01).toFixed(2)}  p50=${pct(sortedY, 0.5).toFixed(2)}  p99=${pct(sortedY, 0.99).toFixed(2)}`,
  );
  console.log(
    `  Z: p1=${pct(sortedZ, 0.01).toFixed(2)}  p50=${pct(sortedZ, 0.5).toFixed(2)}  p99=${pct(sortedZ, 0.99).toFixed(2)}`,
  );
  // Histogram of each axis to see which one has a clear "ground" (a heavy
  // mass at one extreme).
  for (const [name, arr] of [["X", sortedX], ["Y", sortedY], ["Z", sortedZ]]) {
    const lo = arr[0],
      hi = arr[arr.length - 1];
    const span = hi - lo || 1;
    const bins = new Array(10).fill(0);
    for (let i = 0; i < N; i++) {
      const v = arr[i];
      const b = Math.min(9, Math.floor(((v - lo) / span) * 10));
      bins[b]++;
    }
    const max = Math.max(...bins);
    const bar = (n) => "█".repeat(Math.round((n / max) * 18)) || "·";
    console.log(`  ${name} hist:`);
    for (let i = 0; i < 10; i++) {
      const lov = (lo + (span * i) / 10).toFixed(2).padStart(7);
      const hiv = (lo + (span * (i + 1)) / 10).toFixed(2).padStart(7);
      console.log(`    [${lov}, ${hiv}] ${bar(bins[i])} ${bins[i]}`);
    }
  }
}

await dump("NaverLabs VLP-16", "public/data/naverlabs_vel16.pcd");
await dump("KITTI HDL-64", "public/data/kitti00_000000.bin");
