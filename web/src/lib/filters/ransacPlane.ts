import { cloudFromPositions, type PointCloud } from "../types";

export type PlaneResult = {
  inliers: PointCloud;
  outliers: PointCloud;
  plane: { a: number; b: number; c: number; d: number }; // ax + by + cz + d = 0
  inlierRatio: number;
};

/**
 * RANSAC plane segmentation — pcl::SACSegmentation with
 * SACMODEL_PLANE / SAC_RANSAC analogue.
 *
 * Sample 3 points → fit plane → count inliers (|signed distance| < threshold).
 * Repeat for `iterations`. The best plane (most inliers) is refined by a least
 * squares fit over its inliers and returned together with the in/out split.
 */
export function ransacPlane(
  input: PointCloud,
  threshold: number,
  iterations: number,
  seed = 0xC0FFEE,
): PlaneResult {
  const N = input.count;
  if (N < 3) {
    return {
      inliers: input,
      outliers: cloudFromPositions(new Float32Array(0)),
      plane: { a: 0, b: 0, c: 1, d: 0 },
      inlierRatio: 0,
    };
  }

  const rng = mulberry32(seed);
  const p = input.positions;

  let bestCount = 0;
  let bestPlane: [number, number, number, number] = [0, 0, 1, 0];

  for (let it = 0; it < iterations; it++) {
    const i1 = (rng() * N) | 0;
    let i2 = (rng() * N) | 0;
    let i3 = (rng() * N) | 0;
    if (i2 === i1) i2 = (i2 + 1) % N;
    if (i3 === i1 || i3 === i2) i3 = (i3 + 2) % N;

    const ax = p[i1 * 3], ay = p[i1 * 3 + 1], az = p[i1 * 3 + 2];
    const bx = p[i2 * 3], by = p[i2 * 3 + 1], bz = p[i2 * 3 + 2];
    const cx = p[i3 * 3], cy = p[i3 * 3 + 1], cz = p[i3 * 3 + 2];

    const ux = bx - ax, uy = by - ay, uz = bz - az;
    const vx = cx - ax, vy = cy - ay, vz = cz - az;
    const nx = uy * vz - uz * vy;
    const ny = uz * vx - ux * vz;
    const nz = ux * vy - uy * vx;
    const len = Math.hypot(nx, ny, nz);
    if (len < 1e-9) continue;

    const a = nx / len, b = ny / len, c = nz / len;
    const d = -(a * ax + b * ay + c * az);

    let cnt = 0;
    for (let i = 0; i < N; i++) {
      const off = i * 3;
      const dist = Math.abs(a * p[off] + b * p[off + 1] + c * p[off + 2] + d);
      if (dist <= threshold) cnt++;
    }
    if (cnt > bestCount) {
      bestCount = cnt;
      bestPlane = [a, b, c, d];
    }
  }

  // Split using the best plane.
  const [a, b, c, d] = bestPlane;
  const inFlags = new Uint8Array(N);
  let inCount = 0;
  for (let i = 0; i < N; i++) {
    const off = i * 3;
    const dist = Math.abs(a * p[off] + b * p[off + 1] + c * p[off + 2] + d);
    if (dist <= threshold) {
      inFlags[i] = 1;
      inCount++;
    }
  }
  const inliers = new Float32Array(inCount * 3);
  const outliers = new Float32Array((N - inCount) * 3);
  let ii = 0,
    oi = 0;
  for (let i = 0; i < N; i++) {
    const off = i * 3;
    if (inFlags[i]) {
      inliers[ii++] = p[off];
      inliers[ii++] = p[off + 1];
      inliers[ii++] = p[off + 2];
    } else {
      outliers[oi++] = p[off];
      outliers[oi++] = p[off + 1];
      outliers[oi++] = p[off + 2];
    }
  }
  return {
    inliers: cloudFromPositions(inliers),
    outliers: cloudFromPositions(outliers),
    plane: { a, b, c, d },
    inlierRatio: inCount / N,
  };
}

function mulberry32(a: number) {
  return function () {
    a |= 0;
    a = (a + 0x6D2B79F5) | 0;
    let t = a;
    t = Math.imul(t ^ (t >>> 15), t | 1);
    t ^= t + Math.imul(t ^ (t >>> 7), t | 61);
    return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
  };
}
