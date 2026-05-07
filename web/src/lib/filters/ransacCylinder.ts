import { cloudFromPositions, type PointCloud } from "../types";

export type CylinderResult = {
  inliers: PointCloud;
  outliers: PointCloud;
  /** Vertical-axis cylinder: axis is parallel to z, intersecting the
   *  xy plane at (cx, cy) with radius r. */
  cylinder: { cx: number; cy: number; r: number; zMin: number; zMax: number };
  inlierRatio: number;
};

/**
 * RANSAC vertical-axis cylinder fitting — pcl::SACSegmentation with
 * SACMODEL_CYLINDER analogue, *constrained* to a vertical (z-aligned) axis.
 *
 * The vertical-axis assumption simplifies the fit dramatically: project
 * every point onto the xy plane, then fit a 2D circle by sampling 3 random
 * points. The 3-point circumcircle gives (cx, cy, r). Inliers are points
 * whose xy-distance from the axis is within `threshold` of r. Restricting
 * r to [rMin, rMax] keeps the fit on pole-sized features and prevents
 * degenerate solutions (e.g. fitting huge circles through near-colinear
 * planes).
 */
export function ransacVerticalCylinder(
  input: PointCloud,
  threshold: number,
  iterations: number,
  rMin: number,
  rMax: number,
  seed = 0xDECAF,
): CylinderResult {
  const N = input.count;
  if (N < 3) {
    return {
      inliers: input,
      outliers: cloudFromPositions(new Float32Array(0)),
      cylinder: { cx: 0, cy: 0, r: 0, zMin: 0, zMax: 0 },
      inlierRatio: 0,
    };
  }

  const rng = mulberry32(seed);
  const p = input.positions;

  let bestCount = 0;
  let best = { cx: 0, cy: 0, r: 0 };

  for (let it = 0; it < iterations; it++) {
    const i1 = (rng() * N) | 0;
    let i2 = (rng() * N) | 0;
    let i3 = (rng() * N) | 0;
    if (i2 === i1) i2 = (i2 + 1) % N;
    if (i3 === i1 || i3 === i2) i3 = (i3 + 2) % N;

    const x1 = p[i1 * 3], y1 = p[i1 * 3 + 1];
    const x2 = p[i2 * 3], y2 = p[i2 * 3 + 1];
    const x3 = p[i3 * 3], y3 = p[i3 * 3 + 1];

    // Circumcircle of three 2D points.
    const D = 2 * (x1 * (y2 - y3) + x2 * (y3 - y1) + x3 * (y1 - y2));
    if (Math.abs(D) < 1e-9) continue;
    const cx =
      ((x1 * x1 + y1 * y1) * (y2 - y3) +
        (x2 * x2 + y2 * y2) * (y3 - y1) +
        (x3 * x3 + y3 * y3) * (y1 - y2)) /
      D;
    const cy =
      ((x1 * x1 + y1 * y1) * (x3 - x2) +
        (x2 * x2 + y2 * y2) * (x1 - x3) +
        (x3 * x3 + y3 * y3) * (x2 - x1)) /
      D;
    const r = Math.hypot(cx - x1, cy - y1);

    if (r < rMin || r > rMax) continue;

    let cnt = 0;
    for (let i = 0; i < N; i++) {
      const dx = p[i * 3] - cx;
      const dy = p[i * 3 + 1] - cy;
      const d = Math.abs(Math.hypot(dx, dy) - r);
      if (d <= threshold) cnt++;
    }
    if (cnt > bestCount) {
      bestCount = cnt;
      best = { cx, cy, r };
    }
  }

  // Split using the winning cylinder.
  const { cx, cy, r } = best;
  const inFlags = new Uint8Array(N);
  let inCount = 0;
  let zMin = Infinity,
    zMax = -Infinity;
  for (let i = 0; i < N; i++) {
    const dx = p[i * 3] - cx;
    const dy = p[i * 3 + 1] - cy;
    const d = Math.abs(Math.hypot(dx, dy) - r);
    if (d <= threshold) {
      inFlags[i] = 1;
      inCount++;
      const z = p[i * 3 + 2];
      if (z < zMin) zMin = z;
      if (z > zMax) zMax = z;
    }
  }
  if (inCount === 0) {
    zMin = 0;
    zMax = 0;
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
    cylinder: { cx, cy, r, zMin, zMax },
    inlierRatio: inCount / N,
  };
}

/**
 * Build a Float32Array of line segments approximating a vertical cylinder
 * as a wireframe — `rings` horizontal circles plus a few verticals to tie
 * them together. The output is shaped for PointCloudViewer's `lines` prop.
 */
export function cylinderWireframe(
  cx: number,
  cy: number,
  r: number,
  zMin: number,
  zMax: number,
  rings = 8,
  segs = 32,
): Float32Array {
  if (r <= 0 || zMax <= zMin) return new Float32Array(0);
  const horizontal = rings * segs;
  const vertical = segs / 2; // half as many vertical struts to keep it readable
  const out = new Float32Array((horizontal + vertical) * 6);
  let idx = 0;

  // Horizontal rings.
  for (let ri = 0; ri < rings; ri++) {
    const t = ri / Math.max(rings - 1, 1);
    const z = zMin + (zMax - zMin) * t;
    for (let si = 0; si < segs; si++) {
      const a1 = (si / segs) * Math.PI * 2;
      const a2 = ((si + 1) / segs) * Math.PI * 2;
      out[idx++] = cx + r * Math.cos(a1);
      out[idx++] = cy + r * Math.sin(a1);
      out[idx++] = z;
      out[idx++] = cx + r * Math.cos(a2);
      out[idx++] = cy + r * Math.sin(a2);
      out[idx++] = z;
    }
  }
  // Vertical struts.
  for (let si = 0; si < vertical; si++) {
    const a = (si / vertical) * Math.PI * 2;
    const x = cx + r * Math.cos(a);
    const y = cy + r * Math.sin(a);
    out[idx++] = x;
    out[idx++] = y;
    out[idx++] = zMin;
    out[idx++] = x;
    out[idx++] = y;
    out[idx++] = zMax;
  }
  return out;
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
