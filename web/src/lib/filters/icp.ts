import { KdTree } from "../kdtree";
import { buildTransform, transformCloud } from "./transform";
import type { PointCloud } from "../types";

export type IcpStep = {
  /** Updated cumulative transform (row-major 4×4). */
  transform: Float32Array;
  /** Mean correspondence distance after this step. */
  fitness: number;
  /** Number of correspondences used. */
  pairs: number;
  /** Per-pair endpoints, packed [sx,sy,sz, tx,ty,tz, ...] (length = 6*pairs).
   *  src endpoint is the *transformed* source point as it stood before this
   *  step's update — i.e. matches what the user is seeing on screen. */
  pairCoords: Float32Array;
};

/**
 * One ICP iteration:
 *   1. Apply the current transform to the source cloud.
 *   2. For each transformed point, find its nearest point in the target.
 *   3. Compute the rigid (R, t) that best aligns matched pairs (Procrustes).
 *   4. Compose with the current transform.
 *
 * Correspondences whose distance exceeds `maxDist` are discarded.
 */
export function icpStep(
  src: PointCloud,
  tgtTree: KdTree,
  current: Float32Array,
  maxDist: number,
): IcpStep {
  const transformed = transformCloud(src, current);

  // Pass 1: collect filtered correspondences.
  const N = transformed.count;
  const srcIdx = new Uint32Array(N);
  const tgtIdx = new Int32Array(N);
  const max2 = maxDist * maxDist;
  let kept = 0;
  let fitnessSum = 0;
  for (let i = 0; i < N; i++) {
    const x = transformed.positions[i * 3];
    const y = transformed.positions[i * 3 + 1];
    const z = transformed.positions[i * 3 + 2];
    const { indices, sqrDists } = tgtTree.knnSearch(x, y, z, 1);
    if (indices.length === 0 || sqrDists[0] > max2) continue;
    srcIdx[kept] = i;
    tgtIdx[kept] = indices[0];
    fitnessSum += Math.sqrt(sqrDists[0]);
    kept++;
  }
  if (kept < 6) {
    return {
      transform: current,
      fitness: Infinity,
      pairs: kept,
      pairCoords: new Float32Array(0),
    };
  }
  const fitness = fitnessSum / kept;

  // Pack the matched pairs for visualization.
  const pairCoords = new Float32Array(kept * 6);
  for (let k = 0; k < kept; k++) {
    const so = srcIdx[k] * 3;
    const to = tgtIdx[k] * 3;
    pairCoords[k * 6] = transformed.positions[so];
    pairCoords[k * 6 + 1] = transformed.positions[so + 1];
    pairCoords[k * 6 + 2] = transformed.positions[so + 2];
    pairCoords[k * 6 + 3] = tgtTree.positions[to];
    pairCoords[k * 6 + 4] = tgtTree.positions[to + 1];
    pairCoords[k * 6 + 5] = tgtTree.positions[to + 2];
  }

  // Pass 2: centroids of matched pairs (using the transformed src — that
  // way the Procrustes solve gives us the *delta* transform).
  let pcx = 0, pcy = 0, pcz = 0;
  let qcx = 0, qcy = 0, qcz = 0;
  for (let k = 0; k < kept; k++) {
    const so = srcIdx[k] * 3;
    const to = tgtIdx[k] * 3;
    pcx += transformed.positions[so];
    pcy += transformed.positions[so + 1];
    pcz += transformed.positions[so + 2];
    qcx += tgtTree.positions[to];
    qcy += tgtTree.positions[to + 1];
    qcz += tgtTree.positions[to + 2];
  }
  const inv = 1 / kept;
  pcx *= inv; pcy *= inv; pcz *= inv;
  qcx *= inv; qcy *= inv; qcz *= inv;

  // Cross-covariance H = Σ (p - pc) (q - qc)^T   (3×3, row-major).
  let h00 = 0, h01 = 0, h02 = 0;
  let h10 = 0, h11 = 0, h12 = 0;
  let h20 = 0, h21 = 0, h22 = 0;
  for (let k = 0; k < kept; k++) {
    const so = srcIdx[k] * 3;
    const to = tgtIdx[k] * 3;
    const px = transformed.positions[so] - pcx;
    const py = transformed.positions[so + 1] - pcy;
    const pz = transformed.positions[so + 2] - pcz;
    const qx = tgtTree.positions[to] - qcx;
    const qy = tgtTree.positions[to + 1] - qcy;
    const qz = tgtTree.positions[to + 2] - qcz;
    h00 += px * qx; h01 += px * qy; h02 += px * qz;
    h10 += py * qx; h11 += py * qy; h12 += py * qz;
    h20 += pz * qx; h21 += pz * qy; h22 += pz * qz;
  }

  // Procrustes via SVD: H = U Σ V^T,  R = V · diag(1,1,det(V U^T)) · U^T.
  const { U, V } = svd3(h00, h01, h02, h10, h11, h12, h20, h21, h22);

  // R = V * S * U^T, where S = diag(1,1,sign(det(V U^T))).
  // Compute V·U^T first, take its determinant for the sign correction.
  const vu00 = V[0][0] * U[0][0] + V[0][1] * U[0][1] + V[0][2] * U[0][2];
  const vu01 = V[0][0] * U[1][0] + V[0][1] * U[1][1] + V[0][2] * U[1][2];
  const vu02 = V[0][0] * U[2][0] + V[0][1] * U[2][1] + V[0][2] * U[2][2];
  const vu10 = V[1][0] * U[0][0] + V[1][1] * U[0][1] + V[1][2] * U[0][2];
  const vu11 = V[1][0] * U[1][0] + V[1][1] * U[1][1] + V[1][2] * U[1][2];
  const vu12 = V[1][0] * U[2][0] + V[1][1] * U[2][1] + V[1][2] * U[2][2];
  const vu20 = V[2][0] * U[0][0] + V[2][1] * U[0][1] + V[2][2] * U[0][2];
  const vu21 = V[2][0] * U[1][0] + V[2][1] * U[1][1] + V[2][2] * U[1][2];
  const vu22 = V[2][0] * U[2][0] + V[2][1] * U[2][1] + V[2][2] * U[2][2];
  const detVU =
    vu00 * (vu11 * vu22 - vu12 * vu21) -
    vu01 * (vu10 * vu22 - vu12 * vu20) +
    vu02 * (vu10 * vu21 - vu11 * vu20);
  const sign = detVU < 0 ? -1 : 1;

  // R = V · diag(1,1,sign) · U^T. Effectively flip the third column of V.
  const v02 = V[0][2] * sign;
  const v12 = V[1][2] * sign;
  const v22s = V[2][2] * sign;
  const r00 = V[0][0] * U[0][0] + V[0][1] * U[0][1] + v02 * U[0][2];
  const r01 = V[0][0] * U[1][0] + V[0][1] * U[1][1] + v02 * U[1][2];
  const r02 = V[0][0] * U[2][0] + V[0][1] * U[2][1] + v02 * U[2][2];
  const r10 = V[1][0] * U[0][0] + V[1][1] * U[0][1] + v12 * U[0][2];
  const r11 = V[1][0] * U[1][0] + V[1][1] * U[1][1] + v12 * U[1][2];
  const r12 = V[1][0] * U[2][0] + V[1][1] * U[2][1] + v12 * U[2][2];
  const r20 = V[2][0] * U[0][0] + V[2][1] * U[0][1] + v22s * U[0][2];
  const r21 = V[2][0] * U[1][0] + V[2][1] * U[1][1] + v22s * U[1][2];
  const r22 = V[2][0] * U[2][0] + V[2][1] * U[2][1] + v22s * U[2][2];

  const tx = qcx - (r00 * pcx + r01 * pcy + r02 * pcz);
  const ty = qcy - (r10 * pcx + r11 * pcy + r12 * pcz);
  const tz = qcz - (r20 * pcx + r21 * pcy + r22 * pcz);

  // Delta transform.
  const delta = new Float32Array(16);
  delta[0] = r00; delta[1] = r01; delta[2] = r02; delta[3] = tx;
  delta[4] = r10; delta[5] = r11; delta[6] = r12; delta[7] = ty;
  delta[8] = r20; delta[9] = r21; delta[10] = r22; delta[11] = tz;
  delta[12] = 0; delta[13] = 0; delta[14] = 0; delta[15] = 1;

  // Compose: new = delta · current.
  const next = compose(delta, current);
  return { transform: next, fitness, pairs: kept, pairCoords };
}

/** Sub-sample correspondence segments for rendering — line counts blow up
 *  fast, and we only need a representative cross-section. */
export function samplePairCoords(
  pairCoords: Float32Array,
  maxSegments: number,
): Float32Array {
  const total = pairCoords.length / 6;
  if (total <= maxSegments) return pairCoords;
  const out = new Float32Array(maxSegments * 6);
  for (let i = 0; i < maxSegments; i++) {
    const src = Math.floor((i * total) / maxSegments);
    for (let k = 0; k < 6; k++) {
      out[i * 6 + k] = pairCoords[src * 6 + k];
    }
  }
  return out;
}

/** 4×4 row-major matrix multiply. */
function compose(a: Float32Array, b: Float32Array): Float32Array {
  const r = new Float32Array(16);
  for (let i = 0; i < 4; i++) {
    for (let j = 0; j < 4; j++) {
      let s = 0;
      for (let k = 0; k < 4; k++) s += a[i * 4 + k] * b[k * 4 + j];
      r[i * 4 + j] = s;
    }
  }
  return r;
}

/** Convenience starter for the cumulative transform. */
export function identityTransform(): Float32Array {
  return buildTransform(0, 0, 0, 0, 0, 0);
}

// ─── 3×3 SVD via Jacobi eigendecomposition of H^T H ──────────────────────────

type Vec3 = [number, number, number];
type Mat3 = [Vec3, Vec3, Vec3];

function svd3(
  h00: number, h01: number, h02: number,
  h10: number, h11: number, h12: number,
  h20: number, h21: number, h22: number,
): { U: Mat3; S: Vec3; V: Mat3 } {
  // A = H^T H, symmetric 3×3.
  const a00 = h00 * h00 + h10 * h10 + h20 * h20;
  const a01 = h00 * h01 + h10 * h11 + h20 * h21;
  const a02 = h00 * h02 + h10 * h12 + h20 * h22;
  const a11 = h01 * h01 + h11 * h11 + h21 * h21;
  const a12 = h01 * h02 + h11 * h12 + h21 * h22;
  const a22 = h02 * h02 + h12 * h12 + h22 * h22;

  const { values: w, vectors: V } = jacobi3sym(a00, a01, a02, a11, a12, a22);

  const S: Vec3 = [Math.sqrt(Math.max(w[0], 0)), Math.sqrt(Math.max(w[1], 0)), Math.sqrt(Math.max(w[2], 0))];

  // U columns: (1/σ_i) H · V_i. Fall back to a random basis vector for tiny σ.
  const U: Mat3 = [
    [0, 0, 0],
    [0, 0, 0],
    [0, 0, 0],
  ];
  for (let i = 0; i < 3; i++) {
    const vx = V[0][i], vy = V[1][i], vz = V[2][i];
    const ux = h00 * vx + h01 * vy + h02 * vz;
    const uy = h10 * vx + h11 * vy + h12 * vz;
    const uz = h20 * vx + h21 * vy + h22 * vz;
    if (S[i] > 1e-9) {
      const inv = 1 / S[i];
      U[0][i] = ux * inv;
      U[1][i] = uy * inv;
      U[2][i] = uz * inv;
    } else {
      U[0][i] = 0;
      U[1][i] = 0;
      U[2][i] = 0;
    }
  }
  // Patch any degenerate column of U with a vector orthogonal to the others.
  fixOrthonormalU(U, S);
  return { U, S, V };
}

function fixOrthonormalU(U: Mat3, S: Vec3): void {
  // If σ_2 ≈ 0, U[:,2] = U[:,0] × U[:,1].
  if (S[2] < 1e-9) {
    const ax = U[0][0], ay = U[1][0], az = U[2][0];
    const bx = U[0][1], by = U[1][1], bz = U[2][1];
    let cx = ay * bz - az * by;
    let cy = az * bx - ax * bz;
    let cz = ax * by - ay * bx;
    const n = Math.hypot(cx, cy, cz) || 1;
    cx /= n; cy /= n; cz /= n;
    U[0][2] = cx; U[1][2] = cy; U[2][2] = cz;
  }
}

/**
 * Jacobi eigendecomposition of a symmetric 3×3 matrix
 *   A = [[a00, a01, a02], [a01, a11, a12], [a02, a12, a22]]
 * Returns eigenvalues sorted descending, and a 3×3 V whose i-th column is
 * the eigenvector for values[i].
 */
function jacobi3sym(
  a00: number, a01: number, a02: number,
  a11: number, a12: number,
  a22: number,
): { values: Vec3; vectors: Mat3 } {
  // Working copies.
  let m00 = a00, m01 = a01, m02 = a02;
  let m11 = a11, m12 = a12;
  let m22 = a22;
  const v: Mat3 = [
    [1, 0, 0],
    [0, 1, 0],
    [0, 0, 1],
  ];

  for (let iter = 0; iter < 50; iter++) {
    // Find pivot (largest off-diagonal element).
    const o01 = Math.abs(m01);
    const o02 = Math.abs(m02);
    const o12 = Math.abs(m12);
    let p: 0 | 1, q: 1 | 2, mpq: number, mpp: number, mqq: number;
    if (o01 >= o02 && o01 >= o12) {
      p = 0; q = 1; mpq = m01; mpp = m00; mqq = m11;
    } else if (o02 >= o12) {
      p = 0; q = 2; mpq = m02; mpp = m00; mqq = m22;
    } else {
      p = 1; q = 2; mpq = m12; mpp = m11; mqq = m22;
    }
    if (Math.abs(mpq) < 1e-12) break;

    const theta = (mqq - mpp) / (2 * mpq);
    const t =
      theta >= 0
        ? 1 / (theta + Math.sqrt(theta * theta + 1))
        : 1 / (theta - Math.sqrt(theta * theta + 1));
    const c = 1 / Math.sqrt(t * t + 1);
    const s = t * c;

    if (p === 0 && q === 1) {
      const newM00 = m00 - t * m01;
      const newM11 = m11 + t * m01;
      const newM02 = c * m02 - s * m12;
      const newM12 = s * m02 + c * m12;
      m00 = newM00;
      m11 = newM11;
      m02 = newM02;
      m12 = newM12;
      m01 = 0;
    } else if (p === 0 && q === 2) {
      const newM00 = m00 - t * m02;
      const newM22 = m22 + t * m02;
      const newM01 = c * m01 - s * m12;
      const newM12 = s * m01 + c * m12;
      m00 = newM00;
      m22 = newM22;
      m01 = newM01;
      m12 = newM12;
      m02 = 0;
    } else {
      const newM11 = m11 - t * m12;
      const newM22 = m22 + t * m12;
      const newM01 = c * m01 - s * m02;
      const newM02 = s * m01 + c * m02;
      m11 = newM11;
      m22 = newM22;
      m01 = newM01;
      m02 = newM02;
      m12 = 0;
    }

    // Accumulate eigenvectors.
    for (let r = 0; r < 3; r++) {
      const vrp = v[r][p];
      const vrq = v[r][q];
      v[r][p] = c * vrp - s * vrq;
      v[r][q] = s * vrp + c * vrq;
    }
  }

  // Sort eigenvalues descending and reorder columns of V to match.
  const values: Vec3 = [m00, m11, m22];
  const order: [number, number, number] = [0, 1, 2];
  order.sort((a, b) => values[b] - values[a]);
  const sortedValues: Vec3 = [values[order[0]], values[order[1]], values[order[2]]];
  const sortedV: Mat3 = [
    [v[0][order[0]], v[0][order[1]], v[0][order[2]]],
    [v[1][order[0]], v[1][order[1]], v[1][order[2]]],
    [v[2][order[0]], v[2][order[1]], v[2][order[2]]],
  ];
  return { values: sortedValues, vectors: sortedV };
}
