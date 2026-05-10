import { KdTree } from "../kdtree";
import { symEig3x3 } from "../eigen";
import { transformCloud } from "./transform";
import type { PointCloud } from "../types";

/**
 * Simplified TS port of GenZ-ICP (Lee et al., RA-L 2024). The interactive
 * skeleton (Step / Play / convergence stop) is shared with Lec11 ICP, but
 * the per-iteration solve mirrors GenZ's two-stream linear system:
 *
 *   - planar correspondences contribute a point-to-plane residual,
 *     weighted α.
 *   - non-planar correspondences contribute a point-to-point residual,
 *     weighted (1 − α).
 *   - α = planar / (planar + non_planar) — adaptive on every iteration.
 *
 * Each pair's weight is multiplied by a Cauchy-style robust kernel.
 *
 * To keep the per-step cost bounded we precompute target normals and the
 * per-point planarity ratio over a K-NN neighborhood once, instead of
 * re-running the upstream voxel-hash query every iteration.
 */

export type GenzTargetIndex = {
  positions: Float32Array;
  /** Per-tgt-point unit normal (smallest eigenvector of local covariance). */
  normals: Float32Array;
  /** Per-tgt-point planarity ratio = λ_min / Σλ. Smaller = more planar. */
  planarRatio: Float32Array;
  /** Per-tgt-point flag: 1 if ratio < planarityThreshold. */
  isPlanar: Uint8Array;
  tree: KdTree;
};

export function buildGenzTargetIndex(
  tgt: PointCloud,
  k: number,
  planarityThreshold: number,
): GenzTargetIndex {
  const N = tgt.count;
  const tree = new KdTree(tgt.positions);
  const normals = new Float32Array(N * 3);
  const planarRatio = new Float32Array(N);
  const isPlanar = new Uint8Array(N);
  const kk = Math.min(Math.max(k, 5), N);

  for (let i = 0; i < N; i++) {
    const px = tgt.positions[3 * i];
    const py = tgt.positions[3 * i + 1];
    const pz = tgt.positions[3 * i + 2];
    const { indices } = tree.knnSearch(px, py, pz, kk);
    if (indices.length < 5) {
      planarRatio[i] = 1;
      isPlanar[i] = 0;
      normals[3 * i] = 0;
      normals[3 * i + 1] = 0;
      normals[3 * i + 2] = 1;
      continue;
    }
    let mx = 0, my = 0, mz = 0;
    for (const idx of indices) {
      mx += tgt.positions[3 * idx];
      my += tgt.positions[3 * idx + 1];
      mz += tgt.positions[3 * idx + 2];
    }
    const inv = 1 / indices.length;
    mx *= inv; my *= inv; mz *= inv;

    let cxx = 0, cxy = 0, cxz = 0, cyy = 0, cyz = 0, czz = 0;
    for (const idx of indices) {
      const dx = tgt.positions[3 * idx] - mx;
      const dy = tgt.positions[3 * idx + 1] - my;
      const dz = tgt.positions[3 * idx + 2] - mz;
      cxx += dx * dx; cxy += dx * dy; cxz += dx * dz;
      cyy += dy * dy; cyz += dy * dz; czz += dz * dz;
    }
    cxx *= inv; cxy *= inv; cxz *= inv;
    cyy *= inv; cyz *= inv; czz *= inv;

    const eig = symEig3x3(cxx, cxy, cxz, cyy, cyz, czz);
    const sumEig = eig.values[0] + eig.values[1] + eig.values[2] + 1e-20;
    const ratio = eig.values[0] / sumEig;
    planarRatio[i] = ratio;
    isPlanar[i] = ratio < planarityThreshold ? 1 : 0;
    normals[3 * i] = eig.smallestVec[0];
    normals[3 * i + 1] = eig.smallestVec[1];
    normals[3 * i + 2] = eig.smallestVec[2];
  }

  return { positions: tgt.positions, normals, planarRatio, isPlanar, tree };
}

export type GenzIcpStep = {
  transform: Float32Array;
  /** Mean correspondence distance over all kept pairs. */
  fitness: number;
  /** Total kept pairs. */
  pairs: number;
  /** Point-to-plane (planar) pair count. */
  planarPairs: number;
  /** Point-to-point (non-planar) pair count. */
  nonPlanarPairs: number;
  /** α = planar / (planar + non_planar). 0 → all unstructured, 1 → all planar. */
  alpha: number;
  /** Planar pair endpoints, packed [s.x,y,z, t.x,y,z, ...]. */
  planarPairCoords: Float32Array;
  /** Non-planar pair endpoints. */
  nonPlanarPairCoords: Float32Array;
};

/**
 * One GenZ-ICP iteration.
 *   1. find each src point's nearest tgt neighbor (KdTree);
 *   2. classify by tgt's local planarity (precomputed);
 *   3. accumulate JᵀJ / Jᵀr from a point-to-plane block (α weight) and a
 *      point-to-point block ((1 − α) weight);
 *   4. dx = (JᵀJ)⁻¹ (−Jᵀr) via Cholesky; T_new = exp(dx) · T_old.
 */
export function genzIcpStep(
  src: PointCloud,
  index: GenzTargetIndex,
  current: Float32Array,
  maxDist: number,
  kernel: number,
): GenzIcpStep {
  const transformed = transformCloud(src, current);
  const N = transformed.count;
  const max2 = maxDist * maxDist;

  // Pass 1: nearest-neighbor lookup + classification.
  const planarSrc: number[] = [];
  const planarTgt: number[] = [];
  const nonPlanarSrc: number[] = [];
  const nonPlanarTgt: number[] = [];
  let fitnessSum = 0;
  let kept = 0;
  for (let i = 0; i < N; i++) {
    const x = transformed.positions[3 * i];
    const y = transformed.positions[3 * i + 1];
    const z = transformed.positions[3 * i + 2];
    const { indices, sqrDists } = index.tree.knnSearch(x, y, z, 1);
    if (indices.length === 0 || sqrDists[0] > max2) continue;
    const ti = indices[0];
    fitnessSum += Math.sqrt(sqrDists[0]);
    kept++;
    if (index.isPlanar[ti]) {
      planarSrc.push(i);
      planarTgt.push(ti);
    } else {
      nonPlanarSrc.push(i);
      nonPlanarTgt.push(ti);
    }
  }
  const planarCount = planarSrc.length;
  const nonPlanarCount = nonPlanarSrc.length;
  const total = planarCount + nonPlanarCount;
  const fallback = (): GenzIcpStep => ({
    transform: current,
    fitness: total === 0 ? Infinity : fitnessSum / Math.max(kept, 1),
    pairs: total,
    planarPairs: planarCount,
    nonPlanarPairs: nonPlanarCount,
    alpha: total === 0 ? 0 : planarCount / total,
    planarPairCoords: collectPairs(transformed.positions, planarSrc, index.positions, planarTgt),
    nonPlanarPairCoords: collectPairs(transformed.positions, nonPlanarSrc, index.positions, nonPlanarTgt),
  });
  if (total < 6) return fallback();

  const alpha = planarCount / total;
  const fitness = fitnessSum / kept;

  // Cauchy-style robust weight. Mirrors registration.cpp:
  //   w(r²) = k² / (k + r²)².
  const k2 = kernel * kernel;
  const wRobust = (r2: number) => k2 / ((kernel + r2) * (kernel + r2));

  // 6×6 normal equations.
  const JTJ = new Float64Array(36);
  const JTr = new Float64Array(6);

  // Point-to-plane block (planar).
  for (let p = 0; p < planarCount; p++) {
    const si = planarSrc[p];
    const ti = planarTgt[p];
    const sx = transformed.positions[3 * si];
    const sy = transformed.positions[3 * si + 1];
    const sz = transformed.positions[3 * si + 2];
    const tx = index.positions[3 * ti];
    const ty = index.positions[3 * ti + 1];
    const tz = index.positions[3 * ti + 2];
    const nx = index.normals[3 * ti];
    const ny = index.normals[3 * ti + 1];
    const nz = index.normals[3 * ti + 2];

    const r = (sx - tx) * nx + (sy - ty) * ny + (sz - tz) * nz;
    const w = alpha * wRobust(r * r);

    // J = [n^T, (s × n)^T] (1 × 6).
    const j0 = nx, j1 = ny, j2 = nz;
    const j3 = sy * nz - sz * ny;
    const j4 = sz * nx - sx * nz;
    const j5 = sx * ny - sy * nx;
    const J = [j0, j1, j2, j3, j4, j5];

    for (let i = 0; i < 6; i++) {
      JTr[i] += w * J[i] * r;
      const ji = J[i];
      const row = i * 6;
      for (let j = 0; j < 6; j++) JTJ[row + j] += w * ji * J[j];
    }
  }

  // Point-to-point block (non-planar).
  for (let p = 0; p < nonPlanarCount; p++) {
    const si = nonPlanarSrc[p];
    const ti = nonPlanarTgt[p];
    const sx = transformed.positions[3 * si];
    const sy = transformed.positions[3 * si + 1];
    const sz = transformed.positions[3 * si + 2];
    const tx = index.positions[3 * ti];
    const ty = index.positions[3 * ti + 1];
    const tz = index.positions[3 * ti + 2];

    const rx = sx - tx;
    const ry = sy - ty;
    const rz = sz - tz;
    const r2 = rx * rx + ry * ry + rz * rz;
    const w = (1 - alpha) * wRobust(r2);

    // J = [I_3, -hat(s)] (3 × 6).
    //   row 0: [1, 0, 0,  0,  sz, -sy]
    //   row 1: [0, 1, 0, -sz, 0,  sx]
    //   row 2: [0, 0, 1,  sy, -sx, 0]
    const J: number[][] = [
      [1, 0, 0, 0, sz, -sy],
      [0, 1, 0, -sz, 0, sx],
      [0, 0, 1, sy, -sx, 0],
    ];
    const r: [number, number, number] = [rx, ry, rz];
    for (let i = 0; i < 6; i++) {
      let jtr = 0;
      for (let row = 0; row < 3; row++) jtr += J[row][i] * r[row];
      JTr[i] += w * jtr;
      const out = i * 6;
      for (let j = 0; j < 6; j++) {
        let jtj = 0;
        for (let row = 0; row < 3; row++) jtj += J[row][i] * J[row][j];
        JTJ[out + j] += w * jtj;
      }
    }
  }

  // Solve JTJ dx = -JTr via Cholesky.
  const negJTr = new Float64Array(6);
  for (let i = 0; i < 6; i++) negJTr[i] = -JTr[i];
  const dx = solveSpd6(JTJ, negJTr);

  const update = se3Exp(dx);
  const next = compose(update, current);

  return {
    transform: next,
    fitness,
    pairs: total,
    planarPairs: planarCount,
    nonPlanarPairs: nonPlanarCount,
    alpha,
    planarPairCoords: collectPairs(transformed.positions, planarSrc, index.positions, planarTgt),
    nonPlanarPairCoords: collectPairs(transformed.positions, nonPlanarSrc, index.positions, nonPlanarTgt),
  };
}

/** Subsample pair-segment endpoints for rendering. Same shape as the
 *  Lec11 ICP samplePairCoords helper. */
export function sampleGenzPairs(
  pairCoords: Float32Array,
  maxSegments: number,
): Float32Array {
  const total = pairCoords.length / 6;
  if (total <= maxSegments) return pairCoords;
  const out = new Float32Array(maxSegments * 6);
  for (let i = 0; i < maxSegments; i++) {
    const src = Math.floor((i * total) / maxSegments);
    for (let k = 0; k < 6; k++) out[i * 6 + k] = pairCoords[src * 6 + k];
  }
  return out;
}

// ─── helpers ────────────────────────────────────────────────────────────────

function collectPairs(
  srcPositions: Float32Array,
  srcIdxs: number[],
  tgtPositions: Float32Array,
  tgtIdxs: number[],
): Float32Array {
  const out = new Float32Array(srcIdxs.length * 6);
  for (let p = 0; p < srcIdxs.length; p++) {
    const si = srcIdxs[p];
    const ti = tgtIdxs[p];
    out[p * 6] = srcPositions[3 * si];
    out[p * 6 + 1] = srcPositions[3 * si + 1];
    out[p * 6 + 2] = srcPositions[3 * si + 2];
    out[p * 6 + 3] = tgtPositions[3 * ti];
    out[p * 6 + 4] = tgtPositions[3 * ti + 1];
    out[p * 6 + 5] = tgtPositions[3 * ti + 2];
  }
  return out;
}

/** Cholesky solve for a 6×6 symmetric positive-definite system. Returns the
 *  zero vector (no update) if the matrix is not SPD — keeps the iteration
 *  loop from blowing up on a degenerate problem. */
function solveSpd6(A: Float64Array, b: Float64Array): Float64Array {
  const L = new Float64Array(36);
  for (let i = 0; i < 6; i++) {
    for (let j = 0; j <= i; j++) {
      let s = A[i * 6 + j];
      for (let k = 0; k < j; k++) s -= L[i * 6 + k] * L[j * 6 + k];
      if (i === j) {
        if (s <= 1e-20) return new Float64Array(6);
        L[i * 6 + j] = Math.sqrt(s);
      } else {
        L[i * 6 + j] = s / L[j * 6 + j];
      }
    }
  }
  const y = new Float64Array(6);
  for (let i = 0; i < 6; i++) {
    let s = b[i];
    for (let k = 0; k < i; k++) s -= L[i * 6 + k] * y[k];
    y[i] = s / L[i * 6 + i];
  }
  const x = new Float64Array(6);
  for (let i = 5; i >= 0; i--) {
    let s = y[i];
    for (let k = i + 1; k < 6; k++) s -= L[k * 6 + i] * x[k];
    x[i] = s / L[i * 6 + i];
  }
  return x;
}

/** SE(3) exponential. dx = [vx, vy, vz, ωx, ωy, ωz] (translation first). */
function se3Exp(dx: Float64Array): Float32Array {
  const vx = dx[0], vy = dx[1], vz = dx[2];
  const wx = dx[3], wy = dx[4], wz = dx[5];
  const theta = Math.hypot(wx, wy, wz);
  const T = new Float32Array(16);

  if (theta < 1e-12) {
    // First-order: R ≈ I + hat(ω), V ≈ I.
    T[0] = 1;       T[1] = -wz;     T[2] = wy;      T[3] = vx;
    T[4] = wz;      T[5] = 1;       T[6] = -wx;     T[7] = vy;
    T[8] = -wy;     T[9] = wx;      T[10] = 1;      T[11] = vz;
    T[12] = 0; T[13] = 0; T[14] = 0; T[15] = 1;
    return T;
  }

  const t2 = theta * theta;
  const sin = Math.sin(theta);
  const cos = Math.cos(theta);
  const a = sin / theta;
  const b = (1 - cos) / t2;
  const c = (theta - sin) / (t2 * theta);

  // hat(ω)² = ω ω^T - ‖ω‖² I.
  const wxwx = wx * wx, wywy = wy * wy, wzwz = wz * wz;
  const wxwy = wx * wy, wxwz = wx * wz, wywz = wy * wz;

  const h2_00 = wxwx - t2,  h2_01 = wxwy,        h2_02 = wxwz;
  const h2_10 = wxwy,       h2_11 = wywy - t2,   h2_12 = wywz;
  const h2_20 = wxwz,       h2_21 = wywz,        h2_22 = wzwz - t2;

  // R = I + a·hat(ω) + b·hat(ω)².
  const r00 = 1 + b * h2_00;
  const r01 = -a * wz + b * h2_01;
  const r02 =  a * wy + b * h2_02;
  const r10 =  a * wz + b * h2_10;
  const r11 = 1 + b * h2_11;
  const r12 = -a * wx + b * h2_12;
  const r20 = -a * wy + b * h2_20;
  const r21 =  a * wx + b * h2_21;
  const r22 = 1 + b * h2_22;

  // V = I + b·hat(ω) + c·hat(ω)².
  const V00 = 1 + c * h2_00;
  const V01 = -b * wz + c * h2_01;
  const V02 =  b * wy + c * h2_02;
  const V10 =  b * wz + c * h2_10;
  const V11 = 1 + c * h2_11;
  const V12 = -b * wx + c * h2_12;
  const V20 = -b * wy + c * h2_20;
  const V21 =  b * wx + c * h2_21;
  const V22 = 1 + c * h2_22;

  const tx = V00 * vx + V01 * vy + V02 * vz;
  const ty = V10 * vx + V11 * vy + V12 * vz;
  const tz = V20 * vx + V21 * vy + V22 * vz;

  T[0] = r00; T[1] = r01; T[2] = r02; T[3] = tx;
  T[4] = r10; T[5] = r11; T[6] = r12; T[7] = ty;
  T[8] = r20; T[9] = r21; T[10] = r22; T[11] = tz;
  T[12] = 0; T[13] = 0; T[14] = 0; T[15] = 1;
  return T;
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
