import { KdTree } from "../kdtree";
import type { PointCloud } from "../types";

/**
 * Per-point normals via PCA on K nearest neighbors.
 *
 * For each query point we collect its K nearest neighbors, compute the local
 * 3×3 covariance, and take the eigenvector of the smallest eigenvalue as the
 * normal direction. Returns normalized (nx, ny, nz) packed as Float32Array.
 */
export function estimateNormals(
  input: PointCloud,
  k: number,
): Float32Array {
  const N = input.count;
  const out = new Float32Array(N * 3);
  if (N === 0) return out;

  const tree = new KdTree(input.positions);
  const kk = Math.min(Math.max(k, 3), N);

  for (let i = 0; i < N; i++) {
    const px = input.positions[i * 3];
    const py = input.positions[i * 3 + 1];
    const pz = input.positions[i * 3 + 2];
    const { indices } = tree.knnSearch(px, py, pz, kk);

    // Local mean.
    let mx = 0,
      my = 0,
      mz = 0;
    for (let j = 0; j < indices.length; j++) {
      const off = indices[j] * 3;
      mx += input.positions[off];
      my += input.positions[off + 1];
      mz += input.positions[off + 2];
    }
    const inv = 1 / indices.length;
    mx *= inv;
    my *= inv;
    mz *= inv;

    // Covariance (symmetric, only upper triangle).
    let cxx = 0, cxy = 0, cxz = 0, cyy = 0, cyz = 0, czz = 0;
    for (let j = 0; j < indices.length; j++) {
      const off = indices[j] * 3;
      const dx = input.positions[off] - mx;
      const dy = input.positions[off + 1] - my;
      const dz = input.positions[off + 2] - mz;
      cxx += dx * dx;
      cxy += dx * dy;
      cxz += dx * dz;
      cyy += dy * dy;
      cyz += dy * dz;
      czz += dz * dz;
    }

    const n = smallestEigenvector(cxx, cxy, cxz, cyy, cyz, czz);
    out[i * 3] = n[0];
    out[i * 3 + 1] = n[1];
    out[i * 3 + 2] = n[2];
  }
  return out;
}

/**
 * Eigenvector of the smallest eigenvalue of a 3×3 symmetric matrix
 * [[a,b,c],[b,d,e],[c,e,f]] via inverse iteration after solving
 * the characteristic cubic for the eigenvalues. Returns a unit vector.
 */
function smallestEigenvector(
  a: number, b: number, c: number,
  d: number, e: number, f: number,
): [number, number, number] {
  // Eigenvalues of a real symmetric 3×3 matrix — Smith's closed-form.
  const p1 = b * b + c * c + e * e;
  if (p1 === 0) {
    // Already diagonal; smallest diagonal entry's axis is the normal.
    if (a <= d && a <= f) return [1, 0, 0];
    if (d <= a && d <= f) return [0, 1, 0];
    return [0, 0, 1];
  }
  const q = (a + d + f) / 3;
  const p2 = (a - q) ** 2 + (d - q) ** 2 + (f - q) ** 2 + 2 * p1;
  const p = Math.sqrt(p2 / 6);
  const ai = (a - q) / p,
    bi = b / p,
    ci = c / p,
    di = (d - q) / p,
    ei = e / p,
    fi = (f - q) / p;
  const detB =
    ai * (di * fi - ei * ei) - bi * (bi * fi - ei * ci) + ci * (bi * ei - di * ci);
  let r = detB / 2;
  if (r < -1) r = -1;
  if (r > 1) r = 1;
  const phi = Math.acos(r) / 3;
  const eig1 = q + 2 * p * Math.cos(phi);
  const eig3 = q + 2 * p * Math.cos(phi + (2 * Math.PI) / 3);
  const eig2 = 3 * q - eig1 - eig3;
  const lambdaMin = Math.min(eig1, eig2, eig3);

  // (M - lambda*I) v = 0 → take cross product of two rows.
  const m11 = a - lambdaMin;
  const m22 = d - lambdaMin;
  const m33 = f - lambdaMin;
  const r1: [number, number, number] = [m11, b, c];
  const r2: [number, number, number] = [b, m22, e];
  const r3: [number, number, number] = [c, e, m33];

  const candidates: [number, number, number][] = [
    cross(r1, r2),
    cross(r2, r3),
    cross(r1, r3),
  ];
  let best: [number, number, number] = [0, 0, 1];
  let bestLen = -1;
  for (const v of candidates) {
    const l2 = v[0] * v[0] + v[1] * v[1] + v[2] * v[2];
    if (l2 > bestLen) {
      bestLen = l2;
      best = v;
    }
  }
  const len = Math.sqrt(bestLen);
  if (len < 1e-12) return [0, 0, 1];
  return [best[0] / len, best[1] / len, best[2] / len];
}

function cross(
  a: [number, number, number],
  b: [number, number, number],
): [number, number, number] {
  return [
    a[1] * b[2] - a[2] * b[1],
    a[2] * b[0] - a[0] * b[2],
    a[0] * b[1] - a[1] * b[0],
  ];
}
