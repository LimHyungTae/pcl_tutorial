/**
 * Closed-form eigendecomposition of a 3×3 real symmetric matrix
 *   [[a, b, c],
 *    [b, d, e],
 *    [c, e, f]]
 *
 * Returns the eigenvalues sorted ascending and the unit eigenvector
 * corresponding to the smallest eigenvalue. Used by Patchwork's R-GPF
 * (ground plane fitting) and Normal Estimation.
 */
export type SymEig3 = {
  /** Eigenvalues sorted ascending. */
  values: [number, number, number];
  /** Unit eigenvector for the smallest eigenvalue. */
  smallestVec: [number, number, number];
};

export function symEig3x3(
  a: number, b: number, c: number,
  d: number, e: number, f: number,
): SymEig3 {
  const p1 = b * b + c * c + e * e;
  if (p1 === 0) {
    // Already diagonal — eigenvalues = diagonal entries.
    const sorted = [
      { v: a, axis: [1, 0, 0] as [number, number, number] },
      { v: d, axis: [0, 1, 0] as [number, number, number] },
      { v: f, axis: [0, 0, 1] as [number, number, number] },
    ].sort((x, y) => x.v - y.v);
    return {
      values: [sorted[0].v, sorted[1].v, sorted[2].v],
      smallestVec: sorted[0].axis,
    };
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
  const sorted = [eig1, eig2, eig3].sort((x, y) => x - y) as [number, number, number];
  const lambdaMin = sorted[0];

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
  if (len < 1e-12) return { values: sorted, smallestVec: [0, 0, 1] };
  return {
    values: sorted,
    smallestVec: [best[0] / len, best[1] / len, best[2] / len],
  };
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
