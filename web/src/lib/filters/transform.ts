import { cloudFromPositions, type PointCloud } from "../types";

/**
 * Build a 4×4 rigid transform from a translation and ZYX-Euler rotation
 * (degrees). Layout is row-major Float32Array(16) — the same convention used
 * by Eigen::Matrix4f when filled row-by-row.
 *
 *   T = [ R  t ]
 *       [ 0  1 ]
 *
 *   R = Rz(rz) · Ry(ry) · Rx(rx)
 */
export function buildTransform(
  tx: number,
  ty: number,
  tz: number,
  rxDeg: number,
  ryDeg: number,
  rzDeg: number,
): Float32Array {
  const rx = (rxDeg * Math.PI) / 180;
  const ry = (ryDeg * Math.PI) / 180;
  const rz = (rzDeg * Math.PI) / 180;
  const cx = Math.cos(rx),
    sx = Math.sin(rx);
  const cy = Math.cos(ry),
    sy = Math.sin(ry);
  const cz = Math.cos(rz),
    sz = Math.sin(rz);

  const m = new Float32Array(16);
  m[0] = cz * cy;
  m[1] = cz * sy * sx - sz * cx;
  m[2] = cz * sy * cx + sz * sx;
  m[3] = tx;

  m[4] = sz * cy;
  m[5] = sz * sy * sx + cz * cx;
  m[6] = sz * sy * cx - cz * sx;
  m[7] = ty;

  m[8] = -sy;
  m[9] = cy * sx;
  m[10] = cy * cx;
  m[11] = tz;

  m[12] = 0;
  m[13] = 0;
  m[14] = 0;
  m[15] = 1;
  return m;
}

/** Apply a 4×4 (row-major) transform to every point in the cloud. */
export function transformCloud(input: PointCloud, m: Float32Array): PointCloud {
  if (input.count === 0) return input;
  const out = new Float32Array(input.count * 3);
  const p = input.positions;
  for (let i = 0; i < input.count; i++) {
    const x = p[i * 3];
    const y = p[i * 3 + 1];
    const z = p[i * 3 + 2];
    out[i * 3] = m[0] * x + m[1] * y + m[2] * z + m[3];
    out[i * 3 + 1] = m[4] * x + m[5] * y + m[6] * z + m[7];
    out[i * 3 + 2] = m[8] * x + m[9] * y + m[10] * z + m[11];
  }
  return cloudFromPositions(out);
}

/** Format the row-major matrix as Eigen-style C++ code. */
export function formatEigenMatrix4(m: Float32Array): string {
  const f = (v: number) => (Math.abs(v) < 1e-6 ? "0" : v.toFixed(3));
  return [
    `Eigen::Matrix4f T;`,
    `T << ${f(m[0])}, ${f(m[1])}, ${f(m[2])}, ${f(m[3])},`,
    `     ${f(m[4])}, ${f(m[5])}, ${f(m[6])}, ${f(m[7])},`,
    `     ${f(m[8])}, ${f(m[9])}, ${f(m[10])}, ${f(m[11])},`,
    `     0, 0, 0, 1;`,
    `pcl::transformPointCloud(*src, *out, T);`,
  ].join("\n");
}
