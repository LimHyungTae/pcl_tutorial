import { cloudFromPositions, type PointCloud } from "./types";

/**
 * KITTI .bin: tightly packed float32 [x, y, z, intensity] per point.
 * We strip intensity and keep XYZ only.
 */
export function parseKittiBin(buf: ArrayBuffer): PointCloud {
  const f = new Float32Array(buf);
  const n = Math.floor(f.length / 4);
  const xyz = new Float32Array(n * 3);
  for (let i = 0; i < n; i++) {
    xyz[i * 3] = f[i * 4];
    xyz[i * 3 + 1] = f[i * 4 + 1];
    xyz[i * 3 + 2] = f[i * 4 + 2];
  }
  return cloudFromPositions(xyz);
}
