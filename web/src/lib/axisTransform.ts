import { cloudFromPositions, type PointCloud } from "./types";

/**
 * Patchwork and TRAVEL both assume the LiDAR cloud is given in a Z-up frame
 * (sensor at z = +sensor_height above the floor, gravity along -Z). Some
 * preset datasets ship in different conventions:
 *
 *   - KITTI HDL-64: Z-up already (X forward, Y left, Z up).
 *   - NaverLabs VLP-16 PCD: Z-down (floor mass sits at z ≈ +0.6, ceiling at
 *     large negative z) — needs Z to be flipped before either algorithm
 *     will produce meaningful ground / cluster output.
 *
 * Custom user uploads are disabled on the Patchwork / TRAVEL chapters, so
 * a small per-preset switch is enough.
 */
export function ensureZUp(cloud: PointCloud, presetId: string): PointCloud {
  if (presetId !== "naverlabs") return cloud;
  const N = cloud.count;
  const out = new Float32Array(N * 3);
  for (let i = 0; i < N; i++) {
    out[3 * i] = cloud.positions[3 * i];
    out[3 * i + 1] = cloud.positions[3 * i + 1];
    out[3 * i + 2] = -cloud.positions[3 * i + 2];
  }
  return cloudFromPositions(out);
}
