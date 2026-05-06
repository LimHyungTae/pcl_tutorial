import { cloudFromPositions, type PointCloud } from "../types";

/**
 * Voxel-grid downsampling — same idea as pcl::VoxelGrid:
 * partition space into cubic voxels of `leafSize` and emit one point
 * per non-empty voxel (the centroid of points falling into it).
 */
export function voxelGrid(input: PointCloud, leafSize: number): PointCloud {
  if (input.count === 0 || leafSize <= 0) return input;

  // Use a Map keyed by voxel index. Storing sums lets us emit centroids.
  const inv = 1 / leafSize;
  const map = new Map<string, { sx: number; sy: number; sz: number; n: number }>();

  const p = input.positions;
  for (let i = 0; i < p.length; i += 3) {
    const x = p[i],
      y = p[i + 1],
      z = p[i + 2];
    const ix = Math.floor(x * inv);
    const iy = Math.floor(y * inv);
    const iz = Math.floor(z * inv);
    const key = `${ix},${iy},${iz}`;
    const cell = map.get(key);
    if (cell) {
      cell.sx += x;
      cell.sy += y;
      cell.sz += z;
      cell.n += 1;
    } else {
      map.set(key, { sx: x, sy: y, sz: z, n: 1 });
    }
  }

  const out = new Float32Array(map.size * 3);
  let i = 0;
  for (const cell of map.values()) {
    out[i++] = cell.sx / cell.n;
    out[i++] = cell.sy / cell.n;
    out[i++] = cell.sz / cell.n;
  }
  return cloudFromPositions(out);
}
