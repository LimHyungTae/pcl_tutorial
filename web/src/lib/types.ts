/** Plain XYZ point cloud — Float32Array of length 3*N. */
export type PointCloud = {
  positions: Float32Array;
  count: number;
  bbox: { min: [number, number, number]; max: [number, number, number] };
};

export const emptyCloud = (): PointCloud => ({
  positions: new Float32Array(0),
  count: 0,
  bbox: { min: [0, 0, 0], max: [0, 0, 0] },
});

export const computeBBox = (positions: Float32Array): PointCloud["bbox"] => {
  if (positions.length === 0) {
    return { min: [0, 0, 0], max: [0, 0, 0] };
  }
  let minX = Infinity,
    minY = Infinity,
    minZ = Infinity;
  let maxX = -Infinity,
    maxY = -Infinity,
    maxZ = -Infinity;
  for (let i = 0; i < positions.length; i += 3) {
    const x = positions[i],
      y = positions[i + 1],
      z = positions[i + 2];
    if (x < minX) minX = x;
    if (y < minY) minY = y;
    if (z < minZ) minZ = z;
    if (x > maxX) maxX = x;
    if (y > maxY) maxY = y;
    if (z > maxZ) maxZ = z;
  }
  return { min: [minX, minY, minZ], max: [maxX, maxY, maxZ] };
};

export const cloudFromPositions = (positions: Float32Array): PointCloud => ({
  positions,
  count: positions.length / 3,
  bbox: computeBBox(positions),
});
