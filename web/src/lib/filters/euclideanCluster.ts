import { KdTree } from "../kdtree";
import { cloudFromPositions, type PointCloud } from "../types";

export type Cluster = {
  cloud: PointCloud;
  size: number;
};

/**
 * Euclidean cluster extraction — pcl::EuclideanClusterExtraction analogue.
 *
 * BFS through points using `tolerance` as the radius threshold. Two points
 * end up in the same cluster if they're connected by a chain of jumps of
 * size ≤ tolerance. Clusters outside [minSize, maxSize] are dropped.
 */
export function euclideanCluster(
  input: PointCloud,
  tolerance: number,
  minSize: number,
  maxSize: number,
): Cluster[] {
  const N = input.count;
  if (N === 0) return [];

  const tree = new KdTree(input.positions);
  const visited = new Uint8Array(N);
  const clusters: Cluster[] = [];
  const queue: number[] = [];

  for (let seed = 0; seed < N; seed++) {
    if (visited[seed]) continue;
    visited[seed] = 1;
    const members: number[] = [seed];
    queue.length = 0;
    queue.push(seed);

    while (queue.length > 0) {
      const cur = queue.pop()!;
      const off = cur * 3;
      const neigh = tree.radiusSearch(
        input.positions[off],
        input.positions[off + 1],
        input.positions[off + 2],
        tolerance,
      );
      for (const idx of neigh) {
        if (!visited[idx]) {
          visited[idx] = 1;
          members.push(idx);
          queue.push(idx);
        }
      }
    }

    if (members.length < minSize || members.length > maxSize) continue;
    const xyz = new Float32Array(members.length * 3);
    for (let m = 0; m < members.length; m++) {
      const off = members[m] * 3;
      xyz[m * 3] = input.positions[off];
      xyz[m * 3 + 1] = input.positions[off + 1];
      xyz[m * 3 + 2] = input.positions[off + 2];
    }
    clusters.push({ cloud: cloudFromPositions(xyz), size: members.length });
  }

  // Largest first so visualization always pairs the biggest with the brightest hue.
  clusters.sort((a, b) => b.size - a.size);
  return clusters;
}

/** Generate distinct, saturated colors for cluster visualization. */
export function clusterColor(i: number): string {
  const hue = (i * 137.508) % 360; // golden angle → well spread
  return `hsl(${hue.toFixed(0)} 78% 60%)`;
}
