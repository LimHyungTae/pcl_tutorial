import { KdTree } from "../kdtree";
import { cloudFromPositions, type PointCloud } from "../types";

export type Cluster = {
  cloud: PointCloud;
  size: number;
  centroid: [number, number, number];
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
    let cx = 0,
      cy = 0,
      cz = 0;
    for (let m = 0; m < members.length; m++) {
      const off = members[m] * 3;
      const x = input.positions[off];
      const y = input.positions[off + 1];
      const z = input.positions[off + 2];
      xyz[m * 3] = x;
      xyz[m * 3 + 1] = y;
      xyz[m * 3 + 2] = z;
      cx += x;
      cy += y;
      cz += z;
    }
    const inv = 1 / members.length;
    clusters.push({
      cloud: cloudFromPositions(xyz),
      size: members.length,
      centroid: [cx * inv, cy * inv, cz * inv],
    });
  }

  // Largest first so visualization always pairs the biggest with the brightest hue.
  clusters.sort((a, b) => b.size - a.size);
  return clusters;
}

/** Color drawn from the golden angle hue spiral — stable for any index.
 *  Note: three.js's Color parser only accepts the legacy comma-separated
 *  hsl() form, NOT the modern space-separated one. */
export function clusterColor(i: number): string {
  const hue = (i * 137.508) % 360;
  return `hsl(${hue.toFixed(0)}, 78%, 60%)`;
}

/**
 * Persistent color matcher: maps each new cluster to the color of the closest
 * previous cluster (centroid distance), so colors stay stable as the user
 * slides parameters. Clusters with no prior within `matchRadius` get a fresh
 * color.
 */
export class ClusterColorMatcher {
  private next = 0;
  /** Anchors active "after" the most recent assign() call. */
  private anchors: { color: string; centroid: [number, number, number] }[] = [];

  reset(): void {
    this.next = 0;
    this.anchors = [];
  }

  assign(clusters: Cluster[], matchRadius: number): string[] {
    const r2 = matchRadius * matchRadius;
    const used = new Set<number>();
    const out: string[] = [];
    const newAnchors: typeof this.anchors = [];

    for (const c of clusters) {
      // Greedy nearest-anchor by squared centroid distance.
      let best = -1;
      let bestD = Infinity;
      for (let i = 0; i < this.anchors.length; i++) {
        if (used.has(i)) continue;
        const a = this.anchors[i];
        const dx = a.centroid[0] - c.centroid[0];
        const dy = a.centroid[1] - c.centroid[1];
        const dz = a.centroid[2] - c.centroid[2];
        const d = dx * dx + dy * dy + dz * dz;
        if (d < bestD) {
          bestD = d;
          best = i;
        }
      }
      let color: string;
      if (best >= 0 && bestD <= r2) {
        color = this.anchors[best].color;
        used.add(best);
      } else {
        color = clusterColor(this.next++);
      }
      out.push(color);
      newAnchors.push({ color, centroid: c.centroid });
    }
    this.anchors = newAnchors;
    return out;
  }
}
