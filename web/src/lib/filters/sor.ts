import { KdTree } from "../kdtree";
import { cloudFromPositions, type PointCloud } from "../types";

export type SorResult = {
  inliers: PointCloud;
  outliers: PointCloud;
};

/**
 * Statistical Outlier Removal — pcl::StatisticalOutlierRemoval analogue.
 *
 * For each point, average distance to its `meanK` nearest neighbors is
 * computed. A point whose mean distance exceeds (global mean + stddev *
 * stddev_mult) is flagged as an outlier.
 */
export function sor(
  input: PointCloud,
  meanK: number,
  stddevMult: number,
): SorResult {
  if (input.count === 0)
    return { inliers: input, outliers: cloudFromPositions(new Float32Array(0)) };

  const tree = new KdTree(input.positions);
  const k = Math.min(meanK + 1, input.count); // +1 because the point itself is included
  const N = input.count;
  const meanDist = new Float64Array(N);

  // Pass 1: per-point mean distance to k nearest (excluding self).
  for (let i = 0; i < N; i++) {
    const px = input.positions[i * 3];
    const py = input.positions[i * 3 + 1];
    const pz = input.positions[i * 3 + 2];
    const { sqrDists } = tree.knnSearch(px, py, pz, k);
    let sum = 0;
    let counted = 0;
    for (let j = 0; j < sqrDists.length; j++) {
      // Skip the self match (sqrDist ≈ 0).
      if (sqrDists[j] < 1e-12) continue;
      sum += Math.sqrt(sqrDists[j]);
      counted++;
    }
    meanDist[i] = counted > 0 ? sum / counted : 0;
  }

  // Pass 2: global mean and stddev across meanDist.
  let m = 0;
  for (let i = 0; i < N; i++) m += meanDist[i];
  m /= N;
  let v = 0;
  for (let i = 0; i < N; i++) {
    const d = meanDist[i] - m;
    v += d * d;
  }
  const stddev = Math.sqrt(v / N);
  const threshold = m + stddevMult * stddev;

  // Pass 3: split.
  let inN = 0;
  for (let i = 0; i < N; i++) if (meanDist[i] <= threshold) inN++;
  const inliers = new Float32Array(inN * 3);
  const outliers = new Float32Array((N - inN) * 3);
  let ii = 0,
    oi = 0;
  for (let i = 0; i < N; i++) {
    const off = i * 3;
    const x = input.positions[off];
    const y = input.positions[off + 1];
    const z = input.positions[off + 2];
    if (meanDist[i] <= threshold) {
      inliers[ii++] = x;
      inliers[ii++] = y;
      inliers[ii++] = z;
    } else {
      outliers[oi++] = x;
      outliers[oi++] = y;
      outliers[oi++] = z;
    }
  }
  return {
    inliers: cloudFromPositions(inliers),
    outliers: cloudFromPositions(outliers),
  };
}
