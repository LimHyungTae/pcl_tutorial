import { runPatchwork } from "./patchwork";
import { cloudFromPositions, type PointCloud } from "../types";
import type { SensorConfig } from "../sensorConfig";

/**
 * Simplified port of TRAVEL (https://github.com/url-kaist/TRAVEL) — only the
 * Above-ground Object Segmentation (AOS) stage that does range-image-based
 * clustering. The Travelable Ground Segmentation (TGS) stage is replaced by
 * the Patchwork ground mask we already have, so the demo reads as
 * "ground (Patchwork) + clusters on the residual range image".
 *
 * Pipeline:
 *   1. project all non-ground points onto a (rows × cols) range image where
 *      row ↔ vertical FOV bin, col ↔ azimuth bin;
 *   2. flood-fill 4-connected pixels whose depth difference is below
 *      `depthDiffThresh`;
 *   3. drop tiny components ( < `minClusterSize` pixels) as noise.
 *
 * Each pixel stores at most one point — when multiple points fall in the same
 * pixel we keep the closest. This is faithful to TRAVEL's range-image grid.
 */

export type TravelResult = {
  /** Number of clusters produced. */
  numClusters: number;
  /** Length numClusters. Per-cluster point xyz cloud. */
  clusters: PointCloud[];
  /** Length numClusters. (cx, cy, cz). */
  centroids: [number, number, number][];
  /** Range image dimensions. */
  rows: number;
  cols: number;
  /** rows*cols, depth in meters. 0 = empty. */
  depths: Float32Array;
  /** rows*cols, cluster ID in 1-based range. 0 = empty or ground. */
  imageClusterIds: Int32Array;
  /** rows*cols, source point index into input cloud. -1 = empty. */
  imagePointIdx: Int32Array;
  /** Length numClusters. Pixel count of each cluster. */
  clusterPixelCounts: number[];
  /** Cloud holding only the points kept in the range image (1 per pixel). */
  imagedCloud: PointCloud;
  /** Ground sub-cloud from the Patchwork pre-pass — for visualization context. */
  groundCloud: PointCloud;
};

export type TravelOptions = {
  /** Discontinuity threshold in meters between horizontally-adjacent range
   *  pixels (azimuth direction). */
  horzMerge: number;
  /** Discontinuity threshold in meters between vertically-adjacent range
   *  pixels (elevation direction). */
  vertMerge: number;
  /** Minimum pixels for a cluster to be kept. */
  minClusterSize: number;
};

export function defaultTravelOptions(sensor: SensorConfig): TravelOptions {
  return {
    horzMerge: sensor.rangeImage.travelHorzMerge,
    vertMerge: sensor.rangeImage.travelVertMerge,
    minClusterSize: sensor.rangeImage.travelMinClusterSize,
  };
}

export function runTravel(
  input: PointCloud,
  sensor: SensorConfig,
  opts?: TravelOptions,
): TravelResult {
  const o = opts ?? defaultTravelOptions(sensor);
  const ri = sensor.rangeImage;
  const rows = ri.rows;
  const cols = ri.cols;
  const cells = rows * cols;
  const empty = (): TravelResult => ({
    numClusters: 0,
    clusters: [],
    centroids: [],
    rows,
    cols,
    depths: new Float32Array(cells),
    imageClusterIds: new Int32Array(cells),
    imagePointIdx: new Int32Array(cells).fill(-1),
    clusterPixelCounts: [],
    imagedCloud: cloudFromPositions(new Float32Array(0)),
    groundCloud: cloudFromPositions(new Float32Array(0)),
  });
  if (input.count === 0) return empty();

  const ground = runPatchwork(input, sensor);

  const depths = new Float32Array(cells);
  const imagePointIdx = new Int32Array(cells);
  for (let i = 0; i < cells; i++) imagePointIdx[i] = -1;

  const fovLo = (ri.vertFovDeg[0] * Math.PI) / 180;
  const fovHi = (ri.vertFovDeg[1] * Math.PI) / 180;
  const fovSpan = fovHi - fovLo;

  const N = input.count;
  for (let i = 0; i < N; i++) {
    if (ground.isGround[i]) continue;
    const x = input.positions[3 * i];
    const y = input.positions[3 * i + 1];
    const z = input.positions[3 * i + 2];
    const r = Math.sqrt(x * x + y * y + z * z);
    if (r < ri.minRange || r > ri.maxRange) continue;

    const elev = Math.atan2(z, Math.sqrt(x * x + y * y));
    if (elev < fovLo || elev > fovHi) continue;
    const row = Math.min(
      rows - 1,
      Math.max(0, Math.floor(((elev - fovLo) / fovSpan) * rows)),
    );

    const az = Math.atan2(y, x);
    let col = Math.floor(((az + Math.PI) / (2 * Math.PI)) * cols);
    if (col < 0) col = 0;
    if (col >= cols) col = cols - 1;

    // Y-down image: row 0 at the top (highest elevation), row rows-1 at the
    // bottom (lowest). Patchwork+TRAVEL convention.
    const flippedRow = rows - 1 - row;
    const px = flippedRow * cols + col;

    if (imagePointIdx[px] === -1 || r < depths[px]) {
      imagePointIdx[px] = i;
      depths[px] = r;
    }
  }

  // BFS flood-fill on 4-connected pixels with depth-difference threshold.
  const imageClusterIds = new Int32Array(cells);
  const queue = new Int32Array(cells);
  let nextClusterId = 0;
  const clusterPixels: number[][] = [];

  for (let p = 0; p < cells; p++) {
    if (imagePointIdx[p] === -1 || imageClusterIds[p] !== 0) continue;
    nextClusterId++;
    const id = nextClusterId;
    imageClusterIds[p] = id;
    let head = 0, tail = 0;
    queue[tail++] = p;
    const members: number[] = [p];

    while (head < tail) {
      const cur = queue[head++];
      const r = (cur / cols) | 0;
      const c = cur - r * cols;
      const curD = depths[cur];

      // 4-connected neighbors. Wrap horizontally — azimuth is periodic.
      // Vertical (row ±1) uses vertMerge; horizontal (col ±1) uses horzMerge.
      const neighbors: { idx: number; thresh: number }[] = [];
      if (r > 0) neighbors.push({ idx: (r - 1) * cols + c, thresh: o.vertMerge });
      if (r < rows - 1) neighbors.push({ idx: (r + 1) * cols + c, thresh: o.vertMerge });
      neighbors.push({
        idx: r * cols + (c === 0 ? cols - 1 : c - 1),
        thresh: o.horzMerge,
      });
      neighbors.push({
        idx: r * cols + (c === cols - 1 ? 0 : c + 1),
        thresh: o.horzMerge,
      });

      for (const { idx: np, thresh } of neighbors) {
        if (imagePointIdx[np] === -1) continue;
        if (imageClusterIds[np] !== 0) continue;
        const nd = depths[np];
        if (Math.abs(nd - curD) > thresh) continue;
        imageClusterIds[np] = id;
        queue[tail++] = np;
        members.push(np);
      }
    }
    clusterPixels.push(members);
  }

  // Drop small clusters; remap survivors to dense IDs starting at 1.
  const survivors: number[][] = [];
  const idRemap = new Int32Array(nextClusterId + 1); // 1-based
  for (let id = 1; id <= nextClusterId; id++) {
    const pixels = clusterPixels[id - 1];
    if (pixels.length >= o.minClusterSize) {
      survivors.push(pixels);
      idRemap[id] = survivors.length;
    } else {
      idRemap[id] = 0;
    }
  }
  for (let p = 0; p < cells; p++) {
    if (imageClusterIds[p] !== 0) {
      imageClusterIds[p] = idRemap[imageClusterIds[p]];
    }
  }

  // Build per-cluster sub-clouds + centroids; sort largest-first.
  const sortedSurvivors = survivors
    .map((pixels, idx) => ({ pixels, oldId: idx + 1 }))
    .sort((a, b) => b.pixels.length - a.pixels.length);

  const clusters: PointCloud[] = [];
  const centroids: [number, number, number][] = [];
  const clusterPixelCounts: number[] = [];
  // Final remap: oldDenseId → newRankId (1-based after sort).
  const finalRemap = new Int32Array(survivors.length + 1);
  for (let r = 0; r < sortedSurvivors.length; r++) {
    finalRemap[sortedSurvivors[r].oldId] = r + 1;
    const pixels = sortedSurvivors[r].pixels;
    const xyz = new Float32Array(pixels.length * 3);
    let cx = 0, cy = 0, cz = 0;
    for (let k = 0; k < pixels.length; k++) {
      const p = pixels[k];
      const i = imagePointIdx[p];
      const x = input.positions[3 * i];
      const y = input.positions[3 * i + 1];
      const z = input.positions[3 * i + 2];
      xyz[k * 3] = x;
      xyz[k * 3 + 1] = y;
      xyz[k * 3 + 2] = z;
      cx += x;
      cy += y;
      cz += z;
    }
    const inv = 1 / pixels.length;
    clusters.push(cloudFromPositions(xyz));
    centroids.push([cx * inv, cy * inv, cz * inv]);
    clusterPixelCounts.push(pixels.length);
  }
  for (let p = 0; p < cells; p++) {
    if (imageClusterIds[p] !== 0) {
      imageClusterIds[p] = finalRemap[imageClusterIds[p]];
    }
  }

  // Cloud holding every point that survived the range image (one per pixel)
  // — useful as a bbox-only layer for camera framing.
  let imagedN = 0;
  for (let p = 0; p < cells; p++) if (imagePointIdx[p] !== -1) imagedN++;
  const imagedXyz = new Float32Array(imagedN * 3);
  let w = 0;
  for (let p = 0; p < cells; p++) {
    const i = imagePointIdx[p];
    if (i === -1) continue;
    imagedXyz[w++] = input.positions[3 * i];
    imagedXyz[w++] = input.positions[3 * i + 1];
    imagedXyz[w++] = input.positions[3 * i + 2];
  }

  return {
    numClusters: clusters.length,
    clusters,
    centroids,
    rows,
    cols,
    depths,
    imageClusterIds,
    imagePointIdx,
    clusterPixelCounts,
    imagedCloud: cloudFromPositions(imagedXyz),
    groundCloud: ground.groundCloud,
  };
}

/** Same scheme as Euclidean Clustering for visual consistency across chapters. */
export function travelClusterColor(rank: number): string {
  // rank: 0-based index. Use the same golden-angle spiral.
  const hue = (rank * 137.508) % 360;
  return `hsl(${hue.toFixed(0)}, 78%, 60%)`;
}
