import { symEig3x3 } from "../eigen";
import { cloudFromPositions, type PointCloud } from "../types";
import type { CzmConfig, SensorConfig } from "../sensorConfig";

/**
 * Simplified port of Patchwork (https://github.com/LimHyungTae/patchwork).
 *
 * Concentric Zone Model (CZM): split the radial space into 4 zones, each
 * subdivided into rings × sectors → ~few hundred bins. For each bin we run
 * R-GPF (Region-wise Ground Plane Fitting):
 *   1. seed with the lowest points (z within mean+thSeeds of the LPR mean);
 *   2. fit a plane via PCA on the seeds; refit `numIter` times keeping points
 *      within `thDist` of the fitted plane;
 *   3. accept the bin's seeds as ground iff (uprightness, elevation, flatness)
 *      gates pass.
 *
 * We only support sensors whose CZM/R-GPF params are known to us (VLP-16,
 * HDL-64). For other inputs the demo falls back to a "no ground found"
 * result and the page surfaces a caution box pointing to the upstream repo.
 *
 * Returned region IDs are packed as `zone * 100 + ring` so the page can
 * color each (zone, ring) bin uniquely.
 */

export type PatchworkResult = {
  /** Length N — true if the point was classified as ground. */
  isGround: Uint8Array;
  /** Length N — packed region ID = zone * 100 + ring. -1 if unbinned. */
  regionId: Int32Array;
  /** Sub-cloud of ground points. */
  groundCloud: PointCloud;
  /** Sub-cloud of non-ground points. */
  nonGroundCloud: PointCloud;
  /** For each ground point, its parent point's regionId — same length as groundCloud.count. */
  groundRegionIds: Int32Array;
  /** Sorted unique region IDs that contain at least one ground point. */
  uniqueRegions: number[];
  /** Original input cloud (passthrough, useful for visualization plumbing). */
  source: PointCloud;
};

export function runPatchwork(
  input: PointCloud,
  sensor: SensorConfig,
): PatchworkResult {
  const N = input.count;
  const isGround = new Uint8Array(N);
  const regionId = new Int32Array(N);
  for (let i = 0; i < N; i++) regionId[i] = -1;

  if (N === 0) {
    return {
      isGround,
      regionId,
      groundCloud: cloudFromPositions(new Float32Array(0)),
      nonGroundCloud: cloudFromPositions(new Float32Array(0)),
      groundRegionIds: new Int32Array(0),
      uniqueRegions: [],
      source: input,
    };
  }

  const c = sensor.czm;

  // Pre-compute ring boundaries within each zone for uniform radial subdivision.
  const ringBoundaries: number[][] = c.numRingsPerZone.map((nRings, z) => {
    const zMin = c.zoneBoundaries[z];
    const zMax = c.zoneBoundaries[z + 1];
    const arr: number[] = [];
    for (let i = 0; i <= nRings; i++) {
      arr.push(zMin + (zMax - zMin) * (i / nRings));
    }
    return arr;
  });

  // Bin every point. Out-of-range points stay non-ground with regionId = -1.
  const pointZone = new Int32Array(N);
  const pointRing = new Int32Array(N);
  const pointSector = new Int32Array(N);
  const bins = new Map<number, number[]>();
  for (let i = 0; i < N; i++) {
    pointZone[i] = -1;
    pointRing[i] = -1;
    pointSector[i] = -1;
    const x = input.positions[3 * i];
    const y = input.positions[3 * i + 1];
    const r2 = x * x + y * y;
    if (r2 < c.minR * c.minR || r2 > c.maxR * c.maxR) continue;
    const r = Math.sqrt(r2);

    let zone = -1;
    for (let z = 0; z < c.numRingsPerZone.length; z++) {
      if (r >= c.zoneBoundaries[z] && r < c.zoneBoundaries[z + 1]) {
        zone = z;
        break;
      }
    }
    if (zone < 0) continue;

    const rb = ringBoundaries[zone];
    let ring = c.numRingsPerZone[zone] - 1;
    for (let k = 0; k < c.numRingsPerZone[zone]; k++) {
      if (r >= rb[k] && r < rb[k + 1]) {
        ring = k;
        break;
      }
    }

    const angle = Math.atan2(y, x);
    const ns = c.numSectorsPerZone[zone];
    let sector = Math.floor(((angle + Math.PI) / (2 * Math.PI)) * ns);
    if (sector < 0) sector = 0;
    if (sector >= ns) sector = ns - 1;

    pointZone[i] = zone;
    pointRing[i] = ring;
    pointSector[i] = sector;
    regionId[i] = zone * 100 + ring;

    // Pack bin key as zone × 1e6 + ring × 1e3 + sector — fits in 32 bits.
    const key = zone * 1_000_000 + ring * 1_000 + sector;
    let arr = bins.get(key);
    if (!arr) {
      arr = [];
      bins.set(key, arr);
    }
    arr.push(i);
  }

  // R-GPF per bin.
  for (const [, idxs] of bins) {
    if (idxs.length < c.numMinPts) continue;
    const sampleZone = pointZone[idxs[0]];

    // Sort by z ascending to find lowest-point representatives.
    const sorted = idxs.slice().sort(
      (a, b) => input.positions[3 * a + 2] - input.positions[3 * b + 2],
    );

    const lprCount = Math.min(c.numLpr, sorted.length);
    let lprMean = 0;
    for (let k = 0; k < lprCount; k++) {
      lprMean += input.positions[3 * sorted[k] + 2];
    }
    lprMean /= lprCount;

    const seedZThresh = lprMean + c.thSeeds;
    let curSeeds: number[] = [];
    for (const idx of sorted) {
      if (input.positions[3 * idx + 2] < seedZThresh) curSeeds.push(idx);
      else break;
    }
    if (curSeeds.length < 3) continue;

    let normal: [number, number, number] = [0, 0, 1];
    let planeD = 0;
    let smallestEig = 0;
    let sumEig = 1;
    let centroidZ = 0;

    for (let iter = 0; iter < c.numIter; iter++) {
      // Centroid + covariance over current seeds.
      let cx = 0, cy = 0, cz = 0;
      for (const i of curSeeds) {
        cx += input.positions[3 * i];
        cy += input.positions[3 * i + 1];
        cz += input.positions[3 * i + 2];
      }
      const inv = 1 / curSeeds.length;
      cx *= inv;
      cy *= inv;
      cz *= inv;

      let cxx = 0, cxy = 0, cxz = 0, cyy = 0, cyz = 0, czz = 0;
      for (const i of curSeeds) {
        const dx = input.positions[3 * i] - cx;
        const dy = input.positions[3 * i + 1] - cy;
        const dz = input.positions[3 * i + 2] - cz;
        cxx += dx * dx;
        cxy += dx * dy;
        cxz += dx * dz;
        cyy += dy * dy;
        cyz += dy * dz;
        czz += dz * dz;
      }
      cxx *= inv; cxy *= inv; cxz *= inv;
      cyy *= inv; cyz *= inv; czz *= inv;

      const eig = symEig3x3(cxx, cxy, cxz, cyy, cyz, czz);
      normal = eig.smallestVec;
      planeD = -(normal[0] * cx + normal[1] * cy + normal[2] * cz);
      smallestEig = eig.values[0];
      sumEig = eig.values[0] + eig.values[1] + eig.values[2] + 1e-20;
      centroidZ = cz;

      // Refine: keep points within thDist of the current plane.
      const next: number[] = [];
      for (const i of idxs) {
        const dist = Math.abs(
          normal[0] * input.positions[3 * i] +
            normal[1] * input.positions[3 * i + 1] +
            normal[2] * input.positions[3 * i + 2] +
            planeD,
        );
        if (dist < c.thDist) next.push(i);
      }
      if (next.length < 3) {
        curSeeds = [];
        break;
      }
      curSeeds = next;
    }

    if (curSeeds.length < 3) continue;

    const upright = Math.abs(normal[2]) > c.uprightnessThr;
    const elevThr = c.elevationThresholds[Math.min(sampleZone, c.elevationThresholds.length - 1)];
    const flatThr = c.flatnessThresholds[Math.min(sampleZone, c.flatnessThresholds.length - 1)];
    // Patchwork applies the elevation gate strictly only on the outer two zones.
    const elevOk = sampleZone < 2 || centroidZ < -c.sensorHeight + elevThr;
    const flatness = smallestEig / sumEig;
    const flatOk = flatness < flatThr;

    if (!(upright && elevOk && flatOk)) continue;

    for (const i of curSeeds) isGround[i] = 1;
  }

  // Materialize sub-clouds and the per-ground-point region ID parallel array.
  let groundCount = 0;
  for (let i = 0; i < N; i++) if (isGround[i]) groundCount++;
  const groundXyz = new Float32Array(groundCount * 3);
  const ngXyz = new Float32Array((N - groundCount) * 3);
  const groundRegionIds = new Int32Array(groundCount);
  let g = 0,
    ng = 0,
    gi = 0;
  for (let i = 0; i < N; i++) {
    const x = input.positions[3 * i];
    const y = input.positions[3 * i + 1];
    const z = input.positions[3 * i + 2];
    if (isGround[i]) {
      groundXyz[g++] = x;
      groundXyz[g++] = y;
      groundXyz[g++] = z;
      groundRegionIds[gi++] = regionId[i];
    } else {
      ngXyz[ng++] = x;
      ngXyz[ng++] = y;
      ngXyz[ng++] = z;
    }
  }

  const seen = new Set<number>();
  for (let i = 0; i < groundRegionIds.length; i++) seen.add(groundRegionIds[i]);
  const uniqueRegions = Array.from(seen).sort((a, b) => a - b);

  return {
    isGround,
    regionId,
    groundCloud: cloudFromPositions(groundXyz),
    nonGroundCloud: cloudFromPositions(ngXyz),
    groundRegionIds,
    uniqueRegions,
    source: input,
  };
}

/** Pack (zone, ring) → stable color via the golden-angle hue spiral.
 *  Note: three.js's Color parser needs the comma-separated hsl() form. */
export function regionColor(zone: number, ring: number, czm: CzmConfig): string {
  const totalRingsBefore = czm.numRingsPerZone
    .slice(0, zone)
    .reduce((a, b) => a + b, 0);
  const idx = totalRingsBefore + ring;
  const hue = (idx * 137.508) % 360;
  // Outer zones get slightly desaturated to keep the palette cohesive.
  const sat = 78 - zone * 4;
  const lit = 60 - zone * 2;
  return `hsl(${hue.toFixed(0)}, ${sat}%, ${lit}%)`;
}
