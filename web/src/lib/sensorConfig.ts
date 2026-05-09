/**
 * LiDAR sensor specs used by the Patchwork (ground segmentation) and TRAVEL
 * (range-image clustering) demos. Both algorithms are highly sensor-dependent —
 * the rings, sectors, and FOVs below mirror the YAML configs shipped with
 * https://github.com/LimHyungTae/patchwork and https://github.com/url-kaist/TRAVEL.
 *
 * For the demo we only support VLP-16 (NaverLabs preset) and HDL-64 (KITTI
 * preset). CAD inputs like Stanford Bunny have no notion of a LiDAR ring and
 * are intentionally excluded from these chapters.
 */

export type SensorName = "VLP-16" | "HDL-64";

export type CzmConfig = {
  /** Minimum and maximum radii a point must satisfy to participate. */
  minR: number;
  maxR: number;
  /** Length numZones+1 — radial boundaries between concentric zones. */
  zoneBoundaries: number[];
  /** Length numZones — number of rings within each zone (uniform spacing). */
  numRingsPerZone: number[];
  /** Length numZones — number of angular sectors within each zone. */
  numSectorsPerZone: number[];
  /** Per-zone elevation thresholds (m, w.r.t. ground frame). */
  elevationThresholds: number[];
  /** Per-zone flatness thresholds (eigenvalue ratio). */
  flatnessThresholds: number[];
  /** Sensor mount height above ground (positive). */
  sensorHeight: number;
  /** R-GPF: lowest-point-representative count (seed initialization). */
  numLpr: number;
  /** R-GPF: refinement iteration count. */
  numIter: number;
  /** R-GPF: minimum points required in a bin before it's processed. */
  numMinPts: number;
  /** R-GPF: seeds are initial points whose z is within mean+thSeeds. */
  thSeeds: number;
  /** R-GPF: refinement plane-distance threshold. */
  thDist: number;
  /** Uprightness threshold on |n_z| (cos angle). */
  uprightnessThr: number;
};

export type RangeImageConfig = {
  rows: number;
  cols: number;
  minRange: number;
  maxRange: number;
  /** Vertical FOV bounds in degrees (lower, upper). */
  vertFovDeg: [number, number];
};

export type SensorConfig = {
  name: SensorName;
  numChannels: number;
  czm: CzmConfig;
  rangeImage: RangeImageConfig;
};

/** VLP-16 (e.g. NaverLabs indoor preset). Mirrors patchwork/config/velodyne16.yaml. */
export const SENSOR_VLP16: SensorConfig = {
  name: "VLP-16",
  numChannels: 16,
  czm: {
    minR: 0.6,
    maxR: 60.0,
    zoneBoundaries: [0.6, 6.0, 15.0, 30.0, 60.0],
    numRingsPerZone: [2, 2, 2, 1],
    numSectorsPerZone: [16, 32, 56, 32],
    elevationThresholds: [0.523, 0.746, 0.879, 1.125],
    flatnessThresholds: [0.0005, 0.000725, 0.001, 0.001],
    sensorHeight: 0.6,
    numLpr: 20,
    numIter: 3,
    numMinPts: 10,
    thSeeds: 0.2,
    thDist: 0.075,
    uprightnessThr: 0.707,
  },
  rangeImage: {
    rows: 16,
    cols: 1800,
    minRange: 0.6,
    maxRange: 60.0,
    vertFovDeg: [-15.0, 15.0],
  },
};

/** HDL-64 (KITTI preset). Mirrors patchwork/config/velodyne64.yaml. */
export const SENSOR_HDL64: SensorConfig = {
  name: "HDL-64",
  numChannels: 64,
  czm: {
    minR: 2.7,
    maxR: 80.0,
    zoneBoundaries: [2.7, 12.0, 30.0, 50.0, 80.0],
    numRingsPerZone: [2, 4, 4, 5],
    numSectorsPerZone: [16, 32, 56, 32],
    elevationThresholds: [0.523, 0.746, 0.879, 1.125],
    flatnessThresholds: [0.0005, 0.000725, 0.001, 0.001],
    sensorHeight: 1.723,
    numLpr: 20,
    numIter: 3,
    numMinPts: 10,
    thSeeds: 0.5,
    thDist: 0.125,
    uprightnessThr: 0.707,
  },
  rangeImage: {
    rows: 64,
    cols: 1800,
    minRange: 1.0,
    maxRange: 80.0,
    vertFovDeg: [-24.8, 2.0],
  },
};

export const SENSOR_BY_PRESET: Record<string, SensorConfig | null> = {
  bunny: null,
  naverlabs: SENSOR_VLP16,
  kitti: SENSOR_HDL64,
};
