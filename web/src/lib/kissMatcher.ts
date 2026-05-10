import { cloudFromPositions, type PointCloud } from "./types";

/**
 * Shape of the JSON produced by `tools/gen_kiss_matcher_data.py`. The web
 * page never runs KISS-Matcher itself; it just visualizes whatever this
 * file says.
 */
export type KissMatcherData = {
  /** Public URL of the source point cloud — relative to web/public/. */
  srcUrl: string;
  /** Public URL of the target point cloud — relative to web/public/. */
  tgtUrl: string;
  voxelSize: number;
  /** KISS-Matcher's estimate. R is row-major 3×3, length 9. */
  estimatedRotation: number[];
  estimatedTranslation: number[];
  /** Raw KISS-Matcher correspondences. `src` / `tgt` are 3D points already
   *  in their respective frames. `isFinal` flags inliers surviving GNC. */
  matches: Array<{ src: [number, number, number]; tgt: [number, number, number]; isFinal: boolean }>;
  stats: {
    numSrcPoints: number;
    numTgtPoints: number;
    numInitialMatches: number;
    numFinalInliers: number;
    numRotationInliers: number;
  };
};

export async function loadKissMatcherData(url: string): Promise<KissMatcherData> {
  const res = await fetch(url);
  if (!res.ok) throw new Error(`Failed to load ${url}: ${res.status}`);
  return await res.json();
}

/** Apply a rigid 3×3 rotation + translation to a packed XYZ buffer. */
export function applyRigid(
  positions: Float32Array,
  R: number[],
  t: number[],
): PointCloud {
  const r0 = R[0], r1 = R[1], r2 = R[2];
  const r3 = R[3], r4 = R[4], r5 = R[5];
  const r6 = R[6], r7 = R[7], r8 = R[8];
  const tx = t[0], ty = t[1], tz = t[2];
  const out = new Float32Array(positions.length);
  for (let i = 0; i < positions.length; i += 3) {
    const x = positions[i];
    const y = positions[i + 1];
    const z = positions[i + 2];
    out[i] = r0 * x + r1 * y + r2 * z + tx;
    out[i + 1] = r3 * x + r4 * y + r5 * z + ty;
    out[i + 2] = r6 * x + r7 * y + r8 * z + tz;
  }
  return cloudFromPositions(out);
}

/** Pack the first `max` matches passing `filter` into a [s.x,y,z, t.x,y,z, ...]
 *  buffer suitable for `LinesLayer`. */
export function packMatchSegments(
  matches: KissMatcherData["matches"],
  filter: (m: KissMatcherData["matches"][number]) => boolean,
  max: number,
): Float32Array {
  const kept: KissMatcherData["matches"] = [];
  for (const m of matches) {
    if (kept.length >= max) break;
    if (filter(m)) kept.push(m);
  }
  const out = new Float32Array(kept.length * 6);
  for (let i = 0; i < kept.length; i++) {
    out[i * 6] = kept[i].src[0];
    out[i * 6 + 1] = kept[i].src[1];
    out[i * 6 + 2] = kept[i].src[2];
    out[i * 6 + 3] = kept[i].tgt[0];
    out[i * 6 + 4] = kept[i].tgt[1];
    out[i * 6 + 5] = kept[i].tgt[2];
  }
  return out;
}
