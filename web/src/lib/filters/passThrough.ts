import { cloudFromPositions, type PointCloud } from "../types";

export type Axis = "x" | "y" | "z";

/**
 * pcl::PassThrough — keep (or, with `negative=true`, drop) points whose
 * value along `axis` falls inside [min, max].
 */
export function passThrough(
  input: PointCloud,
  axis: Axis,
  min: number,
  max: number,
  negative = false,
): PointCloud {
  if (input.count === 0) return input;
  const stride = axis === "x" ? 0 : axis === "y" ? 1 : 2;

  const src = input.positions;
  // First pass: count.
  let kept = 0;
  for (let i = 0; i < src.length; i += 3) {
    const v = src[i + stride];
    const inside = v >= min && v <= max;
    if (inside !== negative) kept++;
  }
  const out = new Float32Array(kept * 3);
  let w = 0;
  for (let i = 0; i < src.length; i += 3) {
    const v = src[i + stride];
    const inside = v >= min && v <= max;
    if (inside !== negative) {
      out[w++] = src[i];
      out[w++] = src[i + 1];
      out[w++] = src[i + 2];
    }
  }
  return cloudFromPositions(out);
}
