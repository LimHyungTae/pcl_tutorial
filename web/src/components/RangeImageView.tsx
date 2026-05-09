import { useEffect, useRef } from "react";
import * as THREE from "three";

type Props = {
  rows: number;
  cols: number;
  /** Per-pixel cluster ID, 0 = empty/ground. */
  clusterIds: Int32Array;
  /** Per-pixel depth in meters (0 = empty). */
  depths: Float32Array;
  /** Map cluster ID (≥1) → CSS color string. Falls back to grayscale. */
  colorForCluster: (id: number) => string;
  /** Optional minimum cluster size for the legend. */
  numClusters: number;
};

/** Renders a TRAVEL-style range image to a canvas. Cluster pixels get the same
 *  color as their 3D counterparts; empty / ground pixels render as a faint
 *  depth-shaded gray so the overall sweep shape is still readable. */
export default function RangeImageView({
  rows,
  cols,
  clusterIds,
  depths,
  colorForCluster,
  numClusters,
}: Props) {
  const canvasRef = useRef<HTMLCanvasElement>(null);

  useEffect(() => {
    const canvas = canvasRef.current;
    if (!canvas) return;
    const ctx = canvas.getContext("2d");
    if (!ctx) return;

    canvas.width = cols;
    canvas.height = rows;
    const imgData = ctx.createImageData(cols, rows);
    const data = imgData.data;

    // Cache parsed cluster colors so we don't allocate one THREE.Color per pixel.
    const colorCache = new Map<number, [number, number, number]>();
    const tmp = new THREE.Color();

    let maxDepth = 0;
    for (let i = 0; i < depths.length; i++) {
      if (depths[i] > maxDepth) maxDepth = depths[i];
    }
    if (maxDepth === 0) maxDepth = 1;

    for (let p = 0; p < rows * cols; p++) {
      const off = p * 4;
      const id = clusterIds[p];
      const depth = depths[p];

      if (id > 0) {
        let rgb = colorCache.get(id);
        if (!rgb) {
          tmp.setStyle(colorForCluster(id));
          rgb = [tmp.r * 255, tmp.g * 255, tmp.b * 255];
          colorCache.set(id, rgb);
        }
        data[off] = rgb[0];
        data[off + 1] = rgb[1];
        data[off + 2] = rgb[2];
        data[off + 3] = 255;
      } else if (depth > 0) {
        // Empty / ground: faint depth shading.
        const t = 1 - Math.min(1, depth / maxDepth);
        const v = 30 + 50 * t;
        data[off] = v;
        data[off + 1] = v;
        data[off + 2] = v;
        data[off + 3] = 255;
      } else {
        data[off] = 14;
        data[off + 1] = 18;
        data[off + 2] = 28;
        data[off + 3] = 255;
      }
    }

    ctx.putImageData(imgData, 0, 0);
  }, [rows, cols, clusterIds, depths, colorForCluster]);

  return (
    <div className="flex flex-col gap-2">
      <div className="flex items-center justify-between text-[11px]">
        <span className="text-[var(--text)]">Range image (rows × cols)</span>
        <span className="code-font text-[var(--dim)]">
          {rows} × {cols} · {numClusters} clusters
        </span>
      </div>
      <div className="overflow-hidden rounded border border-[var(--border)] bg-[#0a0f1a]">
        <canvas
          ref={canvasRef}
          className="block h-auto w-full"
          style={{ imageRendering: "pixelated" }}
        />
      </div>
    </div>
  );
}
