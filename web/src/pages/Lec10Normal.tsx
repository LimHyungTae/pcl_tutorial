import { useMemo, useState } from "react";
import { findChapter } from "../chapters";
import ChapterHeader from "../components/ChapterHeader";
import DemoAbout from "../components/DemoAbout";
import DemoParams from "../components/DemoParams";
import PointCloudViewer from "../components/PointCloudViewer";
import Slider from "../components/Slider";
import DataSourcePicker from "../components/DataSourcePicker";
import { useT } from "../i18n";
import { estimateNormals } from "../lib/filters/normal";
import { voxelGrid } from "../lib/filters/voxelGrid";
import { emptyCloud, type PointCloud } from "../lib/types";

export default function Lec10Normal() {
  const chapter = findChapter("lec10")!;
  const t = useT();

  const [raw, setRaw] = useState<PointCloud>(emptyCloud());
  const [scale, setScale] = useState(1);
  const [voxel, setVoxel] = useState(0.4);
  const [k, setK] = useState(20);
  const [coloring, setColoring] = useState<"normal" | "flat">("normal");

  const src = useMemo(() => {
    if (raw.count === 0) return raw;
    return voxelGrid(raw, voxel);
  }, [raw, voxel]);

  const normals = useMemo(() => estimateNormals(src, k), [src, k]);

  const colors = useMemo(() => {
    const out = new Float32Array(src.count * 3);
    for (let i = 0; i < src.count; i++) {
      out[i * 3] = Math.abs(normals[i * 3]);
      out[i * 3 + 1] = Math.abs(normals[i * 3 + 1]);
      out[i * 3 + 2] = Math.abs(normals[i * 3 + 2]);
    }
    return out;
  }, [normals, src.count]);

  // Build line segments for the normal vectors — each starts at a point and
  // extends along its normal by a small fraction of the voxel scale so the
  // arrows stay visible without overwhelming the cloud.
  const normalLines = useMemo(() => {
    if (src.count === 0) return new Float32Array(0);
    const len = voxel * 2;
    const out = new Float32Array(src.count * 6);
    for (let i = 0; i < src.count; i++) {
      const x = src.positions[i * 3];
      const y = src.positions[i * 3 + 1];
      const z = src.positions[i * 3 + 2];
      const nx = normals[i * 3];
      const ny = normals[i * 3 + 1];
      const nz = normals[i * 3 + 2];
      out[i * 6] = x;
      out[i * 6 + 1] = y;
      out[i * 6 + 2] = z;
      out[i * 6 + 3] = x + nx * len;
      out[i * 6 + 4] = y + ny * len;
      out[i * 6 + 5] = z + nz * len;
    }
    return out;
  }, [src, normals, voxel]);

  const ptSize = 0.25 * scale;

  return (
    <div className="mx-auto max-w-7xl px-8 py-10">
      <ChapterHeader chapter={chapter} />
      <DemoAbout slug="lec10" />

      <section className="mt-6 grid gap-4 lg:grid-cols-[1fr_18rem]">
        <div className="overflow-hidden rounded-xl border border-slate-800/80">
          <div className="flex items-center justify-between border-b border-slate-800/80 bg-slate-950/60 px-4 py-2 text-xs">
            <div className="text-slate-400">{t.lec10.note}</div>
            <span className="code-font text-slate-500">
              {src.count.toLocaleString()} {t.viewer.pointsSuffix}
            </span>
          </div>
          <div className="aspect-[16/10] w-full">
            <PointCloudViewer
              layers={[
                {
                  cloud: src,
                  color: coloring === "normal" ? colors : "#94a3b8",
                  size: ptSize,
                },
              ]}
              lines={
                normalLines.length > 0
                  ? [{ positions: normalLines, color: "#e2e8f0", opacity: 0.55 }]
                  : undefined
              }
            />
          </div>
        </div>

        <aside className="flex flex-col gap-5 rounded-xl border border-[var(--border)] bg-[var(--surface)] p-5">
          <DemoParams slug="lec10" />
          <DataSourcePicker
            defaultPreset="bunny"
            onCloudChange={(c, info) => {
              setRaw(c);
              setScale(info.suggestedScale);
              // Reasonable per-preset voxel default.
              setVoxel(info.name === "bunny" ? 0.003 : 0.4 * info.suggestedScale);
            }}
          />
          <Slider
            label={t.lec10.voxelPre}
            min={0.001 * scale}
            max={2 * scale}
            step={0.005 * scale}
            value={voxel}
            unit="m"
            hint={t.lec10.voxelPreHint}
            onChange={setVoxel}
          />
          <Slider
            label={t.lec10.k}
            min={4}
            max={60}
            step={1}
            value={k}
            hint={t.lec10.kHint}
            onChange={(v) => setK(Math.round(v))}
          />
          <div>
            <div className="mb-1.5 text-sm text-slate-200">{t.lec10.coloring}</div>
            <div className="grid grid-cols-2 overflow-hidden rounded-md border border-slate-700">
              <button
                onClick={() => setColoring("normal")}
                className={`py-1.5 text-xs transition ${
                  coloring === "normal"
                    ? "bg-emerald-500/15 text-emerald-200"
                    : "text-slate-400 hover:bg-slate-800/60 hover:text-slate-200"
                }`}
              >
                {t.lec10.coloringNormal}
              </button>
              <button
                onClick={() => setColoring("flat")}
                className={`py-1.5 text-xs transition ${
                  coloring === "flat"
                    ? "bg-emerald-500/15 text-emerald-200"
                    : "text-slate-400 hover:bg-slate-800/60 hover:text-slate-200"
                }`}
              >
                {t.lec10.coloringFlat}
              </button>
            </div>
          </div>
          <pre className="code-font overflow-x-auto rounded-md bg-slate-950/60 p-3 text-[11px] leading-relaxed text-slate-300">
            {`pcl::NormalEstimation<
    pcl::PointXYZ, pcl::Normal> ne;
ne.setInputCloud(cloud);
ne.setSearchMethod(tree);
ne.setKSearch(${k});
ne.compute(*normals);`}
          </pre>
        </aside>
      </section>
    </div>
  );
}
