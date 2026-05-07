import { useMemo, useState } from "react";
import { findChapter } from "../chapters";
import ChapterHeader from "../components/ChapterHeader";
import PointCloudViewer from "../components/PointCloudViewer";
import Slider from "../components/Slider";
import DataSourcePicker from "../components/DataSourcePicker";
import { useT } from "../i18n";
import { KdTree } from "../lib/kdtree";
import { voxelGrid } from "../lib/filters/voxelGrid";
import { cloudFromPositions, emptyCloud, type PointCloud } from "../lib/types";

export default function Lec09Knn() {
  const chapter = findChapter("lec09")!;
  const t = useT();

  const [raw, setRaw] = useState<PointCloud>(emptyCloud());
  const [scale, setScale] = useState(1);
  const [qx, setQx] = useState(0);
  const [qy, setQy] = useState(0);
  const [qz, setQz] = useState(0);
  const [k, setK] = useState(500);

  const src = useMemo(() => {
    if (raw.count <= 60_000) return raw;
    let leaf = 0.1 * scale;
    let v = voxelGrid(raw, leaf);
    while (v.count > 60_000 && leaf < 5 * scale) {
      leaf *= 1.4;
      v = voxelGrid(raw, leaf);
    }
    return v;
  }, [raw, scale]);

  const tree = useMemo(() => new KdTree(src.positions), [src]);

  const neighbors = useMemo(() => {
    if (src.count === 0) return cloudFromPositions(new Float32Array(0));
    const { indices } = tree.knnSearch(qx, qy, qz, k);
    const out = new Float32Array(indices.length * 3);
    for (let i = 0; i < indices.length; i++) {
      const off = indices[i] * 3;
      out[i * 3] = src.positions[off];
      out[i * 3 + 1] = src.positions[off + 1];
      out[i * 3 + 2] = src.positions[off + 2];
    }
    return cloudFromPositions(out);
  }, [tree, src, qx, qy, qz, k]);

  const queryCloud = useMemo(
    () => cloudFromPositions(new Float32Array([qx, qy, qz])),
    [qx, qy, qz],
  );
  const ptSize = 0.05 * scale;

  return (
    <div className="mx-auto max-w-7xl px-8 py-10">
      <ChapterHeader chapter={chapter} />

      <section className="mt-6 grid gap-4 lg:grid-cols-[1fr_18rem]">
        <div className="overflow-hidden rounded-xl border border-slate-800/80">
          <div className="flex items-center justify-between border-b border-slate-800/80 bg-slate-950/60 px-4 py-2 text-xs">
            <div className="flex items-center gap-3">
              <Dot color="#475569" /> base
              <Dot color="#34d399" /> {neighbors.count.toLocaleString()} nearest
              <Dot color="#facc15" /> query
            </div>
          </div>
          <div className="aspect-[16/10] w-full">
            <PointCloudViewer
              layers={[
                { cloud: src, color: "#475569", size: ptSize, opacity: 0.55 },
                { cloud: neighbors, color: "#34d399", size: ptSize * 1.3 },
                { cloud: queryCloud, color: "#facc15", size: ptSize * 6 },
              ]}
            />
          </div>
        </div>

        <aside className="flex flex-col gap-5 rounded-xl border border-slate-800/80 bg-slate-900/40 p-5">
          <DataSourcePicker
            onCloudChange={(c, info) => {
              setRaw(c);
              setScale(info.suggestedScale);
            }}
          />
          <Slider
            label={t.lec09.queryX}
            min={-30 * scale}
            max={30 * scale}
            step={0.5 * scale}
            value={qx}
            unit="m"
            onChange={setQx}
          />
          <Slider
            label={t.lec09.queryY}
            min={-30 * scale}
            max={30 * scale}
            step={0.5 * scale}
            value={qy}
            unit="m"
            onChange={setQy}
          />
          <Slider
            label={t.lec09.queryZ}
            min={-10 * scale}
            max={10 * scale}
            step={0.2 * scale}
            value={qz}
            unit="m"
            onChange={setQz}
          />
          <Slider
            label={t.lec09.k}
            min={1}
            max={2000}
            step={1}
            value={k}
            hint={t.lec09.kHint}
            onChange={(v) => setK(Math.round(v))}
          />
          <pre className="code-font overflow-x-auto rounded-md bg-slate-950/60 p-3 text-[11px] leading-relaxed text-slate-300">
            {`pcl::KdTreeFLANN<pcl::PointXYZ> kd;
kd.setInputCloud(src);
pcl::PointXYZ q(${qx.toFixed(2)}, ${qy.toFixed(2)}, ${qz.toFixed(2)});
kd.nearestKSearch(q, ${k},
                  idx, sqr_dists);`}
          </pre>
        </aside>
      </section>
    </div>
  );
}

function Dot({ color }: { color: string }) {
  return (
    <span
      className="inline-block h-2 w-2 rounded-full"
      style={{ background: color }}
    />
  );
}
