import { useMemo, useState } from "react";
import { findChapter } from "../chapters";
import ChapterHeader from "../components/ChapterHeader";
import DemoAbout from "../components/DemoAbout";
import DemoParams from "../components/DemoParams";
import PointCloudViewer from "../components/PointCloudViewer";
import Slider from "../components/Slider";
import DataSourcePicker from "../components/DataSourcePicker";
import { useT } from "../i18n";
import { KdTree } from "../lib/kdtree";
import { voxelGrid } from "../lib/filters/voxelGrid";
import { cloudFromPositions, emptyCloud, type PointCloud } from "../lib/types";

export default function Lec09Knn() {
  const chapter = findChapter("k-nearest-neighbor")!;
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
  const ptSize = 0.25 * scale;

  return (
    <div className="mx-auto max-w-7xl px-4 py-6 sm:px-6 lg:px-8 lg:py-10">
      <ChapterHeader chapter={chapter} />
      <DemoAbout slug="k-nearest-neighbor" />

      <section className="mt-6 grid gap-4 lg:grid-cols-[1fr_18rem]">
        <div className="overflow-hidden rounded-xl border border-[var(--border)]">
          <div className="flex items-center justify-between border-b border-[var(--border)] bg-[color:rgba(10,15,26,0.6)] px-4 py-2 text-[11px]">
            <div className="flex items-center gap-3">
              <Dot color="#475569" /> base
              <Dot color="#34d399" /> {neighbors.count.toLocaleString()} nearest
              <Dot color="#facc15" /> query
            </div>
          </div>
          <div className="aspect-[4/3] md:aspect-[16/10] w-full">
            <PointCloudViewer
              layers={[
                { cloud: src, color: "#475569", size: ptSize, opacity: 0.55 },
                { cloud: neighbors, color: "#34d399", size: ptSize * 1.3 },
                { cloud: queryCloud, color: "#facc15", size: ptSize * 6 },
              ]}
              onPick={([x, y, z]) => {
                setQx(round(x, 2));
                setQy(round(y, 2));
                setQz(round(z, 2));
              }}
            />
          </div>
        </div>

        <aside className="flex flex-col gap-5 rounded-xl border border-[var(--border)] bg-[var(--surface)] p-5">
          <DemoParams slug="k-nearest-neighbor" />
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
            onChange={(v) => setK(Math.round(v))}
          />
          <pre className="code-font overflow-x-auto rounded-md bg-[var(--surface-2)] p-3 text-[10px] leading-relaxed text-[var(--text)]">
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

function round(v: number, digits: number) {
  const f = Math.pow(10, digits);
  return Math.round(v * f) / f;
}

function Dot({ color }: { color: string }) {
  return (
    <span
      className="inline-block h-2 w-2 rounded-full"
      style={{ background: color }}
    />
  );
}
