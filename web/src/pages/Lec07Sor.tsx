import { useMemo, useState } from "react";
import { findChapter } from "../chapters";
import ChapterHeader from "../components/ChapterHeader";
import DemoAbout from "../components/DemoAbout";
import DemoParams from "../components/DemoParams";
import PointCloudViewer from "../components/PointCloudViewer";
import Slider from "../components/Slider";
import DataSourcePicker from "../components/DataSourcePicker";
import { useT } from "../i18n";
import { sor } from "../lib/filters/sor";
import { voxelGrid } from "../lib/filters/voxelGrid";
import { emptyCloud, type PointCloud } from "../lib/types";

/**
 * SOR is O(N · k log N) — for KITTI's 120k points it's slow. We voxelize
 * first to keep things responsive but still informative.
 */
export default function Lec07Sor() {
  const chapter = findChapter("lec07")!;
  const t = useT();

  const [raw, setRaw] = useState<PointCloud>(emptyCloud());
  const [scale, setScale] = useState(1);
  const [meanK, setMeanK] = useState(20);
  const [stddev, setStddev] = useState(1);

  // Cap point count to keep SOR responsive (re-voxelize if too dense).
  const src = useMemo(() => {
    if (raw.count <= 25_000) return raw;
    let leaf = 0.1 * scale;
    let v = voxelGrid(raw, leaf);
    while (v.count > 25_000 && leaf < 5 * scale) {
      leaf *= 1.4;
      v = voxelGrid(raw, leaf);
    }
    return v;
  }, [raw, scale]);

  const result = useMemo(() => sor(src, meanK, stddev), [src, meanK, stddev]);
  const ptSize = 0.25 * scale;

  return (
    <div className="mx-auto max-w-7xl px-4 py-6 sm:px-6 lg:px-8 lg:py-10">
      <ChapterHeader chapter={chapter} />
      <DemoAbout slug="lec07" />

      <section className="mt-6 grid gap-4 lg:grid-cols-[1fr_18rem]">
        <div className="overflow-hidden rounded-xl border border-slate-800/80">
          <div className="flex items-center justify-between border-b border-slate-800/80 bg-slate-950/60 px-4 py-2 text-xs">
            <div className="flex items-center gap-3">
              <Dot color="#34d399" /> {t.lec07.inliers}: {result.inliers.count.toLocaleString()}
              <Dot color="#f87171" /> {t.lec07.outliers}: {result.outliers.count.toLocaleString()}
            </div>
            <span className="code-font text-slate-500">
              {src.count.toLocaleString()} {t.viewer.pointsSuffix}
            </span>
          </div>
          <div className="aspect-[4/3] md:aspect-[16/10] w-full">
            <PointCloudViewer
              layers={[
                { cloud: result.inliers, color: "#34d399", size: ptSize },
                { cloud: result.outliers, color: "#f87171", size: ptSize * 1.4 },
              ]}
            />
          </div>
        </div>

        <aside className="flex flex-col gap-5 rounded-xl border border-[var(--border)] bg-[var(--surface)] p-5">
          <DemoParams slug="lec07" />
          <DataSourcePicker
            defaultPreset="naverlabs"
            onCloudChange={(c, info) => {
              setRaw(c);
              setScale(info.suggestedScale);
            }}
          />
          <Slider
            label={t.lec07.meanK}
            min={3}
            max={60}
            step={1}
            value={meanK}
            hint={t.lec07.meanKHint}
            onChange={(v) => setMeanK(Math.round(v))}
          />
          <Slider
            label={t.lec07.stddev}
            min={0.1}
            max={3}
            step={0.05}
            value={stddev}
            hint={t.lec07.stddevHint}
            onChange={setStddev}
          />
          <pre className="code-font overflow-x-auto rounded-md bg-slate-950/60 p-3 text-[11px] leading-relaxed text-slate-300">
            {`pcl::StatisticalOutlierRemoval<
    pcl::PointXYZ> sor;
sor.setInputCloud(src);
sor.setMeanK(${meanK});
sor.setStddevMulThresh(${stddev.toFixed(2)});
sor.filter(*out);`}
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
