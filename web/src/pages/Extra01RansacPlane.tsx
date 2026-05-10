import { useMemo, useState } from "react";
import { findChapter } from "../chapters";
import ChapterHeader from "../components/ChapterHeader";
import DemoAbout from "../components/DemoAbout";
import DemoParams from "../components/DemoParams";
import PointCloudViewer from "../components/PointCloudViewer";
import Slider from "../components/Slider";
import DataSourcePicker from "../components/DataSourcePicker";
import { useT } from "../i18n";
import { ransacPlane } from "../lib/filters/ransacPlane";
import { voxelGrid } from "../lib/filters/voxelGrid";
import { emptyCloud, type PointCloud } from "../lib/types";

export default function Extra01RansacPlane() {
  const chapter = findChapter("ransac-plane-segmentation")!;
  const t = useT();

  const [raw, setRaw] = useState<PointCloud>(emptyCloud());
  const [scale, setScale] = useState(1);
  const [threshold, setThreshold] = useState(0.15);
  const [iters, setIters] = useState(150);

  // Voxelize first to keep RANSAC fast (each iteration is O(N)).
  const src = useMemo(() => {
    if (raw.count <= 30_000) return raw;
    let leaf = 0.15 * scale;
    let v = voxelGrid(raw, leaf);
    while (v.count > 30_000 && leaf < 5 * scale) {
      leaf *= 1.4;
      v = voxelGrid(raw, leaf);
    }
    return v;
  }, [raw, scale]);

  const result = useMemo(
    () => ransacPlane(src, threshold, iters),
    [src, threshold, iters],
  );
  const ptSize = 0.05 * scale;

  return (
    <div className="mx-auto max-w-7xl px-4 py-6 sm:px-6 lg:px-8 lg:py-10">
      <ChapterHeader chapter={chapter} />
      <DemoAbout slug="ransac-plane-segmentation" />

      <section className="mt-6 grid gap-4 lg:grid-cols-[1fr_18rem]">
        <div className="overflow-hidden rounded-xl border border-[var(--border)]">
          <div className="flex items-center justify-between border-b border-[var(--border)] bg-[color:rgba(10,15,26,0.6)] px-4 py-2 text-xs">
            <div className="flex items-center gap-3">
              <Dot color="#34d399" /> {t.extra01.inliers}: {result.inliers.count.toLocaleString()}
              <Dot color="#94a3b8" /> {t.extra01.outliers}: {result.outliers.count.toLocaleString()}
              <span className="code-font text-[var(--mut)]">
                ({(result.inlierRatio * 100).toFixed(1)}%)
              </span>
            </div>
            <span className="code-font text-[var(--mut)]">
              {result.plane.a.toFixed(2)} x + {result.plane.b.toFixed(2)} y + {result.plane.c.toFixed(2)} z + {result.plane.d.toFixed(2)} = 0
            </span>
          </div>
          <div className="aspect-[4/3] md:aspect-[16/10] w-full">
            <PointCloudViewer
              layers={[
                { cloud: result.outliers, color: "#94a3b8", size: ptSize, opacity: 0.55 },
                { cloud: result.inliers, color: "#34d399", size: ptSize * 1.2 },
              ]}
            />
          </div>
        </div>

        <aside className="flex flex-col gap-5 rounded-xl border border-[var(--border)] bg-[var(--surface)] p-5">
          <DemoParams slug="ransac-plane-segmentation" />
          <DataSourcePicker
            onCloudChange={(c, info) => {
              setRaw(c);
              setScale(info.suggestedScale);
              setThreshold(0.15 * info.suggestedScale);
            }}
          />
          <Slider
            label={t.extra01.threshold}
            min={0.005 * scale}
            max={1.5 * scale}
            step={0.005 * scale}
            value={threshold}
            unit="m"
            hint={t.extra01.thresholdHint}
            onChange={setThreshold}
          />
          <Slider
            label={t.extra01.iters}
            min={20}
            max={500}
            step={10}
            value={iters}
            hint={t.extra01.itersHint}
            onChange={(v) => setIters(Math.round(v))}
          />
          <div className="rounded-md border border-[color:rgba(0,212,170,0.25)] bg-[color:rgba(0,212,170,0.06)] px-3 py-2 text-xs text-[var(--accent)]">
            {t.extra01.note}
          </div>
          <pre className="code-font overflow-x-auto rounded-md bg-[var(--surface-2)] p-3 text-[10px] leading-relaxed text-[var(--text)]">
            {`pcl::SACSegmentation<pcl::PointXYZ> seg;
seg.setModelType(pcl::SACMODEL_PLANE);
seg.setMethodType(pcl::SAC_RANSAC);
seg.setDistanceThreshold(${threshold.toFixed(3)});
seg.setMaxIterations(${iters});
seg.setInputCloud(src);
seg.segment(*inliers, *coeffs);`}
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
