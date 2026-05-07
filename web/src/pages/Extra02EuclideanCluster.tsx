import { useEffect, useMemo, useRef, useState } from "react";
import { findChapter } from "../chapters";
import ChapterHeader from "../components/ChapterHeader";
import PointCloudViewer from "../components/PointCloudViewer";
import Slider from "../components/Slider";
import DataSourcePicker from "../components/DataSourcePicker";
import { useT } from "../i18n";
import {
  ClusterColorMatcher,
  euclideanCluster,
} from "../lib/filters/euclideanCluster";
import { ransacPlane } from "../lib/filters/ransacPlane";
import { voxelGrid } from "../lib/filters/voxelGrid";
import { emptyCloud, type PointCloud } from "../lib/types";

export default function Extra02EuclideanCluster() {
  const chapter = findChapter("extra02")!;
  const t = useT();

  const [raw, setRaw] = useState<PointCloud>(emptyCloud());
  const [scale, setScale] = useState(1);
  const [tolerance, setTolerance] = useState(0.5);
  const [minSize, setMinSize] = useState(50);
  const [removeGround, setRemoveGround] = useState(true);

  // Voxelize hard — clustering is N×radius_search, very expensive.
  const src = useMemo(() => {
    if (raw.count === 0) return raw;
    let leaf = 0.2 * scale;
    let v = voxelGrid(raw, leaf);
    while (v.count > 15_000 && leaf < 5 * scale) {
      leaf *= 1.4;
      v = voxelGrid(raw, leaf);
    }
    return v;
  }, [raw, scale]);

  // Optionally remove the dominant plane first (typical SLAM preprocessing).
  const cleaned = useMemo(() => {
    if (!removeGround || src.count === 0) return src;
    const seg = ransacPlane(src, 0.2 * scale, 80);
    return seg.outliers;
  }, [src, removeGround, scale]);

  const clusters = useMemo(
    () => euclideanCluster(cleaned, tolerance, minSize, 1_000_000),
    [cleaned, tolerance, minSize],
  );

  // Persistent color matcher — colors follow the same cluster across
  // slider tweaks (keyed by centroid proximity).
  const matcherRef = useRef(new ClusterColorMatcher());
  const lastCloudRef = useRef<PointCloud | null>(null);

  // Reset color memory when the underlying cloud changes (preset switch /
  // ground-removal toggle); otherwise stale anchors would stick around.
  useEffect(() => {
    if (lastCloudRef.current !== cleaned) {
      matcherRef.current.reset();
      lastCloudRef.current = cleaned;
    }
  }, [cleaned]);

  const colors = useMemo(
    () => matcherRef.current.assign(clusters, Math.max(tolerance * 3, 0.5 * scale)),
    [clusters, tolerance, scale],
  );

  const ptSize = 0.05 * scale;

  const layers = useMemo(() => {
    const out = clusters.slice(0, 80).map((c, idx) => ({
      cloud: c.cloud,
      color: colors[idx] ?? "#94a3b8",
      size: ptSize * 1.25,
    }));
    return [
      // Phantom layer to keep the viewer's bbox aware of the unclustered
      // points so the camera frames the whole cleaned cloud.
      { cloud: cleaned, color: "#1e293b", size: ptSize, opacity: 0 },
      ...out,
    ];
  }, [clusters, colors, cleaned, ptSize]);

  return (
    <div className="mx-auto max-w-7xl px-8 py-10">
      <ChapterHeader chapter={chapter} />

      <section className="mt-6 grid gap-4 lg:grid-cols-[1fr_18rem]">
        <div className="overflow-hidden rounded-xl border border-[var(--border)]">
          <div className="flex items-center justify-between border-b border-[var(--border)] bg-[color:rgba(10,15,26,0.6)] px-4 py-2 text-[11px]">
            <div className="text-[var(--text)]">
              {t.extra02.found.replace("{n}", clusters.length.toLocaleString())}
            </div>
            <span className="code-font text-[var(--dim)]">
              {cleaned.count.toLocaleString()} {t.viewer.pointsSuffix}
            </span>
          </div>
          <div className="aspect-[16/10] w-full">
            <PointCloudViewer layers={layers} />
          </div>
        </div>

        <aside className="flex flex-col gap-5 rounded-xl border border-[var(--border)] bg-[var(--surface)] p-5">
          <DataSourcePicker
            onCloudChange={(c, info) => {
              setRaw(c);
              setScale(info.suggestedScale);
              setTolerance(0.5 * info.suggestedScale);
            }}
          />
          <label className="flex items-start gap-2 text-sm text-[var(--text)]">
            <input
              type="checkbox"
              checked={removeGround}
              onChange={(e) => setRemoveGround(e.target.checked)}
              className="mt-0.5 h-4 w-4 accent-[var(--accent)]"
            />
            <span>{t.extra02.removeGround}</span>
          </label>
          <Slider
            label={t.extra02.tolerance}
            min={0.05 * scale}
            max={3 * scale}
            step={0.05 * scale}
            value={tolerance}
            unit="m"
            hint={t.extra02.toleranceHint}
            onChange={setTolerance}
          />
          <Slider
            label={t.extra02.minSize}
            min={5}
            max={500}
            step={5}
            value={minSize}
            hint={t.extra02.minSizeHint}
            onChange={(v) => setMinSize(Math.round(v))}
          />
          <pre className="code-font overflow-x-auto rounded-md bg-[var(--surface-2)] p-3 text-[10px] leading-relaxed text-[var(--text)]">
            {`pcl::EuclideanClusterExtraction<
    pcl::PointXYZ> ec;
ec.setClusterTolerance(${tolerance.toFixed(2)});
ec.setMinClusterSize(${minSize});
ec.setSearchMethod(tree);
ec.setInputCloud(src);
ec.extract(indices);`}
          </pre>
        </aside>
      </section>
    </div>
  );
}
