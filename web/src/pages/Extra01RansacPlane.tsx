import { useMemo, useState } from "react";
import { findChapter } from "../chapters";
import ChapterHeader from "../components/ChapterHeader";
import DemoAbout from "../components/DemoAbout";
import DemoParams from "../components/DemoParams";
import PointCloudViewer from "../components/PointCloudViewer";
import Slider from "../components/Slider";
import DataSourcePicker from "../components/DataSourcePicker";
import { useT } from "../i18n";
import { passThrough } from "../lib/filters/passThrough";
import { ransacPlane } from "../lib/filters/ransacPlane";
import {
  cylinderWireframe,
  ransacVerticalCylinder,
} from "../lib/filters/ransacCylinder";
import { voxelGrid } from "../lib/filters/voxelGrid";
import { emptyCloud, type PointCloud } from "../lib/types";

type Model = "plane" | "cylinder";

export default function Extra01RansacPlane() {
  const chapter = findChapter("extra01")!;
  const t = useT();

  const [model, setModel] = useState<Model>("plane");
  const [raw, setRaw] = useState<PointCloud>(emptyCloud());
  const [scale, setScale] = useState(1);
  const [threshold, setThreshold] = useState(0.15);
  const [iters, setIters] = useState(150);

  // Cylinder-specific params: crop above ground so RANSAC isn't dominated by
  // ground-plane points, and constrain radius to pole-sized features.
  const [zMin, setZMin] = useState(0.4);
  const [rMin, setRMin] = useState(0.05);
  const [rMax, setRMax] = useState(0.6);

  // Voxelize first to keep RANSAC fast (each iteration is O(N)).
  const voxelized = useMemo(() => {
    if (raw.count <= 30_000) return raw;
    let leaf = 0.15 * scale;
    let v = voxelGrid(raw, leaf);
    while (v.count > 30_000 && leaf < 5 * scale) {
      leaf *= 1.4;
      v = voxelGrid(raw, leaf);
    }
    return v;
  }, [raw, scale]);

  // For cylinder mode, also pre-crop above ground so the dominant plane
  // doesn't drag the fit downward.
  const src = useMemo(() => {
    if (model !== "cylinder" || voxelized.count === 0) return voxelized;
    return passThrough(voxelized, "z", zMin, Infinity, false);
  }, [voxelized, model, zMin]);

  const planeResult = useMemo(
    () => (model === "plane" ? ransacPlane(src, threshold, iters) : null),
    [model, src, threshold, iters],
  );
  const cylResult = useMemo(
    () =>
      model === "cylinder"
        ? ransacVerticalCylinder(src, threshold, iters, rMin, rMax)
        : null,
    [model, src, threshold, iters, rMin, rMax],
  );

  const inliers =
    planeResult?.inliers ?? cylResult?.inliers ?? emptyCloud();
  const outliers =
    planeResult?.outliers ?? cylResult?.outliers ?? emptyCloud();
  const inlierRatio =
    planeResult?.inlierRatio ?? cylResult?.inlierRatio ?? 0;

  const wireframeLines = useMemo(() => {
    if (model !== "cylinder" || !cylResult || cylResult.inliers.count === 0) {
      return undefined;
    }
    const c = cylResult.cylinder;
    return [
      {
        positions: cylinderWireframe(c.cx, c.cy, c.r, c.zMin, c.zMax, 8, 32),
        color: "#fbbf24",
        opacity: 0.9,
      },
    ];
  }, [model, cylResult]);

  const ptSize = 0.05 * scale;

  return (
    <div className="mx-auto max-w-7xl px-8 py-10">
      <ChapterHeader chapter={chapter} />
      <DemoAbout slug="extra01" />

      <section className="mt-6 grid gap-4 lg:grid-cols-[1fr_18rem]">
        <div className="overflow-hidden rounded-xl border border-[var(--border)]">
          <div className="flex items-center justify-between border-b border-[var(--border)] bg-[color:rgba(10,15,26,0.6)] px-4 py-2 text-xs">
            <div className="flex items-center gap-3">
              <Dot color="#34d399" /> {t.extra01.inliers}:{" "}
              {inliers.count.toLocaleString()}
              <Dot color="#94a3b8" /> {t.extra01.outliers}:{" "}
              {outliers.count.toLocaleString()}
              <span className="code-font text-[var(--mut)]">
                ({(inlierRatio * 100).toFixed(1)}%)
              </span>
            </div>
            <span className="code-font text-[var(--mut)]">
              {planeResult &&
                `${planeResult.plane.a.toFixed(2)} x + ${planeResult.plane.b.toFixed(2)} y + ${planeResult.plane.c.toFixed(2)} z + ${planeResult.plane.d.toFixed(2)} = 0`}
              {cylResult &&
                cylResult.inliers.count > 0 &&
                `axis (x, y) = (${cylResult.cylinder.cx.toFixed(2)}, ${cylResult.cylinder.cy.toFixed(2)})  ·  r = ${cylResult.cylinder.r.toFixed(3)} m`}
            </span>
          </div>
          <div className="aspect-[16/10] w-full">
            <PointCloudViewer
              layers={[
                {
                  cloud: outliers,
                  color: "#94a3b8",
                  size: ptSize,
                  opacity: 0.55,
                },
                { cloud: inliers, color: "#34d399", size: ptSize * 1.2 },
              ]}
              lines={wireframeLines}
            />
          </div>
        </div>

        <aside className="flex flex-col gap-5 rounded-xl border border-[var(--border)] bg-[var(--surface)] p-5">
          <DemoParams slug="extra01" />

          <div>
            <div className="mb-1.5 text-sm text-[var(--text)]">
              {t.extra01.modelLabel}
            </div>
            <div className="grid grid-cols-2 overflow-hidden rounded-md border border-[var(--border-strong)]">
              {(["plane", "cylinder"] as Model[]).map((m) => (
                <button
                  key={m}
                  onClick={() => setModel(m)}
                  className={`code-font py-1.5 text-xs transition ${
                    model === m
                      ? "bg-[color:rgba(0,212,170,0.15)] text-[var(--accent)]"
                      : "text-[var(--dim)] hover:bg-[color:rgba(255,255,255,0.04)] hover:text-[var(--text)]"
                  }`}
                >
                  {t.extra01.modelOptions[m]}
                </button>
              ))}
            </div>
          </div>

          <DataSourcePicker
            onCloudChange={(c, info) => {
              setRaw(c);
              setScale(info.suggestedScale);
              setThreshold(0.15 * info.suggestedScale);
              setZMin(0.4 * info.suggestedScale);
              setRMin(0.05 * info.suggestedScale);
              setRMax(0.6 * info.suggestedScale);
            }}
          />

          {model === "cylinder" && (
            <>
              <Slider
                label={t.extra01.zMin}
                min={-2 * scale}
                max={3 * scale}
                step={0.05 * scale}
                value={zMin}
                unit="m"
                hint={t.extra01.zMinHint}
                onChange={setZMin}
              />
              <Slider
                label={t.extra01.rMin}
                min={0.01 * scale}
                max={0.5 * scale}
                step={0.01 * scale}
                value={rMin}
                unit="m"
                onChange={(v) => setRMin(Math.min(v, rMax))}
              />
              <Slider
                label={t.extra01.rMax}
                min={0.05 * scale}
                max={2 * scale}
                step={0.01 * scale}
                value={rMax}
                unit="m"
                hint={t.extra01.rRangeHint}
                onChange={(v) => setRMax(Math.max(v, rMin))}
              />
            </>
          )}

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
            {model === "plane" ? t.extra01.note : t.extra01.cylinderNote}
          </div>

          <pre className="code-font overflow-x-auto rounded-md bg-[var(--surface-2)] p-3 text-[10px] leading-relaxed text-[var(--text)]">
            {model === "plane"
              ? `pcl::SACSegmentation<pcl::PointXYZ> seg;
seg.setModelType(pcl::SACMODEL_PLANE);
seg.setMethodType(pcl::SAC_RANSAC);
seg.setDistanceThreshold(${threshold.toFixed(3)});
seg.setMaxIterations(${iters});
seg.setInputCloud(src);
seg.segment(*inliers, *coeffs);`
              : `pcl::SACSegmentation<pcl::PointXYZ> seg;
seg.setModelType(pcl::SACMODEL_CYLINDER);
seg.setMethodType(pcl::SAC_RANSAC);
seg.setAxis(Eigen::Vector3f::UnitZ());
seg.setEpsAngle(0.1);
seg.setRadiusLimits(${rMin.toFixed(2)}, ${rMax.toFixed(2)});
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
