import { type ReactNode, useMemo, useState } from "react";
import { findChapter } from "../chapters";
import ChapterHeader from "../components/ChapterHeader";
import DemoAbout from "../components/DemoAbout";
import DemoParams from "../components/DemoParams";
import DataSourcePicker from "../components/DataSourcePicker";
import PointCloudViewer from "../components/PointCloudViewer";
import Slider from "../components/Slider";
import { useT } from "../i18n";
import { CameraSyncStore } from "../lib/cameraSync";
import { passThrough } from "../lib/filters/passThrough";
import { ransacPlane } from "../lib/filters/ransacPlane";
import { voxelGrid } from "../lib/filters/voxelGrid";
import { emptyCloud, type PointCloud } from "../lib/types";

const PATCHWORK = "https://github.com/LimHyungTae/patchwork";
const PATCHWORK_PLUS_PLUS = "https://github.com/url-kaist/patchwork-plusplus";

export default function Extra03GroundRemovalPipeline() {
  const chapter = findChapter("extra03")!;
  const t = useT();

  const [raw, setRaw] = useState<PointCloud>(emptyCloud());
  const [scale, setScale] = useState(1);
  const [zMin, setZMin] = useState(-3);
  const [zMax, setZMax] = useState(2);
  const [leaf, setLeaf] = useState(0.25);
  const [threshold, setThreshold] = useState(0.2);
  const [iters, setIters] = useState(120);

  const cropped = useMemo(
    () => passThrough(raw, "z", zMin, zMax, false),
    [raw, zMin, zMax],
  );
  const voxelized = useMemo(() => voxelGrid(cropped, leaf), [cropped, leaf]);
  const segmented = useMemo(
    () => ransacPlane(voxelized, threshold, iters),
    [voxelized, threshold, iters],
  );
  const sync = useMemo(() => new CameraSyncStore(), []);
  const ptSize = 0.06 * scale;

  return (
    <div className="mx-auto max-w-7xl px-8 py-10">
      <ChapterHeader chapter={chapter} />
      <DemoAbout slug="extra03" />
      <ResearchNote />

      <section className="mt-6 grid gap-4 lg:grid-cols-[1fr_18rem]">
        <div className="flex flex-col gap-3">
          <PipelineStats
            stages={[
              { label: t.extra03.rawStage, count: raw.count, color: "#94a3b8" },
              { label: t.extra03.passStage, count: cropped.count, color: "#4da6ff" },
              { label: t.extra03.voxelStage, count: voxelized.count, color: "#00d4aa" },
              { label: t.extra03.nonGroundStage, count: segmented.outliers.count, color: "#f97316" },
            ]}
          />
          <div className="grid grid-cols-1 gap-3 lg:grid-cols-2">
            <Pane
              label={t.extra03.inputPane}
              meta={`${raw.count.toLocaleString()} ${t.viewer.pointsSuffix}`}
              dot="#94a3b8"
            >
              <PointCloudViewer
                sync={sync}
                layers={[
                  { cloud: raw, color: "#94a3b8", size: ptSize, opacity: 0.7 },
                  { cloud: segmented.outliers, color: "#000", boundsOnly: true },
                ]}
              />
            </Pane>
            <Pane
              label={t.extra03.outputPane}
              meta={`${segmented.outliers.count.toLocaleString()} ${t.viewer.pointsSuffix}`}
              dot="#f97316"
            >
              <PointCloudViewer
                sync={sync}
                layers={[
                  { cloud: raw, color: "#000", boundsOnly: true },
                  {
                    cloud: segmented.outliers,
                    color: "#f97316",
                    size: ptSize * 1.15,
                    opacity: 0.9,
                  },
                  {
                    cloud: segmented.inliers,
                    color: "#00d4aa",
                    size: ptSize * 1.25,
                    opacity: 0.9,
                  },
                ]}
              />
            </Pane>
          </div>
        </div>

        <aside className="flex flex-col gap-5 rounded-xl border border-[var(--border)] bg-[var(--surface)] p-5">
          <DemoParams slug="extra03" />
          <DataSourcePicker
            defaultPreset="kitti"
            onCloudChange={(c, info) => {
              setRaw(c);
              setScale(info.suggestedScale);
              setZMin(-3 * info.suggestedScale);
              setZMax(2 * info.suggestedScale);
              setLeaf(0.25 * info.suggestedScale);
              setThreshold(0.2 * info.suggestedScale);
            }}
          />
          <Slider
            label={t.extra03.zMin}
            min={-10 * scale}
            max={5 * scale}
            step={0.05 * scale}
            value={zMin}
            unit="m"
            hint={t.extra03.zHint}
            onChange={(v) => setZMin(Math.min(v, zMax))}
          />
          <Slider
            label={t.extra03.zMax}
            min={-10 * scale}
            max={5 * scale}
            step={0.05 * scale}
            value={zMax}
            unit="m"
            onChange={(v) => setZMax(Math.max(v, zMin))}
          />
          <Slider
            label={t.extra03.leaf}
            min={0.02 * scale}
            max={1.5 * scale}
            step={0.01 * scale}
            value={leaf}
            unit="m"
            hint={t.extra03.leafHint}
            onChange={setLeaf}
          />
          <Slider
            label={t.extra03.threshold}
            min={0.02 * scale}
            max={1 * scale}
            step={0.01 * scale}
            value={threshold}
            unit="m"
            hint={t.extra03.thresholdHint}
            onChange={setThreshold}
          />
          <Slider
            label={t.extra03.iters}
            min={20}
            max={300}
            step={10}
            value={iters}
            hint={t.extra03.itersHint}
            onChange={(v) => setIters(Math.round(v))}
          />
          <Legend />
          <pre className="code-font overflow-x-auto rounded-md bg-[var(--surface-2)] p-3 text-[10px] leading-relaxed text-[var(--text)]">
            {`pcl::PassThrough<pcl::PointXYZ> pass;
pass.setInputCloud(src);
pass.setFilterFieldName("z");
pass.setFilterLimits(${zMin.toFixed(2)}, ${zMax.toFixed(2)});
pass.filter(*cropped);

pcl::VoxelGrid<pcl::PointXYZ> vg;
vg.setInputCloud(cropped);
vg.setLeafSize(${leaf.toFixed(2)}, ${leaf.toFixed(2)}, ${leaf.toFixed(2)});
vg.filter(*down);

pcl::SACSegmentation<pcl::PointXYZ> seg;
seg.setModelType(pcl::SACMODEL_PLANE);
seg.setDistanceThreshold(${threshold.toFixed(2)});
seg.segment(*ground, *coeffs);`}
          </pre>
        </aside>
      </section>
    </div>
  );
}

function ResearchNote() {
  const t = useT();
  return (
    <div className="mt-5 rounded-lg border border-[color:rgba(77,166,255,0.25)] bg-[color:rgba(77,166,255,0.06)] px-4 py-3 text-[13px] leading-relaxed text-[var(--dim)]">
      {t.extra03.roughTerrainPrefix}{" "}
      <ExternalLink href={PATCHWORK}>Patchwork</ExternalLink>
      {" / "}
      <ExternalLink href={PATCHWORK_PLUS_PLUS}>Patchwork++</ExternalLink>
      {t.extra03.roughTerrainSuffix}
    </div>
  );
}

function Legend() {
  const t = useT();
  return (
    <div className="grid grid-cols-2 gap-2 text-[11px] text-[var(--dim)]">
      <LegendItem color="#00d4aa" label={t.extra03.groundLegend} />
      <LegendItem color="#f97316" label={t.extra03.nonGroundLegend} />
    </div>
  );
}

function LegendItem({ color, label }: { color: string; label: string }) {
  return (
    <div className="flex items-center gap-2">
      <span className="inline-block h-2 w-2 rounded-full" style={{ background: color }} />
      <span>{label}</span>
    </div>
  );
}

function PipelineStats({
  stages,
}: {
  stages: { label: string; count: number; color: string }[];
}) {
  const t = useT();
  return (
    <div className="grid gap-2 sm:grid-cols-4">
      {stages.map((s) => (
        <div
          key={s.label}
          className="rounded-lg border border-[var(--border)] bg-[color:rgba(10,15,26,0.55)] px-3 py-2"
        >
          <div className="flex items-center gap-2">
            <span className="inline-block h-2 w-2 rounded-full" style={{ background: s.color }} />
            <span className="code-font text-[10px] uppercase tracking-wider text-[var(--mut)]">
              {s.label}
            </span>
          </div>
          <div className="code-font mt-1 text-sm font-bold text-[var(--text-strong)]">
            {s.count.toLocaleString()} {t.viewer.pointsSuffix}
          </div>
        </div>
      ))}
    </div>
  );
}

function Pane({
  label,
  meta,
  dot,
  children,
}: {
  label: string;
  meta: string;
  dot: string;
  children: ReactNode;
}) {
  return (
    <div className="overflow-hidden rounded-xl border border-[var(--border)]">
      <div className="flex items-center justify-between border-b border-[var(--border)] bg-[color:rgba(10,15,26,0.6)] px-4 py-2 text-[11px]">
        <div className="flex items-center gap-2">
          <span className="inline-block h-2 w-2 rounded-full" style={{ background: dot }} />
          <span className="font-semibold text-[var(--text)]">{label}</span>
        </div>
        <span className="code-font text-[var(--dim)]">{meta}</span>
      </div>
      <div className="aspect-[4/3] w-full">{children}</div>
    </div>
  );
}

function ExternalLink({
  href,
  children,
}: {
  href: string;
  children: ReactNode;
}) {
  return (
    <a
      href={href}
      target="_blank"
      rel="noreferrer"
      className="code-font font-bold text-[var(--accent-2)] underline decoration-[color:rgba(77,166,255,0.35)] underline-offset-2 transition hover:text-[var(--accent)]"
    >
      {children}
    </a>
  );
}
