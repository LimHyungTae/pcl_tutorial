import { useEffect, useMemo, useRef, useState } from "react";
import { findChapter } from "../chapters";
import ChapterHeader from "../components/ChapterHeader";
import DemoAbout from "../components/DemoAbout";
import DemoParams from "../components/DemoParams";
import PointCloudViewer from "../components/PointCloudViewer";
import Slider from "../components/Slider";
import DataSourcePicker from "../components/DataSourcePicker";
import { useT } from "../i18n";
import { KdTree } from "../lib/kdtree";
import {
  buildTransform,
  transformCloud,
} from "../lib/filters/transform";
import {
  icpStep,
  identityTransform,
  samplePairCoords,
} from "../lib/filters/icp";
import { voxelGrid } from "../lib/filters/voxelGrid";
import { emptyCloud, type PointCloud } from "../lib/types";

export default function Lec11Icp() {
  const chapter = findChapter("iterative-closest-point")!;
  const t = useT();

  const [raw, setRaw] = useState<PointCloud>(emptyCloud());
  const [scale, setScale] = useState(1);
  const [maxDist, setMaxDist] = useState(1);
  const [transform, setTransform] = useState<Float32Array>(identityTransform());
  const [iter, setIter] = useState(0);
  const [fitness, setFitness] = useState<number | null>(null);
  const [pairs, setPairs] = useState<number | null>(null);
  const [pairCoords, setPairCoords] = useState<Float32Array>(new Float32Array(0));
  const [playing, setPlaying] = useState(false);
  const [converged, setConverged] = useState(false);
  // Bumped on preset switch and Reset — that's when we want the camera to
  // refit. Per-iteration bbox shifts during Play do NOT bump this, so the
  // user's manual orbit persists across iterations.
  const [framingEpoch, setFramingEpoch] = useState(0);
  const lastSrcId = useRef<PointCloud | null>(null);

  const CONVERGE_EPS = 1e-9;

  // Voxelize aggressively — interactive ICP wants a few thousand points.
  const tgt = useMemo(() => {
    if (raw.count === 0) return raw;
    let leaf = 0.2 * scale;
    let v = voxelGrid(raw, leaf);
    while (v.count > 12_000 && leaf < 5 * scale) {
      leaf *= 1.4;
      v = voxelGrid(raw, leaf);
    }
    return v;
  }, [raw, scale]);

  const tgtTree = useMemo(() => new KdTree(tgt.positions), [tgt]);

  const [src, setSrc] = useState<PointCloud>(emptyCloud());

  useEffect(() => {
    if (tgt.count === 0 || lastSrcId.current === tgt) return;
    lastSrcId.current = tgt;
    seed(tgt, scale);
    setFramingEpoch((e) => e + 1);
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [tgt]);

  function seed(target: PointCloud, sc: number) {
    const tx = (Math.random() - 0.5) * 2.5 * sc;
    const ty = (Math.random() - 0.5) * 2.5 * sc;
    const tz = (Math.random() - 0.5) * 0.6 * sc;
    const rx = (Math.random() - 0.5) * 25;
    const ry = (Math.random() - 0.5) * 25;
    const rz = (Math.random() - 0.5) * 25;
    const groundTruth = buildTransform(tx, ty, tz, rx, ry, rz);
    setSrc(transformCloud(target, groundTruth));
    setTransform(identityTransform());
    setIter(0);
    setFitness(null);
    setPairs(null);
    setPairCoords(new Float32Array(0));
    setPlaying(false);
    setConverged(false);
  }

  const reset = () => {
    seed(tgt, scale);
    setFramingEpoch((e) => e + 1);
  };

  const step = () => {
    if (src.count === 0 || tgt.count === 0) return;
    if (converged) return;
    const result = icpStep(src, tgtTree, transform, maxDist);
    const delta = transformDelta(transform, result.transform);
    setTransform(result.transform);
    setFitness(result.fitness);
    setPairs(result.pairs);
    setPairCoords(samplePairCoords(result.pairCoords, 400));
    setIter((i) => i + 1);
    if (result.fitness === 0 || delta < CONVERGE_EPS) {
      setConverged(true);
      setPlaying(false);
    }
  };

  useEffect(() => {
    if (!playing || converged) return;
    const id = window.setTimeout(() => {
      step();
    }, 220);
    return () => window.clearTimeout(id);
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [playing, converged, transform, src, tgt, maxDist]);

  const transformed = useMemo(
    () => transformCloud(src, transform),
    [src, transform],
  );
  const ptSize = 0.05 * scale;

  return (
    <div className="mx-auto max-w-7xl px-4 py-6 sm:px-6 lg:px-8 lg:py-10">
      <ChapterHeader chapter={chapter} />
      <DemoAbout slug="iterative-closest-point" />

      <section className="mt-6 grid gap-4 lg:grid-cols-[1fr_18rem]">
        <div className="overflow-hidden rounded-xl border border-[var(--border)]">
          <div className="flex flex-wrap items-center justify-between gap-3 border-b border-[var(--border)] bg-[color:rgba(10,15,26,0.6)] px-4 py-2 text-[11px]">
            <div className="flex items-center gap-3">
              <Dot color="#f87171" /> {t.lec11.legendSrc}
              <Dot color="#00d4aa" /> {t.lec11.legendTgt}
              <Dot color="#facc15" /> pairs
            </div>
            <div className="code-font flex items-center gap-3 text-[var(--dim)]">
              <span>iter {iter}</span>
              <span>
                {t.lec11.fitness}{" "}
                <span className="text-[var(--text)]">
                  {fitness == null ? "—" : fitness.toFixed(3)}
                </span>
              </span>
              <span>
                {t.lec11.pairs}{" "}
                <span className="text-[var(--text)]">
                  {pairs == null ? "—" : pairs.toLocaleString()}
                </span>
              </span>
            </div>
          </div>
          <div className="aspect-[4/3] md:aspect-[16/10] w-full">
            <PointCloudViewer
              layers={[
                { cloud: tgt, color: "#00d4aa", size: ptSize, opacity: 0.85 },
                { cloud: transformed, color: "#f87171", size: ptSize, opacity: 0.85 },
              ]}
              lines={
                pairCoords.length > 0
                  ? [{ positions: pairCoords, color: "#facc15", opacity: 0.95, width: 2 }]
                  : undefined
              }
              framingKey={framingEpoch}
              framingZoom={0.7}
            />
          </div>
        </div>

        <aside className="flex flex-col gap-4 rounded-xl border border-[var(--border)] bg-[var(--surface)] p-5">
          <DemoParams slug="iterative-closest-point" />
          <DataSourcePicker
            defaultPreset="naverlabs"
            onCloudChange={(c, info) => {
              setRaw(c);
              setScale(info.suggestedScale);
              setMaxDist(1 * info.suggestedScale);
            }}
          />

          <div className="grid grid-cols-3 gap-1.5">
            <button
              onClick={reset}
              className="code-font rounded-md border border-[var(--border-strong)] bg-[var(--surface-2)] px-2 py-1.5 text-[11px] text-[var(--dim)] transition hover:text-[var(--text)]"
            >
              {t.lec11.reset}
            </button>
            <button
              onClick={step}
              disabled={playing || converged}
              className="code-font rounded-md border border-[color:rgba(0,212,170,0.5)] bg-[color:rgba(0,212,170,0.10)] px-2 py-1.5 text-[11px] font-bold text-[var(--accent)] transition hover:bg-[color:rgba(0,212,170,0.18)] disabled:opacity-40"
            >
              {t.lec11.step}
            </button>
            <button
              onClick={() => setPlaying((p) => !p)}
              disabled={converged}
              className={`code-font rounded-md border px-2 py-1.5 text-[11px] font-bold transition disabled:opacity-40 ${
                playing
                  ? "border-[color:rgba(255,140,66,0.5)] bg-[color:rgba(255,140,66,0.12)] text-[#ff8c42]"
                  : "border-[color:rgba(77,166,255,0.5)] bg-[color:rgba(77,166,255,0.10)] text-[var(--accent-2)] hover:bg-[color:rgba(77,166,255,0.18)]"
              }`}
            >
              {converged ? t.lec11.converged : playing ? t.lec11.pause : t.lec11.play}
            </button>
          </div>

          <Slider
            label={t.lec11.maxDist}
            min={0.05 * scale}
            max={5 * scale}
            step={0.05 * scale}
            value={maxDist}
            unit="m"
            onChange={setMaxDist}
          />

          <pre className="code-font overflow-x-auto rounded-md bg-[var(--surface-2)] p-3 text-[10px] leading-relaxed text-[var(--text)]">
            {`pcl::IterativeClosestPoint<
    pcl::PointXYZ, pcl::PointXYZ> icp;
icp.setMaxCorrespondenceDistance(${maxDist.toFixed(2)});
icp.setMaximumIterations(1);  // step-by-step
icp.setInputSource(src);
icp.setInputTarget(tgt);
icp.align(*aligned, T);`}
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

/** Frobenius norm of the difference between two 4×4 transforms. Captures
 *  both rotation and translation change in one scalar — used as the ICP
 *  convergence threshold (matches PCL's transformation_epsilon spirit). */
function transformDelta(prev: Float32Array, next: Float32Array): number {
  let s = 0;
  for (let i = 0; i < 16; i++) {
    const d = next[i] - prev[i];
    s += d * d;
  }
  return Math.sqrt(s);
}
