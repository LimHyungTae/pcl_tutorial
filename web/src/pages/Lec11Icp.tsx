import { useEffect, useMemo, useRef, useState } from "react";
import { findChapter } from "../chapters";
import ChapterHeader from "../components/ChapterHeader";
import DemoAbout from "../components/DemoAbout";
import DemoParams from "../components/DemoParams";
import PointCloudViewer from "../components/PointCloudViewer";
import PoseOffsetControls, {
  randomPose,
  type PoseOffset,
  ZERO_POSE,
} from "../components/PoseOffsetControls";
import RegistrationStatus, {
  type RegistrationStatusKind,
} from "../components/RegistrationStatus";
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
  // 6-DoF offset applied to tgt to fabricate the src — the "initial pose
  // gap" the user can dial in via the sidebar sliders.
  const [poseOffset, setPoseOffset] = useState<PoseOffset>(ZERO_POSE);
  // Bumped on preset switch and Randomize/Reset — that's when we want the
  // camera to refit. Slider tweaks do NOT bump this, so the user's orbit
  // persists while they drag.
  const [framingEpoch, setFramingEpoch] = useState(0);
  const lastTgtRef = useRef<PointCloud | null>(null);

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

  // src = tgt with the user-controlled rigid offset applied. Iteration
  // state auto-resets whenever src changes (see effect below).
  const src = useMemo(() => {
    if (tgt.count === 0) return emptyCloud();
    const T = buildTransform(
      poseOffset.tx, poseOffset.ty, poseOffset.tz,
      poseOffset.rxDeg, poseOffset.ryDeg, poseOffset.rzDeg,
    );
    return transformCloud(tgt, T);
  }, [tgt, poseOffset]);

  // First time we see a target (preset switch / file drop), seed the offset
  // sliders with random values so the demo lands in an interesting state.
  useEffect(() => {
    if (tgt.count === 0 || lastTgtRef.current === tgt) return;
    lastTgtRef.current = tgt;
    setPoseOffset(randomPose(scale));
    setFramingEpoch((e) => e + 1);
  }, [tgt, scale]);

  // Whenever src changes (offset move, preset switch), drop the iteration
  // state — old fitness / transform are no longer meaningful.
  useEffect(() => {
    setTransform(identityTransform());
    setIter(0);
    setFitness(null);
    setPairs(null);
    setPairCoords(new Float32Array(0));
    setPlaying(false);
    setConverged(false);
  }, [src]);

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

  // Heuristic: ICP "succeeded" when it converged AND the mean residual
  // (fitness) is small relative to the input scale. fitness > 0.5·scale is
  // usually a sign of a local minimum.
  const status = useMemo<{ kind: RegistrationStatusKind; text: string } | null>(() => {
    if (iter === 0) return null;
    const f = fitness == null ? "—" : fitness.toFixed(3);
    const p = pairs == null ? "—" : pairs.toLocaleString();
    if (!converged) {
      return { kind: "iterating", text: t.lec11.statusIterating.replace("{fitness}", f).replace("{pairs}", p) };
    }
    if (pairs != null && pairs < 6) {
      return { kind: "fail", text: t.lec11.statusFail.replace("{pairs}", p) };
    }
    if (fitness != null && fitness > 0.5 * scale) {
      return { kind: "warn", text: t.lec11.statusWarn.replace("{fitness}", f) };
    }
    return { kind: "ok", text: t.lec11.statusOk.replace("{fitness}", f).replace("{pairs}", p) };
  }, [iter, converged, fitness, pairs, scale, t.lec11]);

  return (
    <div className="mx-auto max-w-7xl px-4 py-6 sm:px-6 lg:px-8 lg:py-10">
      <ChapterHeader chapter={chapter} />
      <DemoAbout slug="iterative-closest-point" />

      <section className="mt-6 grid gap-4 lg:grid-cols-[1fr_18rem]">
        <div className="overflow-hidden rounded-xl border border-[var(--border)]">
          <div className="flex flex-wrap items-center justify-between gap-x-3 gap-y-2 border-b border-[var(--border)] bg-[color:rgba(10,15,26,0.6)] px-3 py-2 text-[11px] sm:px-4">
            <div className="flex flex-wrap items-center gap-x-3 gap-y-1.5">
              <Legend color="#f87171" label={t.lec11.legendSrc} />
              <Legend color="#00d4aa" label={t.lec11.legendTgt} />
              <Legend color="#facc15" label="pairs" />
            </div>
            <div className="code-font flex flex-wrap items-center gap-x-3 gap-y-1.5 text-[var(--dim)]">
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
          {status && <RegistrationStatus kind={status.kind} text={status.text} />}
        </div>

        <aside className="flex flex-col gap-4 rounded-xl border border-[var(--border)] bg-[var(--surface)] p-5">
          <DemoParams slug="iterative-closest-point" />
          <DataSourcePicker
            defaultPreset="naverlabs"
            onCloudChange={(c, info) => {
              setRaw(c);
              setScale(info.suggestedScale);
              // KITTI Vel64 scenes have wider scan-to-scan motion; 5 m is a
              // saner default than 1 m for that preset.
              const baseDist = info.name === "kitti" ? 5 : 1;
              setMaxDist(baseDist * info.suggestedScale);
            }}
          />

          <div className="grid grid-cols-2 gap-1.5">
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
            max={10 * scale}
            step={0.05 * scale}
            value={maxDist}
            unit="m"
            onChange={setMaxDist}
          />

          <PoseOffsetControls
            value={poseOffset}
            scale={scale}
            onChange={setPoseOffset}
            onCommit={() => setFramingEpoch((e) => e + 1)}
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

function Legend({ color, label }: { color: string; label: string }) {
  return (
    <span className="inline-flex items-center gap-1.5 whitespace-nowrap">
      <Dot color={color} />
      {label}
    </span>
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
