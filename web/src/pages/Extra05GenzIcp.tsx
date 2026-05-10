import { useEffect, useMemo, useRef, useState } from "react";
import { findChapter } from "../chapters";
import BibTexBlock, { type BibEntry } from "../components/BibTexBlock";
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
import {
  buildTransform,
  transformCloud,
} from "../lib/filters/transform";
import {
  buildGenzTargetIndex,
  genzIcpStep,
  sampleGenzPairs,
} from "../lib/filters/genzIcp";
import { identityTransform } from "../lib/filters/icp";
import { voxelGrid } from "../lib/filters/voxelGrid";
import { emptyCloud, type PointCloud } from "../lib/types";

/** Wong-palette colors used by the upstream GenZ-ICP rviz config. */
const PLANAR_COLOR = "#0077bb";
const NON_PLANAR_COLOR = "#ee3377";

const PLANARITY_THRESHOLD = 0.1;
const NORMAL_K = 10;

const GENZ_BIBTEX: BibEntry[] = [
  {
    key: "lee2024genz",
    bibtex: `@article{lee2024genz,
  title={GenZ-ICP: Generalizable and Degeneracy-Robust LiDAR Odometry Using an Adaptive Weighting},
  author={Lee, Daehan and Lim, Hyungtae and Han, Soohee},
  journal={IEEE Robotics and Automation Letters},
  year={2024},
  doi={10.1109/LRA.2024.3498779}
}`,
  },
];

export default function Extra05GenzIcp() {
  const chapter = findChapter("genz-icp")!;
  const t = useT();

  const [raw, setRaw] = useState<PointCloud>(emptyCloud());
  const [scale, setScale] = useState(1);
  const [maxDist, setMaxDist] = useState(1);
  const [transform, setTransform] = useState<Float32Array>(identityTransform());
  const [iter, setIter] = useState(0);
  const [fitness, setFitness] = useState<number | null>(null);
  const [pairs, setPairs] = useState<number | null>(null);
  const [planarPairs, setPlanarPairs] = useState(0);
  const [nonPlanarPairs, setNonPlanarPairs] = useState(0);
  const [alpha, setAlpha] = useState<number | null>(null);
  const [planarLines, setPlanarLines] = useState<Float32Array>(new Float32Array(0));
  const [nonPlanarLines, setNonPlanarLines] = useState<Float32Array>(new Float32Array(0));
  const [playing, setPlaying] = useState(false);
  const [converged, setConverged] = useState(false);
  const [poseOffset, setPoseOffset] = useState<PoseOffset>(ZERO_POSE);
  const [framingEpoch, setFramingEpoch] = useState(0);
  const lastTgtRef = useRef<PointCloud | null>(null);

  const CONVERGE_EPS = 1e-9;

  // Voxelize aggressively — interactive GenZ-ICP wants a few thousand points.
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

  // Pre-compute per-tgt normals + planarity classification once per cloud.
  // Each iteration's nearest-neighbor lookup just reads from this.
  const tgtIndex = useMemo(() => {
    if (tgt.count === 0) return null;
    return buildGenzTargetIndex(tgt, NORMAL_K, PLANARITY_THRESHOLD);
  }, [tgt]);

  // src = tgt with the user-controlled rigid offset applied.
  const src = useMemo(() => {
    if (tgt.count === 0) return emptyCloud();
    const T = buildTransform(
      poseOffset.tx, poseOffset.ty, poseOffset.tz,
      poseOffset.rxDeg, poseOffset.ryDeg, poseOffset.rzDeg,
    );
    return transformCloud(tgt, T);
  }, [tgt, poseOffset]);

  useEffect(() => {
    if (tgt.count === 0 || lastTgtRef.current === tgt) return;
    lastTgtRef.current = tgt;
    setPoseOffset(randomPose(scale));
    setFramingEpoch((e) => e + 1);
  }, [tgt, scale]);

  // src changed → reset iteration state.
  useEffect(() => {
    setTransform(identityTransform());
    setIter(0);
    setFitness(null);
    setPairs(null);
    setPlanarPairs(0);
    setNonPlanarPairs(0);
    setAlpha(null);
    setPlanarLines(new Float32Array(0));
    setNonPlanarLines(new Float32Array(0));
    setPlaying(false);
    setConverged(false);
  }, [src]);

  const step = () => {
    if (src.count === 0 || tgt.count === 0 || !tgtIndex) return;
    if (converged) return;
    // GenZ uses kernel ≈ sigma/3 with sigma being its adaptive threshold.
    // Without that machinery, derive a reasonable kernel from maxDist.
    const kernel = maxDist / 3;
    const result = genzIcpStep(src, tgtIndex, transform, maxDist, kernel);
    const delta = transformDelta(transform, result.transform);
    setTransform(result.transform);
    setFitness(result.fitness);
    setPairs(result.pairs);
    setPlanarPairs(result.planarPairs);
    setNonPlanarPairs(result.nonPlanarPairs);
    setAlpha(result.alpha);
    setPlanarLines(sampleGenzPairs(result.planarPairCoords, 250));
    setNonPlanarLines(sampleGenzPairs(result.nonPlanarPairCoords, 250));
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

  const lines = useMemo(() => {
    const out: { positions: Float32Array; color: string; opacity: number; width: number }[] = [];
    if (planarLines.length > 0) {
      out.push({ positions: planarLines, color: PLANAR_COLOR, opacity: 0.95, width: 2 });
    }
    if (nonPlanarLines.length > 0) {
      out.push({ positions: nonPlanarLines, color: NON_PLANAR_COLOR, opacity: 0.95, width: 2 });
    }
    return out;
  }, [planarLines, nonPlanarLines]);

  return (
    <div className="mx-auto max-w-7xl px-4 py-6 sm:px-6 lg:px-8 lg:py-10">
      <ChapterHeader chapter={chapter} />
      <DemoAbout slug="genz-icp" />

      <section className="mt-6 grid gap-4 lg:grid-cols-[1fr_18rem]">
        <div className="overflow-hidden rounded-xl border border-[var(--border)]">
          <div className="flex flex-wrap items-center justify-between gap-x-3 gap-y-2 border-b border-[var(--border)] bg-[color:rgba(10,15,26,0.6)] px-3 py-2 text-[11px] sm:px-4">
            <div className="flex flex-wrap items-center gap-x-3 gap-y-1.5">
              <Legend color="#f87171" label={t.lec11.legendSrc} />
              <Legend color="#00d4aa" label={t.lec11.legendTgt} />
              <Legend color={PLANAR_COLOR} label={t.extra05.planar} />
              <Legend color={NON_PLANAR_COLOR} label={t.extra05.nonPlanar} />
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
                α{" "}
                <span className="text-[var(--text)]">
                  {alpha == null ? "—" : alpha.toFixed(2)}
                </span>
              </span>
              <span>
                {t.lec11.pairs}{" "}
                <span className="text-[var(--text)]">
                  {pairs == null ? "—" : pairs.toLocaleString()}
                </span>
                <span className="ml-1 text-[var(--mut)]">
                  ({planarPairs.toLocaleString()}/{nonPlanarPairs.toLocaleString()})
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
              lines={lines.length > 0 ? lines : undefined}
              framingKey={framingEpoch}
              framingZoom={0.7}
            />
          </div>
          {status && <RegistrationStatus kind={status.kind} text={status.text} />}
        </div>

        <aside className="flex flex-col gap-4 rounded-xl border border-[var(--border)] bg-[var(--surface)] p-5">
          <DemoParams slug="genz-icp" />
          <DataSourcePicker
            defaultPreset="naverlabs"
            onCloudChange={(c, info) => {
              setRaw(c);
              setScale(info.suggestedScale);
              setMaxDist(1 * info.suggestedScale);
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
            max={5 * scale}
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
            {`genz_icp::Registration reg(
    /*max_iter*/ 100,
    /*conv_eps*/ 1e-4);
auto T = reg.RegisterFrame(
    src, voxel_map, T_init,
    /*max_corr_dist*/ ${maxDist.toFixed(2)},
    /*kernel*/ ${(maxDist / 3).toFixed(2)});`}
          </pre>
        </aside>
      </section>

      <BibTexBlock entries={GENZ_BIBTEX} />
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

function transformDelta(prev: Float32Array, next: Float32Array): number {
  let s = 0;
  for (let i = 0; i < 16; i++) {
    const d = next[i] - prev[i];
    s += d * d;
  }
  return Math.sqrt(s);
}
