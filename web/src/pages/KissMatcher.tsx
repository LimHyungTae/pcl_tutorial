import { useEffect, useMemo, useState } from "react";
import { findChapter } from "../chapters";
import BibTexBlock, { type BibEntry } from "../components/BibTexBlock";
import CautionBox from "../components/CautionBox";
import ChapterHeader from "../components/ChapterHeader";
import DemoAbout from "../components/DemoAbout";
import DemoParams from "../components/DemoParams";
import PointCloudViewer from "../components/PointCloudViewer";
import PoseOffsetControls, {
  type PoseOffset,
  ZERO_POSE,
} from "../components/PoseOffsetControls";
import RegistrationStatus, {
  type RegistrationStatusKind,
} from "../components/RegistrationStatus";
import { useT } from "../i18n";
import { buildTransform } from "../lib/filters/transform";
import { asset, useCloudFromUrl } from "../lib/useCloud";
import {
  applyRigid,
  loadKissMatcherData,
  loadKissMatcherIndex,
  packMatchSegments,
  type KissMatcherData,
  type KissMatcherPresetSummary,
} from "../lib/kissMatcher";
import { emptyCloud } from "../lib/types";

const SRC_COLOR = "#f87171";   // red — matches Lec11 ICP convention
const TGT_COLOR = "#00d4aa";   // teal — matches Lec11 ICP convention
const FINAL_COLOR = "#facc15"; // amber — survived ROBIN + GNC
const INITIAL_COLOR = "#cbd5e1"; // slate-300 — pre-pruning, kept dim but visible

const KISS_BIBTEX: BibEntry[] = [
  {
    key: "lim2025icra-KISSMatcher",
    bibtex: `@inproceedings{lim2025icra-KISSMatcher,
  title={KISS-Matcher: Fast and Robust Point Cloud Registration Revisited},
  author={Lim, Hyungtae and Kim, Daebeom and Shin, Gunhee and Shi, Jingnan and Vizzo, Ignacio and Myung, Hyun and Park, Jaesik and Carlone, Luca},
  booktitle={Proc. IEEE Int. Conf. Robot. Automat.},
  year={2025}
}`,
  },
];

export default function KissMatcher() {
  const chapter = findChapter("kiss-matcher")!;
  const t = useT();

  const [presets, setPresets] = useState<KissMatcherPresetSummary[]>([]);
  const [activeId, setActiveId] = useState<string>("vel64");
  const [data, setData] = useState<KissMatcherData | null>(null);
  const [dataError, setDataError] = useState<string | null>(null);
  const [aligned, setAligned] = useState(false);
  const [showInitial, setShowInitial] = useState(true);
  const [showFinal, setShowFinal] = useState(true);
  // 6-DoF offset the user dials in to displace the source cloud relative to
  // the target. KISS-Matcher's recovered transform doesn't depend on this
  // — but visually, dragging the sliders shows that global registration
  // doesn't need a good initial guess.
  const [poseOffset, setPoseOffset] = useState<PoseOffset>(ZERO_POSE);
  const [framingEpoch, setFramingEpoch] = useState(0);

  // Load the preset index once. Defaults activeId to vel64 if present, else
  // first available.
  useEffect(() => {
    let cancelled = false;
    loadKissMatcherIndex(asset("data/precomputed/kiss_matcher_index.json"))
      .then((idx) => {
        if (cancelled) return;
        setPresets(idx.presets);
        if (idx.presets.length > 0 && !idx.presets.some((p) => p.id === "vel64")) {
          setActiveId(idx.presets[0].id);
        }
      })
      .catch((e) => {
        if (!cancelled) setDataError(e instanceof Error ? e.message : String(e));
      });
    return () => {
      cancelled = true;
    };
  }, []);

  // Load the active preset's full JSON whenever the user switches.
  useEffect(() => {
    if (!activeId) return;
    let cancelled = false;
    setData(null);
    setDataError(null);
    setAligned(false);
    setPoseOffset(ZERO_POSE);
    setFramingEpoch((e) => e + 1);
    loadKissMatcherData(asset(`data/precomputed/kiss_matcher_${activeId}.json`))
      .then((d) => {
        if (!cancelled) setData(d);
      })
      .catch((e) => {
        if (!cancelled) setDataError(e instanceof Error ? e.message : String(e));
      });
    return () => {
      cancelled = true;
    };
  }, [activeId]);

  // src and tgt URLs come from the JSON, so swapping scans is a JSON-only
  // change.
  const srcUrl = data ? asset(data.srcUrl) : "";
  const tgtUrl = data ? asset(data.tgtUrl) : "";
  const { cloud: srcCloud, loading: srcLoading, error: srcError } =
    useCloudFromUrl(srcUrl);
  const { cloud: tgtCloud, loading: tgtLoading, error: tgtError } =
    useCloudFromUrl(tgtUrl);

  // Apply the user's offset to src for the "before alignment" view. The
  // offset is ignored once the user clicks Apply T — KISS-Matcher's
  // recovered transform takes over.
  const offsetMatrix = useMemo(
    () =>
      buildTransform(
        poseOffset.tx, poseOffset.ty, poseOffset.tz,
        poseOffset.rxDeg, poseOffset.ryDeg, poseOffset.rzDeg,
      ),
    [poseOffset],
  );

  const offsetMatrixRowMajor = useMemo(
    () => [
      offsetMatrix[0], offsetMatrix[1], offsetMatrix[2],
      offsetMatrix[4], offsetMatrix[5], offsetMatrix[6],
      offsetMatrix[8], offsetMatrix[9], offsetMatrix[10],
    ],
    [offsetMatrix],
  );
  const offsetT = useMemo(
    () => [offsetMatrix[3], offsetMatrix[7], offsetMatrix[11]],
    [offsetMatrix],
  );

  const srcOffsetCloud = useMemo(() => {
    if (srcCloud.count === 0) return emptyCloud();
    return applyRigid(srcCloud.positions, offsetMatrixRowMajor, offsetT);
  }, [srcCloud, offsetMatrixRowMajor, offsetT]);

  const srcAligned = useMemo(() => {
    if (!data || srcCloud.count === 0) return emptyCloud();
    return applyRigid(srcCloud.positions, data.estimatedRotation, data.estimatedTranslation);
  }, [data, srcCloud]);

  const displayedSrc = aligned ? srcAligned : srcOffsetCloud;

  // Match-line buffers. Everything KISS-Matcher produced at this voxel_size
  // is rendered — no client-side throttling, since `voxel_size` (set in
  // tools/gen_kiss_matcher_data.py) already determines the correspondence
  // count. src endpoints get the user offset applied so the lines stay
  // attached to the displayed src cloud.
  const transformSrcEndpoint = (m: { src: [number, number, number] }): [number, number, number] => {
    const [x, y, z] = m.src;
    const r = offsetMatrixRowMajor;
    return [
      r[0] * x + r[1] * y + r[2] * z + offsetT[0],
      r[3] * x + r[4] * y + r[5] * z + offsetT[1],
      r[6] * x + r[7] * y + r[8] * z + offsetT[2],
    ];
  };

  const initialLines = useMemo(() => {
    if (!data) return new Float32Array(0);
    const remapped = data.matches
      .filter((m) => !m.isFinal)
      .map((m) => ({ ...m, src: transformSrcEndpoint(m) }));
    return packMatchSegments(remapped, () => true, Number.POSITIVE_INFINITY);
  }, [data, offsetMatrixRowMajor, offsetT]);

  const finalLines = useMemo(() => {
    if (!data) return new Float32Array(0);
    const remapped = data.matches
      .filter((m) => m.isFinal)
      .map((m) => ({ ...m, src: transformSrcEndpoint(m) }));
    return packMatchSegments(remapped, () => true, Number.POSITIVE_INFINITY);
  }, [data, offsetMatrixRowMajor, offsetT]);

  const lines = useMemo(() => {
    // Suppress lines after Apply T — they would no longer connect to the
    // moved src points.
    if (aligned) return undefined;
    const out: { positions: Float32Array; color: string; opacity: number; width: number }[] = [];
    if (showInitial && initialLines.length > 0) {
      out.push({ positions: initialLines, color: INITIAL_COLOR, opacity: 0.55, width: 1 });
    }
    if (showFinal && finalLines.length > 0) {
      out.push({ positions: finalLines, color: FINAL_COLOR, opacity: 0.95, width: 2 });
    }
    return out.length > 0 ? out : undefined;
  }, [aligned, showInitial, showFinal, initialLines, finalLines]);

  const ptSize = 0.12;
  const layers = useMemo(
    () => [
      { cloud: tgtCloud, color: TGT_COLOR, size: ptSize, opacity: 0.85 },
      { cloud: displayedSrc, color: SRC_COLOR, size: ptSize, opacity: 0.85 },
    ],
    [tgtCloud, displayedSrc],
  );

  const stats = data?.stats;
  const dataMissing = !data && !!dataError;
  void srcLoading;
  void tgtLoading;

  // Mirrors the upstream Python example's success heuristic:
  //   num_final_inliers >= 5 → likely succeeded; otherwise likely failed.
  const status = useMemo<{ kind: RegistrationStatusKind; text: string } | null>(() => {
    if (!stats) return null;
    const n = stats.numFinalInliers.toString();
    if (stats.numFinalInliers >= 5) {
      return {
        kind: "ok",
        text: t.kissMatcher.statusOk
          .replace("{n}", n)
          .replace("{rot}", stats.numRotationInliers.toString())
          .replace("{trans}", stats.numFinalInliers.toString()),
      };
    }
    return { kind: "fail", text: t.kissMatcher.statusFail.replace("{n}", n) };
  }, [stats, t.kissMatcher]);

  return (
    <div className="mx-auto max-w-7xl px-4 py-6 sm:px-6 lg:px-8 lg:py-10">
      <ChapterHeader chapter={chapter} />
      <DemoAbout slug="kiss-matcher" />

      <section className="mt-6 grid gap-4 lg:grid-cols-[1fr_18rem]">
        <div className="overflow-hidden rounded-xl border border-[var(--border)]">
          <div className="flex flex-wrap items-center justify-between gap-x-3 gap-y-2 border-b border-[var(--border)] bg-[color:rgba(10,15,26,0.6)] px-3 py-2 text-[11px] sm:px-4">
            <div className="flex flex-wrap items-center gap-x-3 gap-y-1.5">
              <Legend color={SRC_COLOR} label={t.lec11.legendSrc} />
              <Legend color={TGT_COLOR} label={t.lec11.legendTgt} />
              <Legend color={INITIAL_COLOR} label={t.kissMatcher.initialMatches} />
              <Legend color={FINAL_COLOR} label={t.kissMatcher.finalInliers} />
            </div>
            <div className="code-font flex flex-wrap items-center gap-3 text-[var(--dim)]">
              {stats && (
                <>
                  <span>
                    init <span className="text-[var(--text)]">{stats.numInitialMatches.toLocaleString()}</span>
                  </span>
                  <span>
                    final <span className="text-[var(--text)]">{stats.numFinalInliers}</span>
                  </span>
                  <span>
                    src/tgt{" "}
                    <span className="text-[var(--text)]">
                      {(stats.numSrcPoints / 1000).toFixed(0)}k / {(stats.numTgtPoints / 1000).toFixed(0)}k
                    </span>
                  </span>
                </>
              )}
            </div>
          </div>
          <div className="aspect-[4/3] md:aspect-[16/10] w-full">
            <PointCloudViewer
              layers={layers}
              lines={lines}
              framingZoom={0.85}
              framingKey={framingEpoch}
            />
          </div>
          {status && <RegistrationStatus kind={status.kind} text={status.text} />}
          {(dataError || srcError || tgtError) && (
            <div className="border-t border-[var(--border)] bg-[color:rgba(248,113,113,0.06)] px-4 py-2 text-[12px] text-rose-300">
              {dataError ?? srcError ?? tgtError}
            </div>
          )}
        </div>

        <aside className="flex flex-col gap-4 rounded-xl border border-[var(--border)] bg-[var(--surface)] p-5">
          <DemoParams slug="kiss-matcher" />

          {presets.length > 0 && (
            <div>
              <div className="mb-1.5 text-sm text-slate-200">{t.kissMatcher.preset}</div>
              <div className="grid grid-cols-1 gap-1.5">
                {presets.map((p) => {
                  const active = p.id === activeId;
                  return (
                    <button
                      key={p.id}
                      onClick={() => setActiveId(p.id)}
                      className={`code-font rounded-md border px-2.5 py-1.5 text-left text-[11px] transition ${
                        active
                          ? "border-[color:rgba(0,212,170,0.5)] bg-[color:rgba(0,212,170,0.08)] text-[var(--text-strong)]"
                          : "border-[var(--border-strong)] bg-[var(--surface-2)] text-[var(--dim)] hover:text-[var(--text)]"
                      }`}
                    >
                      <div className="truncate font-bold">{p.label}</div>
                      <div className="mt-0.5 text-[9px] text-[var(--mut)]">
                        {(p.stats.numSrcPoints / 1000).toFixed(0)}k / {(p.stats.numTgtPoints / 1000).toFixed(0)}k pts ·{" "}
                        {p.stats.numFinalInliers} final inliers
                      </div>
                    </button>
                  );
                })}
              </div>
            </div>
          )}

          <div className="grid grid-cols-2 gap-1.5">
            <button
              onClick={() => setAligned(false)}
              disabled={!data}
              className={`code-font rounded-md border px-2 py-1.5 text-[11px] font-bold transition disabled:opacity-40 ${
                !aligned
                  ? "border-[color:rgba(248,113,113,0.5)] bg-[color:rgba(248,113,113,0.10)] text-[#f87171]"
                  : "border-[var(--border-strong)] bg-[var(--surface-2)] text-[var(--dim)] hover:text-[var(--text)]"
              }`}
            >
              {t.kissMatcher.original}
            </button>
            <button
              onClick={() => setAligned(true)}
              disabled={!data}
              className={`code-font rounded-md border px-2 py-1.5 text-[11px] font-bold transition disabled:opacity-40 ${
                aligned
                  ? "border-[color:rgba(0,212,170,0.5)] bg-[color:rgba(0,212,170,0.10)] text-[var(--accent)]"
                  : "border-[var(--border-strong)] bg-[var(--surface-2)] text-[var(--dim)] hover:text-[var(--text)]"
              }`}
            >
              {t.kissMatcher.applyT}
            </button>
          </div>

          <div className="flex flex-col gap-1.5 text-[12px] text-[var(--text)]">
            <label className="flex items-center gap-2">
              <input
                type="checkbox"
                checked={showInitial}
                onChange={(e) => setShowInitial(e.target.checked)}
                disabled={aligned}
                className="h-4 w-4 accent-[var(--accent)] disabled:opacity-30"
              />
              <span className={aligned ? "opacity-40" : ""}>{t.kissMatcher.toggleInitial}</span>
            </label>
            <label className="flex items-center gap-2">
              <input
                type="checkbox"
                checked={showFinal}
                onChange={(e) => setShowFinal(e.target.checked)}
                disabled={aligned}
                className="h-4 w-4 accent-[var(--accent)] disabled:opacity-30"
              />
              <span className={aligned ? "opacity-40" : ""}>{t.kissMatcher.toggleFinal}</span>
            </label>
          </div>

          <PoseOffsetControls
            value={poseOffset}
            scale={1}
            onChange={(p) => {
              setPoseOffset(p);
              if (aligned) setAligned(false);
            }}
            onCommit={(p) => {
              if (p === ZERO_POSE || (p.tx === 0 && p.ty === 0 && p.tz === 0 && p.rxDeg === 0 && p.ryDeg === 0 && p.rzDeg === 0)) {
                // Reset went back to origin — refit camera so the cloud stays in view.
                setFramingEpoch((e) => e + 1);
              } else {
                setFramingEpoch((e) => e + 1);
              }
            }}
          />

          <p className="-mt-2 text-[11px] leading-snug text-[var(--mut)]">
            {t.kissMatcher.poseInvarianceNote}
          </p>

          <CautionBox
            title={t.kissMatcher.precomputedTitle}
            body={t.kissMatcher.precomputedNote}
            links={[
              { label: "KISS-Matcher (GitHub)", url: "https://github.com/MIT-SPARK/KISS-Matcher" },
              { label: "Paper (arXiv)", url: "https://arxiv.org/abs/2409.15615" },
            ]}
          />

          {dataMissing && (
            <div className="rounded-md border border-[color:rgba(248,113,113,0.4)] bg-[color:rgba(248,113,113,0.06)] px-3 py-2.5 text-[12px] text-rose-200">
              <div className="font-semibold text-rose-300">{t.kissMatcher.missingDataTitle}</div>
              <p className="mt-1 leading-relaxed">{t.kissMatcher.missingDataBody}</p>
              <pre className="code-font mt-2 overflow-x-auto rounded bg-[var(--surface-2)] p-2 text-[10px]">
{`pip install kiss-matcher
python tools/gen_kiss_matcher_data.py`}
              </pre>
            </div>
          )}

          <pre className="code-font overflow-x-auto rounded-md bg-[var(--surface-2)] p-3 text-[10px] leading-relaxed text-[var(--text)]">
{`kiss_matcher::KISSMatcherConfig cfg(${data?.voxelSize.toFixed(2) ?? "—"});
kiss_matcher::KISSMatcher matcher(cfg);
auto sol = matcher.estimate(src, tgt);
// sol.rotation, sol.translation`}
          </pre>
        </aside>
      </section>

      <BibTexBlock entries={KISS_BIBTEX} />
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

/** Dot + label kept on the same line when the parent flex-wraps. */
function Legend({ color, label }: { color: string; label: string }) {
  return (
    <span className="inline-flex items-center gap-1.5 whitespace-nowrap">
      <Dot color={color} />
      {label}
    </span>
  );
}
