import { useEffect, useMemo, useState } from "react";
import { findChapter } from "../chapters";
import BibTexBlock, { type BibEntry } from "../components/BibTexBlock";
import CautionBox from "../components/CautionBox";
import ChapterHeader from "../components/ChapterHeader";
import DemoAbout from "../components/DemoAbout";
import DemoParams from "../components/DemoParams";
import PointCloudViewer from "../components/PointCloudViewer";
import { useT } from "../i18n";
import { asset, useCloudFromUrl } from "../lib/useCloud";
import {
  applyRigid,
  loadKissMatcherData,
  packMatchSegments,
  type KissMatcherData,
} from "../lib/kissMatcher";
import { emptyCloud } from "../lib/types";

const SRC_COLOR = "#f87171";   // red — matches Lec11 ICP convention
const TGT_COLOR = "#00d4aa";   // teal — matches Lec11 ICP convention
const FINAL_COLOR = "#facc15"; // amber — survived ROBIN + GNC
const INITIAL_COLOR = "#475569"; // slate — pre-pruning

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

  const [data, setData] = useState<KissMatcherData | null>(null);
  const [dataError, setDataError] = useState<string | null>(null);
  const [aligned, setAligned] = useState(false);
  const [showInitial, setShowInitial] = useState(true);
  const [showFinal, setShowFinal] = useState(true);

  useEffect(() => {
    let cancelled = false;
    loadKissMatcherData(asset("data/precomputed/kiss_matcher.json"))
      .then((d) => {
        if (!cancelled) setData(d);
      })
      .catch((e) => {
        if (!cancelled) setDataError(e instanceof Error ? e.message : String(e));
      });
    return () => {
      cancelled = true;
    };
  }, []);

  // src and tgt URLs come from the JSON, so swapping scans is a JSON-only
  // change.
  const srcUrl = data ? asset(data.srcUrl) : "";
  const tgtUrl = data ? asset(data.tgtUrl) : "";
  const { cloud: srcCloud, loading: srcLoading, error: srcError } =
    useCloudFromUrl(srcUrl);
  const { cloud: tgtCloud, loading: tgtLoading, error: tgtError } =
    useCloudFromUrl(tgtUrl);

  const srcAligned = useMemo(() => {
    if (!data || srcCloud.count === 0) return emptyCloud();
    return applyRigid(srcCloud.positions, data.estimatedRotation, data.estimatedTranslation);
  }, [data, srcCloud]);

  const displayedSrc = aligned ? srcAligned : srcCloud;

  // Match-line buffers. Everything KISS-Matcher produced at this voxel_size
  // is rendered — no client-side throttling, since `voxel_size` (set in
  // tools/gen_kiss_matcher_data.py) already determines the correspondence
  // count.
  const initialLines = useMemo(() => {
    if (!data) return new Float32Array(0);
    return packMatchSegments(data.matches, (m) => !m.isFinal, Number.POSITIVE_INFINITY);
  }, [data]);

  const finalLines = useMemo(() => {
    if (!data) return new Float32Array(0);
    return packMatchSegments(data.matches, (m) => m.isFinal, Number.POSITIVE_INFINITY);
  }, [data]);

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

  return (
    <div className="mx-auto max-w-7xl px-4 py-6 sm:px-6 lg:px-8 lg:py-10">
      <ChapterHeader chapter={chapter} />
      <DemoAbout slug="kiss-matcher" />

      <section className="mt-6 grid gap-4 lg:grid-cols-[1fr_18rem]">
        <div className="overflow-hidden rounded-xl border border-[var(--border)]">
          <div className="flex flex-wrap items-center justify-between gap-3 border-b border-[var(--border)] bg-[color:rgba(10,15,26,0.6)] px-4 py-2 text-[11px]">
            <div className="flex flex-wrap items-center gap-3">
              <Dot color={SRC_COLOR} /> {t.lec11.legendSrc}
              <Dot color={TGT_COLOR} /> {t.lec11.legendTgt}
              <Dot color={INITIAL_COLOR} /> {t.kissMatcher.initialMatches}
              <Dot color={FINAL_COLOR} /> {t.kissMatcher.finalInliers}
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
            <PointCloudViewer layers={layers} lines={lines} framingZoom={0.85} />
          </div>
          {(dataError || srcError || tgtError) && (
            <div className="border-t border-[var(--border)] bg-[color:rgba(248,113,113,0.06)] px-4 py-2 text-[12px] text-rose-300">
              {dataError ?? srcError ?? tgtError}
            </div>
          )}
        </div>

        <aside className="flex flex-col gap-4 rounded-xl border border-[var(--border)] bg-[var(--surface)] p-5">
          <DemoParams slug="kiss-matcher" />

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

          <CautionBox
            title={t.caution.title}
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
