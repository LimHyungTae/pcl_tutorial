import { useState } from "react";
import { ChapterMeta } from "../chapters";
import ChapterHeader from "../components/ChapterHeader";
import PointCloudViewer from "../components/PointCloudViewer";
import { useT } from "../i18n";
import { asset, useCloudFromUrl } from "../lib/useCloud";

type Mode = "before" | "after";

type Props = {
  chapter: ChapterMeta;
  /** PCD url stems, e.g. "icp" → /precomputed/{icp_src,icp_tgt,icp_aligned}.pcd. */
  prefix: string;
  codeBlock: string;
};

export default function PrecomputedRegistration({
  chapter,
  prefix,
  codeBlock,
}: Props) {
  const t = useT();
  const [mode, setMode] = useState<Mode>("before");

  const src = useCloudFromUrl(asset(`precomputed/${prefix}_src.pcd`));
  const tgt = useCloudFromUrl(asset(`precomputed/${prefix}_tgt.pcd`));
  const aligned = useCloudFromUrl(asset(`precomputed/${prefix}_aligned.pcd`));

  const loading = src.loading || tgt.loading || aligned.loading;
  const anyError = src.error ?? tgt.error ?? aligned.error;

  const layers =
    mode === "before"
      ? [
          { cloud: src.cloud, color: "#f87171", size: 0.05, opacity: 0.85 },
          { cloud: tgt.cloud, color: "#34d399", size: 0.05, opacity: 0.85 },
        ]
      : [
          { cloud: aligned.cloud, color: "#f87171", size: 0.05, opacity: 0.85 },
          { cloud: tgt.cloud, color: "#34d399", size: 0.05, opacity: 0.85 },
        ];

  return (
    <div className="mx-auto max-w-7xl px-8 py-10">
      <ChapterHeader chapter={chapter} />

      <section className="mt-6 grid gap-4 lg:grid-cols-[1fr_18rem]">
        <div className="overflow-hidden rounded-xl border border-slate-800/80">
          <div className="flex items-center justify-between border-b border-slate-800/80 bg-slate-950/60 px-4 py-2 text-xs">
            <div className="flex items-center gap-3">
              <span className="inline-flex items-center gap-1.5">
                <span className="inline-block h-2 w-2 rounded-full bg-rose-400" />
                <span className="text-slate-300">
                  {mode === "before" ? t.viewer.src : t.viewer.aligned}
                </span>
              </span>
              <span className="inline-flex items-center gap-1.5">
                <span className="inline-block h-2 w-2 rounded-full bg-emerald-400" />
                <span className="text-slate-300">{t.viewer.tgt}</span>
              </span>
            </div>
            <span className="code-font text-slate-500">
              {(src.cloud.count + tgt.cloud.count).toLocaleString()} {t.viewer.pointsSuffix}
            </span>
          </div>
          <div className="aspect-[16/10] w-full">
            <PointCloudViewer layers={layers} />
          </div>
        </div>

        <aside className="flex flex-col gap-5 rounded-xl border border-slate-800/80 bg-slate-900/40 p-5">
          <div>
            <div className="mb-1.5 text-sm text-slate-200">{t.lec11.mode}</div>
            <div className="grid grid-cols-2 overflow-hidden rounded-md border border-slate-700">
              <button
                onClick={() => setMode("before")}
                className={`py-2 text-sm transition ${
                  mode === "before"
                    ? "bg-rose-500/15 text-rose-200"
                    : "text-slate-400 hover:bg-slate-800/60 hover:text-slate-200"
                }`}
              >
                {t.lec11.before}
              </button>
              <button
                onClick={() => setMode("after")}
                className={`py-2 text-sm transition ${
                  mode === "after"
                    ? "bg-emerald-500/15 text-emerald-200"
                    : "text-slate-400 hover:bg-slate-800/60 hover:text-slate-200"
                }`}
              >
                {t.lec11.after}
              </button>
            </div>
            <p className="mt-2 text-xs leading-relaxed text-slate-500">
              <strong>{t.lec11.before}</strong> · {t.lec11.beforeHint}
              <br />
              <strong>{t.lec11.after}</strong> · {t.lec11.afterHint}
            </p>
          </div>

          <pre className="code-font overflow-x-auto rounded-md bg-slate-950/60 p-3 text-[11px] leading-relaxed text-slate-300">
            {codeBlock}
          </pre>

          <div className="rounded-md border border-sky-500/20 bg-sky-500/5 px-3 py-2 text-xs text-sky-200">
            {t.lec11.note}
          </div>
        </aside>
      </section>

      {loading && (
        <div className="mt-6 text-sm text-slate-500">…</div>
      )}
      {anyError && (
        <div className="mt-6 rounded-md border border-rose-500/30 bg-rose-500/10 px-3 py-2 text-sm text-rose-200">
          {anyError}
        </div>
      )}
    </div>
  );
}
