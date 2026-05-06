import { useState } from "react";
import { findChapter } from "../chapters";
import ChapterHeader from "../components/ChapterHeader";
import PointCloudViewer from "../components/PointCloudViewer";
import { asset, useCloudFromUrl } from "../lib/useCloud";

type Mode = "before" | "after";

export default function Lec11Icp() {
  const chapter = findChapter("lec11")!;
  const [mode, setMode] = useState<Mode>("before");

  const src = useCloudFromUrl(asset("precomputed/icp_src.pcd"));
  const tgt = useCloudFromUrl(asset("precomputed/icp_tgt.pcd"));
  const aligned = useCloudFromUrl(asset("precomputed/icp_aligned.pcd"));

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
                  {mode === "before" ? "src" : "src(aligned)"}
                </span>
              </span>
              <span className="inline-flex items-center gap-1.5">
                <span className="inline-block h-2 w-2 rounded-full bg-emerald-400" />
                <span className="text-slate-300">tgt</span>
              </span>
            </div>
            <span className="code-font text-slate-500">
              {(src.cloud.count + tgt.cloud.count).toLocaleString()} pts
            </span>
          </div>
          <div className="aspect-[16/10] w-full">
            <PointCloudViewer layers={layers} />
          </div>
        </div>

        <aside className="flex flex-col gap-5 rounded-xl border border-slate-800/80 bg-slate-900/40 p-5">
          <div>
            <div className="mb-1.5 text-sm text-slate-200">Mode</div>
            <div className="grid grid-cols-2 overflow-hidden rounded-md border border-slate-700">
              <button
                onClick={() => setMode("before")}
                className={`py-2 text-sm transition ${
                  mode === "before"
                    ? "bg-rose-500/15 text-rose-200"
                    : "text-slate-400 hover:bg-slate-800/60 hover:text-slate-200"
                }`}
              >
                Before
              </button>
              <button
                onClick={() => setMode("after")}
                className={`py-2 text-sm transition ${
                  mode === "after"
                    ? "bg-emerald-500/15 text-emerald-200"
                    : "text-slate-400 hover:bg-slate-800/60 hover:text-slate-200"
                }`}
              >
                After
              </button>
            </div>
            <p className="mt-2 text-xs leading-relaxed text-slate-500">
              <strong>Before</strong>: ICP 적용 전 src(빨강) ↔ tgt(초록) — 일부러
              x축으로 +2m 오프셋을 준 상태입니다.
              <br />
              <strong>After</strong>: ICP가 추정한 변환을 src에 적용한 결과 — tgt에
              겹쳐져야 정렬 성공.
            </p>
          </div>

          <pre className="code-font overflow-x-auto rounded-md bg-slate-950/60 p-3 text-[11px] leading-relaxed text-slate-300">
            {`pcl::IterativeClosestPoint<
    pcl::PointXYZ, pcl::PointXYZ> icp;
icp.setMaxCorrespondenceDistance(1.0);
icp.setTransformationEpsilon(0.003);
icp.setMaximumIterations(1000);
icp.setInputSource(src);
icp.setInputTarget(tgt);
icp.align(*aligned);`}
          </pre>

          <div className="rounded-md border border-sky-500/20 bg-sky-500/5 px-3 py-2 text-xs text-sky-200">
            이 챕터의 결과는{" "}
            <span className="code-font">scripts/export_precomputed.cpp</span>가
            로컬에서 한 번 돌고 PCD로 저장한 결과입니다 — 브라우저는 fetch만 합니다.
          </div>
        </aside>
      </section>

      {loading && (
        <div className="mt-6 text-sm text-slate-500">데이터 불러오는 중…</div>
      )}
      {anyError && (
        <div className="mt-6 rounded-md border border-rose-500/30 bg-rose-500/10 px-3 py-2 text-sm text-rose-200">
          Pre-computed 자산을 불러오지 못했습니다: {anyError}
        </div>
      )}
    </div>
  );
}
