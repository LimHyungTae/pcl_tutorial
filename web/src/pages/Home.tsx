import { Link } from "react-router-dom";
import { chapters, statusColor, statusLabel } from "../chapters";

export default function Home() {
  return (
    <div className="mx-auto max-w-5xl px-8 py-12">
      <div className="mb-10">
        <div className="mb-2 text-xs font-semibold uppercase tracking-widest text-emerald-400">
          PCL Tutorial · 한글
        </div>
        <h1 className="text-4xl font-semibold tracking-tight text-white">
          Point Cloud Library를{" "}
          <span className="text-emerald-300">손으로 만져 보는</span> 튜토리얼
        </h1>
        <p className="mt-4 max-w-2xl text-base leading-relaxed text-slate-400">
          C++ 코드 그대로의 동작을 브라우저에서 슬라이더로 만져볼 수 있게
          옮겼습니다. 가벼운 필터(Voxel / PassThrough / SOR / KNN)는 TypeScript로
          재구현했고, 무거운 연산(ICP / GICP / Normal)은 PCL 바이너리로 미리
          뽑은 결과를 before / after로 비교합니다.
        </p>
      </div>

      <div className="mb-8 flex flex-wrap gap-2 text-xs">
        <Legend label="인터랙티브" tone="emerald" hint="슬라이더로 실시간 갱신" />
        <Legend label="Pre-computed" tone="sky" hint="C++ 결과를 토글" />
        <Legend label="코드 설명" tone="amber" hint="개념·소스 해설" />
        <Legend label="준비중" tone="slate" hint="추후 추가" />
      </div>

      <div className="grid gap-3 sm:grid-cols-2">
        {chapters.map((c) => (
          <Link
            key={c.slug}
            to={`/${c.slug}`}
            className="group flex flex-col gap-3 rounded-xl border border-slate-800/80 bg-slate-900/40 p-5 transition hover:border-slate-600 hover:bg-slate-900"
          >
            <div className="flex items-start justify-between gap-3">
              <div className="min-w-0">
                <div className="code-font text-xs text-slate-500">
                  Chapter {c.number}
                </div>
                <div className="mt-1 truncate text-base font-medium text-white">
                  {c.title}
                </div>
              </div>
              <span
                className={`shrink-0 rounded px-2 py-0.5 text-[10px] ring-1 ring-inset ${statusColor[c.status]}`}
              >
                {statusLabel[c.status]}
              </span>
            </div>
            <p className="text-sm leading-relaxed text-slate-400">
              {c.subtitle}
            </p>
          </Link>
        ))}
      </div>
    </div>
  );
}

function Legend({
  label,
  hint,
  tone,
}: {
  label: string;
  hint: string;
  tone: "emerald" | "sky" | "amber" | "slate";
}) {
  const tones = {
    emerald: "bg-emerald-500/15 text-emerald-300 ring-emerald-500/30",
    sky: "bg-sky-500/15 text-sky-300 ring-sky-500/30",
    amber: "bg-amber-500/15 text-amber-300 ring-amber-500/30",
    slate: "bg-slate-500/15 text-slate-400 ring-slate-500/30",
  } as const;
  return (
    <span className="inline-flex items-center gap-2">
      <span className={`rounded px-1.5 py-0.5 text-[10px] ring-1 ring-inset ${tones[tone]}`}>
        {label}
      </span>
      <span className="text-slate-500">— {hint}</span>
    </span>
  );
}
