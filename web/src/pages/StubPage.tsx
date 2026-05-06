import { Chapter, statusColor, statusLabel } from "../chapters";
import ChapterHeader from "../components/ChapterHeader";

export default function StubPage({ chapter }: { chapter: Chapter }) {
  return (
    <div className="mx-auto max-w-5xl px-8 py-10">
      <ChapterHeader chapter={chapter} />

      <div className="mt-8 rounded-xl border border-slate-800/80 bg-slate-900/30 p-8 text-center">
        <span
          className={`inline-flex items-center rounded px-2 py-0.5 text-xs ring-1 ring-inset ${statusColor[chapter.status]}`}
        >
          {statusLabel[chapter.status]}
        </span>
        <h2 className="mt-4 text-xl font-medium text-slate-100">
          이 챕터는 곧 추가됩니다
        </h2>
        <p className="mx-auto mt-2 max-w-md text-sm leading-relaxed text-slate-400">
          소스코드와 블로그 글은 아래 링크에서 먼저 확인하실 수 있습니다.
          인터랙티브 데모는 작업 중입니다.
        </p>
      </div>
    </div>
  );
}
