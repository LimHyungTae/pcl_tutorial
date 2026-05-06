import { Chapter, statusColor, statusLabel } from "../chapters";

const REPO = "https://github.com/LimHyungTae/pcl_tutorial/blob/main";

export default function ChapterHeader({ chapter }: { chapter: Chapter }) {
  return (
    <header className="border-b border-slate-800/80 pb-6">
      <div className="flex items-center gap-3 text-xs">
        <span className="code-font text-slate-500">
          Chapter {chapter.number}
        </span>
        <span
          className={`rounded px-1.5 py-0.5 text-[10px] ring-1 ring-inset ${statusColor[chapter.status]}`}
        >
          {statusLabel[chapter.status]}
        </span>
      </div>
      <h1 className="mt-2 text-3xl font-semibold tracking-tight text-white">
        {chapter.title}
      </h1>
      <p className="mt-2 text-base text-slate-400">{chapter.subtitle}</p>

      <div className="mt-4 flex flex-wrap gap-3 text-xs">
        <a
          href={`${REPO}/${chapter.source}`}
          target="_blank"
          rel="noreferrer"
          className="code-font rounded-md border border-slate-700/80 px-3 py-1.5 text-slate-300 transition hover:border-slate-500 hover:text-white"
        >
          {chapter.source}
        </a>
        {chapter.blog && (
          <a
            href={chapter.blog}
            target="_blank"
            rel="noreferrer"
            className="rounded-md border border-slate-700/80 px-3 py-1.5 text-slate-300 transition hover:border-slate-500 hover:text-white"
          >
            블로그 글 ↗
          </a>
        )}
      </div>
    </header>
  );
}
