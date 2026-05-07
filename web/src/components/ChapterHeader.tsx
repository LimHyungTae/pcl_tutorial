import { ChapterMeta, statusColor } from "../chapters";
import { useLocale, useT } from "../i18n";

const REPO = "https://github.com/LimHyungTae/pcl_tutorial/blob/main";

export default function ChapterHeader({ chapter }: { chapter: ChapterMeta }) {
  const t = useT();
  const { locale } = useLocale();
  const tc = t.chapters[chapter.slug as keyof typeof t.chapters];
  return (
    <header className="border-b border-[var(--border)] pb-7 fade-up">
      <div className="flex items-center gap-3 text-[11px]">
        <span className="code-font uppercase tracking-[0.2em] text-[var(--mut)]">
          {t.home.chapterPrefix} {chapter.number.padStart(2, "0")}
        </span>
        <span
          className={`rounded px-1.5 py-0.5 text-[10px] ring-1 ring-inset ${statusColor[chapter.status]}`}
        >
          {t.status[chapter.status]}
        </span>
      </div>
      <h1 className="heading-mono mt-3 text-3xl font-extrabold tracking-tight">
        <span className="gradient-text">{tc.title}</span>
      </h1>
      <p className="mt-2 max-w-3xl text-[14px] leading-relaxed text-[var(--dim)]">
        {tc.subtitle}
      </p>

      <div className="mt-4 flex flex-wrap gap-2 text-xs">
        {chapter.source && (
          <a
            href={`${REPO}/${chapter.source}`}
            target="_blank"
            rel="noreferrer"
            className="code-font rounded-md border border-[var(--border-strong)] bg-[var(--surface)] px-3 py-1.5 text-[var(--dim)] transition hover:border-[var(--accent)] hover:text-[var(--accent)]"
          >
            {chapter.source}
          </a>
        )}
        {chapter.blog && locale !== "en" && (
          <a
            href={chapter.blog}
            target="_blank"
            rel="noreferrer"
            className="rounded-md border border-[var(--border-strong)] bg-[var(--surface)] px-3 py-1.5 text-[var(--dim)] transition hover:border-[var(--accent-2)] hover:text-[var(--accent-2)]"
          >
            {t.nav.blogPost} ↗
          </a>
        )}
      </div>
    </header>
  );
}
