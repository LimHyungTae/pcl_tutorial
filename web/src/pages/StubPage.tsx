import { ChapterMeta, statusColor } from "../chapters";
import ChapterHeader from "../components/ChapterHeader";
import { useT } from "../i18n";

export default function StubPage({ chapter }: { chapter: ChapterMeta }) {
  const t = useT();
  return (
    <div className="mx-auto max-w-5xl px-8 py-10">
      <ChapterHeader chapter={chapter} />

      <div className="mt-8 rounded-xl border border-slate-800/80 bg-slate-900/30 p-8 text-center">
        <span
          className={`inline-flex items-center rounded px-2 py-0.5 text-xs ring-1 ring-inset ${statusColor[chapter.status]}`}
        >
          {t.stub.badge}
        </span>
        <h2 className="mt-4 text-xl font-medium text-slate-100">
          {t.stub.headline}
        </h2>
        <p className="mx-auto mt-2 max-w-md text-sm leading-relaxed text-slate-400">
          {t.stub.body}
        </p>
      </div>
    </div>
  );
}
