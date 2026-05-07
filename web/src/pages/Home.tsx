import { Link } from "react-router-dom";
import { chapters, statusColor } from "../chapters";
import { useT } from "../i18n";

export default function Home() {
  const t = useT();
  return (
    <div className="mx-auto max-w-5xl px-8 pt-14 pb-16">
      <header className="mb-12 text-center fade-up">
        <div className="lab-tag">{t.home.eyebrow}</div>
        <h1 className="heading-mono mt-4 text-4xl font-extrabold leading-[1.1] sm:text-5xl">
          <span className="gradient-text">
            {t.home.headline1} {t.home.headlineEm} {t.home.headline2}
          </span>
        </h1>
        <p className="mx-auto mt-5 max-w-2xl text-[15px] leading-relaxed text-[var(--dim)]">
          {t.home.intro}
        </p>
        <div className="pill mt-6">PCL · ROS · LiDAR · SLAM</div>
      </header>

      <div className="mb-6 flex flex-wrap gap-x-4 gap-y-2 text-xs">
        <Legend label={t.status.interactive} dot="#00d4aa" hint={t.legend.interactive} />
        <Legend label={t.status.precomputed} dot="#4da6ff" hint={t.legend.precomputed} />
        <Legend label={t.status.stub} dot="#475569" hint={t.legend.stub} />
      </div>

      <div className="grid gap-3 sm:grid-cols-2">
        {chapters.map((c, i) => {
          const tc = t.chapters[c.slug as keyof typeof t.chapters];
          return (
            <Link
              key={c.slug}
              to={`/${c.slug}`}
              className="app-card fade-up flex flex-col gap-3 px-5 py-4"
              style={{ animationDelay: `${0.04 * i}s` }}
            >
              <div className="flex items-start justify-between gap-3">
                <div className="min-w-0">
                  <div className="code-font text-[11px] uppercase tracking-wider text-[var(--mut)]">
                    {t.home.chapterPrefix} {c.number}
                  </div>
                  <div className="code-font mt-1 truncate text-[15px] font-bold text-[var(--text-strong)]">
                    {tc.title}
                  </div>
                </div>
                <span
                  className={`shrink-0 rounded px-2 py-0.5 text-[10px] font-medium ring-1 ring-inset ${statusColor[c.status]}`}
                >
                  {t.status[c.status]}
                </span>
              </div>
              <p className="text-[13px] leading-relaxed text-[var(--dim)]">
                {tc.subtitle}
              </p>
            </Link>
          );
        })}
      </div>
    </div>
  );
}

function Legend({
  label,
  hint,
  dot,
}: {
  label: string;
  hint: string;
  dot: string;
}) {
  return (
    <span className="inline-flex items-center gap-2 text-[var(--dim)]">
      <span
        className="inline-block h-1.5 w-1.5 rounded-full"
        style={{ background: dot }}
      />
      <span className="text-[var(--text)]">{label}</span>
      <span>· {hint}</span>
    </span>
  );
}
