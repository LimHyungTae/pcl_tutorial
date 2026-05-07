import { NavLink, Outlet } from "react-router-dom";
import { chapters, statusColor } from "../chapters";
import { useT } from "../i18n";
import LocaleToggle from "../components/LocaleToggle";

const REPO_URL = "https://github.com/LimHyungTae/pcl_tutorial";

export default function AppLayout() {
  const t = useT();
  return (
    <div className="grid h-full grid-cols-[18rem_minmax(0,1fr)] grid-rows-[auto_minmax(0,1fr)]">
      <header className="col-span-2 flex items-center justify-between border-b border-[var(--border)] bg-[color:rgba(10,15,26,0.85)] px-6 py-3 backdrop-blur-md">
        <NavLink to="/" className="flex items-center gap-3 group">
          <span className="text-2xl">📍</span>
          <div>
            <div className="code-font text-sm font-bold tracking-wide text-[var(--text-strong)]">
              {t.app.title}
            </div>
            <div className="text-[11px] text-[var(--dim)]">{t.app.tagline}</div>
          </div>
        </NavLink>
        <div className="flex items-center gap-2 text-xs">
          <LocaleToggle />
          <a
            href={REPO_URL}
            target="_blank"
            rel="noreferrer"
            className="rounded-md border border-[var(--border-strong)] bg-[var(--surface)] px-3 py-1.5 text-[var(--dim)] transition hover:border-[var(--accent)] hover:text-[var(--accent)]"
          >
            {t.nav.github}
          </a>
        </div>
      </header>

      <aside className="row-start-2 overflow-y-auto border-r border-[var(--border)] bg-[var(--surface-2)] py-4">
        <div className="code-font px-4 pb-2 text-[10px] font-semibold uppercase tracking-[0.25em] text-[var(--mut)]">
          {t.nav.chapters}
        </div>
        <nav className="flex flex-col">
          {chapters.map((c) => {
            const tc = t.chapters[c.slug as keyof typeof t.chapters];
            return (
              <NavLink
                key={c.slug}
                to={`/${c.slug}`}
                className={({ isActive }) =>
                  [
                    "group relative flex items-start gap-3 px-4 py-2.5 text-[13px] transition",
                    isActive
                      ? "bg-[color:rgba(0,212,170,0.06)] text-[var(--text-strong)]"
                      : "text-[var(--dim)] hover:bg-[color:rgba(255,255,255,0.02)] hover:text-[var(--text-strong)]",
                  ].join(" ")
                }
              >
                {({ isActive }) => (
                  <>
                    {isActive && (
                      <span className="absolute inset-y-0 left-0 w-[2px] bg-gradient-to-b from-[var(--accent)] to-[var(--accent-2)]" />
                    )}
                    <span className="code-font mt-0.5 w-7 shrink-0 text-[11px] text-[var(--mut)]">
                      {c.number.padStart(2, "0")}
                    </span>
                    <span className="min-w-0 flex-1">
                      <span className="block truncate">{tc.title}</span>
                      <span
                        className={`mt-1 inline-flex items-center rounded px-1.5 py-0.5 text-[10px] ring-1 ring-inset ${statusColor[c.status]}`}
                      >
                        {t.status[c.status]}
                      </span>
                    </span>
                  </>
                )}
              </NavLink>
            );
          })}
        </nav>
      </aside>

      <main className="row-start-2 overflow-y-auto">
        <Outlet />
      </main>
    </div>
  );
}
