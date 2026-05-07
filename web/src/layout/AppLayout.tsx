import { useEffect, useState } from "react";
import { NavLink, Outlet, useLocation } from "react-router-dom";
import { chapters, statusColor } from "../chapters";
import { useT } from "../i18n";
import LocaleToggle from "../components/LocaleToggle";

const REPO_URL = "https://github.com/LimHyungTae/pcl_tutorial";

export default function AppLayout() {
  const t = useT();
  const [drawerOpen, setDrawerOpen] = useState(false);
  const location = useLocation();

  // Close the mobile drawer whenever the route changes.
  useEffect(() => {
    setDrawerOpen(false);
  }, [location.pathname]);

  // Lock body scroll while the drawer is open.
  useEffect(() => {
    if (typeof document === "undefined") return;
    const prev = document.body.style.overflow;
    if (drawerOpen) document.body.style.overflow = "hidden";
    return () => {
      document.body.style.overflow = prev;
    };
  }, [drawerOpen]);

  return (
    <>
      <div className="grid h-full grid-rows-[auto_minmax(0,1fr)] md:grid-cols-[18rem_minmax(0,1fr)]">
        <header className="flex items-center justify-between gap-2 border-b border-[var(--border)] bg-[color:rgba(10,15,26,0.85)] px-3 py-3 backdrop-blur-md md:col-span-2 md:px-6">
          <div className="flex min-w-0 items-center gap-2">
            <button
              type="button"
              className="md:hidden inline-flex h-9 w-9 items-center justify-center rounded-md border border-[var(--border-strong)] bg-[var(--surface)] text-[var(--text)] transition hover:border-[var(--accent)] hover:text-[var(--accent)]"
              onClick={() => setDrawerOpen(true)}
              aria-label="Open chapters"
            >
              <HamburgerIcon />
            </button>
            <NavLink to="/" className="group flex min-w-0 items-center gap-2 sm:gap-3">
              <span className="text-2xl">📍</span>
              <div className="min-w-0">
                <div className="code-font truncate text-sm font-bold tracking-wide text-[var(--text-strong)]">
                  {t.app.title}
                </div>
                <div className="hidden truncate text-[11px] text-[var(--dim)] sm:block">
                  {t.app.tagline}
                </div>
              </div>
            </NavLink>
          </div>
          <div className="flex shrink-0 items-center gap-2 text-xs">
            <LocaleToggle />
            <a
              href={REPO_URL}
              target="_blank"
              rel="noreferrer"
              className="hidden rounded-md border border-[var(--border-strong)] bg-[var(--surface)] px-3 py-1.5 text-[var(--dim)] transition hover:border-[var(--accent)] hover:text-[var(--accent)] sm:inline-block"
            >
              {t.nav.github}
            </a>
            <a
              href={REPO_URL}
              target="_blank"
              rel="noreferrer"
              aria-label="GitHub"
              className="inline-flex h-9 w-9 items-center justify-center rounded-md border border-[var(--border-strong)] bg-[var(--surface)] text-[var(--dim)] transition hover:border-[var(--accent)] hover:text-[var(--accent)] sm:hidden"
            >
              <GitHubIcon />
            </a>
          </div>
        </header>

        {/* Desktop sidebar */}
        <aside className="row-start-2 hidden overflow-y-auto border-r border-[var(--border)] bg-[var(--surface-2)] py-4 md:block">
          <div className="code-font px-4 pb-2 text-[10px] font-semibold uppercase tracking-[0.25em] text-[var(--mut)]">
            {t.nav.chapters}
          </div>
          <ChapterList />
        </aside>

        <main className="row-start-2 overflow-y-auto md:col-start-2">
          <Outlet />
        </main>
      </div>

      {/* Mobile drawer overlay */}
      <div
        className={`fixed inset-0 z-40 bg-black/60 backdrop-blur-sm transition-opacity md:hidden ${
          drawerOpen ? "opacity-100" : "pointer-events-none opacity-0"
        }`}
        onClick={() => setDrawerOpen(false)}
        aria-hidden="true"
      />
      {/* Mobile drawer panel */}
      <aside
        className={`fixed inset-y-0 left-0 z-50 w-72 overflow-y-auto border-r border-[var(--border)] bg-[var(--surface-2)] py-4 shadow-2xl transition-transform md:hidden ${
          drawerOpen ? "translate-x-0" : "-translate-x-full"
        }`}
        aria-hidden={!drawerOpen}
      >
        <div className="flex items-center justify-between px-4 pb-2">
          <div className="code-font text-[10px] font-semibold uppercase tracking-[0.25em] text-[var(--mut)]">
            {t.nav.chapters}
          </div>
          <button
            type="button"
            className="inline-flex h-8 w-8 items-center justify-center rounded-md border border-[var(--border-strong)] bg-[var(--surface)] text-[var(--dim)] transition hover:text-[var(--text)]"
            onClick={() => setDrawerOpen(false)}
            aria-label="Close"
          >
            ×
          </button>
        </div>
        <ChapterList onNavigate={() => setDrawerOpen(false)} />
      </aside>
    </>
  );
}

function ChapterList({ onNavigate }: { onNavigate?: () => void }) {
  const t = useT();
  return (
    <nav className="flex flex-col">
      {chapters.map((c) => {
        const tc = t.chapters[c.slug as keyof typeof t.chapters];
        return (
          <NavLink
            key={c.slug}
            to={`/${c.slug}`}
            onClick={onNavigate}
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
  );
}

function HamburgerIcon() {
  return (
    <svg
      width="18"
      height="18"
      viewBox="0 0 24 24"
      fill="none"
      stroke="currentColor"
      strokeWidth="2"
      strokeLinecap="round"
      strokeLinejoin="round"
    >
      <path d="M3 6h18M3 12h18M3 18h18" />
    </svg>
  );
}

function GitHubIcon() {
  return (
    <svg width="16" height="16" viewBox="0 0 24 24" fill="currentColor">
      <path d="M12 .5C5.65.5.5 5.65.5 12c0 5.08 3.29 9.39 7.86 10.91.58.1.79-.25.79-.55 0-.27-.01-1-.01-1.95-3.2.69-3.87-1.54-3.87-1.54-.52-1.32-1.27-1.67-1.27-1.67-1.04-.71.08-.7.08-.7 1.15.08 1.76 1.18 1.76 1.18 1.02 1.76 2.69 1.25 3.34.96.1-.74.4-1.25.72-1.54-2.55-.29-5.24-1.28-5.24-5.69 0-1.26.45-2.29 1.18-3.1-.12-.29-.51-1.46.11-3.04 0 0 .96-.31 3.16 1.18.92-.26 1.9-.39 2.88-.39.98 0 1.96.13 2.88.39 2.2-1.49 3.16-1.18 3.16-1.18.62 1.58.23 2.75.11 3.04.74.81 1.18 1.84 1.18 3.1 0 4.42-2.69 5.4-5.25 5.68.41.36.78 1.06.78 2.14 0 1.55-.01 2.79-.01 3.17 0 .31.21.66.8.55 4.56-1.52 7.85-5.83 7.85-10.91C23.5 5.65 18.35.5 12 .5z" />
    </svg>
  );
}
