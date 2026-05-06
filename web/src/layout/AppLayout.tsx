import { NavLink, Outlet } from "react-router-dom";
import { chapters, statusColor, statusLabel } from "../chapters";

const REPO_URL = "https://github.com/LimHyungTae/pcl_tutorial";

export default function AppLayout() {
  return (
    <div className="grid h-full grid-cols-[18rem_minmax(0,1fr)] grid-rows-[auto_minmax(0,1fr)] bg-slate-950 text-slate-100">
      <header className="col-span-2 flex items-center justify-between border-b border-slate-800/80 bg-slate-950/80 px-6 py-3 backdrop-blur">
        <NavLink to="/" className="flex items-center gap-3">
          <span className="text-2xl">📍</span>
          <div>
            <div className="text-sm font-semibold tracking-wide text-slate-100">
              PCL Tutorial
            </div>
            <div className="text-xs text-slate-400">한글 인터랙티브 버전</div>
          </div>
        </NavLink>
        <div className="flex items-center gap-3 text-xs">
          <a
            href={REPO_URL}
            target="_blank"
            rel="noreferrer"
            className="rounded-md border border-slate-700/80 px-3 py-1.5 text-slate-300 transition hover:border-slate-500 hover:text-white"
          >
            GitHub
          </a>
        </div>
      </header>

      <aside className="row-start-2 overflow-y-auto border-r border-slate-800/80 bg-slate-950/60 py-4">
        <div className="px-4 pb-2 text-[11px] font-semibold uppercase tracking-widest text-slate-500">
          Chapters
        </div>
        <nav className="flex flex-col">
          {chapters.map((c) => (
            <NavLink
              key={c.slug}
              to={`/${c.slug}`}
              className={({ isActive }) =>
                [
                  "group relative flex items-start gap-3 px-4 py-2.5 text-sm transition",
                  isActive
                    ? "bg-slate-800/60 text-white"
                    : "text-slate-300 hover:bg-slate-800/30 hover:text-white",
                ].join(" ")
              }
            >
              <span className="code-font mt-0.5 w-8 shrink-0 text-xs text-slate-500">
                {c.number}
              </span>
              <span className="min-w-0 flex-1">
                <span className="block truncate">{c.title}</span>
                <span
                  className={`mt-1 inline-flex items-center rounded px-1.5 py-0.5 text-[10px] ring-1 ring-inset ${statusColor[c.status]}`}
                >
                  {statusLabel[c.status]}
                </span>
              </span>
            </NavLink>
          ))}
        </nav>
      </aside>

      <main className="row-start-2 overflow-y-auto">
        <Outlet />
      </main>
    </div>
  );
}
