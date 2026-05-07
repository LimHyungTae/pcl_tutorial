import { useT } from "../i18n";

type Param = { name: string; desc: string; effect: string };

/** Compact "Parameters" list for the sidebar — explains each control + effect. */
export default function DemoParams({ slug }: { slug: string }) {
  const t = useT();
  const c = t.chapters[slug as keyof typeof t.chapters] as
    | { params?: Param[] }
    | undefined;
  if (!c?.params || c.params.length === 0) return null;
  return (
    <div>
      <div className="code-font mb-2 text-[10px] font-semibold uppercase tracking-[0.2em] text-[var(--mut)]">
        {t.demo.parameters}
      </div>
      <ul className="space-y-2 text-[12px] leading-relaxed">
        {c.params.map((p, i) => (
          <li key={i}>
            <span className="code-font font-bold text-[var(--accent)]">
              {p.name}
            </span>
            <span className="text-[var(--text)]"> — {p.desc}</span>
            {p.effect && (
              <span className="text-[var(--accent-2)]"> {p.effect}</span>
            )}
          </li>
        ))}
      </ul>
    </div>
  );
}
