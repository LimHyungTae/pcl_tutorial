type Props = {
  title: string;
  body: string;
  links: { label: string; url: string }[];
};

/** Amber-tinted notice used for chapters whose params are sensor-specific
 *  (Patchwork, TRAVEL). Surfaces the upstream GitHub link prominently so
 *  visitors can adjust params themselves. */
export default function CautionBox({ title, body, links }: Props) {
  return (
    <div className="rounded-md border border-[color:rgba(250,204,21,0.4)] bg-[color:rgba(250,204,21,0.06)] px-3 py-2.5">
      <div className="flex items-start gap-2">
        <span aria-hidden="true" className="text-[#facc15]">⚠</span>
        <div className="min-w-0 flex-1">
          <div className="text-[12px] font-semibold text-[#facc15]">{title}</div>
          <p className="mt-1 text-[12px] leading-relaxed text-[var(--text)]">{body}</p>
          {links.length > 0 && (
            <div className="mt-2 flex flex-wrap gap-1.5">
              {links.map((l) => (
                <a
                  key={l.url}
                  href={l.url}
                  target="_blank"
                  rel="noreferrer"
                  className="code-font rounded border border-[color:rgba(250,204,21,0.35)] bg-[color:rgba(250,204,21,0.08)] px-1.5 py-0.5 text-[10px] text-[#facc15] hover:bg-[color:rgba(250,204,21,0.15)]"
                >
                  {l.label} ↗
                </a>
              ))}
            </div>
          )}
        </div>
      </div>
    </div>
  );
}
