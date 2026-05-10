type Kind = "idle" | "iterating" | "ok" | "warn" | "fail";

type Props = {
  kind: Kind;
  text: string;
  detail?: string;
};

const PALETTE: Record<Kind, string> = {
  idle: "border-[var(--border)] bg-[color:rgba(10,15,26,0.4)] text-[var(--mut)]",
  iterating: "border-sky-400/40 bg-sky-500/[.06] text-sky-200",
  ok: "border-emerald-400/40 bg-emerald-500/[.06] text-emerald-200",
  warn: "border-amber-400/40 bg-amber-500/[.06] text-amber-200",
  fail: "border-rose-400/40 bg-rose-500/[.06] text-rose-200",
};

const ICON: Record<Kind, string> = {
  idle: "·",
  iterating: "⏳",
  ok: "✓",
  warn: "⚠",
  fail: "✗",
};

/** Single-line success / warn / fail badge that lives directly under the
 *  point-cloud viewer. Kept generic so registration chapters (ICP, GenZ-ICP,
 *  KISS-Matcher) can all reuse it. */
export default function RegistrationStatus({ kind, text, detail }: Props) {
  return (
    <div className={`flex items-start gap-2 border-t px-4 py-2 text-[12px] ${PALETTE[kind]}`}>
      <span aria-hidden="true" className="leading-tight">{ICON[kind]}</span>
      <div className="min-w-0 flex-1">
        <span className="code-font">{text}</span>
        {detail && <div className="code-font mt-0.5 text-[11px] opacity-75">{detail}</div>}
      </div>
    </div>
  );
}

export type RegistrationStatusKind = Kind;
