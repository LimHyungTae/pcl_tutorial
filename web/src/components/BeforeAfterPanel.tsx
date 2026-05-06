import { ReactNode } from "react";

type Props = {
  beforeLabel?: string;
  afterLabel?: string;
  beforeMeta?: string;
  afterMeta?: string;
  before: ReactNode;
  after: ReactNode;
};

export default function BeforeAfterPanel({
  beforeLabel = "Before",
  afterLabel = "After",
  beforeMeta,
  afterMeta,
  before,
  after,
}: Props) {
  return (
    <div className="grid grid-cols-1 gap-3 lg:grid-cols-2">
      <Pane label={beforeLabel} meta={beforeMeta} tone="rose">
        {before}
      </Pane>
      <Pane label={afterLabel} meta={afterMeta} tone="emerald">
        {after}
      </Pane>
    </div>
  );
}

function Pane({
  label,
  meta,
  tone,
  children,
}: {
  label: string;
  meta?: string;
  tone: "rose" | "emerald";
  children: ReactNode;
}) {
  const tones = {
    rose: "border-rose-500/30 bg-rose-500/[.04]",
    emerald: "border-emerald-500/30 bg-emerald-500/[.04]",
  } as const;
  const dots = {
    rose: "bg-rose-400",
    emerald: "bg-emerald-400",
  } as const;
  return (
    <div className={`overflow-hidden rounded-xl border ${tones[tone]}`}>
      <div className="flex items-center justify-between border-b border-white/5 bg-slate-950/40 px-4 py-2 text-xs">
        <div className="flex items-center gap-2">
          <span className={`inline-block h-2 w-2 rounded-full ${dots[tone]}`} />
          <span className="font-medium text-slate-200">{label}</span>
        </div>
        {meta && <span className="code-font text-slate-500">{meta}</span>}
      </div>
      <div className="aspect-[4/3] w-full">{children}</div>
    </div>
  );
}
