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
      <Pane label={beforeLabel} meta={beforeMeta} dot="#f87171" tint="rgba(248,113,113,0.06)">
        {before}
      </Pane>
      <Pane label={afterLabel} meta={afterMeta} dot="#00d4aa" tint="rgba(0,212,170,0.06)">
        {after}
      </Pane>
    </div>
  );
}

function Pane({
  label,
  meta,
  dot,
  tint,
  children,
}: {
  label: string;
  meta?: string;
  dot: string;
  tint: string;
  children: ReactNode;
}) {
  return (
    <div
      className="overflow-hidden rounded-xl border border-[var(--border)]"
      style={{ background: tint }}
    >
      <div className="flex items-center justify-between border-b border-white/5 bg-[color:rgba(10,15,26,0.4)] px-4 py-2 text-[11px]">
        <div className="flex items-center gap-2">
          <span
            className="inline-block h-2 w-2 rounded-full"
            style={{ background: dot }}
          />
          <span className="font-semibold text-[var(--text)]">{label}</span>
        </div>
        {meta && <span className="code-font text-[var(--dim)]">{meta}</span>}
      </div>
      <div className="aspect-[4/3] w-full">{children}</div>
    </div>
  );
}
