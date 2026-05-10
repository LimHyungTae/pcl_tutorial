import { useT } from "../i18n";
import Slider from "./Slider";

export type PoseOffset = {
  tx: number;
  ty: number;
  tz: number;
  rxDeg: number;
  ryDeg: number;
  rzDeg: number;
};

export const ZERO_POSE: PoseOffset = {
  tx: 0,
  ty: 0,
  tz: 0,
  rxDeg: 0,
  ryDeg: 0,
  rzDeg: 0,
};

/** Default ranges scale with the input cloud's spatial extent (`scale`). */
export function randomPose(scale: number): PoseOffset {
  const r = 4 * scale;
  return {
    tx: round((Math.random() - 0.5) * r, 2),
    ty: round((Math.random() - 0.5) * r, 2),
    tz: round((Math.random() - 0.5) * r * 0.25, 2),
    rxDeg: Math.round((Math.random() - 0.5) * 60),
    ryDeg: Math.round((Math.random() - 0.5) * 60),
    rzDeg: Math.round((Math.random() - 0.5) * 60),
  };
}

type Props = {
  value: PoseOffset;
  onChange: (next: PoseOffset) => void;
  /** Sets the slider min/max range (translation = ±10·scale, rotation = ±180°). */
  scale: number;
  /** Optional hook called after the user clicks Randomize / Reset. Lets the
   *  parent piggyback on those events (e.g. ICP needs to clear its iteration
   *  state when the offset changes via these buttons). */
  onCommit?: (next: PoseOffset) => void;
};

/** Reusable 6-DoF rigid offset controls — six sliders + Randomize / Reset.
 *  Same UX as the Transformation chapter. */
export default function PoseOffsetControls({ value, onChange, scale, onCommit }: Props) {
  const t = useT();
  const set = (patch: Partial<PoseOffset>) => onChange({ ...value, ...patch });

  const fire = (next: PoseOffset) => {
    onChange(next);
    onCommit?.(next);
  };

  return (
    <div className="flex flex-col gap-3">
      <div className="grid grid-cols-2 gap-1.5">
        <button
          onClick={() => fire(randomPose(scale))}
          className="code-font rounded-md border border-[var(--border-strong)] bg-[var(--surface-2)] px-2 py-1.5 text-[11px] text-[var(--text)] transition hover:border-[var(--accent)] hover:text-[var(--accent)]"
        >
          {t.lec03.randomize}
        </button>
        <button
          onClick={() => fire(ZERO_POSE)}
          className="code-font rounded-md border border-[var(--border-strong)] bg-[var(--surface-2)] px-2 py-1.5 text-[11px] text-[var(--dim)] transition hover:text-[var(--text)]"
        >
          {t.lec03.reset}
        </button>
      </div>

      <div>
        <div className="mb-1 text-[11px] uppercase tracking-wider text-[var(--mut)]">
          {t.lec03.translation}
        </div>
        <div className="flex flex-col gap-3">
          <Slider label="tx" min={-10 * scale} max={10 * scale} step={0.1 * scale} value={value.tx} unit="m" onChange={(v) => set({ tx: v })} />
          <Slider label="ty" min={-10 * scale} max={10 * scale} step={0.1 * scale} value={value.ty} unit="m" onChange={(v) => set({ ty: v })} />
          <Slider label="tz" min={-5 * scale} max={5 * scale} step={0.1 * scale} value={value.tz} unit="m" onChange={(v) => set({ tz: v })} />
        </div>
      </div>

      <div>
        <div className="mb-1 text-[11px] uppercase tracking-wider text-[var(--mut)]">
          {t.lec03.rotation}
        </div>
        <div className="flex flex-col gap-3">
          <Slider label="rx" min={-180} max={180} step={1} value={value.rxDeg} unit="°" onChange={(v) => set({ rxDeg: Math.round(v) })} />
          <Slider label="ry" min={-180} max={180} step={1} value={value.ryDeg} unit="°" onChange={(v) => set({ ryDeg: Math.round(v) })} />
          <Slider label="rz" min={-180} max={180} step={1} value={value.rzDeg} unit="°" onChange={(v) => set({ rzDeg: Math.round(v) })} />
        </div>
      </div>
    </div>
  );
}

function round(v: number, digits: number) {
  const f = Math.pow(10, digits);
  return Math.round(v * f) / f;
}
