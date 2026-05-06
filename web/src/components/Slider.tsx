type Props = {
  label: string;
  min: number;
  max: number;
  step: number;
  value: number;
  unit?: string;
  hint?: string;
  onChange: (v: number) => void;
};

export default function Slider({
  label,
  min,
  max,
  step,
  value,
  unit,
  hint,
  onChange,
}: Props) {
  return (
    <label className="block">
      <div className="mb-1.5 flex items-baseline justify-between gap-3">
        <span className="text-sm text-slate-200">{label}</span>
        <span className="code-font text-xs text-slate-400">
          {value}
          {unit ? ` ${unit}` : ""}
        </span>
      </div>
      <input
        type="range"
        min={min}
        max={max}
        step={step}
        value={value}
        onChange={(e) => onChange(parseFloat(e.target.value))}
        className="h-1.5 w-full cursor-pointer appearance-none rounded-full bg-slate-800 accent-emerald-400"
      />
      {hint && <div className="mt-1 text-xs text-slate-500">{hint}</div>}
    </label>
  );
}
