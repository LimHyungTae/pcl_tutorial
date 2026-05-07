import { useEffect, useMemo, useRef, useState } from "react";
import { useT } from "../i18n";
import { asset } from "../lib/useCloud";
import { loadCloud, parseFromBuffer } from "../lib/pcd";
import { emptyCloud, type PointCloud } from "../lib/types";

export type PresetId = "bunny" | "naverlabs" | "kitti";

/** Order here drives the on-screen button order (left → right). */
const PRESET_ORDER: PresetId[] = ["bunny", "naverlabs", "kitti"];

const SCENE_TONES: Record<PresetId, { fg: string; bg: string; bd: string }> = {
  bunny: {
    fg: "#fbbf24",
    bg: "rgba(251, 191, 36, 0.10)",
    bd: "rgba(251, 191, 36, 0.30)",
  },
  naverlabs: {
    fg: "#4da6ff",
    bg: "rgba(77, 166, 255, 0.10)",
    bd: "rgba(77, 166, 255, 0.30)",
  },
  kitti: {
    fg: "#00d4aa",
    bg: "rgba(0, 212, 170, 0.10)",
    bd: "rgba(0, 212, 170, 0.30)",
  },
};

const PRESETS: Record<PresetId, { url: string; suggestedScale: number }> = {
  bunny: { url: "data/bun_zipper_res3.ply", suggestedScale: 0.01 },
  naverlabs: { url: "data/naverlabs_vel16.pcd", suggestedScale: 1 },
  kitti: { url: "data/kitti00_000000.bin", suggestedScale: 1 },
};

type Props = {
  defaultPreset?: PresetId;
  onCloudChange: (cloud: PointCloud, info: { name: string; isPreset: boolean; suggestedScale: number }) => void;
};

export default function DataSourcePicker({
  defaultPreset = "kitti",
  onCloudChange,
}: Props) {
  const t = useT();
  const [active, setActive] = useState<PresetId | "custom">(defaultPreset);
  const [customName, setCustomName] = useState<string | null>(null);
  const [error, setError] = useState<string | null>(null);
  const [hover, setHover] = useState(false);
  const cbRef = useRef(onCloudChange);
  cbRef.current = onCloudChange;

  // Load initial preset.
  useEffect(() => {
    let cancelled = false;
    loadCloud(asset(PRESETS[defaultPreset].url))
      .then((c) => {
        if (!cancelled)
          cbRef.current(c, {
            name: defaultPreset,
            isPreset: true,
            suggestedScale: PRESETS[defaultPreset].suggestedScale,
          });
      })
      .catch((e) => !cancelled && setError(String(e)));
    return () => {
      cancelled = true;
    };
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [defaultPreset]);

  const pick = async (id: PresetId) => {
    setError(null);
    setActive(id);
    try {
      const c = await loadCloud(asset(PRESETS[id].url));
      cbRef.current(c, {
        name: id,
        isPreset: true,
        suggestedScale: PRESETS[id].suggestedScale,
      });
    } catch (e) {
      cbRef.current(emptyCloud(), { name: id, isPreset: true, suggestedScale: 1 });
      setError(e instanceof Error ? e.message : String(e));
    }
  };

  const handleFile = async (f: File) => {
    setError(null);
    try {
      const buf = await f.arrayBuffer();
      const c = parseFromBuffer(buf, f.name);
      setActive("custom");
      setCustomName(f.name);
      cbRef.current(c, { name: f.name, isPreset: false, suggestedScale: 1 });
    } catch (e) {
      setError(e instanceof Error ? e.message : String(e));
    }
  };

  const presetIds = useMemo(() => PRESET_ORDER, []);

  return (
    <div>
      <div className="mb-1.5 flex items-baseline justify-between">
        <span className="text-sm text-slate-200">{t.source.label}</span>
      </div>
      <div className="grid grid-cols-3 gap-1.5">
        {presetIds.map((id) => {
          const isActive = active === id;
          const sceneTone = SCENE_TONES[id];
          return (
            <button
              key={id}
              onClick={() => pick(id)}
              className={`rounded-md border px-2 py-2 text-left transition ${
                isActive
                  ? "border-[color:rgba(0,212,170,0.5)] bg-[color:rgba(0,212,170,0.08)] text-[var(--text-strong)]"
                  : "border-[var(--border-strong)] bg-[var(--surface-2)] text-[var(--dim)] hover:border-[var(--border-strong)] hover:text-[var(--text)]"
              }`}
            >
              <div className="flex items-center gap-1.5">
                <span className="code-font text-[11px] font-bold">
                  {t.source.presets[id].name}
                </span>
                <span
                  className="code-font rounded-sm px-1 py-px text-[9px] font-semibold ring-1 ring-inset"
                  style={{
                    color: sceneTone.fg,
                    background: sceneTone.bg,
                    boxShadow: `inset 0 0 0 1px ${sceneTone.bd}`,
                  }}
                >
                  {t.source.presets[id].scene}
                </span>
              </div>
              <div className="code-font mt-1 text-[9px] leading-tight text-[var(--mut)]">
                {t.source.presets[id].hint}
              </div>
            </button>
          );
        })}
      </div>

      <div
        onDragOver={(e) => {
          e.preventDefault();
          setHover(true);
        }}
        onDragLeave={() => setHover(false)}
        onDrop={(e) => {
          e.preventDefault();
          setHover(false);
          const f = e.dataTransfer.files?.[0];
          if (f) void handleFile(f);
        }}
        className={`mt-2 flex flex-col items-center justify-center gap-1.5 rounded-md border border-dashed px-3 py-3 text-center text-xs transition ${
          hover
            ? "border-emerald-400/60 bg-emerald-500/5 text-emerald-200"
            : active === "custom"
              ? "border-emerald-500/40 bg-emerald-500/[.04] text-emerald-100"
              : "border-slate-700 bg-slate-900/30 text-slate-400"
        }`}
      >
        <span>
          {active === "custom" && customName
            ? `${t.source.using}: ${customName}`
            : t.source.dropHint}
        </span>
        <label className="cursor-pointer rounded-md border border-slate-700 px-2.5 py-1 text-[11px] text-slate-300 hover:border-slate-500 hover:text-white">
          {t.source.pickFile}
          <input
            type="file"
            accept=".bin,.pcd,.ply"
            className="hidden"
            onChange={(e) => {
              const f = e.target.files?.[0];
              if (f) void handleFile(f);
              e.target.value = "";
            }}
          />
        </label>
      </div>

      {error && (
        <div className="mt-2 text-xs text-rose-400">⚠️ {error}</div>
      )}
    </div>
  );
}
