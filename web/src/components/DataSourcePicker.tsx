import { useEffect, useMemo, useRef, useState } from "react";
import { useT } from "../i18n";
import { asset } from "../lib/useCloud";
import { loadCloud, parseFromBuffer } from "../lib/pcd";
import { emptyCloud, type PointCloud } from "../lib/types";

export type PresetId = "kitti" | "naverlabs" | "bunny";

const PRESETS: Record<PresetId, { url: string; suggestedScale: number }> = {
  kitti: { url: "data/kitti00_000000.bin", suggestedScale: 1 },
  naverlabs: { url: "data/naverlabs_vel16.pcd", suggestedScale: 1 },
  bunny: { url: "data/bun_zipper_res3.ply", suggestedScale: 0.01 },
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

  const presetIds = useMemo(() => Object.keys(PRESETS) as PresetId[], []);

  return (
    <div>
      <div className="mb-1.5 flex items-baseline justify-between">
        <span className="text-sm text-slate-200">{t.source.label}</span>
      </div>
      <div className="grid grid-cols-3 gap-1.5">
        {presetIds.map((id) => (
          <button
            key={id}
            onClick={() => pick(id)}
            className={`rounded-md border px-2 py-2 text-left transition ${
              active === id
                ? "border-emerald-500/50 bg-emerald-500/10 text-emerald-100"
                : "border-slate-700 bg-slate-900/40 text-slate-300 hover:border-slate-500 hover:text-white"
            }`}
          >
            <div className="text-xs font-medium">{t.source.presets[id].name}</div>
            <div className="mt-0.5 text-[10px] leading-tight text-slate-500">
              {t.source.presets[id].hint}
            </div>
          </button>
        ))}
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
