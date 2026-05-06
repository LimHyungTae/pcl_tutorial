import { useCallback, useState } from "react";
import { parseKittiBin } from "../lib/kitti";
import { parsePcd } from "../lib/pcd";
import type { PointCloud } from "../lib/types";

type Props = {
  onLoad: (cloud: PointCloud, filename: string) => void;
  hint?: string;
};

export default function CloudDropZone({ onLoad, hint }: Props) {
  const [hover, setHover] = useState(false);
  const [error, setError] = useState<string | null>(null);

  const handle = useCallback(
    async (file: File) => {
      setError(null);
      try {
        const buf = await file.arrayBuffer();
        const cloud = file.name.toLowerCase().endsWith(".bin")
          ? parseKittiBin(buf)
          : parsePcd(buf);
        onLoad(cloud, file.name);
      } catch (e) {
        setError(e instanceof Error ? e.message : String(e));
      }
    },
    [onLoad],
  );

  return (
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
        if (f) void handle(f);
      }}
      className={`flex flex-col items-center justify-center gap-2 rounded-xl border border-dashed p-6 text-center text-sm transition ${
        hover
          ? "border-emerald-400/60 bg-emerald-500/5 text-emerald-200"
          : "border-slate-700 bg-slate-900/40 text-slate-400"
      }`}
    >
      <span className="text-2xl">🗂️</span>
      <span>
        <strong className="text-slate-200">.bin / .pcd / .ply</strong> 파일을 여기에
        드래그하거나
      </span>
      <label className="cursor-pointer rounded-md border border-slate-700 px-3 py-1.5 text-xs text-slate-300 hover:border-slate-500 hover:text-white">
        파일 선택
        <input
          type="file"
          accept=".bin,.pcd,.ply"
          className="hidden"
          onChange={(e) => {
            const f = e.target.files?.[0];
            if (f) void handle(f);
            e.target.value = "";
          }}
        />
      </label>
      {hint && <div className="text-xs text-slate-500">{hint}</div>}
      {error && <div className="text-xs text-rose-400">⚠️ {error}</div>}
    </div>
  );
}
