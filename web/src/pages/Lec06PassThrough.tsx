import { useMemo, useState } from "react";
import { findChapter } from "../chapters";
import ChapterHeader from "../components/ChapterHeader";
import BeforeAfterPanel from "../components/BeforeAfterPanel";
import PointCloudViewer from "../components/PointCloudViewer";
import Slider from "../components/Slider";
import CloudDropZone from "../components/CloudDropZone";
import { asset, useCloudFromUrl } from "../lib/useCloud";
import { passThrough, type Axis } from "../lib/filters/passThrough";
import type { PointCloud } from "../lib/types";

export default function Lec06PassThrough() {
  const chapter = findChapter("lec06")!;
  const initial = useCloudFromUrl(asset("data/kitti00_000000.bin"));
  const [override, setOverride] = useState<PointCloud | null>(null);
  const [overrideName, setOverrideName] = useState<string | null>(null);

  const [axis, setAxis] = useState<Axis>("z");
  const [min, setMin] = useState(-2);
  const [max, setMax] = useState(0);
  const [negative, setNegative] = useState(false);

  const src = override ?? initial.cloud;

  const filtered = useMemo(
    () => passThrough(src, axis, min, max, negative),
    [src, axis, min, max, negative],
  );

  return (
    <div className="mx-auto max-w-7xl px-8 py-10">
      <ChapterHeader chapter={chapter} />

      <section className="mt-6 grid gap-4 lg:grid-cols-[1fr_18rem]">
        <BeforeAfterPanel
          beforeMeta={`${src.count.toLocaleString()} pts`}
          afterMeta={`${filtered.count.toLocaleString()} pts`}
          before={
            <PointCloudViewer
              layers={[{ cloud: src, color: "#f87171", size: 0.05 }]}
            />
          }
          after={
            <PointCloudViewer
              layers={[{ cloud: filtered, color: "#34d399", size: 0.05 }]}
            />
          }
        />

        <aside className="flex flex-col gap-5 rounded-xl border border-slate-800/80 bg-slate-900/40 p-5">
          <div>
            <div className="mb-1.5 text-sm text-slate-200">axis</div>
            <div className="grid grid-cols-3 overflow-hidden rounded-md border border-slate-700">
              {(["x", "y", "z"] as Axis[]).map((a) => (
                <button
                  key={a}
                  onClick={() => setAxis(a)}
                  className={`code-font py-1.5 text-sm transition ${
                    axis === a
                      ? "bg-emerald-500/15 text-emerald-200"
                      : "text-slate-400 hover:bg-slate-800/60 hover:text-slate-200"
                  }`}
                >
                  {a}
                </button>
              ))}
            </div>
          </div>

          <Slider
            label="min"
            min={-50}
            max={50}
            step={0.5}
            value={min}
            unit="m"
            onChange={(v) => setMin(Math.min(v, max))}
          />
          <Slider
            label="max"
            min={-50}
            max={50}
            step={0.5}
            value={max}
            unit="m"
            onChange={(v) => setMax(Math.max(v, min))}
          />

          <label className="flex items-center gap-2 text-sm text-slate-300">
            <input
              type="checkbox"
              checked={negative}
              onChange={(e) => setNegative(e.target.checked)}
              className="h-4 w-4 accent-emerald-400"
            />
            <span>
              <span className="code-font">setNegative(true)</span> — 범위 바깥 추출
            </span>
          </label>

          <CloudDropZone
            hint={
              overrideName
                ? `로드됨: ${overrideName}`
                : "기본 입력은 KITTI 첫 프레임입니다."
            }
            onLoad={(c, n) => {
              setOverride(c);
              setOverrideName(n);
            }}
          />

          <pre className="code-font overflow-x-auto rounded-md bg-slate-950/60 p-3 text-[11px] leading-relaxed text-slate-300">
            {`pcl::PassThrough<pcl::PointXYZ> pt;
pt.setInputCloud(src);
pt.setFilterFieldName("${axis}");
pt.setFilterLimits(${min}, ${max});
${negative ? "pt.setNegative(true);\n" : ""}pt.filter(*out);`}
          </pre>
        </aside>
      </section>

      {initial.error && (
        <div className="mt-6 rounded-md border border-rose-500/30 bg-rose-500/10 px-3 py-2 text-sm text-rose-200">
          기본 입력 로드 실패: {initial.error}
        </div>
      )}
    </div>
  );
}
