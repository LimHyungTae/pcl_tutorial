import { useMemo, useState } from "react";
import { findChapter } from "../chapters";
import ChapterHeader from "../components/ChapterHeader";
import DemoAbout from "../components/DemoAbout";
import DemoParams from "../components/DemoParams";
import BeforeAfterPanel from "../components/BeforeAfterPanel";
import PointCloudViewer from "../components/PointCloudViewer";
import Slider from "../components/Slider";
import DataSourcePicker from "../components/DataSourcePicker";
import { useT } from "../i18n";
import { CameraSyncStore } from "../lib/cameraSync";
import { passThrough, type Axis } from "../lib/filters/passThrough";
import { emptyCloud, type PointCloud } from "../lib/types";

export default function Lec06PassThrough() {
  const chapter = findChapter("pass-through")!;
  const t = useT();

  const [src, setSrc] = useState<PointCloud>(emptyCloud());
  const [scale, setScale] = useState(1);
  const [axis, setAxis] = useState<Axis>("z");
  const [min, setMin] = useState(-2);
  const [max, setMax] = useState(0);
  const [negative, setNegative] = useState(false);

  const filtered = useMemo(
    () => passThrough(src, axis, min, max, negative),
    [src, axis, min, max, negative],
  );
  const ptSize = 0.05 * scale;
  const sync = useMemo(() => new CameraSyncStore(), []);

  return (
    <div className="mx-auto max-w-7xl px-4 py-6 sm:px-6 lg:px-8 lg:py-10">
      <ChapterHeader chapter={chapter} />
      <DemoAbout slug="pass-through" />

      <section className="mt-6 grid gap-4 lg:grid-cols-[1fr_18rem]">
        <BeforeAfterPanel
          beforeMeta={`${src.count.toLocaleString()} ${t.viewer.pointsSuffix}`}
          afterMeta={`${filtered.count.toLocaleString()} ${t.viewer.pointsSuffix}`}
          before={
            <PointCloudViewer
              sync={sync}
              layers={[
                { cloud: src, color: "#f87171", size: ptSize },
                { cloud: filtered, color: "#000", boundsOnly: true },
              ]}
            />
          }
          after={
            <PointCloudViewer
              sync={sync}
              layers={[
                { cloud: src, color: "#000", boundsOnly: true },
                { cloud: filtered, color: "#34d399", size: ptSize },
              ]}
            />
          }
        />

        <aside className="flex flex-col gap-5 rounded-xl border border-[var(--border)] bg-[var(--surface)] p-5">
          <DemoParams slug="pass-through" />
          <DataSourcePicker
            onCloudChange={(c, info) => {
              setSrc(c);
              setScale(info.suggestedScale);
              setMin(-2 * info.suggestedScale);
              setMax(0);
            }}
          />
          <div>
            <div className="mb-1.5 text-sm text-slate-200">{t.lec06.axis}</div>
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
            label={t.lec06.min}
            min={-50 * scale}
            max={50 * scale}
            step={0.05 * scale}
            value={min}
            unit="m"
            onChange={(v) => setMin(Math.min(v, max))}
          />
          <Slider
            label={t.lec06.max}
            min={-50 * scale}
            max={50 * scale}
            step={0.05 * scale}
            value={max}
            unit="m"
            onChange={(v) => setMax(Math.max(v, min))}
          />
          <label className="flex items-start gap-2 text-sm text-slate-300">
            <input
              type="checkbox"
              checked={negative}
              onChange={(e) => setNegative(e.target.checked)}
              className="mt-0.5 h-4 w-4 accent-emerald-400"
            />
            <span>
              {t.lec06.negative}
              <span className="code-font ml-1 text-xs text-slate-500">
                {t.lec06.negativeNote}
              </span>
            </span>
          </label>
          <pre className="code-font overflow-x-auto rounded-md bg-slate-950/60 p-3 text-[11px] leading-relaxed text-slate-300">
            {`pcl::PassThrough<pcl::PointXYZ> pt;
pt.setInputCloud(src);
pt.setFilterFieldName("${axis}");
pt.setFilterLimits(${min.toFixed(2)}, ${max.toFixed(2)});
${negative ? "pt.setNegative(true);\n" : ""}pt.filter(*out);`}
          </pre>
        </aside>
      </section>
    </div>
  );
}
