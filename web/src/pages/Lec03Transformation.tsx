import { useEffect, useMemo, useRef, useState } from "react";
import { findChapter } from "../chapters";
import ChapterHeader from "../components/ChapterHeader";
import DemoAbout from "../components/DemoAbout";
import DemoParams from "../components/DemoParams";
import PointCloudViewer from "../components/PointCloudViewer";
import Slider from "../components/Slider";
import DataSourcePicker from "../components/DataSourcePicker";
import { useT } from "../i18n";
import {
  buildTransform,
  formatEigenMatrix4,
  transformCloud,
} from "../lib/filters/transform";
import { voxelGrid } from "../lib/filters/voxelGrid";
import { emptyCloud, type PointCloud } from "../lib/types";

export default function Lec03Transformation() {
  const chapter = findChapter("transformation")!;
  const t = useT();

  const [raw, setRaw] = useState<PointCloud>(emptyCloud());
  const [scale, setScale] = useState(1);
  const [tx, setTx] = useState(0);
  const [ty, setTy] = useState(0);
  const [tz, setTz] = useState(0);
  const [rx, setRx] = useState(0);
  const [ry, setRy] = useState(0);
  const [rz, setRz] = useState(0);
  const lastSrcId = useRef<PointCloud | null>(null);

  // Keep dense KITTI responsive — voxelize for visualization purposes.
  const src = useMemo(() => {
    if (raw.count <= 80_000) return raw;
    let leaf = 0.15 * scale;
    let v = voxelGrid(raw, leaf);
    while (v.count > 80_000 && leaf < 5 * scale) {
      leaf *= 1.4;
      v = voxelGrid(raw, leaf);
    }
    return v;
  }, [raw, scale]);

  // First time we see a cloud (preset switch, file drop), randomize a small
  // translation so the demo lands in an interesting state immediately.
  useEffect(() => {
    if (src.count === 0 || lastSrcId.current === src) return;
    lastSrcId.current = src;
    const r = 4 * scale;
    setTx(round((Math.random() - 0.5) * r, 2));
    setTy(round((Math.random() - 0.5) * r, 2));
    setTz(round((Math.random() - 0.5) * r * 0.25, 2));
    setRx(0);
    setRy(0);
    setRz(0);
  }, [src, scale]);

  const matrix = useMemo(
    () => buildTransform(tx, ty, tz, rx, ry, rz),
    [tx, ty, tz, rx, ry, rz],
  );
  const transformed = useMemo(
    () => transformCloud(src, matrix),
    [src, matrix],
  );
  const ptSize = 0.05 * scale;

  const randomize = () => {
    const r = 4 * scale;
    setTx(round((Math.random() - 0.5) * r, 2));
    setTy(round((Math.random() - 0.5) * r, 2));
    setTz(round((Math.random() - 0.5) * r * 0.25, 2));
    setRx(round((Math.random() - 0.5) * 60, 0));
    setRy(round((Math.random() - 0.5) * 60, 0));
    setRz(round((Math.random() - 0.5) * 60, 0));
  };
  const reset = () => {
    setTx(0);
    setTy(0);
    setTz(0);
    setRx(0);
    setRy(0);
    setRz(0);
  };

  return (
    <div className="mx-auto max-w-7xl px-4 py-6 sm:px-6 lg:px-8 lg:py-10">
      <ChapterHeader chapter={chapter} />
      <DemoAbout slug="transformation" />

      <section className="mt-6 grid gap-4 lg:grid-cols-[1fr_18rem]">
        <div className="overflow-hidden rounded-xl border border-[var(--border)]">
          <div className="flex items-center justify-between border-b border-[var(--border)] bg-[color:rgba(10,15,26,0.6)] px-4 py-2 text-[11px]">
            <div className="flex items-center gap-3">
              <Dot color="#f87171" /> {t.lec03.legendSrc}
              <Dot color="#00d4aa" /> {t.lec03.legendOut}
            </div>
            <span className="code-font text-[var(--dim)]">
              {src.count.toLocaleString()} {t.viewer.pointsSuffix}
            </span>
          </div>
          <div className="aspect-[4/3] md:aspect-[16/10] w-full">
            <PointCloudViewer
              layers={[
                { cloud: src, color: "#f87171", size: ptSize, opacity: 0.65 },
                { cloud: transformed, color: "#00d4aa", size: ptSize, opacity: 0.85 },
              ]}
            />
          </div>
        </div>

        <aside className="flex flex-col gap-4 rounded-xl border border-[var(--border)] bg-[var(--surface)] p-5">
          <DemoParams slug="transformation" />
          <DataSourcePicker
            onCloudChange={(c, info) => {
              setRaw(c);
              setScale(info.suggestedScale);
            }}
          />

          <div className="grid grid-cols-2 gap-1.5">
            <button
              onClick={randomize}
              className="code-font rounded-md border border-[var(--border-strong)] bg-[var(--surface-2)] px-2 py-1.5 text-[11px] text-[var(--text)] transition hover:border-[var(--accent)] hover:text-[var(--accent)]"
            >
              {t.lec03.randomize}
            </button>
            <button
              onClick={reset}
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
              <Slider
                label="tx"
                min={-10 * scale}
                max={10 * scale}
                step={0.1 * scale}
                value={tx}
                unit="m"
                onChange={setTx}
              />
              <Slider
                label="ty"
                min={-10 * scale}
                max={10 * scale}
                step={0.1 * scale}
                value={ty}
                unit="m"
                onChange={setTy}
              />
              <Slider
                label="tz"
                min={-5 * scale}
                max={5 * scale}
                step={0.1 * scale}
                value={tz}
                unit="m"
                onChange={setTz}
              />
            </div>
          </div>

          <div>
            <div className="mb-1 text-[11px] uppercase tracking-wider text-[var(--mut)]">
              {t.lec03.rotation}
            </div>
            <div className="flex flex-col gap-3">
              <Slider label="rx" min={-180} max={180} step={1} value={rx} unit="°" onChange={(v) => setRx(Math.round(v))} />
              <Slider label="ry" min={-180} max={180} step={1} value={ry} unit="°" onChange={(v) => setRy(Math.round(v))} />
              <Slider label="rz" min={-180} max={180} step={1} value={rz} unit="°" onChange={(v) => setRz(Math.round(v))} />
            </div>
          </div>

          <pre className="code-font overflow-x-auto rounded-md bg-[var(--surface-2)] p-3 text-[10px] leading-relaxed text-[var(--text)]">
            {formatEigenMatrix4(matrix)}
          </pre>
        </aside>
      </section>
    </div>
  );
}

function round(v: number, digits: number) {
  const f = Math.pow(10, digits);
  return Math.round(v * f) / f;
}

function Dot({ color }: { color: string }) {
  return (
    <span
      className="inline-block h-2 w-2 rounded-full"
      style={{ background: color }}
    />
  );
}
