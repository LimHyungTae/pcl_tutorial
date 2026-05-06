import { useMemo, useState } from "react";
import { findChapter } from "../chapters";
import ChapterHeader from "../components/ChapterHeader";
import BeforeAfterPanel from "../components/BeforeAfterPanel";
import PointCloudViewer from "../components/PointCloudViewer";
import Slider from "../components/Slider";
import CloudDropZone from "../components/CloudDropZone";
import { asset, useCloudFromUrl } from "../lib/useCloud";
import { voxelGrid } from "../lib/filters/voxelGrid";
import type { PointCloud } from "../lib/types";

export default function Lec05Voxelization() {
  const chapter = findChapter("lec05")!;
  const initial = useCloudFromUrl(asset("data/kitti00_000000.bin"));
  const [override, setOverride] = useState<PointCloud | null>(null);
  const [overrideName, setOverrideName] = useState<string | null>(null);
  const [leaf, setLeaf] = useState(0.4);

  const src = override ?? initial.cloud;

  const filtered = useMemo(() => voxelGrid(src, leaf), [src, leaf]);

  const reduction =
    src.count > 0 ? (1 - filtered.count / src.count) * 100 : 0;

  return (
    <div className="mx-auto max-w-7xl px-8 py-10">
      <ChapterHeader chapter={chapter} />

      <section className="mt-6 grid gap-4 lg:grid-cols-[1fr_18rem]">
        <BeforeAfterPanel
          beforeMeta={`${src.count.toLocaleString()} pts`}
          afterMeta={`${filtered.count.toLocaleString()} pts (-${reduction.toFixed(1)}%)`}
          before={
            <PointCloudViewer
              layers={[{ cloud: src, color: "#f87171", size: 0.06 }]}
            />
          }
          after={
            <PointCloudViewer
              layers={[{ cloud: filtered, color: "#34d399", size: 0.06 }]}
            />
          }
        />

        <aside className="flex flex-col gap-6 rounded-xl border border-slate-800/80 bg-slate-900/40 p-5">
          <Slider
            label="leaf size"
            min={0.05}
            max={2}
            step={0.05}
            value={leaf}
            unit="m"
            hint="Voxel 한 변의 길이. 클수록 더 거칠게 다운샘플."
            onChange={setLeaf}
          />
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
            {`pcl::VoxelGrid<pcl::PointXYZ> vg;
vg.setInputCloud(src);
vg.setLeafSize(${leaf}, ${leaf}, ${leaf});
vg.filter(*out);`}
          </pre>
        </aside>
      </section>

      {initial.error && (
        <div className="mt-6 rounded-md border border-rose-500/30 bg-rose-500/10 px-3 py-2 text-sm text-rose-200">
          기본 입력 로드 실패: {initial.error} — 위 드롭존으로 직접 파일을
          올리시면 동작합니다.
        </div>
      )}
    </div>
  );
}
