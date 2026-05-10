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
import { voxelGrid } from "../lib/filters/voxelGrid";
import { emptyCloud, type PointCloud } from "../lib/types";

export default function Lec05Voxelization() {
  const chapter = findChapter("voxelization")!;
  const t = useT();

  const [src, setSrc] = useState<PointCloud>(emptyCloud());
  const [scale, setScale] = useState(1);
  const [leaf, setLeaf] = useState(0.4);

  const filtered = useMemo(() => voxelGrid(src, leaf), [src, leaf]);
  const reduction =
    src.count > 0 ? (1 - filtered.count / src.count) * 100 : 0;
  const ptSize = 0.05 * scale;
  const sync = useMemo(() => new CameraSyncStore(), []);

  return (
    <div className="mx-auto max-w-7xl px-4 py-6 sm:px-6 lg:px-8 lg:py-10">
      <ChapterHeader chapter={chapter} />
      <DemoAbout slug="voxelization" />

      <section className="mt-6 grid gap-4 lg:grid-cols-[1fr_18rem]">
        <BeforeAfterPanel
          beforeMeta={`${src.count.toLocaleString()} ${t.viewer.pointsSuffix}`}
          afterMeta={`${filtered.count.toLocaleString()} ${t.viewer.pointsSuffix} (-${reduction.toFixed(1)}%)`}
          before={
            <PointCloudViewer
              sync={sync}
              layers={[
                { cloud: src, color: "#f87171", size: ptSize },
                // bounds anchor → keeps both panes' bbox identical so framing stays in sync
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
          <DemoParams slug="voxelization" />
          <DataSourcePicker
            onCloudChange={(c, info) => {
              setSrc(c);
              setScale(info.suggestedScale);
              setLeaf(0.4 * info.suggestedScale);
            }}
          />
          <Slider
            label={t.lec05.leafSize}
            min={0.001 * scale}
            max={2 * scale}
            step={0.005 * scale}
            value={leaf}
            unit="m"
            hint={t.lec05.leafHint}
            onChange={setLeaf}
          />
          <pre className="code-font overflow-x-auto rounded-md bg-slate-950/60 p-3 text-[11px] leading-relaxed text-slate-300">
            {`pcl::VoxelGrid<pcl::PointXYZ> vg;
vg.setInputCloud(src);
vg.setLeafSize(${leaf.toFixed(4)},
               ${leaf.toFixed(4)},
               ${leaf.toFixed(4)});
vg.filter(*out);`}
          </pre>
        </aside>
      </section>
    </div>
  );
}
