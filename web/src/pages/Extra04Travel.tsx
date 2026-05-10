import { useEffect, useMemo, useRef, useState } from "react";
import { findChapter } from "../chapters";
import BibTexBlock, { type BibEntry } from "../components/BibTexBlock";
import CautionBox from "../components/CautionBox";
import ChapterHeader from "../components/ChapterHeader";
import DataSourcePicker, { type PresetId } from "../components/DataSourcePicker";
import DemoAbout from "../components/DemoAbout";
import DemoParams from "../components/DemoParams";
import PointCloudViewer from "../components/PointCloudViewer";
import RangeImageView from "../components/RangeImageView";
import { useT } from "../i18n";
import { ClusterColorMatcher, type Cluster } from "../lib/filters/euclideanCluster";
import { runTravel, travelClusterColor } from "../lib/filters/travel";
import { ensureZUp } from "../lib/axisTransform";
import { SENSOR_BY_PRESET, type SensorConfig } from "../lib/sensorConfig";
import { emptyCloud, type PointCloud } from "../lib/types";

const ENABLED_PRESETS: PresetId[] = ["naverlabs", "kitti"];

const TRAVEL_BIBTEX: BibEntry[] = [
  {
    key: "oh2022travel",
    bibtex: `@article{oh2022travel,
  title={TRAVEL: Traversable ground and above-ground object segmentation using graph representation of 3D LiDAR scans},
  author={Oh, Minho and Jung, Euigon and Lim, Hyungtae and Song, Wonho and Hu, Sumin and Lee, Eungchang Mason and Park, Junghee and Kim, Jaekyung and Lee, Jangwoo and Myung, Hyun},
  journal={IEEE Robotics and Automation Letters},
  volume={7},
  number={3},
  pages={7255--7262},
  year={2022}
}`,
  },
];

export default function Extra04Travel() {
  const chapter = findChapter("travel")!;
  const t = useT();

  const [raw, setRaw] = useState<PointCloud>(emptyCloud());
  const [presetId, setPresetId] = useState<PresetId>("kitti");
  const sensor: SensorConfig | null = SENSOR_BY_PRESET[presetId] ?? null;

  // TRAVEL inherits the Z-up assumption from its Patchwork-based ground
  // pre-pass; normalize per-preset before passing to runTravel.
  const zUpCloud = useMemo(() => ensureZUp(raw, presetId), [raw, presetId]);

  const result = useMemo(() => {
    if (!sensor || zUpCloud.count === 0) return null;
    return runTravel(zUpCloud, sensor);
  }, [zUpCloud, sensor]);

  // Persistent color matcher so cluster colors stay stable when the user
  // re-selects the same preset (avoids flicker on remount).
  const matcherRef = useRef(new ClusterColorMatcher());
  const lastSrcRef = useRef<PointCloud | null>(null);
  useEffect(() => {
    if (lastSrcRef.current !== raw) {
      matcherRef.current.reset();
      lastSrcRef.current = raw;
    }
  }, [raw]);

  const colors = useMemo(() => {
    if (!result) return [] as string[];
    const clusters: Cluster[] = result.clusters.map((c, i) => ({
      cloud: c,
      size: result.clusterPixelCounts[i],
      centroid: result.centroids[i],
    }));
    // matchRadius scaled to sensor — VLP-16 indoor scenes are tighter.
    const matchR = sensor?.name === "VLP-16" ? 1.5 : 3.0;
    return matcherRef.current.assign(clusters, matchR);
  }, [result, sensor]);

  const colorForCluster = useMemo(() => {
    return (id1Based: number) => colors[id1Based - 1] ?? travelClusterColor(id1Based - 1);
  }, [colors]);

  const layers = useMemo(() => {
    if (!result || !sensor) return [];
    const ptSize = sensor.name === "VLP-16" ? 0.06 : 0.14;
    const out = result.clusters.slice(0, 80).map((c, idx) => ({
      cloud: c,
      color: colors[idx] ?? travelClusterColor(idx),
      size: ptSize * 1.25,
    }));
    return [
      // Ground (Patchwork output) as muted background.
      { cloud: result.groundCloud, color: "#1f2937", size: ptSize, opacity: 0.6 },
      // Phantom layer to keep camera framing aware of the full sweep.
      { cloud: result.imagedCloud, color: "#0a0f1a", size: ptSize, opacity: 0, boundsOnly: true },
      ...out,
    ];
  }, [result, colors, sensor]);

  const numClusters = result?.numClusters ?? 0;

  return (
    <div className="mx-auto max-w-7xl px-4 py-6 sm:px-6 lg:px-8 lg:py-10">
      <ChapterHeader chapter={chapter} />
      <DemoAbout slug="travel" />

      <section className="mt-6 grid gap-4 lg:grid-cols-[1fr_18rem]">
        <div className="flex flex-col gap-4">
          <div className="overflow-hidden rounded-xl border border-[var(--border)]">
            <div className="flex items-center justify-between border-b border-[var(--border)] bg-[color:rgba(10,15,26,0.6)] px-4 py-2 text-[11px]">
              <div className="text-[var(--text)]">
                {t.extra04.found.replace("{n}", numClusters.toLocaleString())}
              </div>
              <span className="code-font text-[var(--dim)]">
                sensor: {sensor?.name ?? "—"}
              </span>
            </div>
            <div className="aspect-[4/3] md:aspect-[16/10] w-full">
              <PointCloudViewer layers={layers} />
            </div>
          </div>

          {result && (
            <div className="rounded-xl border border-[var(--border)] bg-[var(--surface)] p-3">
              <RangeImageView
                rows={result.rows}
                cols={result.cols}
                clusterIds={result.imageClusterIds}
                depths={result.depths}
                colorForCluster={colorForCluster}
                numClusters={result.numClusters}
              />
            </div>
          )}
        </div>

        <aside className="flex flex-col gap-5 rounded-xl border border-[var(--border)] bg-[var(--surface)] p-5">
          <DemoParams slug="travel" />
          <DataSourcePicker
            defaultPreset="kitti"
            enabledPresets={ENABLED_PRESETS}
            allowCustom={false}
            onCloudChange={(c, info) => {
              setRaw(c);
              if (info.isPreset) setPresetId(info.name as PresetId);
            }}
          />
          <CautionBox
            title={t.caution.title}
            body={t.caution.travelBody}
            links={[
              { label: "TRAVEL (GitHub)", url: "https://github.com/url-kaist/TRAVEL" },
            ]}
          />
          <pre className="code-font overflow-x-auto rounded-md bg-[var(--surface-2)] p-3 text-[10px] leading-relaxed text-[var(--text)]">
            {`TravelGroundSeg<pcl::PointXYZ> tgs;
TravelObjectSeg<pcl::PointXYZ> aos;
tgs.set_lidar_param(${sensor?.numChannels ?? "—"});
tgs.estimate_ground(cloud, ground, obstacles);
aos.cluster(obstacles, clusters);`}
          </pre>
        </aside>
      </section>

      <BibTexBlock entries={TRAVEL_BIBTEX} />
    </div>
  );
}
