import { useMemo, useState } from "react";
import * as THREE from "three";
import { findChapter } from "../chapters";
import BibTexBlock, { type BibEntry } from "../components/BibTexBlock";
import CautionBox from "../components/CautionBox";
import ChapterHeader from "../components/ChapterHeader";
import DataSourcePicker, { type PresetId } from "../components/DataSourcePicker";
import DemoAbout from "../components/DemoAbout";
import DemoParams from "../components/DemoParams";
import PointCloudViewer from "../components/PointCloudViewer";
import { useT } from "../i18n";
import { regionColor, runPatchwork, unpackPatch } from "../lib/filters/patchwork";
import { ensureZUp } from "../lib/axisTransform";
import { SENSOR_BY_PRESET, type SensorConfig } from "../lib/sensorConfig";
import { emptyCloud, type PointCloud } from "../lib/types";

const ENABLED_PRESETS: PresetId[] = ["naverlabs", "kitti"];

const PATCHWORK_BIBTEX: BibEntry[] = [
  {
    key: "lim2021patchwork",
    bibtex: `@article{lim2021patchwork,
  title={Patchwork: Concentric zone-based region-wise ground segmentation with ground likelihood estimation using a 3D LiDAR sensor},
  author={Lim, Hyungtae and Oh, Minho and Myung, Hyun},
  journal={IEEE Robotics and Automation Letters},
  volume={6},
  number={4},
  pages={6458--6465},
  year={2021}
}`,
  },
  {
    key: "lee2022patchwork++",
    bibtex: `@inproceedings{lee2022patchwork++,
  title={Patchwork++: Fast and robust ground segmentation solving partial under-segmentation using 3D point cloud},
  author={Lee, Seungjae and Lim, Hyungtae and Myung, Hyun},
  booktitle={2022 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)},
  pages={13276--13283},
  year={2022}
}`,
  },
];

export default function Extra03Patchwork() {
  const chapter = findChapter("patchwork")!;
  const t = useT();

  const [raw, setRaw] = useState<PointCloud>(emptyCloud());
  const [presetId, setPresetId] = useState<PresetId>("kitti");
  const sensor: SensorConfig | null = SENSOR_BY_PRESET[presetId] ?? null;

  // Patchwork assumes Z-up; some presets (NaverLabs) ship Z-down.
  // ensureZUp normalizes per-preset before we run the algorithm.
  const zUpCloud = useMemo(() => ensureZUp(raw, presetId), [raw, presetId]);

  const result = useMemo(() => {
    if (!sensor || zUpCloud.count === 0) return null;
    return runPatchwork(zUpCloud, sensor);
  }, [zUpCloud, sensor]);

  // Per-vertex color buffer for the ground cloud — every point gets a
  // unique (zone, ring, sector) patch color so individual CZM patches are
  // distinguishable, not just rings.
  const groundColors = useMemo(() => {
    if (!result || !sensor) return new Float32Array(0);
    const tmp = new THREE.Color();
    const out = new Float32Array(result.groundCloud.count * 3);
    for (let i = 0; i < result.groundCloud.count; i++) {
      const { zone, ring, sector } = unpackPatch(result.groundRegionIds[i]);
      tmp.setStyle(regionColor(zone, ring, sector, sensor.czm));
      out[i * 3] = tmp.r;
      out[i * 3 + 1] = tmp.g;
      out[i * 3 + 2] = tmp.b;
    }
    return out;
  }, [result, sensor]);

  const groundCount = result?.groundCloud.count ?? 0;
  const nonGroundCount = result?.nonGroundCloud.count ?? 0;
  const numPatches = result?.uniqueRegions.length ?? 0;

  const layers = useMemo(() => {
    if (!result || !sensor) return [];
    const ptSize = sensor.name === "VLP-16" ? 0.06 : 0.12;
    return [
      // Non-ground in muted slate so the colored ground stays readable.
      { cloud: result.nonGroundCloud, color: "#475569", size: ptSize, opacity: 0.55 },
      { cloud: result.groundCloud, color: groundColors, size: ptSize * 1.2 },
    ];
  }, [result, groundColors, sensor]);

  const chip = chapter.status;
  void chip;

  return (
    <div className="mx-auto max-w-7xl px-4 py-6 sm:px-6 lg:px-8 lg:py-10">
      <ChapterHeader chapter={chapter} />
      <DemoAbout slug="patchwork" />

      <section className="mt-6 grid gap-4 lg:grid-cols-[1fr_18rem]">
        <div className="overflow-hidden rounded-xl border border-[var(--border)]">
          <div className="flex flex-wrap items-center justify-between gap-2 border-b border-[var(--border)] bg-[color:rgba(10,15,26,0.6)] px-4 py-2 text-[11px]">
            <div className="flex items-center gap-3 text-[var(--text)]">
              <span>
                <span className="text-[var(--accent)]">{groundCount.toLocaleString()}</span> ground
              </span>
              <span className="text-[var(--mut)]">·</span>
              <span>
                <span className="text-[var(--text-strong)]">{nonGroundCount.toLocaleString()}</span> non-ground
              </span>
              <span className="text-[var(--mut)]">·</span>
              <span>
                <span className="text-[var(--text-strong)]">{numPatches}</span> patches
              </span>
            </div>
            <span className="code-font text-[var(--dim)]">
              sensor: {sensor?.name ?? "—"}
            </span>
          </div>
          <div className="aspect-[4/3] md:aspect-[16/10] w-full">
            <PointCloudViewer layers={layers} />
          </div>
        </div>

        <aside className="flex flex-col gap-5 rounded-xl border border-[var(--border)] bg-[var(--surface)] p-5">
          <DemoParams slug="patchwork" />
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
            body={t.caution.patchworkBody}
            links={[
              { label: "Patchwork (GitHub)", url: "https://github.com/LimHyungTae/patchwork" },
              { label: "Patchwork++", url: "https://github.com/url-kaist/patchwork-plusplus" },
            ]}
          />
          <pre className="code-font overflow-x-auto rounded-md bg-[var(--surface-2)] p-3 text-[10px] leading-relaxed text-[var(--text)]">
            {`PatchWork<pcl::PointXYZ> pw;
pw.set_sensor_height(${sensor?.czm.sensorHeight ?? "—"});
pw.set_zone_boundaries(
    {${sensor?.czm.zoneBoundaries.join(", ") ?? "—"}});
pw.set_uprightness_thr(${sensor?.czm.uprightnessThr ?? "—"});
pw.estimate_ground(cloud, ground, nonground);`}
          </pre>
        </aside>
      </section>

      <BibTexBlock entries={PATCHWORK_BIBTEX} />
    </div>
  );
}
