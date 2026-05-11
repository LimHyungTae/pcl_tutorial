#!/usr/bin/env python3
"""
Generate KISS-Matcher demo data for the web tutorial.

For each preset listed in PRESETS, loads the bundled src / tgt PCDs from
materials/, runs KISS-Matcher (`pip install kiss-matcher`) once, and
writes a small JSON the web page reads at runtime.

The web page never re-runs KISS-Matcher; the slider on the page just
re-renders the precomputed correspondences.

Usage
-----
    pip install kiss-matcher numpy
    python tools/gen_kiss_matcher_data.py

Re-run this whenever you swap input scans, add a preset, or re-tune the
voxel size for an existing one.
"""
import json
import sys
from pathlib import Path

import numpy as np

try:
    import kiss_matcher as km
except ImportError:
    sys.exit("`pip install kiss-matcher` first")

try:
    from kiss_matcher.io_utils import read_pcd
except ImportError:
    read_pcd = None

# ---------------------------------------------------------------------------

REPO = Path(__file__).resolve().parent.parent
MATERIALS = REPO / "materials"
OUT_DIR = MATERIALS / "precomputed"

# id            label                   sensor      src                          tgt                          voxel
PRESETS = [
    {
        "id": "vel16",
        "label": "Velodyne VLP-16 (outdoor)",
        "src": MATERIALS / "kiss_vel16_src.pcd",
        "tgt": MATERIALS / "kiss_vel16_tgt.pcd",
        "voxel": 0.3,
    },
    {
        "id": "vel64",
        "label": "Velodyne HDL-64 (KITTI scan-to-scan)",
        "src": MATERIALS / "kiss_vel64_src.pcd",
        "tgt": MATERIALS / "kiss_vel64_tgt.pcd",
        "voxel": 0.3,
    },
    {
        "id": "kitti0007",
        "label": "KITTI seq 00 ↔ seq 07 (cross-trajectory submaps)",
        "src": MATERIALS / "kiss_kitti0007_src.pcd",
        "tgt": MATERIALS / "kiss_kitti0007_tgt.pcd",
        "voxel": 1.5,
    },
    {
        "id": "vbr",
        "label": "VBR Colosseum (handheld submaps)",
        "src": MATERIALS / "kiss_vbr_src.pcd",
        "tgt": MATERIALS / "kiss_vbr_tgt.pcd",
        "voxel": 0.4,
    },
]

MAX_MATCHES_IN_JSON = 4000

# ---------------------------------------------------------------------------


def load_pcd_xyz(path: Path) -> np.ndarray:
    if read_pcd is None:
        raise SystemExit(
            "kiss_matcher.io_utils.read_pcd not found — please install a newer kiss-matcher"
        )
    return np.asarray(read_pcd(str(path)), dtype=np.float32)


def run_one(preset: dict) -> dict:
    print(f"\n=== {preset['id']} ({preset['label']}) ===")
    src_xyz = load_pcd_xyz(preset["src"])
    tgt_xyz = load_pcd_xyz(preset["tgt"])
    print(f"  src / tgt: {len(src_xyz):,} / {len(tgt_xyz):,} points")
    print(f"  voxel_size: {preset['voxel']}")

    config = km.KISSMatcherConfig(preset["voxel"])
    matcher = km.KISSMatcher(config)
    result = matcher.estimate(src_xyz, tgt_xyz)
    matcher.print()

    R_est = np.asarray(result.rotation, dtype=np.float32)
    t_est = np.asarray(result.translation, dtype=np.float32)

    src_kpts, tgt_kpts = matcher.get_keypoints_from_faster_pfh()
    src_kpts = np.asarray(src_kpts, dtype=np.float32)
    tgt_kpts = np.asarray(tgt_kpts, dtype=np.float32)

    init_corres = matcher.get_initial_correspondences()
    final_corres_set = set(map(tuple, matcher.get_final_correspondences()))
    print(
        f"  kpts (src/tgt) : {len(src_kpts):,} / {len(tgt_kpts):,}\n"
        f"  init corres    : {len(init_corres):,}\n"
        f"  final corres   : {len(final_corres_set):,}"
    )

    matches = []
    for s_idx, t_idx in init_corres:
        s_idx, t_idx = int(s_idx), int(t_idx)
        if s_idx < 0 or s_idx >= len(src_kpts) or t_idx < 0 or t_idx >= len(tgt_kpts):
            continue
        is_final = (s_idx, t_idx) in final_corres_set
        matches.append(
            {
                "src": [round(float(v), 4) for v in src_kpts[s_idx]],
                "tgt": [round(float(v), 4) for v in tgt_kpts[t_idx]],
                "isFinal": bool(is_final),
            }
        )

    matches.sort(key=lambda m: 0 if m["isFinal"] else 1)
    if len(matches) > MAX_MATCHES_IN_JSON:
        finals = [m for m in matches if m["isFinal"]]
        non_finals = [m for m in matches if not m["isFinal"]]
        keep_non = MAX_MATCHES_IN_JSON - len(finals)
        if keep_non > 0:
            stride = max(1, len(non_finals) // keep_non)
            non_finals = non_finals[::stride][:keep_non]
        matches = finals + non_finals

    payload = {
        "id": preset["id"],
        "label": preset["label"],
        "srcUrl": f"data/{preset['src'].name}",
        "tgtUrl": f"data/{preset['tgt'].name}",
        "voxelSize": preset["voxel"],
        "estimatedRotation": [float(v) for v in R_est.flatten()],
        "estimatedTranslation": [float(v) for v in t_est],
        "matches": matches,
        "stats": {
            "numSrcPoints": int(len(src_xyz)),
            "numTgtPoints": int(len(tgt_xyz)),
            "numInitialMatches": int(len(init_corres)),
            "numFinalInliers": int(len(final_corres_set)),
            "numRotationInliers": int(matcher.get_num_rotation_inliers()),
        },
    }
    return payload


def main() -> None:
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    index = []
    for preset in PRESETS:
        payload = run_one(preset)
        out = OUT_DIR / f"kiss_matcher_{preset['id']}.json"
        with open(out, "w") as f:
            json.dump(payload, f)
        size_kb = out.stat().st_size / 1024
        print(f"  → {out.relative_to(REPO)} ({size_kb:.1f} KB)")
        index.append({
            "id": preset["id"],
            "label": preset["label"],
            "url": f"data/precomputed/{out.name}",
            "stats": payload["stats"],
        })

    index_path = OUT_DIR / "kiss_matcher_index.json"
    with open(index_path, "w") as f:
        json.dump({"presets": index}, f, indent=2)
    print(f"\nWrote index: {index_path.relative_to(REPO)} ({len(index)} presets)")


if __name__ == "__main__":
    main()
