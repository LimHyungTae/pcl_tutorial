#!/usr/bin/env python3
"""
Generate KISS-Matcher demo data for the web tutorial.

Loads materials/kiss_matcher_src.pcd + materials/kiss_matcher_tgt.pcd
(KITTI HDL-64 frames 540 / 1319 from the official Vel64 sample bundled
with KISS-Matcher), runs KISS-Matcher (`pip install kiss-matcher`) to
recover the rigid transform that aligns src to tgt, and writes a small
JSON to web/public/data/precomputed/kiss_matcher.json that the web page
reads at runtime.

The web page never re-runs KISS-Matcher; the slider on the page just
re-renders the precomputed correspondences.

Usage
-----
    pip install kiss-matcher numpy
    python tools/gen_kiss_matcher_data.py

Re-run this whenever you swap the input scans or re-tune voxel size.
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
    # Older kiss_matcher releases shipped read_pcd at the top level. Fall
    # back to a tiny ascii/binary reader if neither path is available.
    read_pcd = None

# ---------------------------------------------------------------------------

REPO = Path(__file__).resolve().parent.parent
SRC_PCD = REPO / "materials" / "kiss_matcher_src.pcd"
TGT_PCD = REPO / "materials" / "kiss_matcher_tgt.pcd"
# Lives under materials/ so it gets picked up by web/scripts/sync-data.mjs
# at prebuild time and copied into web/public/data/precomputed/.
OUT_JSON = REPO / "materials" / "precomputed" / "kiss_matcher.json"

VOXEL_SIZE = 0.6  # tune to KITTI's typical point spacing
MAX_MATCHES_IN_JSON = 4000  # cap to keep the bundle small

# ---------------------------------------------------------------------------


def load_pcd_xyz(path: Path) -> np.ndarray:
    if read_pcd is not None:
        return np.asarray(read_pcd(str(path)), dtype=np.float32)
    # Minimal fallback (header-only inspection). Most KISS-Matcher releases
    # include io_utils so this branch should rarely run.
    raise SystemExit(
        "kiss_matcher.io_utils.read_pcd not found — please install a newer kiss-matcher"
    )


def main() -> None:
    print(f"Loading src: {SRC_PCD}")
    src_xyz = load_pcd_xyz(SRC_PCD)
    print(f"  src points: {len(src_xyz):,}")

    print(f"Loading tgt: {TGT_PCD}")
    tgt_xyz = load_pcd_xyz(TGT_PCD)
    print(f"  tgt points: {len(tgt_xyz):,}")

    print(f"\nRunning KISS-Matcher (voxel_size={VOXEL_SIZE})...")
    config = km.KISSMatcherConfig(VOXEL_SIZE)
    matcher = km.KISSMatcher(config)
    result = matcher.estimate(
        src_xyz.astype(np.float32),
        tgt_xyz.astype(np.float32),
    )
    matcher.print()

    R_est = np.asarray(result.rotation, dtype=np.float32)
    t_est = np.asarray(result.translation, dtype=np.float32)

    # FasterPFH keypoints — initial / final correspondences index into
    # these. Going via the keypoint sets keeps things consistent regardless
    # of how many pairs survive each pruning stage.
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

    # Sort: final inliers first so the slider's first-K naturally surfaces them.
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
        "srcUrl": "data/kiss_matcher_src.pcd",
        "tgtUrl": "data/kiss_matcher_tgt.pcd",
        "voxelSize": VOXEL_SIZE,
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

    OUT_JSON.parent.mkdir(parents=True, exist_ok=True)
    with open(OUT_JSON, "w") as f:
        json.dump(payload, f)
    size_kb = OUT_JSON.stat().st_size / 1024
    print(f"\nWrote: {OUT_JSON} ({size_kb:.1f} KB)")
    print(f"  initial matches in JSON : {len(matches):,} (capped at {MAX_MATCHES_IN_JSON:,})")
    print(f"  final inliers           : {payload['stats']['numFinalInliers']}")


if __name__ == "__main__":
    main()
