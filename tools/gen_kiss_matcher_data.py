#!/usr/bin/env python3
"""
Generate KISS-Matcher demo data for the web tutorial.

Loads materials/kitti00_000000.bin, applies a known yaw rotation +
translation to make a "src" view of the scene, runs KISS-Matcher
(`pip install kiss-matcher`) on the (src, tgt) pair, and writes a small
JSON to web/public/data/precomputed/kiss_matcher.json that the web page
reads at runtime.

The web page never re-runs KISS-Matcher; the slider on the page just
re-renders the precomputed correspondences.

Usage
-----
    pip install kiss-matcher numpy
    python tools/gen_kiss_matcher_data.py

Re-run this whenever you want to:
  - swap the input scan,
  - change the ground-truth offset,
  - re-tune the voxel size.
"""
import json
import sys
from pathlib import Path

import numpy as np

try:
    import kiss_matcher as km
except ImportError:
    sys.exit("`pip install kiss-matcher` first")

# ---------------------------------------------------------------------------

REPO = Path(__file__).resolve().parent.parent
SRC_BIN = REPO / "materials" / "kitti00_000000.bin"
OUT_JSON = REPO / "web" / "public" / "data" / "precomputed" / "kiss_matcher.json"

# Ground-truth offset applied to the original cloud to fabricate a "src" view.
# Large enough that a local method (ICP) would fail without a good init,
# which is the whole point of showing global registration.
YAW_DEG = 75.0
TX = 8.0
TY = -3.0
TZ = 0.0

VOXEL_SIZE = 0.6  # tune to KITTI's typical point spacing

MAX_MATCHES_IN_JSON = 4000  # cap to keep the bundle small

# ---------------------------------------------------------------------------


def load_bin_xyz(path: Path) -> np.ndarray:
    raw = np.fromfile(path, dtype=np.float32)
    return raw.reshape(-1, 4)[:, :3]


def make_src(tgt_xyz: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    yaw = np.radians(YAW_DEG)
    R = np.array(
        [
            [np.cos(yaw), -np.sin(yaw), 0.0],
            [np.sin(yaw), np.cos(yaw), 0.0],
            [0.0, 0.0, 1.0],
        ],
        dtype=np.float32,
    )
    t = np.array([TX, TY, TZ], dtype=np.float32)
    src_xyz = (R @ tgt_xyz.T).T + t
    return src_xyz, R, t


def main() -> None:
    print(f"Loading: {SRC_BIN}")
    tgt_xyz = load_bin_xyz(SRC_BIN)
    print(f"  tgt points: {len(tgt_xyz):,}")

    src_xyz, R_aug, t_aug = make_src(tgt_xyz)
    print(f"  applied yaw={YAW_DEG}°, t=({TX}, {TY}, {TZ}) → src points: {len(src_xyz):,}")

    print(f"Running KISS-Matcher (voxel_size={VOXEL_SIZE})...")
    config = km.KISSMatcherConfig(VOXEL_SIZE)
    matcher = km.KISSMatcher(config)
    result = matcher.estimate(
        src_xyz.astype(np.float32),
        tgt_xyz.astype(np.float32),
    )
    matcher.print()

    R_est = np.asarray(result.rotation, dtype=np.float32)
    t_est = np.asarray(result.translation, dtype=np.float32)

    # Sanity print: rotation + translation residuals against R_aug, t_aug.
    R_gt_inv = R_aug.T
    t_gt_inv = -R_gt_inv @ t_aug
    rot_err_deg = np.degrees(
        np.arccos(np.clip((np.trace(R_gt_inv.T @ R_est) - 1) / 2, -1.0, 1.0))
    )
    trans_err = np.linalg.norm(t_est - t_gt_inv)
    print(f"  rotation error vs GT⁻¹     : {rot_err_deg:.2f}°")
    print(f"  translation error vs GT⁻¹  : {trans_err:.3f} m")

    src_matched, tgt_matched = matcher.get_keypoints_from_initial_matching()
    src_matched = np.asarray(src_matched, dtype=np.float32)
    tgt_matched = np.asarray(tgt_matched, dtype=np.float32)

    init_corres = matcher.get_initial_correspondences()
    final_corres_set = set(map(tuple, matcher.get_final_correspondences()))

    matches = []
    for i, (s_idx, t_idx) in enumerate(init_corres):
        is_final = (int(s_idx), int(t_idx)) in final_corres_set
        matches.append(
            {
                "src": [round(float(v), 4) for v in src_matched[i]],
                "tgt": [round(float(v), 4) for v in tgt_matched[i]],
                "isFinal": bool(is_final),
            }
        )

    # Sort: final inliers first so the slider's first-K naturally surfaces them.
    matches.sort(key=lambda m: 0 if m["isFinal"] else 1)
    if len(matches) > MAX_MATCHES_IN_JSON:
        # Always keep all final inliers; subsample the non-final tail.
        finals = [m for m in matches if m["isFinal"]]
        non_finals = [m for m in matches if not m["isFinal"]]
        keep_non = MAX_MATCHES_IN_JSON - len(finals)
        if keep_non > 0:
            stride = max(1, len(non_finals) // keep_non)
            non_finals = non_finals[::stride][:keep_non]
        matches = finals + non_finals

    payload = {
        "tgtUrl": "data/kitti00_000000.bin",
        "voxelSize": VOXEL_SIZE,
        "srcOffset": {
            "yawDeg": float(YAW_DEG),
            "tx": float(TX),
            "ty": float(TY),
            "tz": float(TZ),
        },
        "estimatedRotation": [float(v) for v in R_est.flatten()],
        "estimatedTranslation": [float(v) for v in t_est],
        "matches": matches,
        "stats": {
            "numTgtPoints": int(len(tgt_xyz)),
            "numInitialMatches": int(len(init_corres)),
            "numFinalInliers": int(len(final_corres_set)),
            "numRotationInliers": int(matcher.get_num_rotation_inliers()),
            "rotErrorDeg": float(rot_err_deg),
            "transErrorM": float(trans_err),
        },
    }

    OUT_JSON.parent.mkdir(parents=True, exist_ok=True)
    with open(OUT_JSON, "w") as f:
        json.dump(payload, f)
    size_kb = OUT_JSON.stat().st_size / 1024
    print(f"\nWrote: {OUT_JSON} ({size_kb:.1f} KB)")
    print(f"  initial matches in JSON: {len(matches):,} (capped at {MAX_MATCHES_IN_JSON:,})")
    print(f"  final inliers           : {payload['stats']['numFinalInliers']}")


if __name__ == "__main__":
    main()
