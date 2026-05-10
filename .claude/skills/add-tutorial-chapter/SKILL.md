---
name: add-tutorial-chapter
description: Use when adding a new chapter to the PCL Tutorial web app (web/src/). Establishes the kebab-case slug + matching PascalCase page file convention, the chapters.ts / routes.tsx / i18n / README wiring, sensor-config and BibTeX patterns, and the academic copy style.
---

# Adding a new tutorial chapter

This repo's interactive demo lives in `web/`. Every chapter is a TS port of a PCL or open-source perception algorithm with a small page that wraps `PointCloudViewer`. Follow this skill end-to-end when adding a new chapter; deviating from the file-naming convention (or skipping a wiring step) will silently break routing or i18n.

## Naming convention — single source of truth

Pick a **kebab-case algorithm name** first. Everything else is derived from it.

| Concern | Format | Example |
|---|---|---|
| URL slug (`chapters.ts`, `routes.tsx`) | `kebab-case` | `genz-icp` |
| Page file (`web/src/pages/`) | `PascalCase` matching slug | `GenzIcp.tsx` |
| Algorithm file (`web/src/lib/filters/`) | `camelCase` matching slug | `genzIcp.ts` |
| i18n `chapters` key | same as slug, quote when hyphenated | `chapters["genz-icp"]` |
| i18n UI string block (top-level) | concise camelCase nickname | `t.genzIcp` |

DO NOT use codename prefixes like `Lec11`, `Extra05` for new files — those exist on legacy chapters only and must not be propagated. New page modules go in `web/src/pages/<PascalCase>.tsx`.

When you must rename an existing slug, ALSO add a `<Navigate to="/<new>" replace />` redirect in `routes.tsx` so old links / bookmarks still resolve.

## Step-by-step checklist

### 1. Algorithm file — `web/src/lib/filters/<camelCase>.ts`

- Pure TS. No React / DOM imports.
- Reuse what exists: `KdTree` (`../kdtree`), `symEig3x3` (`../eigen`), `cloudFromPositions` / `transformCloud`, etc.
- Export the main entry function plus all result types. Document any sensor-specific assumptions in the file header.
- For **iterative** algorithms (ICP family), match the existing step shape:
  ```ts
  function xxxStep(src, targetIndex, currentTransform, ...params): {
    transform: Float32Array;   // 4×4 row-major
    fitness: number;
    pairs: number;
    pairCoords: Float32Array;   // packed [s.x,y,z, t.x,y,z, ...]
    // + algo-specific fields
  }
  ```
- For algorithms that need expensive precomputation (normals, planarity classification), expose a `build…Index(target, params)` helper that runs once per target and is reused across iterations.

### 2. Page component — `web/src/pages/<PascalCase>.tsx`

- Copy the closest existing page as a skeleton:
  - Iterative ICP-family: `Extra05GenzIcp.tsx`
  - Filter / single-pass: `Lec05Voxelization.tsx`
  - Two-pane synced before/after: `Lec06PassThrough.tsx`
  - Cluster-style: `Extra02EuclideanCluster.tsx`
- Wire it up:
  ```tsx
  const chapter = findChapter("<kebab-slug>")!;
  // ...
  <ChapterHeader chapter={chapter} />
  <DemoAbout slug="<kebab-slug>" />
  // ...
  <DemoParams slug="<kebab-slug>" />
  ```
- For sensor-specific demos, restrict the data picker and add a caution box:
  ```tsx
  <DataSourcePicker enabledPresets={["naverlabs", "kitti"]} allowCustom={false} ... />
  <CautionBox title={t.caution.title} body={t.caution.xxxBody} links={[...]} />
  ```
- For papers, append a citation block at the bottom:
  ```tsx
  <BibTexBlock entries={[{ key: "...", bibtex: `@article{...}` }]} />
  ```
- For iterative demos, set `framingKey={epoch}` and `framingZoom={0.7}` on `<PointCloudViewer>`. The epoch state should bump on Reset and on preset switch ONLY; that keeps the user's manual orbit during Play. See `Lec11Icp.tsx` for the canonical pattern.
- For correspondence / pair line layers, use a width ≥ 2 (drei's fat-line is wired into `LinesLayer`). Native WebGL line width caps at 1 px and looks faint.

### 3. Register in `web/src/chapters.ts`

Append:
```ts
{
  slug: "<kebab-slug>",
  number: "<N>",            // 1-based, shown in the UI
  source: "<lecXX_xxx.cpp>", // path under repo root, or "" for web-only extras
  blog: `${BLOG}/...`,        // blog post URL, or ""
  status: "interactive",
}
```

Order in this array determines the homepage card order.

### 4. Wire the route — `web/src/routes.tsx`

Add:
```tsx
{ path: "<kebab-slug>", element: <NewChapter /> },
```

If you renamed an existing slug, add the legacy redirect alongside:
```tsx
{ path: "<old-slug>", element: <Navigate to="/<kebab-slug>" replace /> },
```

### 5. i18n — `web/src/i18n/locales/en.ts` AND `ko.ts`

Add a `chapters[<slug>]` block in BOTH files (KO must mirror EN's shape — `LocaleDict = typeof en`):

```ts
chapters: {
  // ...
  "<kebab-slug>": {
    title: "...",
    subtitle: "...",
    about: "...",
    params: [
      { name: "...", desc: "...", effect: "..." },
    ],
  },
}
```

If the chapter needs new UI strings (legend labels, status text), add a top-level block keyed by the camelCase nickname:
```ts
genzIcp: {
  planar: "planar (point-to-plane)",
  nonPlanar: "non-planar (point-to-point)",
}
```

### 6. README tables — `README.md` AND `README_kr.md`

Append a row to the `Interactive Demo` table at the right number. Keep the Highlights column under ~70 chars.

### 7. Build & verify

```bash
cd web && npm run build
```

`tsc -b && vite build` must both pass. The chunk size warning (`> 1MB gzipped`) is acceptable, not a blocker.

For algorithms that are easy to validate with synthetic data, drop a smoke script under `web/scripts/` (Node tsx). See `web/scripts/smoke-patchwork.mjs` for the pattern: build a synthetic scene, run the algo, assert ground / non-ground splits, exit non-zero on failure.

## Style conventions

### Academic / about copy

- **No em dashes (`—`) or en dashes (`–`) used as em dashes** in academic copy (chapter `about`, paper-style writing). Replace with comma, semicolon, colon, parentheses, or split sentences. Hyphens in compound words (`state-of-the-art`) and number ranges (`pp. 10–15` is technically en dash but in citations only) are fine. This rule comes from `~/.claude/CLAUDE.md` and applies to BOTH EN and KO chapter copy.
- **Citation format**: `Author et al., Venue, YYYY` — e.g. `Lim et al., RA-L, 2021`. Use a comma between Venue and Year. This format is consistent across all chapters.
- **Tone** (per author preference, Patchwork chapter as the canonical example): open with motivation, name the algorithm's components explicitly (CZM, R-GPF, GLE; TGS, AOS; etc.), close with what is shown in the visualization.

### Code

- **Default to no comments**. Add one only when the WHY is non-obvious — a hidden constraint, a workaround, behavior that would surprise a reader. Don't restate WHAT the code does.
- **No backwards-compat hacks**. Don't keep deprecated paths "just in case" unless the user asks.
- **Reuse rviz colors when porting from ROS packages**. Document the source in a code comment. Examples in repo:
  - GenZ-ICP planar = `#0077bb`, non-planar = `#ee3377`

### Commit messages

Follow the repo's pattern (see recent commits):
- Subject under ~72 chars, imperative.
- Body explains WHY in short paragraphs / bullets, references upstream files (`patchwork.hpp:818-827`) when porting.
- HEREDOC for multi-line bodies.

## Common pitfalls

- **Z-up assumption**: NaverLabs preset PCD is Z-down (floor mass at `z ≈ +0.6`). Algorithms that assume Z-up (Patchwork, TRAVEL, GenZ-ICP) MUST call `ensureZUp(cloud, presetId)` from `web/src/lib/axisTransform.ts` before processing. KITTI is already Z-up.
- **i18n type widening**: Do NOT use `as const` on `en.ts`. Plain object literal preserves the inferred type that `ko.ts` then satisfies via `LocaleDict`.
- **Camera framing during Play**: Iterative chapters MUST pass `framingKey` to `<PointCloudViewer>` (or the camera resets every frame as bbox shifts).
- **Line width on WebGL**: Native `lineSegments` cap at 1 px regardless of `linewidth`. `LinesLayer` already routes through drei's fat-line when `width > 1` — pass `width: 2` (or higher) on the `LineLayer` to actually see thick lines.
- **Hyphenated keys in i18n must be quoted**: `chapters["pass-through"]: { ... }` — bare `pass-through:` is a syntax error.
- **Sensor configs are baked**: Sensor-specific demos (Patchwork, TRAVEL) currently support only VLP-16 (`naverlabs` preset) and HDL-64 (`kitti` preset). Disable Bunny via `enabledPresets` and surface a `<CautionBox>` linking to the upstream repo for arbitrary data.

## Where the C++ originals live (Hyungtae's machine)

| Algorithm | Path |
|---|---|
| Patchwork | `~/git/patchwork/` |
| Patchwork++ | `~/git/patchwork-plusplus/` (if present) |
| TRAVEL | `~/git/TRAVEL/` |
| GenZ-ICP | `~/git/genz-icp/` |
| KISS-Matcher | `~/git/KISS-Matcher/` (if present) |

When porting, read the algorithm core under `cpp/.../core/` or `include/.../`, the rviz config under `ros/rviz/`, and the parameter YAMLs under `ros/config/`. Cross-reference rviz colors / parameter defaults rather than guessing.
