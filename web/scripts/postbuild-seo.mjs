// Postbuild SEO step. Runs after `vite build`.
//
// For every canonical chapter route, write dist/<slug>/index.html that
// reuses the bundled index.html but with chapter-specific <title>, meta
// description, og:* / twitter:* tags, and canonical URL. GH Pages then
// serves a real file (no SPA 404 round-trip) and Google indexes each
// chapter on its own merits.
//
// Also writes dist/sitemap.xml and dist/robots.txt so crawlers can find
// every chapter URL in one shot.
//
// Source of truth for chapter titles / subtitles is web/src/i18n/locales/en.ts;
// we mirror them here intentionally — the alternative (parsing TS from
// Node) is fragile. If you rename a chapter's title there, update it
// here too.

import { readFile, writeFile, mkdir } from "node:fs/promises";
import { dirname, join, resolve } from "node:path";
import { fileURLToPath } from "node:url";

const __filename = fileURLToPath(import.meta.url);
const __dirname = dirname(__filename);
const repoRoot = resolve(__dirname, "..", "..");
const distDir = join(repoRoot, "web", "dist");

const SITE_ORIGIN = "https://limhyungtae.github.io";
const BASE_PATH = "/pcl_tutorial";
const SITE_ROOT = SITE_ORIGIN + BASE_PATH + "/";

/** Mirrors t.chapters.<slug> from web/src/i18n/locales/en.ts. Edit there
 *  AND here in tandem. Kept short so this list is easy to audit. */
const CHAPTERS = [
  { slug: "transformation",                title: "Transformation",                                            subtitle: "4×4 rigid transform for 3D motion" },
  { slug: "voxelization",                  title: "Voxelization",                                              subtitle: "Voxel-grid downsampling for point clouds" },
  { slug: "pass-through",                  title: "PassThrough Filter",                                        subtitle: "Crop a point cloud with an axis-aligned interval filter" },
  { slug: "statistical-outlier-removal",   title: "Statistical Outlier Removal",                               subtitle: "Outlier removal via neighbor-distance statistics" },
  { slug: "radius-search",                 title: "Radius Search",                                             subtitle: "KdTree-based neighbor lookup within a fixed radius" },
  { slug: "k-nearest-neighbor",            title: "K-Nearest Neighbor Search",                                 subtitle: "KdTree-based K closest-neighbor search" },
  { slug: "normal-estimation",             title: "Normal Estimation",                                         subtitle: "Per-point normals via KdTree + SVD" },
  { slug: "iterative-closest-point",       title: "Iterative Closest Point (ICP)",                             subtitle: "Step-by-step local point cloud registration" },
  { slug: "ransac-plane-segmentation",     title: "RANSAC Plane Segmentation",                                 subtitle: "Find the dominant plane and split inliers / outliers" },
  { slug: "euclidean-clustering",          title: "Euclidean Clustering",                                      subtitle: "Group spatially adjacent points into clusters" },
  { slug: "patchwork",                     title: "Patchwork — Ground Segmentation",                           subtitle: "Concentric Zone Model + region-wise plane fitting for LiDAR ground" },
  { slug: "travel",                        title: "TRAVEL — Range Image Clustering",                           subtitle: "Range-image flood-fill of above-ground objects" },
  { slug: "genz-icp",                      title: "GenZ-ICP — Degeneracy-Robust ICP",                          subtitle: "Adaptive point-to-plane / point-to-point ICP" },
  { slug: "kiss-matcher",                  title: "KISS-Matcher — Global Registration",                        subtitle: "Initial-guess-free global point cloud registration (FasterPFH + ROBIN + GNC)" },
];

const SITE_DESC_EN =
  "PCL Tutorial — an interactive Point Cloud Library tutorial covering voxel, passthrough, KdTree, normals, ICP, RANSAC and Euclidean clustering, all in your browser.";
const SITE_DESC_KO =
  "Point Cloud Library(PCL) 인터랙티브 튜토리얼 — 브라우저에서 바로 실습하는 PCL 튜토리얼.";

function escapeHtml(s) {
  return s
    .replace(/&/g, "&amp;")
    .replace(/</g, "&lt;")
    .replace(/>/g, "&gt;")
    .replace(/"/g, "&quot;");
}

/** Replace a meta tag's `content` attribute, identified by an attribute
 *  selector match (e.g. `name="description"` or `property="og:title"`).
 *  No-op (and warns) if the tag is missing.
 */
function setMeta(html, attrSel, value) {
  const safe = escapeHtml(value);
  const re = new RegExp(
    `(<meta[^>]*\\b${attrSel}[^>]*\\bcontent=)"[^"]*"`,
    "i"
  );
  if (!re.test(html)) {
    console.warn(`[postbuild-seo] could not find <meta ${attrSel}>`);
    return html;
  }
  return html.replace(re, `$1"${safe}"`);
}

function setTitle(html, title) {
  return html.replace(/<title>[\s\S]*?<\/title>/i, `<title>${escapeHtml(title)}</title>`);
}

function setCanonical(html, url) {
  return html.replace(
    /<link\s+rel="canonical"\s+href="[^"]*"\s*\/>/i,
    `<link rel="canonical" href="${escapeHtml(url)}" />`
  );
}

async function generateChapterStub(template, ch) {
  const url = SITE_ROOT + ch.slug;
  // Per-chapter title pattern: keeps the brand at the end so search
  // snippets read "<Chapter> · PCL Tutorial". Tested with Google's title
  // truncation rules — we stay under ~60 chars on the canonical slugs.
  const title = `${ch.title} · PCL Tutorial`;
  const desc = `${ch.subtitle}. Interactive PCL tutorial — runs in your browser.`;
  const ogTitle = `【Interactive】 ${ch.title} · PCL Tutorial · PCL 튜토리얼`;

  let html = template;
  html = setTitle(html, title);
  html = setMeta(html, 'name="description"', desc);
  html = setMeta(html, 'property="og:title"', ogTitle);
  html = setMeta(html, 'property="og:description"', desc);
  html = setMeta(html, 'property="og:url"', url);
  html = setMeta(html, 'name="twitter:title"', ogTitle);
  html = setMeta(html, 'name="twitter:description"', desc);
  html = setCanonical(html, url);

  const outDir = join(distDir, ch.slug);
  await mkdir(outDir, { recursive: true });
  await writeFile(join(outDir, "index.html"), html);
}

function buildSitemap() {
  const now = new Date().toISOString().slice(0, 10);
  const urls = [
    SITE_ROOT,
    ...CHAPTERS.map((c) => SITE_ROOT + c.slug),
  ];
  const body = urls
    .map(
      (u) =>
        `  <url><loc>${u}</loc><lastmod>${now}</lastmod><changefreq>monthly</changefreq><priority>${
          u === SITE_ROOT ? "1.0" : "0.8"
        }</priority></url>`
    )
    .join("\n");
  return `<?xml version="1.0" encoding="UTF-8"?>\n<urlset xmlns="http://www.sitemaps.org/schemas/sitemap/0.9">\n${body}\n</urlset>\n`;
}

function buildRobots() {
  return [
    "User-agent: *",
    "Allow: /",
    "",
    `Sitemap: ${SITE_ORIGIN}${BASE_PATH}/sitemap.xml`,
    "",
  ].join("\n");
}

async function main() {
  const indexPath = join(distDir, "index.html");
  let template;
  try {
    template = await readFile(indexPath, "utf8");
  } catch (e) {
    console.error(`[postbuild-seo] ${indexPath} not found — run after vite build`);
    process.exit(1);
  }

  // Each per-chapter stub starts from the same bundled index.html so it
  // hydrates into the same React app; only the head meta differ.
  for (const ch of CHAPTERS) {
    await generateChapterStub(template, ch);
  }
  console.log(`[postbuild-seo] wrote ${CHAPTERS.length} chapter stubs`);

  await writeFile(join(distDir, "sitemap.xml"), buildSitemap());
  await writeFile(join(distDir, "robots.txt"), buildRobots());
  console.log("[postbuild-seo] wrote sitemap.xml and robots.txt");

  // Silence the "unused" lint warning for SITE_DESC_EN / SITE_DESC_KO,
  // which are kept as documentation of the canonical strings used in
  // web/index.html.
  void SITE_DESC_EN;
  void SITE_DESC_KO;
}

main().catch((e) => {
  console.error(e);
  process.exit(1);
});
