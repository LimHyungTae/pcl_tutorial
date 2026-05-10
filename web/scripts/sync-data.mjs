// Mirrors materials/ → web/public/data/, recursively. Runs as a prebuild /
// predev hook (see package.json scripts).
import { mkdirSync, copyFileSync, existsSync, readdirSync, statSync } from "node:fs";
import { dirname, join, resolve } from "node:path";
import { fileURLToPath } from "node:url";

const __dirname = dirname(fileURLToPath(import.meta.url));
const repoRoot = resolve(__dirname, "..", "..");
const src = join(repoRoot, "materials");
const dst = join(__dirname, "..", "public", "data");

if (!existsSync(src)) {
  console.error(`[sync-data] materials/ not found at ${src}`);
  process.exit(1);
}

let copied = 0;
function copyRecursive(srcDir, dstDir) {
  mkdirSync(dstDir, { recursive: true });
  for (const name of readdirSync(srcDir)) {
    const s = join(srcDir, name);
    const d = join(dstDir, name);
    const st = statSync(s);
    if (st.isDirectory()) {
      copyRecursive(s, d);
    } else if (st.isFile()) {
      copyFileSync(s, d);
      copied++;
    }
  }
}

copyRecursive(src, dst);
console.log(`[sync-data] copied ${copied} file(s) → ${dst}`);
