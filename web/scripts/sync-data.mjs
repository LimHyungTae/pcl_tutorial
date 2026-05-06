// Copies materials/* → web/public/data/*
// Run automatically before `vite dev` and `vite build` (see package.json scripts).
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

mkdirSync(dst, { recursive: true });

let copied = 0;
for (const name of readdirSync(src)) {
  const s = join(src, name);
  if (!statSync(s).isFile()) continue;
  copyFileSync(s, join(dst, name));
  copied++;
}
console.log(`[sync-data] copied ${copied} file(s) → ${dst}`);
