import { parseKittiBin } from "./kitti";
import { parsePly } from "./ply";
import { cloudFromPositions, type PointCloud } from "./types";

/**
 * Minimal PCD parser that handles ASCII and binary (uncompressed) variants
 * with FIELDS that include x, y, z. Anything else is read but ignored.
 */
export function parsePcd(buf: ArrayBuffer): PointCloud {
  const bytes = new Uint8Array(buf);
  const headerEnd = findHeaderEnd(bytes);
  if (headerEnd < 0) throw new Error("PCD: header end not found");

  const headerText = new TextDecoder("ascii").decode(bytes.subarray(0, headerEnd));
  const meta = parseHeader(headerText);

  const xIdx = meta.fields.indexOf("x");
  const yIdx = meta.fields.indexOf("y");
  const zIdx = meta.fields.indexOf("z");
  if (xIdx < 0 || yIdx < 0 || zIdx < 0) {
    throw new Error("PCD: x/y/z fields missing");
  }

  const positions = new Float32Array(meta.points * 3);

  if (meta.data === "ascii") {
    const body = new TextDecoder("ascii").decode(bytes.subarray(headerEnd));
    let p = 0;
    const lines = body.split(/\r?\n/);
    for (const line of lines) {
      if (!line) continue;
      const tok = line.trim().split(/\s+/);
      if (tok.length < meta.fields.length) continue;
      positions[p * 3] = parseFloat(tok[xIdx]);
      positions[p * 3 + 1] = parseFloat(tok[yIdx]);
      positions[p * 3 + 2] = parseFloat(tok[zIdx]);
      p++;
      if (p >= meta.points) break;
    }
  } else if (meta.data === "binary") {
    const view = new DataView(bytes.buffer, bytes.byteOffset + headerEnd);
    const pointStride = meta.sizes.reduce((a, b) => a + b, 0);
    let off = 0;
    for (let p = 0; p < meta.points; p++) {
      let inner = 0;
      for (let f = 0; f < meta.fields.length; f++) {
        const size = meta.sizes[f];
        const typ = meta.types[f];
        if (f === xIdx || f === yIdx || f === zIdx) {
          const v = readScalar(view, off + inner, size, typ);
          if (f === xIdx) positions[p * 3] = v;
          else if (f === yIdx) positions[p * 3 + 1] = v;
          else positions[p * 3 + 2] = v;
        }
        inner += size;
      }
      off += pointStride;
    }
  } else {
    throw new Error(`PCD: unsupported DATA mode "${meta.data}"`);
  }

  return cloudFromPositions(positions);
}

function findHeaderEnd(bytes: Uint8Array): number {
  // Find the end of the line that begins with "DATA "
  const dec = new TextDecoder("ascii");
  let lineStart = 0;
  for (let i = 0; i < bytes.length; i++) {
    if (bytes[i] === 0x0a /* \n */) {
      const line = dec.decode(bytes.subarray(lineStart, i));
      if (/^\s*DATA\s+/i.test(line)) {
        return i + 1;
      }
      lineStart = i + 1;
    }
  }
  return -1;
}

type PcdHeader = {
  fields: string[];
  sizes: number[];
  types: string[]; // F (float), U (unsigned), I (signed)
  points: number;
  data: "ascii" | "binary" | "binary_compressed";
};

function parseHeader(text: string): PcdHeader {
  const out: PcdHeader = {
    fields: [],
    sizes: [],
    types: [],
    points: 0,
    data: "ascii",
  };
  for (const raw of text.split(/\r?\n/)) {
    const line = raw.trim();
    if (!line || line.startsWith("#")) continue;
    const [key, ...rest] = line.split(/\s+/);
    const k = key.toUpperCase();
    if (k === "FIELDS") out.fields = rest;
    else if (k === "SIZE") out.sizes = rest.map((v) => parseInt(v, 10));
    else if (k === "TYPE") out.types = rest.map((v) => v.toUpperCase());
    else if (k === "POINTS") out.points = parseInt(rest[0], 10);
    else if (k === "DATA") out.data = rest[0].toLowerCase() as PcdHeader["data"];
  }
  return out;
}

function readScalar(view: DataView, offset: number, size: number, typ: string): number {
  if (typ === "F") {
    if (size === 4) return view.getFloat32(offset, true);
    if (size === 8) return view.getFloat64(offset, true);
  } else if (typ === "U") {
    if (size === 1) return view.getUint8(offset);
    if (size === 2) return view.getUint16(offset, true);
    if (size === 4) return view.getUint32(offset, true);
  } else if (typ === "I") {
    if (size === 1) return view.getInt8(offset);
    if (size === 2) return view.getInt16(offset, true);
    if (size === 4) return view.getInt32(offset, true);
  }
  throw new Error(`PCD: unsupported field type "${typ}${size}"`);
}

/** Convenience: detect by extension and parse. */
export async function loadCloud(url: string): Promise<PointCloud> {
  const res = await fetch(url);
  if (!res.ok) throw new Error(`fetch ${url}: ${res.status}`);
  const buf = await res.arrayBuffer();
  const lower = url.toLowerCase();
  if (lower.endsWith(".bin")) return parseKittiBin(buf);
  if (lower.endsWith(".ply")) return parsePly(buf);
  return parsePcd(buf);
}

/** Parse from an in-memory buffer with explicit format hint. */
export function parseFromBuffer(buf: ArrayBuffer, filename: string): PointCloud {
  const lower = filename.toLowerCase();
  if (lower.endsWith(".bin")) return parseKittiBin(buf);
  if (lower.endsWith(".ply")) return parsePly(buf);
  return parsePcd(buf);
}
