import { cloudFromPositions, type PointCloud } from "./types";

/**
 * Minimal PLY parser supporting:
 *  - format ascii 1.0
 *  - format binary_little_endian 1.0
 * with one `element vertex N` block whose properties contain x/y/z (any
 * order, any extra properties). Faces and other elements are skipped.
 */
export function parsePly(buf: ArrayBuffer): PointCloud {
  const bytes = new Uint8Array(buf);
  const headerEnd = findHeaderEnd(bytes);
  if (headerEnd < 0) throw new Error("PLY: header end not found");

  const headerText = new TextDecoder("ascii").decode(bytes.subarray(0, headerEnd));
  const meta = parseHeader(headerText);
  if (meta.vertexCount < 0) throw new Error("PLY: missing 'element vertex'");

  const xIdx = meta.vertexProps.findIndex((p) => p.name === "x");
  const yIdx = meta.vertexProps.findIndex((p) => p.name === "y");
  const zIdx = meta.vertexProps.findIndex((p) => p.name === "z");
  if (xIdx < 0 || yIdx < 0 || zIdx < 0) {
    throw new Error("PLY: vertex properties x/y/z missing");
  }

  const positions = new Float32Array(meta.vertexCount * 3);

  if (meta.format === "ascii") {
    const body = new TextDecoder("ascii").decode(bytes.subarray(headerEnd));
    let idx = 0;
    for (const raw of body.split(/\r?\n/)) {
      if (idx >= meta.vertexCount) break;
      const line = raw.trim();
      if (!line) continue;
      const tok = line.split(/\s+/);
      if (tok.length < meta.vertexProps.length) continue;
      positions[idx * 3] = parseFloat(tok[xIdx]);
      positions[idx * 3 + 1] = parseFloat(tok[yIdx]);
      positions[idx * 3 + 2] = parseFloat(tok[zIdx]);
      idx++;
    }
  } else if (meta.format === "binary_little_endian") {
    const view = new DataView(bytes.buffer, bytes.byteOffset + headerEnd);
    let off = 0;
    for (let i = 0; i < meta.vertexCount; i++) {
      let inner = 0;
      for (let p = 0; p < meta.vertexProps.length; p++) {
        const prop = meta.vertexProps[p];
        if (p === xIdx || p === yIdx || p === zIdx) {
          const v = readScalar(view, off + inner, prop.type);
          if (p === xIdx) positions[i * 3] = v;
          else if (p === yIdx) positions[i * 3 + 1] = v;
          else positions[i * 3 + 2] = v;
        }
        inner += scalarSize(prop.type);
      }
      off += meta.vertexStride;
    }
  } else {
    throw new Error(`PLY: unsupported format "${meta.format}"`);
  }

  return cloudFromPositions(positions);
}

type PlyType =
  | "char" | "uchar" | "short" | "ushort" | "int" | "uint" | "float" | "double"
  | "int8" | "uint8" | "int16" | "uint16" | "int32" | "uint32" | "float32" | "float64";

type PlyProp = { name: string; type: PlyType };

type PlyHeader = {
  format: "ascii" | "binary_little_endian" | "binary_big_endian";
  vertexCount: number;
  vertexProps: PlyProp[];
  vertexStride: number;
};

function findHeaderEnd(bytes: Uint8Array): number {
  const dec = new TextDecoder("ascii");
  let lineStart = 0;
  for (let i = 0; i < bytes.length; i++) {
    if (bytes[i] === 0x0a /* \n */) {
      const line = dec.decode(bytes.subarray(lineStart, i)).trim();
      if (line === "end_header") return i + 1;
      lineStart = i + 1;
    }
  }
  return -1;
}

function parseHeader(text: string): PlyHeader {
  const out: PlyHeader = {
    format: "ascii",
    vertexCount: -1,
    vertexProps: [],
    vertexStride: 0,
  };
  let inVertex = false;
  for (const raw of text.split(/\r?\n/)) {
    const line = raw.trim();
    if (!line || line.startsWith("comment ") || line === "ply") continue;
    if (line.startsWith("format ")) {
      const tok = line.split(/\s+/);
      out.format = tok[1] as PlyHeader["format"];
    } else if (line.startsWith("element ")) {
      const tok = line.split(/\s+/);
      if (tok[1] === "vertex") {
        inVertex = true;
        out.vertexCount = parseInt(tok[2], 10);
      } else {
        inVertex = false;
      }
    } else if (line.startsWith("property ")) {
      if (!inVertex) continue;
      const tok = line.split(/\s+/);
      // Skip 'property list ...' (faces etc.) — only scalar vertex props are
      // relevant for our purposes.
      if (tok[1] === "list") continue;
      out.vertexProps.push({ type: tok[1] as PlyType, name: tok[tok.length - 1] });
    }
  }
  out.vertexStride = out.vertexProps.reduce((a, p) => a + scalarSize(p.type), 0);
  return out;
}

function scalarSize(t: PlyType): number {
  switch (t) {
    case "char": case "uchar": case "int8": case "uint8":
      return 1;
    case "short": case "ushort": case "int16": case "uint16":
      return 2;
    case "int": case "uint": case "float": case "int32": case "uint32": case "float32":
      return 4;
    case "double": case "float64":
      return 8;
  }
}

function readScalar(view: DataView, off: number, t: PlyType): number {
  switch (t) {
    case "char": case "int8":   return view.getInt8(off);
    case "uchar": case "uint8": return view.getUint8(off);
    case "short": case "int16":   return view.getInt16(off, true);
    case "ushort": case "uint16": return view.getUint16(off, true);
    case "int": case "int32":     return view.getInt32(off, true);
    case "uint": case "uint32":   return view.getUint32(off, true);
    case "float": case "float32": return view.getFloat32(off, true);
    case "double": case "float64": return view.getFloat64(off, true);
  }
}
