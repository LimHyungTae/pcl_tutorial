import { useMemo } from "react";
import { Canvas } from "@react-three/fiber";
import { OrbitControls, GizmoHelper, GizmoViewport } from "@react-three/drei";
import * as THREE from "three";
import type { PointCloud } from "../lib/types";

type Layer = {
  cloud: PointCloud;
  /** Either a single CSS color (applied to all points), or per-point RGB
   *  (Float32Array of length 3*count, values in [0,1]). */
  color: string | Float32Array;
  size?: number;
  opacity?: number;
};

type Props = {
  layers: Layer[];
  background?: string;
};

export default function PointCloudViewer({
  layers,
  background = "#0b1220",
}: Props) {
  const { center, radius } = useMemo(() => unionBounds(layers), [layers]);

  // Camera sized so the union bbox comfortably fills view.
  const camPos: [number, number, number] = [
    center.x + radius * 1.6,
    center.y + radius * 1.0,
    center.z + radius * 1.4,
  ];

  return (
    <Canvas
      style={{ background }}
      camera={{ position: camPos, fov: 50, near: Math.max(radius * 0.001, 0.01), far: radius * 50 + 1000 }}
      dpr={[1, 2]}
    >
      <ambientLight intensity={0.6} />
      {layers.map((l, idx) => (
        <PointsLayer key={idx} layer={l} />
      ))}
      <axesHelper args={[Math.max(1, radius * 0.2)]} />
      <OrbitControls
        target={center}
        enableDamping
        dampingFactor={0.08}
        makeDefault
      />
      <GizmoHelper alignment="bottom-right" margin={[64, 64]}>
        <GizmoViewport
          axisColors={["#ef4444", "#22c55e", "#3b82f6"]}
          labelColor="#e2e8f0"
        />
      </GizmoHelper>
    </Canvas>
  );
}

function PointsLayer({ layer }: { layer: Layer }) {
  const geom = useMemo(() => {
    const g = new THREE.BufferGeometry();
    g.setAttribute("position", new THREE.BufferAttribute(layer.cloud.positions, 3));
    if (layer.color instanceof Float32Array) {
      g.setAttribute("color", new THREE.BufferAttribute(layer.color, 3));
    }
    return g;
  }, [layer.cloud, layer.color]);

  const mat = useMemo(() => {
    if (layer.color instanceof Float32Array) {
      return new THREE.PointsMaterial({
        vertexColors: true,
        size: layer.size ?? 0.05,
        sizeAttenuation: true,
        transparent: (layer.opacity ?? 1) < 1,
        opacity: layer.opacity ?? 1,
      });
    }
    return new THREE.PointsMaterial({
      color: new THREE.Color(layer.color),
      size: layer.size ?? 0.05,
      sizeAttenuation: true,
      transparent: (layer.opacity ?? 1) < 1,
      opacity: layer.opacity ?? 1,
    });
  }, [layer.color, layer.size, layer.opacity]);

  return <points geometry={geom} material={mat} />;
}

function unionBounds(layers: Layer[]) {
  let minX = Infinity,
    minY = Infinity,
    minZ = Infinity;
  let maxX = -Infinity,
    maxY = -Infinity,
    maxZ = -Infinity;
  let any = false;
  for (const l of layers) {
    if (l.cloud.count === 0) continue;
    any = true;
    const { min, max } = l.cloud.bbox;
    if (min[0] < minX) minX = min[0];
    if (min[1] < minY) minY = min[1];
    if (min[2] < minZ) minZ = min[2];
    if (max[0] > maxX) maxX = max[0];
    if (max[1] > maxY) maxY = max[1];
    if (max[2] > maxZ) maxZ = max[2];
  }
  if (!any) {
    return {
      center: new THREE.Vector3(0, 0, 0),
      radius: 10,
    };
  }
  const cx = (minX + maxX) / 2;
  const cy = (minY + maxY) / 2;
  const cz = (minZ + maxZ) / 2;
  const r = Math.max(maxX - minX, maxY - minY, maxZ - minZ) * 0.6 + 0.1;
  return { center: new THREE.Vector3(cx, cy, cz), radius: r };
}
