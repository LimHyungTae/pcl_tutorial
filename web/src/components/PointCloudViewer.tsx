import { useEffect, useMemo, useRef, useState } from "react";
import { Canvas, useThree } from "@react-three/fiber";
import { OrbitControls, GizmoHelper, GizmoViewport } from "@react-three/drei";
import * as THREE from "three";
import { CameraSyncStore, nextSyncId } from "../lib/cameraSync";
import type { PointCloud } from "../lib/types";

type Layer = {
  cloud: PointCloud;
  color: string | Float32Array;
  size?: number;
  opacity?: number;
  /** Contributes to camera framing only — skip rendering. Useful when two
   *  synced viewers should share a bbox: each pane includes the other's
   *  cloud as a bounds-only layer. */
  boundsOnly?: boolean;
};

type LineLayer = {
  positions: Float32Array;
  color: string;
  opacity?: number;
};

type Props = {
  layers: Layer[];
  lines?: LineLayer[];
  background?: string;
  onPick?: (xyz: [number, number, number]) => void;
  defaultSizeMult?: number;
  /** When two viewers share a CameraSyncStore, orbiting one mirrors to the
   *  other. */
  sync?: CameraSyncStore;
};

export default function PointCloudViewer({
  layers,
  lines,
  background = "#0a0f1a",
  onPick,
  defaultSizeMult = 1,
  sync,
}: Props) {
  const [sizeMult, setSizeMult] = useState(defaultSizeMult);
  const { center, radius } = useMemo(() => unionBounds(layers), [layers]);

  const initialPos: [number, number, number] = [
    center.x + radius * 1.6,
    center.y + radius * 1.0,
    center.z + radius * 1.4,
  ];

  const scaledLayers = useMemo(
    () =>
      layers.map((l) => ({
        ...l,
        size: (l.size ?? 0.05) * sizeMult,
      })),
    [layers, sizeMult],
  );

  return (
    <div className="relative h-full w-full">
      <Canvas
        style={{ background }}
        camera={{
          position: initialPos,
          fov: 50,
          near: Math.max(radius * 0.001, 0.0001),
          far: radius * 50 + 1000,
        }}
        dpr={[1, 2]}
      >
        <ambientLight intensity={0.6} />
        {scaledLayers.map((l, idx) =>
          l.boundsOnly ? null : <PointsLayer key={idx} layer={l} onPick={onPick} />,
        )}
        {lines?.map((l, idx) => <LinesLayer key={`l${idx}`} layer={l} />)}
        <axesHelper args={[Math.max(0.05, radius * 0.2)]} />
        <ViewFrame
          cx={center.x}
          cy={center.y}
          cz={center.z}
          radius={radius}
        />
        <PickConfig radius={radius} pickable={!!onPick} />
        <OrbitControls enableDamping dampingFactor={0.08} makeDefault />
        {sync && <CameraSyncBridge store={sync} />}
        <GizmoHelper alignment="bottom-right" margin={[64, 64]}>
          <GizmoViewport
            axisColors={["#ef4444", "#22c55e", "#3b82f6"]}
            labelColor="#e2e8f0"
          />
        </GizmoHelper>
      </Canvas>

      <PointSizeControl value={sizeMult} onChange={setSizeMult} />
      {onPick && <PickHint />}
    </div>
  );
}

function PickHint() {
  return (
    <div className="absolute right-3 top-3 z-10 rounded-md border border-[color:rgba(250,204,21,0.4)] bg-[color:rgba(250,204,21,0.08)] px-2 py-1 text-[10px] code-font text-[#facc15]">
      click to set query
    </div>
  );
}

function PointSizeControl({
  value,
  onChange,
}: {
  value: number;
  onChange: (v: number) => void;
}) {
  return (
    <div
      className="absolute left-3 top-3 z-10 flex items-center gap-2 rounded-md border border-[var(--border)] bg-[color:rgba(10,15,26,0.7)] px-2.5 py-1 backdrop-blur-sm"
      onPointerDown={(e) => e.stopPropagation()}
      onWheel={(e) => e.stopPropagation()}
    >
      <span className="code-font text-[10px] uppercase tracking-wider text-[var(--mut)]">
        pt
      </span>
      <input
        type="range"
        min={0.25}
        max={8}
        step={0.05}
        value={value}
        onChange={(e) => onChange(parseFloat(e.target.value))}
        className="w-24 sm:w-28"
        aria-label="Point size"
      />
      <span className="code-font min-w-[2.6em] text-right text-[10px] text-[var(--text)]">
        {value.toFixed(2)}×
      </span>
    </div>
  );
}

function ViewFrame({
  cx,
  cy,
  cz,
  radius,
}: {
  cx: number;
  cy: number;
  cz: number;
  radius: number;
}) {
  const camera = useThree((s) => s.camera);
  const controls = useThree((s) => s.controls) as
    | (THREE.EventDispatcher & {
        target: THREE.Vector3;
        update?: () => void;
      })
    | null;

  useEffect(() => {
    if (!camera) return;
    camera.position.set(
      cx + radius * 1.6,
      cy + radius * 1.0,
      cz + radius * 1.4,
    );
    if ((camera as THREE.PerspectiveCamera).isPerspectiveCamera) {
      const c = camera as THREE.PerspectiveCamera;
      c.near = Math.max(radius * 0.001, 0.0001);
      c.far = radius * 50 + 1000;
      c.updateProjectionMatrix();
    }
    if (controls?.target) {
      controls.target.set(cx, cy, cz);
      controls.update?.();
    }
  }, [cx, cy, cz, radius, camera, controls]);

  return null;
}

/** Bridge between this viewer's OrbitControls and a shared store, so two
 *  viewers can keep their cameras locked together. */
function CameraSyncBridge({ store }: { store: CameraSyncStore }) {
  const camera = useThree((s) => s.camera);
  const controls = useThree((s) => s.controls) as
    | (THREE.EventDispatcher & {
        target: THREE.Vector3;
        update?: () => void;
        addEventListener: (type: string, fn: () => void) => void;
        removeEventListener: (type: string, fn: () => void) => void;
      })
    | null;

  // Per-instance ID — used to ignore our own echoed publishes.
  const idRef = useRef<number | null>(null);
  if (idRef.current === null) idRef.current = nextSyncId();

  // Tracks the most recently *applied* remote state. The change handler
  // checks against this to filter out events that are just our own
  // remote-applied updates being re-broadcast through OrbitControls.
  const lastAppliedRef = useRef<{
    pos: THREE.Vector3;
    tgt: THREE.Vector3;
  } | null>(null);

  // Tracks whether the local user is actively dragging this viewer's
  // OrbitControls. We only broadcast while userActive=true, so frame-rate
  // damping ticks and remote-applied jumps don't echo back into the loop.
  const userActiveRef = useRef(false);

  useEffect(() => {
    if (!camera || !controls) return;
    return store.subscribe((e) => {
      if (e.sourceId === idRef.current) return;
      // Set the guard *before* mutating the camera so the synchronous
      // "change" event that controls.update() fires sees the right ref.
      lastAppliedRef.current = {
        pos: e.pos.clone(),
        tgt: e.tgt.clone(),
      };
      camera.position.copy(e.pos);
      controls.target.copy(e.tgt);
      controls.update?.();
    });
  }, [store, camera, controls]);

  useEffect(() => {
    if (!camera || !controls) return;
    const onStart = () => {
      userActiveRef.current = true;
    };
    const onEnd = () => {
      userActiveRef.current = false;
    };
    const onChange = () => {
      // Only the viewer the user is currently dragging publishes — keeps
      // the bus quiet during damping interpolation and remote-applied
      // updates, which in turn prevents echo loops.
      if (!userActiveRef.current) return;
      const last = lastAppliedRef.current;
      if (
        last &&
        camera.position.distanceTo(last.pos) < 1e-4 &&
        controls.target.distanceTo(last.tgt) < 1e-4
      ) {
        return;
      }
      store.publish({
        pos: camera.position.clone(),
        tgt: controls.target.clone(),
        sourceId: idRef.current!,
      });
      lastAppliedRef.current = {
        pos: camera.position.clone(),
        tgt: controls.target.clone(),
      };
    };
    controls.addEventListener("start", onStart);
    controls.addEventListener("end", onEnd);
    controls.addEventListener("change", onChange);
    return () => {
      controls.removeEventListener("start", onStart);
      controls.removeEventListener("end", onEnd);
      controls.removeEventListener("change", onChange);
    };
  }, [store, camera, controls]);

  return null;
}

function PickConfig({ radius, pickable }: { radius: number; pickable: boolean }) {
  const raycaster = useThree((s) => s.raycaster);
  useEffect(() => {
    if (!raycaster) return;
    raycaster.params.Points = {
      threshold: pickable ? Math.max(radius * 0.008, 0.0005) : 0,
    };
  }, [raycaster, radius, pickable]);
  return null;
}

function PointsLayer({
  layer,
  onPick,
}: {
  layer: Layer;
  onPick?: (xyz: [number, number, number]) => void;
}) {
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

  return (
    <points
      geometry={geom}
      material={mat}
      onClick={
        onPick
          ? (e) => {
              e.stopPropagation();
              onPick([e.point.x, e.point.y, e.point.z]);
            }
          : undefined
      }
    />
  );
}

function LinesLayer({ layer }: { layer: LineLayer }) {
  const geom = useMemo(() => {
    const g = new THREE.BufferGeometry();
    g.setAttribute("position", new THREE.BufferAttribute(layer.positions, 3));
    return g;
  }, [layer.positions]);

  const mat = useMemo(
    () =>
      new THREE.LineBasicMaterial({
        color: new THREE.Color(layer.color),
        transparent: (layer.opacity ?? 1) < 1,
        opacity: layer.opacity ?? 1,
      }),
    [layer.color, layer.opacity],
  );

  return <lineSegments geometry={geom} material={mat} />;
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
    return { center: new THREE.Vector3(0, 0, 0), radius: 10 };
  }
  const cx = (minX + maxX) / 2;
  const cy = (minY + maxY) / 2;
  const cz = (minZ + maxZ) / 2;
  const r = Math.max(maxX - minX, maxY - minY, maxZ - minZ) * 0.6 + 0.05;
  return { center: new THREE.Vector3(cx, cy, cz), radius: r };
}
