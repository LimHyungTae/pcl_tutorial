import * as THREE from "three";

let nextId = 0;
export const nextSyncId = (): number => ++nextId;

export type CameraSyncEvent = {
  pos: THREE.Vector3;
  tgt: THREE.Vector3;
  sourceId: number;
};

/**
 * Tiny pub-sub for syncing OrbitControls state across multiple viewers
 * (e.g. before / after panes). Each subscriber stamps publishes with its
 * own sourceId so it can ignore its own echoes.
 */
export class CameraSyncStore {
  private subs = new Set<(e: CameraSyncEvent) => void>();

  publish(e: CameraSyncEvent): void {
    this.subs.forEach((fn) => fn(e));
  }

  subscribe(fn: (e: CameraSyncEvent) => void): () => void {
    this.subs.add(fn);
    return () => {
      this.subs.delete(fn);
    };
  }
}
