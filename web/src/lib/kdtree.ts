/**
 * Static, array-backed 3D KD-tree.
 *
 * Built once over a Float32Array of [x,y,z,...] points. Internally we store
 * a permutation `perm` and a parallel array of "cut axis" choices for each
 * tree node (recorded implicitly by the recursive median split — the axis
 * cycles 0,1,2,0,1,2,... at depths 0,1,2,3,...).
 */
export class KdTree {
  readonly positions: Float32Array;
  readonly perm: Uint32Array; // index → original point index
  readonly count: number;

  constructor(positions: Float32Array) {
    this.positions = positions;
    this.count = positions.length / 3;
    this.perm = new Uint32Array(this.count);
    for (let i = 0; i < this.count; i++) this.perm[i] = i;
    if (this.count > 0) this.build(0, this.count - 1, 0);
  }

  /** All point indices within `radius` of (qx, qy, qz). */
  radiusSearch(qx: number, qy: number, qz: number, radius: number): number[] {
    const r2 = radius * radius;
    const out: number[] = [];
    if (this.count === 0) return out;
    this.recurseRadius(0, this.count - 1, 0, qx, qy, qz, r2, out);
    return out;
  }

  /**
   * K nearest neighbors of (qx, qy, qz). Returns indices sorted by distance
   * (closest first), plus the squared distances.
   */
  knnSearch(qx: number, qy: number, qz: number, k: number) {
    const heap = new MaxHeap(k); // by squared distance
    if (this.count === 0) return { indices: [] as number[], sqrDists: [] as number[] };
    this.recurseKnn(0, this.count - 1, 0, qx, qy, qz, heap);

    const arr = heap.entries.slice(0, heap.size);
    arr.sort((a, b) => a.d - b.d);
    return {
      indices: arr.map((e) => e.i),
      sqrDists: arr.map((e) => e.d),
    };
  }

  // ---- internals ----

  private build(lo: number, hi: number, depth: number): void {
    if (lo >= hi) return;
    const axis = depth % 3;
    const mid = (lo + hi) >> 1;
    this.nthElement(lo, hi, mid, axis);
    this.build(lo, mid - 1, depth + 1);
    this.build(mid + 1, hi, depth + 1);
  }

  /** Quickselect-style partition: places median at `mid`. */
  private nthElement(lo: number, hi: number, mid: number, axis: number): void {
    while (lo < hi) {
      const pivot = this.partition(lo, hi, axis);
      if (pivot === mid) return;
      if (pivot < mid) lo = pivot + 1;
      else hi = pivot - 1;
    }
  }

  private partition(lo: number, hi: number, axis: number): number {
    // Hoare-like partition with center-pivot.
    const pivotIdx = (lo + hi) >> 1;
    const pivotVal = this.coord(pivotIdx, axis);
    this.swap(pivotIdx, hi);
    let store = lo;
    for (let i = lo; i < hi; i++) {
      if (this.coord(i, axis) < pivotVal) {
        this.swap(i, store);
        store++;
      }
    }
    this.swap(store, hi);
    return store;
  }

  private coord(i: number, axis: number): number {
    return this.positions[this.perm[i] * 3 + axis];
  }

  private swap(a: number, b: number): void {
    const t = this.perm[a];
    this.perm[a] = this.perm[b];
    this.perm[b] = t;
  }

  private sqrDistTo(idx: number, qx: number, qy: number, qz: number): number {
    const off = this.perm[idx] * 3;
    const dx = this.positions[off] - qx;
    const dy = this.positions[off + 1] - qy;
    const dz = this.positions[off + 2] - qz;
    return dx * dx + dy * dy + dz * dz;
  }

  private recurseRadius(
    lo: number,
    hi: number,
    depth: number,
    qx: number,
    qy: number,
    qz: number,
    r2: number,
    out: number[],
  ): void {
    if (lo > hi) return;
    const mid = (lo + hi) >> 1;
    const axis = depth % 3;
    const split = this.coord(mid, axis);
    const q = axis === 0 ? qx : axis === 1 ? qy : qz;
    const diff = q - split;

    if (this.sqrDistTo(mid, qx, qy, qz) <= r2) out.push(this.perm[mid]);

    if (diff <= 0) {
      this.recurseRadius(lo, mid - 1, depth + 1, qx, qy, qz, r2, out);
      if (diff * diff <= r2) {
        this.recurseRadius(mid + 1, hi, depth + 1, qx, qy, qz, r2, out);
      }
    } else {
      this.recurseRadius(mid + 1, hi, depth + 1, qx, qy, qz, r2, out);
      if (diff * diff <= r2) {
        this.recurseRadius(lo, mid - 1, depth + 1, qx, qy, qz, r2, out);
      }
    }
  }

  private recurseKnn(
    lo: number,
    hi: number,
    depth: number,
    qx: number,
    qy: number,
    qz: number,
    heap: MaxHeap,
  ): void {
    if (lo > hi) return;
    const mid = (lo + hi) >> 1;
    const axis = depth % 3;
    const split = this.coord(mid, axis);
    const q = axis === 0 ? qx : axis === 1 ? qy : qz;
    const diff = q - split;

    heap.push(this.perm[mid], this.sqrDistTo(mid, qx, qy, qz));

    if (diff <= 0) {
      this.recurseKnn(lo, mid - 1, depth + 1, qx, qy, qz, heap);
      if (!heap.full() || diff * diff <= heap.worst()) {
        this.recurseKnn(mid + 1, hi, depth + 1, qx, qy, qz, heap);
      }
    } else {
      this.recurseKnn(mid + 1, hi, depth + 1, qx, qy, qz, heap);
      if (!heap.full() || diff * diff <= heap.worst()) {
        this.recurseKnn(lo, mid - 1, depth + 1, qx, qy, qz, heap);
      }
    }
  }
}

/** A simple bounded max-heap over (index, sqr-distance) pairs. */
class MaxHeap {
  readonly entries: { i: number; d: number }[];
  size = 0;

  constructor(public capacity: number) {
    this.entries = new Array(capacity);
  }

  full(): boolean {
    return this.size >= this.capacity;
  }

  worst(): number {
    return this.size === 0 ? Infinity : this.entries[0].d;
  }

  push(idx: number, d: number): void {
    if (this.size < this.capacity) {
      this.entries[this.size] = { i: idx, d };
      this.size++;
      this.bubbleUp(this.size - 1);
    } else if (d < this.entries[0].d) {
      this.entries[0] = { i: idx, d };
      this.bubbleDown(0);
    }
  }

  private bubbleUp(i: number): void {
    while (i > 0) {
      const p = (i - 1) >> 1;
      if (this.entries[p].d < this.entries[i].d) {
        const tmp = this.entries[p];
        this.entries[p] = this.entries[i];
        this.entries[i] = tmp;
        i = p;
      } else break;
    }
  }

  private bubbleDown(i: number): void {
    while (true) {
      const l = 2 * i + 1;
      const r = 2 * i + 2;
      let m = i;
      if (l < this.size && this.entries[l].d > this.entries[m].d) m = l;
      if (r < this.size && this.entries[r].d > this.entries[m].d) m = r;
      if (m === i) break;
      const tmp = this.entries[m];
      this.entries[m] = this.entries[i];
      this.entries[i] = tmp;
      i = m;
    }
  }
}
