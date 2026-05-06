import { useEffect, useState } from "react";
import { loadCloud } from "./pcd";
import { emptyCloud, type PointCloud } from "./types";

type State =
  | { status: "idle" }
  | { status: "loading" }
  | { status: "ready"; cloud: PointCloud }
  | { status: "error"; message: string };

/** Loads a cloud from a public URL. Re-runs when `url` changes. */
export function useCloudFromUrl(url: string): {
  cloud: PointCloud;
  loading: boolean;
  error: string | null;
} {
  const [state, setState] = useState<State>({ status: "idle" });
  useEffect(() => {
    let cancelled = false;
    setState({ status: "loading" });
    loadCloud(url)
      .then((cloud) => {
        if (!cancelled) setState({ status: "ready", cloud });
      })
      .catch((e) => {
        if (!cancelled)
          setState({
            status: "error",
            message: e instanceof Error ? e.message : String(e),
          });
      });
    return () => {
      cancelled = true;
    };
  }, [url]);

  return {
    cloud: state.status === "ready" ? state.cloud : emptyCloud(),
    loading: state.status === "loading",
    error: state.status === "error" ? state.message : null,
  };
}

/** Resolves a public-relative URL with Vite's base path baked in. */
export function asset(path: string): string {
  const base = import.meta.env.BASE_URL || "/";
  const trimmed = path.startsWith("/") ? path.slice(1) : path;
  return base.endsWith("/") ? base + trimmed : `${base}/${trimmed}`;
}
