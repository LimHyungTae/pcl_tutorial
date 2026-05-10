import { createHashRouter, Navigate } from "react-router-dom";
import AppLayout from "./layout/AppLayout";
import Home from "./pages/Home";
import Lec03Transformation from "./pages/Lec03Transformation";
import Lec05Voxelization from "./pages/Lec05Voxelization";
import Lec06PassThrough from "./pages/Lec06PassThrough";
import Lec07Sor from "./pages/Lec07Sor";
import Lec08RadiusSearch from "./pages/Lec08RadiusSearch";
import Lec09Knn from "./pages/Lec09Knn";
import Lec10Normal from "./pages/Lec10Normal";
import Lec11Icp from "./pages/Lec11Icp";
import Extra01RansacPlane from "./pages/Extra01RansacPlane";
import Extra02EuclideanCluster from "./pages/Extra02EuclideanCluster";
import Extra03Patchwork from "./pages/Extra03Patchwork";
import Extra04Travel from "./pages/Extra04Travel";
import Extra05GenzIcp from "./pages/Extra05GenzIcp";

// Hash routing keeps GH Pages happy: no 404 on deep-linked refresh.
//
// Slugs are the human-readable algorithm names (e.g. /voxelization). The
// previous codename routes (/lec05, /extra01, ...) redirect to the new
// names so any existing link or bookmark still lands on the right page.
export const router = createHashRouter([
  {
    path: "/",
    element: <AppLayout />,
    children: [
      { index: true, element: <Home /> },

      // Current routes.
      { path: "transformation", element: <Lec03Transformation /> },
      { path: "voxelization", element: <Lec05Voxelization /> },
      { path: "pass-through", element: <Lec06PassThrough /> },
      { path: "statistical-outlier-removal", element: <Lec07Sor /> },
      { path: "radius-search", element: <Lec08RadiusSearch /> },
      { path: "k-nearest-neighbor", element: <Lec09Knn /> },
      { path: "normal-estimation", element: <Lec10Normal /> },
      { path: "iterative-closest-point", element: <Lec11Icp /> },
      { path: "ransac-plane-segmentation", element: <Extra01RansacPlane /> },
      { path: "euclidean-clustering", element: <Extra02EuclideanCluster /> },
      { path: "patchwork", element: <Extra03Patchwork /> },
      { path: "travel", element: <Extra04Travel /> },
      { path: "genz-icp", element: <Extra05GenzIcp /> },

      // Legacy redirects — keep old links working.
      { path: "lec03", element: <Navigate to="/transformation" replace /> },
      { path: "lec05", element: <Navigate to="/voxelization" replace /> },
      { path: "lec06", element: <Navigate to="/pass-through" replace /> },
      { path: "lec07", element: <Navigate to="/statistical-outlier-removal" replace /> },
      { path: "lec08", element: <Navigate to="/radius-search" replace /> },
      { path: "lec09", element: <Navigate to="/k-nearest-neighbor" replace /> },
      { path: "lec10", element: <Navigate to="/normal-estimation" replace /> },
      { path: "lec11", element: <Navigate to="/iterative-closest-point" replace /> },
      { path: "extra01", element: <Navigate to="/ransac-plane-segmentation" replace /> },
      { path: "extra02", element: <Navigate to="/euclidean-clustering" replace /> },
      { path: "extra03", element: <Navigate to="/patchwork" replace /> },
      { path: "extra04", element: <Navigate to="/travel" replace /> },
      { path: "extra05", element: <Navigate to="/genz-icp" replace /> },

      { path: "*", element: <Navigate to="/" replace /> },
    ],
  },
]);
