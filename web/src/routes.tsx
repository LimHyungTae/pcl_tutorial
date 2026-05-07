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
import Extra03GroundRemovalPipeline from "./pages/Extra03GroundRemovalPipeline";

// Hash routing keeps GH Pages happy: no 404 on deep-linked refresh.
export const router = createHashRouter([
  {
    path: "/",
    element: <AppLayout />,
    children: [
      { index: true, element: <Home /> },
      { path: "lec03", element: <Lec03Transformation /> },
      { path: "lec05", element: <Lec05Voxelization /> },
      { path: "lec06", element: <Lec06PassThrough /> },
      { path: "lec07", element: <Lec07Sor /> },
      { path: "lec08", element: <Lec08RadiusSearch /> },
      { path: "lec09", element: <Lec09Knn /> },
      { path: "lec10", element: <Lec10Normal /> },
      { path: "lec11", element: <Lec11Icp /> },
      { path: "extra01", element: <Extra01RansacPlane /> },
      { path: "extra03", element: <Extra03GroundRemovalPipeline /> },
      { path: "extra02", element: <Extra02EuclideanCluster /> },
      { path: "*", element: <Navigate to="/" replace /> },
    ],
  },
]);
