import { createHashRouter, Navigate } from "react-router-dom";
import AppLayout from "./layout/AppLayout";
import Home from "./pages/Home";
import Lec05Voxelization from "./pages/Lec05Voxelization";
import Lec06PassThrough from "./pages/Lec06PassThrough";
import Lec07Sor from "./pages/Lec07Sor";
import Lec08RadiusSearch from "./pages/Lec08RadiusSearch";
import Lec09Knn from "./pages/Lec09Knn";
import Lec10Normal from "./pages/Lec10Normal";
import Lec11Icp from "./pages/Lec11Icp";
import Lec12Gicp from "./pages/Lec12Gicp";
import Extra01RansacPlane from "./pages/Extra01RansacPlane";
import Extra02EuclideanCluster from "./pages/Extra02EuclideanCluster";

// Hash routing keeps GH Pages happy: no 404 on deep-linked refresh.
export const router = createHashRouter([
  {
    path: "/",
    element: <AppLayout />,
    children: [
      { index: true, element: <Home /> },
      { path: "lec05", element: <Lec05Voxelization /> },
      { path: "lec06", element: <Lec06PassThrough /> },
      { path: "lec07", element: <Lec07Sor /> },
      { path: "lec08", element: <Lec08RadiusSearch /> },
      { path: "lec09", element: <Lec09Knn /> },
      { path: "lec10", element: <Lec10Normal /> },
      { path: "lec11", element: <Lec11Icp /> },
      { path: "lec12", element: <Lec12Gicp /> },
      { path: "extra01", element: <Extra01RansacPlane /> },
      { path: "extra02", element: <Extra02EuclideanCluster /> },
      { path: "*", element: <Navigate to="/" replace /> },
    ],
  },
]);
