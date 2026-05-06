import { createHashRouter, Navigate } from "react-router-dom";
import AppLayout from "./layout/AppLayout";
import Home from "./pages/Home";
import Lec05Voxelization from "./pages/Lec05Voxelization";
import Lec06PassThrough from "./pages/Lec06PassThrough";
import Lec11Icp from "./pages/Lec11Icp";
import StubPage from "./pages/StubPage";
import { chapters } from "./chapters";

// Hash routing keeps GH Pages happy: no 404 on deep-linked refresh.
export const router = createHashRouter([
  {
    path: "/",
    element: <AppLayout />,
    children: [
      { index: true, element: <Home /> },
      { path: "lec05", element: <Lec05Voxelization /> },
      { path: "lec06", element: <Lec06PassThrough /> },
      { path: "lec11", element: <Lec11Icp /> },
      ...chapters
        .filter((c) => !["lec05", "lec06", "lec11"].includes(c.slug))
        .map((c) => ({
          path: c.slug,
          element: <StubPage chapter={c} />,
        })),
      { path: "*", element: <Navigate to="/" replace /> },
    ],
  },
]);
