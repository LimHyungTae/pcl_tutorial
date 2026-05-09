<div align="center">
<h1>PCL Tutorial</h1>

<a href="https://limhyungtae.github.io/pcl_tutorial/"><img src="https://img.shields.io/badge/Live%20Demo-limhyungtae.github.io%2Fpcl__tutorial-10b981.svg" alt="Live Demo"></a>
<img src="https://img.shields.io/badge/WebGL-via%20three.js-990000.svg" alt="WebGL">
<img src="https://img.shields.io/badge/Language-C%2B%2B17-blue.svg" alt="C++17">
<img src="https://img.shields.io/badge/PCL-%E2%89%A51.8-brightgreen.svg" alt="PCL">
<img src="https://img.shields.io/badge/CMake-%E2%89%A53.10-064F8C.svg" alt="CMake">
<img src="https://img.shields.io/badge/License-MIT-yellow.svg" alt="MIT">

<p align="center"><img src="https://github.com/user-attachments/assets/5b727855-353f-4a10-baa1-89ec9f019187" alt="thumbnail" width="80%"/></p>

<p><strong><em>A hands-on, browser-based tutorial for the Point Cloud Library.</em></strong></p>
</div>

**English** · [한국어](README_kr.md)

______________________________________________________________________

## :rocket: Overview

This tutorial focuses on **how to actually use PCL well** rather than deriving optimization math or diving deep into C++ syntax.
Each chapter is paired 1:1 with a [blog post](https://limhyungtae.github.io/), and this repo serves as the maintained home for every code sample referenced in those posts.

- Original author: Hyungtae Lim (`shapelim` at `mit` dot `edu`)
- Tested OS: Ubuntu 24.04

______________________________________________________________________

## :sparkles: Interactive Demo (`web/`)

Play with the sliders right in your browser to see each chapter in action — **<https://limhyungtae.github.io/pcl_tutorial/>**.

Every interactive demo runs **entirely in the browser**, with the algorithms reimplemented in TypeScript (no remote compute). Inputs include KITTI (outdoor LiDAR), NaverLabs (indoor LiDAR), and Stanford Bunny (CAD) presets, plus drag-and-drop for your own data.

| #  | Chapter                               | Highlights                                               |
| -- | ------------------------------------- | -------------------------------------------------------- |
| 1  | Voxelization                          | leaf size slider, synced before/after view               |
| 2  | PassThrough                           | axis + min/max + negative toggle, synced before/after    |
| 3  | Transformation                        | tx/ty/tz/rx/ry/rz, random offset on entry                |
| 4  | Statistical Outlier Removal           | mean K + stddev mult, inlier/outlier split               |
| 5  | Radius Search                         | KdTree-based; click any point to set the query           |
| 6  | K-Nearest Neighbor                    | KdTree-based K-NN; click any point to set the query      |
| 7  | Normal Estimation                     | KdTree + SVD; normals overlaid as white line segments    |
| 8  | RANSAC Plane Segmentation             | ground / wall detection (extra)                          |
| 9  | **Patchwork** — Ground Segmentation   | CZM zones × rings colored per region (VLP-16 / HDL-64)   |
| 10 | Euclidean Clustering                  | stable per-cluster colors across slider changes (extra)  |
| 11 | **TRAVEL** — Range Image Clustering   | 3D + 2D range image, same cluster colors (VLP-16 / HDL-64) |
| 12 | Iterative Closest Point               | **Step / Play to watch each iteration's correspondences** |

> Chapters 9 and 11 are sensor-specific demos: parameters are baked in for VLP-16 (NaverLabs preset) and HDL-64 (KITTI preset) only. Each page links out to the upstream [Patchwork](https://github.com/LimHyungTae/patchwork) and [TRAVEL](https://github.com/url-kaist/TRAVEL) repos for running on your own data.

### Run the site locally

```bash
cd web
npm install
npm run dev          # http://localhost:5173/pcl_tutorial/
```

The `predev` / `prebuild` hooks automatically copy data files (`.bin` / `.pcd` / `.ply`) from `materials/` into `web/public/data/`.

### Deployment

When changes under `web/**` are pushed to `main`, GitHub Actions (`.github/workflows/deploy.yml`) builds the site and deploys it to GitHub Pages. Before the first deploy, set the repo's **Settings → Pages → Source** to **GitHub Actions**.

______________________________________________________________________

## :hammer: Prerequisites & Build

### Prerequisites

- CMake ≥ 3.10
- PCL ≥ 1.8
- Boost ≥ 1.58

### Build

```bash
mkdir build && cd build
cmake ..
make -j$(nproc)
```

The build step copies the data files (`.bin`, `.pcd`, `.ply`) under `materials/` into `build/auxiliary/`, so always run the executables **from inside** the `build/` directory:

```bash
cd build
./lec00_usage
./lec04_visualization
# ...
```

______________________________________________________________________

## :books: Chapters & Source (Blog posts are written in Korean)

| #     | Code                                                                                | Topic                               | Blog post                                                                                                                                                                                                  |
| :---: | :-----------------------------------------------------------------------------------: | :-----------------------------------: | :---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| 0     | [`lec00_usage.cpp`](lec00_usage.cpp)                                                | PCL basics                          | [0. Tutorial 및 기본 사용법](https://limhyungtae.github.io/2021-09-09-ROS-Point-Cloud-Library-(PCL)-0.-Tutorial-및-기본-사용법/)                                                                          |
| 1-1   | [`lec01_1_shared_ptr.cpp`](lec01_1_shared_ptr.cpp)                                  | `shared_ptr` fundamentals           | [1-(1). Ptr/ConstPtr의 완벽 이해 — `shared_ptr`](https://limhyungtae.github.io/2021-09-09-ROS-Point-Cloud-Library-(PCL)-1.-Ptr,-ConstPtr의-완벽-이해-(1)-shared_ptr/)                                     |
| 1-2   | [`lec01_2_ptr.cpp`](lec01_2_ptr.cpp)                                                | Using Ptr in PCL                    | [1-(2). Ptr/ConstPtr의 완벽 이해 — Ptr in PCL](https://limhyungtae.github.io/2021-09-10-ROS-Point-Cloud-Library-(PCL)-1.-Ptr,-ConstPtr의-완벽-이해-(2)-Ptr-in-PCL/)                                       |
| 1-3   | [`lec01_3_ptr_in_class.cpp`](lec01_3_ptr_in_class.cpp)                              | Ptr as a class member               | [1-(3). Ptr/ConstPtr의 완벽 이해 — Ptr in 클래스 멤버변수](https://limhyungtae.github.io/2021-09-10-ROS-Point-Cloud-Library-(PCL)-1.-Ptr,-ConstPtr의-완벽-이해-(3)-Ptr-in-클래스-멤버변수/)               |
| 2     | *(no code)*                                                                         | Conversions — `toROSMsg` / `fromROSMsg` | [2. 형변환 — toROSMsg, fromROSMsg](https://limhyungtae.github.io/2021-09-10-ROS-Point-Cloud-Library-(PCL)-2.-형변환-toROSMsg,-fromROSMsg/)                                                                |
| 3     | [`lec03_transformation.cpp`](lec03_transformation.cpp)                              | 4×4 matrix transforms               | [3. Transformation](https://limhyungtae.github.io/2021-09-10-ROS-Point-Cloud-Library-(PCL)-3.-Transformation/)                                                                                              |
| 4     | [`lec04_visualization.cpp`](lec04_visualization.cpp)                                | Using PCLVisualizer                 | [4. Viewer로 visualization하는 법](https://limhyungtae.github.io/2021-09-10-ROS-Point-Cloud-Library-(PCL)-4.-Viewer로-visualization하는-법/)                                                              |
| 5     | [`lec05_voxelization.cpp`](lec05_voxelization.cpp)                                  | Voxel Grid downsampling             | [5. Voxelization](https://limhyungtae.github.io/2021-09-12-ROS-Point-Cloud-Library-(PCL)-5.-Voxelization/)                                                                                                  |
| 6     | [`lec06_pass_through.cpp`](lec06_pass_through.cpp)                                  | PassThrough filtering               | [6. PassThrough](https://limhyungtae.github.io/2021-09-12-ROS-Point-Cloud-Library-(PCL)-6.-PassThrough/)                                                                                                    |
| 7     | [`lec07_sor.cpp`](lec07_sor.cpp)                                                    | Statistical Outlier Removal         | [7. Statistical Outlier Removal](https://limhyungtae.github.io/2021-09-12-ROS-Point-Cloud-Library-(PCL)-7.-Statistical-Outlier-Removal/)                                                                    |
| 8     | [`lec08_radius_search.cpp`](lec08_radius_search.cpp)                                | KdTree-based radius search          | [8. KdTree를 활용한 Radius Search](https://limhyungtae.github.io/2021-09-12-ROS-Point-Cloud-Library-(PCL)-8.-KdTree를-활용한-Radius-Search/)                                                              |
| 9     | [`lec09_knn.cpp`](lec09_knn.cpp)                                                    | K-Nearest Neighbor search           | [9. KdTree를 활용한 K-NN Search](https://limhyungtae.github.io/2021-09-12-ROS-Point-Cloud-Library-(PCL)-9.-KdTree를-활용한-K-nearest-Neighbor-Search-(KNN)/)                                              |
| 10-1  | [`lec10_1_normal.cpp`](lec10_1_normal.cpp)                                          | Normal estimation via KdTree + SVD  | [10. Normal Estimation](https://limhyungtae.github.io/2021-09-13-ROS-Point-Cloud-Library-(PCL)-10.-Normal-Estimation/)                                                                                      |
| 10-2  | [`lec10_2_normal_corner.cpp`](lec10_2_normal_corner.cpp)                            | Normal computation in a simple case | (corner-case example for the post above)                                                                                                                                                                  |
| 11    | [`lec11_icp.cpp`](lec11_icp.cpp)                                                    | Iterative Closest Point             | [11. Iterative Closest Point (ICP)](https://limhyungtae.github.io/2021-09-14-ROS-Point-Cloud-Library-(PCL)-11.-Iterative-Closest-Point-(ICP)/)                                                              |
| 12    | [`lec12_gicp.cpp`](lec12_gicp.cpp)                                                  | Generalized ICP                     | [12. Generalized ICP (G-ICP)](https://limhyungtae.github.io/2021-09-14-ROS-Point-Cloud-Library-(PCL)-12.-Generalized-Iterative-Closest-Point-(G-ICP)/)                                                      |

> Note: `auxiliary/pass_by_address.cpp` is a side snippet (a Ceres + Eigen learning example) and is not part of the main build.

______________________________________________________________________

## :file_folder: Directory layout

```
pcl_tutorial/
├── CMakeLists.txt
├── lec*.cpp                         # per-chapter examples (PCL/C++)
├── auxiliary/                       # side snippets, not part of the main build
├── img/                             # images for README and blog posts
├── materials/                       # sample point cloud data (.bin / .pcd / .ply)
└── web/                             # interactive site (React + Vite)
    ├── src/
    │   ├── pages/                   # one page per chapter
    │   ├── components/              # viewer, sliders, drop zone
    │   ├── i18n/                    # EN/KO dictionaries + toggle
    │   └── lib/                     # KdTree, PCD/BIN/PLY parsers,
    │       └── filters/             # voxel · passthrough · sor · normal ·
    │                                #   ransacPlane · euclideanCluster · icp ·
    │                                #   transform (all reimplemented in TS)
    └── public/data/                 # auto-copied from materials/ at build time
```

______________________________________________________________________

## :page_facing_up: License

MIT License. See [`package.xml`](package.xml) for the license declaration.
