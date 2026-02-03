# PCL Tutorial (한글.ver)

Original author: Hyungtae Lim (shapelim@kaist.ac.kr)

---

## PCL tutorial 코드의 유지보수를 위한 repository

수식적으로 optimization이나 C++ 문법에 대한 상세한 설명보다는 오롯이 **어떻게 잘 쓰는지**에 대한 tutorial

### Prerequisites

- CMake >= 3.10
- PCL >= 1.8
- Boost >= 1.58

### Compile

```bash
mkdir build && cd build
cmake ..
make -j$(nproc)
```

### 실행

빌드 후 `build` 폴더 안에서 실행 파일들을 실행할 수 있습니다:

```bash
cd build
./lec00_usage
./lec04_visualization
# etc.
```

### 코드 구성

| 파일명 | 내용 |
|--------|------|
| `lec00_usage.cpp` | PCL 기본 사용법 |
| `lec01_1_shared_ptr.cpp` | shared_ptr 사용법 |
| `lec01_2_ptr.cpp` | PCL에서의 포인터 사용 |
| `lec01_3_ptr_in_class.cpp` | 클래스 내 포인터 멤버 |
| `lec03_transformation.cpp` | Point cloud 변환 (4x4 행렬) |
| `lec04_visualization.cpp` | PCL Visualizer 사용법 |
| `lec05_voxelization.cpp` | Voxel Grid 필터링 |
| `lec06_pass_through.cpp` | PassThrough 필터링 |
| `lec07_sor.cpp` | Statistical Outlier Removal |
| `lec08_radius_search.cpp` | KdTree Radius Search |
| `lec09_knn.cpp` | K-Nearest Neighbor Search |
| `lec10_1_normal.cpp` | Normal 추정 |
| `lec10_2_normal_corner.cpp` | 간단한 Normal 계산 |
| `lec11_icp.cpp` | Iterative Closest Point |
| `lec12_gicp.cpp` | Generalized ICP |
