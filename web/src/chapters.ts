export type ChapterStatus = "interactive" | "precomputed" | "code" | "stub";

export type Chapter = {
  slug: string;
  number: string;
  title: string;
  subtitle: string;
  source: string; // path to the .cpp file at the repo root
  blog: string; // full URL to the matching blog post (or "" if none)
  status: ChapterStatus;
};

const BLOG = "https://limhyungtae.github.io";

export const chapters: Chapter[] = [
  {
    slug: "lec00",
    number: "0",
    title: "PCL 기본 사용법",
    subtitle: "PointCloud 선언, point 타입, 합치기",
    source: "lec00_usage.cpp",
    blog: `${BLOG}/2021-09-09-ROS-Point-Cloud-Library-(PCL)-0.-Tutorial-및-기본-사용법/`,
    status: "code",
  },
  {
    slug: "lec01_1",
    number: "1-1",
    title: "shared_ptr 기본",
    subtitle: "C++ shared_ptr 동작 이해",
    source: "lec01_1_shared_ptr.cpp",
    blog: `${BLOG}/2021-09-09-ROS-Point-Cloud-Library-(PCL)-1.-Ptr,-ConstPtr의-완벽-이해-(1)-shared_ptr/`,
    status: "code",
  },
  {
    slug: "lec01_2",
    number: "1-2",
    title: "PCL에서의 Ptr",
    subtitle: "Ptr / ConstPtr 사용법, 주소 공유 시 함정",
    source: "lec01_2_ptr.cpp",
    blog: `${BLOG}/2021-09-10-ROS-Point-Cloud-Library-(PCL)-1.-Ptr,-ConstPtr의-완벽-이해-(2)-Ptr-in-PCL/`,
    status: "code",
  },
  {
    slug: "lec01_3",
    number: "1-3",
    title: "클래스 멤버변수 Ptr",
    subtitle: "선언과 .reset()을 통한 초기화",
    source: "lec01_3_ptr_in_class.cpp",
    blog: `${BLOG}/2021-09-10-ROS-Point-Cloud-Library-(PCL)-1.-Ptr,-ConstPtr의-완벽-이해-(3)-Ptr-in-클래스-멤버변수/`,
    status: "code",
  },
  {
    slug: "lec03",
    number: "3",
    title: "Transformation",
    subtitle: "4×4 행렬 기반 변환",
    source: "lec03_transformation.cpp",
    blog: `${BLOG}/2021-09-10-ROS-Point-Cloud-Library-(PCL)-3.-Transformation/`,
    status: "code",
  },
  {
    slug: "lec04",
    number: "4",
    title: "Visualization",
    subtitle: "PCLVisualizer 사용법",
    source: "lec04_visualization.cpp",
    blog: `${BLOG}/2021-09-10-ROS-Point-Cloud-Library-(PCL)-4.-Viewer로-visualization하는-법/`,
    status: "code",
  },
  {
    slug: "lec05",
    number: "5",
    title: "Voxelization",
    subtitle: "Voxel Grid 필터링 — 크기를 슬라이더로 조절",
    source: "lec05_voxelization.cpp",
    blog: `${BLOG}/2021-09-12-ROS-Point-Cloud-Library-(PCL)-5.-Voxelization/`,
    status: "interactive",
  },
  {
    slug: "lec06",
    number: "6",
    title: "PassThrough",
    subtitle: "축별 박스 필터로 점군 잘라내기",
    source: "lec06_pass_through.cpp",
    blog: `${BLOG}/2021-09-12-ROS-Point-Cloud-Library-(PCL)-6.-PassThrough/`,
    status: "interactive",
  },
  {
    slug: "lec07",
    number: "7",
    title: "Statistical Outlier Removal",
    subtitle: "이웃 거리 분포로 outlier 제거",
    source: "lec07_sor.cpp",
    blog: `${BLOG}/2021-09-12-ROS-Point-Cloud-Library-(PCL)-7.-Statistical-Outlier-Removal/`,
    status: "stub",
  },
  {
    slug: "lec08",
    number: "8",
    title: "Radius Search",
    subtitle: "KdTree로 반경 내 이웃 찾기",
    source: "lec08_radius_search.cpp",
    blog: `${BLOG}/2021-09-12-ROS-Point-Cloud-Library-(PCL)-8.-KdTree를-활용한-Radius-Search/`,
    status: "stub",
  },
  {
    slug: "lec09",
    number: "9",
    title: "K-Nearest Neighbor",
    subtitle: "KdTree로 K개 최근접점 찾기",
    source: "lec09_knn.cpp",
    blog: `${BLOG}/2021-09-12-ROS-Point-Cloud-Library-(PCL)-9.-KdTree를-활용한-K-nearest-Neighbor-Search-(KNN)/`,
    status: "stub",
  },
  {
    slug: "lec10",
    number: "10",
    title: "Normal Estimation",
    subtitle: "KdTree + SVD로 Normal 추정",
    source: "lec10_1_normal.cpp",
    blog: `${BLOG}/2021-09-13-ROS-Point-Cloud-Library-(PCL)-10.-Normal-Estimation/`,
    status: "stub",
  },
  {
    slug: "lec11",
    number: "11",
    title: "Iterative Closest Point",
    subtitle: "Pre-computed src ↔ tgt ↔ aligned 비교",
    source: "lec11_icp.cpp",
    blog: `${BLOG}/2021-09-14-ROS-Point-Cloud-Library-(PCL)-11.-Iterative-Closest-Point-(ICP)/`,
    status: "precomputed",
  },
  {
    slug: "lec12",
    number: "12",
    title: "Generalized ICP",
    subtitle: "공분산 기반 ICP 변형",
    source: "lec12_gicp.cpp",
    blog: `${BLOG}/2021-09-14-ROS-Point-Cloud-Library-(PCL)-12.-Generalized-Iterative-Closest-Point-(G-ICP)/`,
    status: "stub",
  },
];

export const findChapter = (slug: string) =>
  chapters.find((c) => c.slug === slug);

export const statusLabel: Record<ChapterStatus, string> = {
  interactive: "인터랙티브",
  precomputed: "Pre-computed",
  code: "코드 설명",
  stub: "준비중",
};

export const statusColor: Record<ChapterStatus, string> = {
  interactive: "bg-emerald-500/15 text-emerald-300 ring-emerald-500/30",
  precomputed: "bg-sky-500/15 text-sky-300 ring-sky-500/30",
  code: "bg-amber-500/15 text-amber-300 ring-amber-500/30",
  stub: "bg-slate-500/15 text-slate-400 ring-slate-500/30",
};
