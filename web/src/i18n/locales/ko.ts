import type { LocaleDict } from "./en";

export const ko: LocaleDict = {
  app: {
    title: "PCL Tutorial",
    tagline: "직접 만져 보는 PCL",
  },
  nav: {
    chapters: "챕터",
    github: "GitHub",
    sourceCode: "소스",
    blogPost: "블로그 글",
  },
  status: {
    interactive: "인터랙티브",
    precomputed: "Pre-computed",
    code: "코드 설명",
    stub: "준비중",
  },
  legend: {
    interactive: "슬라이더로 실시간",
    precomputed: "C++ 결과 토글",
    code: "개념·소스 해설",
    stub: "추후 추가",
  },
  home: {
    eyebrow: "PCL Tutorial · 한글",
    headline1: "Point Cloud Library를",
    headlineEm: "손으로 만져 보는",
    headline2: "튜토리얼",
    intro:
      "가벼운 필터(Voxel / PassThrough / SOR / Radius / KNN / Normal)는 TypeScript로 재구현해 브라우저에서 실시간으로 동작합니다. 무거운 정합(ICP / GICP)은 PCL 바이너리로 한 번 돌리고 그 결과 PCD를 정적 자산으로 fetch합니다.",
    chapterPrefix: "Chapter",
  },
  stub: {
    badge: "준비중",
    headline: "이 챕터는 곧 추가됩니다",
    body: "소스코드와 블로그 글은 위 링크에서 먼저 확인하실 수 있습니다.",
  },
  source: {
    label: "입력",
    presets: {
      kitti: { name: "KITTI", hint: "야외 LiDAR · 약 12만 pt" },
      naverlabs: { name: "NaverLabs", hint: "실내 LiDAR · 약 2.9만 pt" },
      bunny: { name: "Stanford Bunny", hint: "메시 꼭짓점 · 약 1.9k pt" },
    },
    custom: "직접 업로드",
    dropHint: ".bin / .pcd / .ply 파일을 여기에 끌어다 놓기",
    pickFile: "또는 파일 선택",
    using: "사용 중",
  },
  viewer: {
    pointsSuffix: "pts",
    before: "Before",
    after: "After",
    src: "src",
    tgt: "tgt",
    aligned: "src (정렬됨)",
  },
  lec05: {
    leafSize: "leaf size",
    leafHint: "Voxel 한 변의 길이. 클수록 더 거칠게 다운샘플링됩니다.",
    reduction: "감소율",
  },
  lec06: {
    axis: "axis",
    min: "min",
    max: "max",
    negative: "범위 바깥의 점만 남기기",
    negativeNote: "(setNegative(true) 와 동일)",
  },
  lec07: {
    meanK: "mean K",
    meanKHint: "각 점의 지역 밀도를 추정할 때 사용할 이웃 수.",
    stddev: "stddev multiplier",
    stddevHint:
      "임계값 = 전체 평균 + 배수 × 전체 표준편차. 작을수록 outlier 제거가 더 엄격해집니다.",
    inliers: "inlier",
    outliers: "outlier",
  },
  lec08: {
    queryX: "query x",
    queryY: "query y",
    queryZ: "query z",
    radius: "radius",
    radiusHint: "쿼리 점에서 이 거리 이내의 점들이 강조됩니다.",
    queryHint: "쿼리 점은 노란색으로 표시됩니다.",
    found: "이웃 점 발견",
  },
  lec09: {
    queryX: "query x",
    queryY: "query y",
    queryZ: "query z",
    k: "K",
    kHint: "가져올 최근접 이웃의 수.",
    queryHint: "쿼리 점은 노란색으로 표시됩니다.",
  },
  lec10: {
    voxelPre: "전처리 voxel",
    voxelPreHint:
      "Normal은 voxel로 줄인 cloud에서 추정합니다. 작을수록 세밀하지만 느려집니다.",
    k: "K (SVD 이웃 수)",
    kHint:
      "K개 이웃 점들로 공분산 행렬을 만들고, 가장 작은 eigenvalue의 eigenvector가 normal입니다.",
    coloring: "색상",
    coloringNormal: "Normal RGB",
    coloringFlat: "단색",
    note: "색상 = (|nx|, |ny|, |nz|).",
  },
  lec11: {
    mode: "Mode",
    before: "Before",
    after: "After",
    beforeHint:
      "ICP 적용 전 — src (빨강) vs tgt (초록). 일부러 x축으로 +2m 이동시킨 상태입니다.",
    afterHint:
      "ICP가 추정한 변환을 src에 적용한 결과. 정렬됐다면 target과 겹쳐야 합니다.",
    note: "이 결과는 scripts/export_precomputed.cpp가 미리 만든 PCD입니다 — 브라우저는 정적 자산을 fetch만 합니다.",
  },
  extra01: {
    threshold: "distance threshold",
    thresholdHint: "한 점이 plane의 inlier로 판정되는 최대 거리.",
    iters: "RANSAC 반복",
    itersHint: "3개 점을 무작위로 뽑아 plane을 만드는 반복 횟수. 클수록 정답 plane 찾을 확률 ↑.",
    inliers: "plane inlier",
    outliers: "그 외",
    note: "SLAM/로보틱스에서 지면(ground) 또는 벽 제거에 흔히 쓰는 전처리.",
  },
  extra02: {
    tolerance: "cluster tolerance",
    toleranceHint: "이 거리 이하로 연결된 점들은 같은 cluster.",
    minSize: "최소 cluster 크기",
    minSizeHint: "이보다 작은 cluster는 버립니다.",
    removeGround: "전처리로 RANSAC 지면 제거",
    found: "{n}개 cluster 발견",
  },
  chapters: {
    lec05: { title: "Voxelization", subtitle: "Voxel grid 다운샘플링 — leaf size를 슬라이더로." },
    lec06: { title: "PassThrough", subtitle: "축별 박스 필터로 cloud 잘라내기." },
    lec07: { title: "Statistical Outlier Removal", subtitle: "이웃 거리 분포로 outlier 제거." },
    lec08: { title: "Radius Search", subtitle: "쿼리 점의 반경 내 이웃 (KdTree)." },
    lec09: { title: "K-Nearest Neighbor", subtitle: "쿼리 점의 K개 최근접 이웃 (KdTree)." },
    lec10: { title: "Normal Estimation", subtitle: "KdTree + SVD로 점별 normal 추정." },
    lec11: { title: "Iterative Closest Point", subtitle: "src ↔ tgt ↔ aligned 토글 (C++ 결과)." },
    lec12: { title: "Generalized ICP", subtitle: "공분산 기반 ICP 변형 (C++ 결과)." },
    extra01: { title: "RANSAC Plane Segmentation", subtitle: "주된 plane(예: 지면)을 찾아 inlier/outlier로 분리." },
    extra02: { title: "Euclidean Clustering", subtitle: "공간 인접성으로 연결된 점들끼리 묶기." },
  },
};
