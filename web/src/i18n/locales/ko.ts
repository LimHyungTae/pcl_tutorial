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
    headline1: "손으로 직접 해보는",
    headlineEm: "Point Cloud Library Tutorial",
    headline2: "",
    intro:
      "3D point cloud을 다루는 핵심 기법들 — voxelization, KdTree, RANSAC, ICP 등 — 을 TypeScript로 재구현해 브라우저에서 실시간으로 돌립니다.",
    chapterPrefix: "Chapter",
  },
  author: {
    toggle: "How's Hyungtae Lim?",
    title: "임형태 (Hyungtae Lim, Ph.D.)",
    role: "Postdoctoral Associate · MIT SPARK Lab",
    focus: "Robust perception · simultaneous localization and mapping (SLAM) · state estimation 연구.",
    bio: "임형태는 MIT SPARK Lab에서 Luca Carlone 교수 아래 Postdoctoral Associate으로 일하고 있으며, 2020·2023년 KAIST에서 명현 교수의 지도 하에 전기 및 전자공학 석사·박사 학위를 받았고, 2018년에는 KAIST 기계공학과를 졸업했습니다. 주로 로봇의 강인한 인지 기술에 관심이 많으며, 모바일 로봇과 자율주행을 위한 robust perception, state estimation, lifelong mapping 등을 연구합니다. 대외적으로는 RSS Pioneers 2024와 2022 IEEE RA-L Best Paper Award 수상자이며, LiDAR point cloud를 다루는 IEEE ICRA의 NSS 2024 Challenge와 2023 HILTI SLAM Challenge에서 2년 간 1등을 수상하기도 했습니다. 또한 널리 사용되는 LiDAR 기반 오픈소스 라이브러리들의 저자이기도 합니다.",
    libsLabel: "오픈소스 라이브러리",
    libs: [
      { name: "GenZ-ICP", url: "https://github.com/cocel-postech/genz-icp" },
      { name: "Patchwork", url: "https://github.com/LimHyungTae/patchwork" },
      { name: "Patchwork++", url: "https://github.com/url-kaist/patchwork-plusplus" },
      { name: "TRAVEL", url: "https://github.com/url-kaist/TRAVEL" },
      { name: "KISS-Matcher", url: "https://github.com/MIT-SPARK/KISS-Matcher" },
    ],
    blog: "개인 사이트",
    blogUrl: "https://limhyungtae.github.io",
    github: "GitHub",
    githubUrl: "https://github.com/LimHyungTae",
    email: "Email",
    emailAddress: "shapelim@mit.edu",
  },
  stub: {
    badge: "준비중",
    headline: "이 챕터는 곧 추가됩니다",
    body: "소스코드와 블로그 글은 위 링크에서 먼저 확인하실 수 있습니다.",
  },
  source: {
    label: "입력",
    presets: {
      bunny:     { name: "Stanford Bunny", scene: "CAD",  hint: "약 1.9k pt" },
      naverlabs: { name: "VLP-16",         scene: "실내", hint: "NaverLabs · 약 2.9만 pt" },
      kitti:     { name: "HDL-64",         scene: "실외", hint: "KITTI · 약 12만 pt" },
    },
    custom: "직접 업로드",
    dropHint: ".bin / .pcd / .ply 파일을 여기에 끌어다 놓기",
    pickFile: "또는 파일 선택",
    using: "사용 중",
  },
  demo: {
    parameters: "파라미터",
    bibtex: "인용 (BibTeX)",
    copy: "복사",
    copied: "복사됨",
  },
  caution: {
    title: "센서 의존적 데모",
    patchworkBody:
      "Patchwork의 CZM·R-GPF 파라미터는 LiDAR마다 달라집니다. 이 데모는 VLP-16과 HDL-64 설정만 포함되어 있습니다. 자체 데이터에 적용하려면 원본 C++ 구현을 빌드해 YAML을 센서에 맞게 튜닝하세요:",
    travelBody:
      "TRAVEL의 range image 크기와 클러스터링 임계값은 센서마다 다릅니다. 이 데모는 VLP-16과 HDL-64 설정만 포함합니다. 자체 LiDAR라면 원본 C++/ROS 패키지를 빌드해 YAML을 튜닝하세요:",
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
  lec03: {
    translation: "Translation",
    rotation: "Rotation (Euler)",
    randomize: "랜덤화",
    reset: "리셋",
    legendSrc: "src",
    legendOut: "변환됨",
  },
  lec11: {
    reset: "리셋 · 랜덤 offset",
    step: "▶ Step",
    play: "▶ Play",
    pause: "⏸ Pause",
    maxDist: "max correspondence distance",
    maxDistHint: "이 거리보다 먼 pair는 매 iteration마다 버립니다.",
    fitness: "fitness:",
    pairs: "pairs:",
    legendSrc: "src (현재 T 적용)",
    legendTgt: "tgt",
    howItWorks: "동작 원리",
    step1: "각 src 점에 대해 tgt에서 가장 가까운 점 찾기 (KdTree).",
    step2: "매칭된 pair들에 가장 잘 맞는 강체 (R, t)를 SVD로 추정 (Procrustes).",
    step3: "(R, t)를 src에 누적 적용. 수렴할 때까지 반복.",
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
  extra04: {
    found: "{n}개 cluster 발견",
  },
  chapters: {
    lec03: {
      title: "Transformation",
      subtitle: "3D motion을 위한 4×4 강체 변환.",
      about:
        "모든 점에 4×4 강체 변환 T = [R | t; 0 | 1]을 적용하는 연산을 transformation이라고 부르며, 이 4×4 행렬을 transformation matrix라고 부릅니다. 정합 · 센서 fusion · world ↔ body 좌표계 변환 등 거의 모든 SLAM 파이프라인의 토대가 되는 기본 연산입니다.",
      params: [
        { name: "tx / ty / tz", desc: "각 축 방향 평행이동 (미터).", effect: "" },
        { name: "rx / ry / rz", desc: "Euler 회전각 (도). Rz · Ry · Rx 순으로 합성.", effect: "" },
      ],
    },
    lec05: {
      title: "Voxelization",
      subtitle: "Voxel grid 다운샘플링.",
      about:
        "Voxel-grid 필터(voxelization)는 3차원 공간을 균일한 정육면체 셀로 나누고, 각 셀에 들어가는 점들을 그 centroid 한 점으로 대체합니다. 거의 모든 point cloud 처리 파이프라인의 첫 전처리 단계로 사용되며, redundant한 점 수를 줄여 뒤이은 알고리즘의 연산 효율을 크게 높여줍니다.",
      params: [
        {
          name: "leaf size",
          desc: "Voxel 한 변의 길이.",
          effect: "클수록 더 sparse하고 빠르며 디테일이 줄어듦.",
        },
      ],
    },
    lec06: {
      title: "PassThrough",
      subtitle: "축별 박스 필터로 cloud 잘라내기.",
      about:
        "PassThrough 필터는 한 축의 좌표가 [min, max] 구간 안에 들어가는 점만 남기거나(또는 negative 옵션으로) 빼냅니다. 평평한 실내 환경에서 바닥을 빠르게 제거하거나, 야외 주행에서 도로 판단과 무관한 높이의 점들을 사전에 잘라낼 때 자주 사용됩니다.",
      params: [
        { name: "axis", desc: "필터를 적용할 공간 축.", effect: "" },
        { name: "min / max", desc: "구간 경계 (미터).", effect: "넓을수록 더 많이 남음." },
        { name: "negative", desc: "필터 반전 — 구간 바깥의 점을 남김.", effect: "" },
      ],
    },
    lec07: {
      title: "Statistical Outlier Removal",
      subtitle: "이웃 거리 분포 기반 outlier 제거.",
      about:
        "각 점의 K개 최근접 이웃까지의 평균 거리를 구하고, 전체 cloud의 그 평균과 표준편차를 계산해 (평균 + mult × 표준편차)를 넘는 점을 outlier로 제거합니다.",
      params: [
        {
          name: "mean K",
          desc: "각 점의 지역 밀도 추정에 사용할 이웃 수.",
          effect: "클수록 밀도 추정이 부드러워짐.",
        },
        {
          name: "stddev mult",
          desc: "표준편차 위에 곱해지는 임계값 배수.",
          effect: "작을수록 엄격 (더 많이 제거).",
        },
      ],
    },
    lec08: {
      title: "Radius Search",
      subtitle: "KdTree 기반 쿼리 점의 반경 내 이웃 찾기.",
      about:
        "KdTree를 활용해 주어진 쿼리(query) 점 주변 반경 내의 모든 이웃을 반환합니다. 뷰어의 점을 클릭하면 쿼리 점이 그 위치로 옮겨집니다.",
      params: [
        {
          name: "query x / y / z",
          desc: "쿼리 점 좌표 (미터). 뷰어의 점 클릭으로 설정 가능.",
          effect: "",
        },
        { name: "radius", desc: "검색 반경 (미터).", effect: "클수록 더 많은 이웃." },
      ],
    },
    lec09: {
      title: "K-Nearest Neighbor",
      subtitle: "KdTree 기반 쿼리 점의 K개 최근접 이웃 찾기.",
      about:
        "KdTree가 쿼리 점에서 가장 가까운 K개를 거리 순으로 반환합니다. 뷰어의 점을 클릭하면 쿼리 점이 옮겨갑니다.",
      params: [
        {
          name: "query x / y / z",
          desc: "쿼리 점 좌표 (미터). 뷰어의 점 클릭으로 설정 가능.",
          effect: "",
        },
        { name: "K", desc: "가져올 최근접 이웃 수.", effect: "클수록 더 넓은 영역이 강조됨." },
      ],
    },
    lec10: {
      title: "Normal Estimation",
      subtitle: "KdTree + SVD로 point 별 normal 추정.",
      about:
        "각 점의 K개 이웃으로 공분산 행렬을 만들고, 가장 작은 eigenvalue의 eigenvector가 그 점의 표면 법선이 됩니다. 색 = (|nx|, |ny|, |nz|).",
      params: [
        {
          name: "preprocess voxel",
          desc: "Normal 계산 전에 cloud를 줄일 voxel 크기.",
          effect: "작을수록 디테일↑, 속도↓.",
        },
        {
          name: "K",
          desc: "각 점의 지역 PCA에 사용할 이웃 수.",
          effect: "클수록 normal이 부드럽지만 흐려짐.",
        },
      ],
    },
    lec11: {
      title: "Iterative Closest Point",
      subtitle: "Step / Play로 ICP 매 iteration의 짝 찾기 → pose update 관찰해보기.",
      about:
        "Iterative Closest Point(ICP)는 매 iteration마다 가장 인접한 점을 유효한 대응(correspondence)으로 보고 최적화하는 local registration 기법입니다. (1) 각 src 점의 nearest tgt 점 찾기, (2) 매칭된 pair들에 가장 잘 맞는 강체 (R, t)를 SVD로 추정(Procrustes)하는 두 단계로 구성되며, pose 변화량이 임계값 아래로 떨어질 때까지 반복합니다.",
      params: [
        {
          name: "max correspondence distance",
          desc: "이 거리 이상으로 떨어진 pair는 매 iteration마다 버립니다.",
          effect: "작을수록 outlier에 강건하지만 멀리 떨어진 정합에선 정체될 수 있음.",
        },
        { name: "Step", desc: "한 iteration씩 수동 실행.", effect: "" },
        { name: "Play", desc: "약 180 ms마다 자동 반복.", effect: "" },
      ],
    },
    extra01: {
      title: "RANSAC Plane Segmentation",
      subtitle: "주된 plane(예: 지면)을 찾아 inlier/outlier로 분리.",
      about:
        "RANSAC(Random Sample Consensus)을 활용해 무작위로 3점을 뽑아 plane을 만들고, 그 plane에서 threshold 이내에 들어오는 점을 inlier로 셉니다. 가장 많은 inlier를 갖는 plane을 채택하며, 지면 · 벽 · 테이블 추출에 널리 쓰이는 고전적인 방법입니다.",
      params: [
        {
          name: "distance threshold",
          desc: "한 점이 inlier로 인정되는 최대 plane 거리.",
          effect: "클수록 노이즈를 더 많이 받아들임.",
        },
        {
          name: "iterations",
          desc: "무작위 3점 sample을 시도하는 횟수.",
          effect: "클수록 정답 plane을 안정적으로 찾음.",
        },
      ],
    },
    extra02: {
      title: "Euclidean Clustering",
      subtitle: "공간 인접성으로 연결된 점들끼리 cluster로 묶기.",
      about:
        "공간 상에 인접한 점들을 하나의 객체로 묶는 작업을 clustering이라고 부릅니다. 여기서는 BFS로 tolerance 이내로 연결된 점들을 같은 cluster로 묶습니다.",
      params: [
        {
          name: "tolerance",
          desc: "같은 cluster로 인정되는 최대 점 간 거리.",
          effect: "클수록 cluster가 크고 적어짐.",
        },
        {
          name: "min size",
          desc: "이보다 작은 cluster는 버림.",
          effect: "클수록 노이즈 cluster를 더 많이 제거.",
        },
        { name: "remove ground", desc: "전처리로 RANSAC plane 제거 적용.", effect: "" },
      ],
    },
    extra03: {
      title: "Patchwork - \"Ground Segmentation\"",
      subtitle: "Concentric Zone Model + region-wise plane fitting으로 LiDAR 지면 추출.",
      about:
        "Patchwork (Lim et al., RA-L, 2021)는 중심에서 멀어질수록 sparse해지는 LiDAR 점들에도 강인하게 ground segmentation을 수행하기 위해, 반경 공간을 4개의 동심 zone으로 나누고 각 zone을 ring × sector bin으로 세분화합니다. 그 후 각 bin마다 region-wise ground plane fitting (R-GPF)을 수행하고, uprightness · elevation · flatness 세 게이트로 검증하는 ground likelihood estimation (GLE)을 거쳐 추정된 평면이 실제 ground인지 판정합니다. 각 patch는 (zone, ring, sector)에 따라 다른 색으로 시각화되어 CZM 분할이 그대로 드러납니다.",
      params: [
        { name: "sensor", desc: "VLP-16 / HDL-64만 지원 (파라미터 고정).", effect: "" },
        { name: "CZM", desc: "Concentric Zone Model: 4 zone × ring × sector.", effect: "" },
        { name: "R-GPF + GLE", desc: "각 bin 별 plane fit 후 uprightness · elevation · flatness 검증.", effect: "" },
      ],
    },
    extra04: {
      title: "TRAVEL — Range Image Clustering",
      subtitle: "Range image에 투영해 above-ground 객체를 flood-fill, 3D와 2D에 동일 색으로.",
      about:
        "TRAVEL (Oh et al., RA-L, 2022)은 LiDAR range image의 2D 격자 구조를 활용해 효율적으로 ground-aware 객체 clustering을 수행합니다. 파이프라인은 두 단계로 구성됩니다. 먼저 Traversable Ground Segmentation (TGS)이 지면을 제거하고, Above-ground Object Segmentation (AOS)이 남은 점들을 range image 위에서 직접 cluster로 묶습니다. 본 데모에서는 TGS를 Patchwork로 대체하고 AOS는 원 코드의 pipeline을 따랐습니다. 각 non-ground 점이 (rows × cols) range image의 (row, col) 픽셀 한 곳에 투영되고, 4-인접 픽셀의 깊이차가 센서별 수평·수직 임계값 이하이면 같은 cluster로 병합합니다. 3D 뷰어와 2D range image는 동일한 cluster 색을 공유하고 있습니다.",
      params: [
        { name: "sensor", desc: "VLP-16 / HDL-64만 지원 (파라미터 고정).", effect: "" },
        { name: "TGS", desc: "Patchwork로 대체 (원본은 Travelable Ground Segmentation).", effect: "" },
        { name: "AOS", desc: "range image 위 4-인접 BFS, 깊이차 임계값으로 분리.", effect: "" },
      ],
    },
  },
};
