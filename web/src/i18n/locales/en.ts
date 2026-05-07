export const en = {
  app: {
    title: "PCL Tutorial",
    tagline: "Hands-on Point Cloud Library",
  },
  nav: {
    chapters: "Chapters",
    github: "GitHub",
    sourceCode: "Source",
    blogPost: "Blog post",
  },
  status: {
    interactive: "Interactive",
    precomputed: "Pre-computed",
    code: "Code only",
    stub: "Coming soon",
  },
  legend: {
    interactive: "Live sliders",
    precomputed: "Toggle C++ result",
    code: "Concept only",
    stub: "Will be added",
  },
  home: {
    eyebrow: "PCL Tutorial",
    headline1: "A Point Cloud Library tutorial",
    headlineEm: "you can play with",
    headline2: "",
    intro:
      "Must-to-know techniques for processing 3D point clouds — voxelization, KdTree, RANSAC, ICP and more — reimplemented in TypeScript and running live in your browser.",
    chapterPrefix: "Chapter",
  },
  author: {
    toggle: "How's Hyungtae Lim?",
    title: "Hyungtae Lim",
    role: "Postdoctoral Associate · MIT SPARK Lab",
    focus: "Focuses on robust perception, simultaneous localization and mapping (SLAM), and state estimation.",
    bio: "Hyungtae Lim is a Postdoctoral Associate at MIT's SPARK Lab, working with Prof. Luca Carlone. He received his Ph.D. in Electrical Engineering from KAIST in 2023, advised by Prof. Hyun Myung. His research focuses on robust perception, state estimation, and lifelong mapping for mobile robots and autonomous vehicles. He is a recipient of the RSS Pioneers 2024 award and the 2022 IEEE RA-L Best Paper Award, an Associate Editor for IEEE RA-L, and the author of widely used open-source LiDAR-based libraries.",
    libsLabel: "Open-source libraries",
    libs: [
      { name: "GenZ-ICP", url: "https://github.com/cocel-postech/genz-icp" },
      { name: "Patchwork", url: "https://github.com/LimHyungTae/patchwork" },
      { name: "Patchwork++", url: "https://github.com/url-kaist/patchwork-plusplus" },
      { name: "TRAVEL", url: "https://github.com/url-kaist/TRAVEL" },
      { name: "KISS-Matcher", url: "https://github.com/MIT-SPARK/KISS-Matcher" },
    ],
    blog: "Personal site",
    blogUrl: "https://limhyungtae.github.io",
    github: "GitHub",
    githubUrl: "https://github.com/LimHyungTae",
    email: "Email",
    emailAddress: "shapelim@mit.edu",
  },
  stub: {
    badge: "Coming soon",
    headline: "This chapter is still being prepared",
    body: "The source code and blog post are linked above — the interactive demo will follow.",
  },
  source: {
    label: "Input",
    presets: {
      bunny:     { name: "Stanford Bunny", scene: "CAD",     hint: "~1.9k pts" },
      naverlabs: { name: "VLP-16",         scene: "Indoor",  hint: "NaverLabs · ~29k pts" },
      kitti:     { name: "HDL-64",         scene: "Outdoor", hint: "KITTI · ~120k pts" },
    },
    custom: "Custom",
    dropHint: "Drop a .bin / .pcd / .ply here",
    pickFile: "or choose a file",
    using: "Using",
  },
  demo: {
    parameters: "Parameters",
  },
  viewer: {
    pointsSuffix: "pts",
    before: "Before",
    after: "After",
    src: "src",
    tgt: "tgt",
    aligned: "src (aligned)",
  },
  lec05: {
    leafSize: "leaf size",
    leafHint: "Side length of a voxel. Larger values downsample more aggressively.",
    reduction: "reduction",
  },
  lec06: {
    axis: "axis",
    min: "min",
    max: "max",
    negative: "Keep points outside the range",
    negativeNote: "(equivalent to setNegative(true))",
  },
  lec07: {
    meanK: "mean K",
    meanKHint: "Number of neighbors used to estimate each point's local density.",
    stddev: "stddev multiplier",
    stddevHint:
      "Threshold = global mean + multiplier × global stddev. Lower = stricter outlier removal.",
    inliers: "inliers",
    outliers: "outliers",
  },
  lec08: {
    queryX: "query x",
    queryY: "query y",
    queryZ: "query z",
    radius: "radius",
    radiusHint: "All points within this distance from the query are highlighted.",
    queryHint: "The query point shown in yellow.",
    found: "neighbors found",
  },
  lec09: {
    queryX: "query x",
    queryY: "query y",
    queryZ: "query z",
    k: "K",
    kHint: "Number of nearest neighbors to fetch.",
    queryHint: "The query point shown in yellow.",
  },
  lec10: {
    voxelPre: "preprocess voxel",
    voxelPreHint:
      "Normals are estimated on the voxelized cloud. Smaller = more detail but slower.",
    k: "K (neighbors for SVD)",
    kHint:
      "K nearest neighbors are used to compute the local covariance. Smallest eigenvector → normal.",
    coloring: "coloring",
    coloringNormal: "Normal RGB",
    coloringFlat: "Flat",
    note: "Color = (|nx|, |ny|, |nz|).",
  },
  lec03: {
    translation: "Translation",
    rotation: "Rotation (Euler)",
    randomize: "Randomize",
    reset: "Reset",
    legendSrc: "src",
    legendOut: "transformed",
  },
  lec11: {
    reset: "Reset · random offset",
    step: "▶ Step",
    play: "▶ Play",
    pause: "⏸ Pause",
    maxDist: "max correspondence distance",
    maxDistHint: "Pairs farther apart than this are dropped each iteration.",
    fitness: "fitness:",
    pairs: "pairs:",
    legendSrc: "src (under current T)",
    legendTgt: "tgt",
    howItWorks: "How it works",
    step1: "Find each src point's nearest tgt point (KdTree).",
    step2: "Solve the rigid (R, t) that best aligns the matched pairs (Procrustes).",
    step3: "Apply (R, t) to src; repeat until convergence.",
  },
  extra01: {
    threshold: "distance threshold",
    thresholdHint: "Maximum distance from a point to the plane for it to count as inlier.",
    iters: "RANSAC iterations",
    itersHint: "Number of random 3-point samples; more = better odds of finding the dominant plane.",
    inliers: "plane inliers",
    outliers: "rest",
    note: "Useful for ground-plane / wall removal in SLAM and robotics pipelines.",
  },
  extra02: {
    tolerance: "cluster tolerance",
    toleranceHint: "Two points belong to the same cluster if connected by jumps ≤ this distance.",
    minSize: "min cluster size",
    minSizeHint: "Clusters smaller than this are dropped.",
    removeGround: "Remove dominant plane first (RANSAC)",
    found: "{n} clusters found",
  },
  chapters: {
    lec03: {
      title: "Transformation",
      subtitle: "Apply a 4×4 rigid transform — translate and rotate the cloud.",
      about:
        "Apply a 4×4 rigid transform T = [R | t; 0 | 1] to every point. The same operation behind point-cloud registration, multi-sensor fusion and world ↔ body frame conversions.",
      params: [
        { name: "tx / ty / tz", desc: "Translation along each axis (meters).", effect: "" },
        { name: "rx / ry / rz", desc: "Euler rotation in degrees, applied as Rz · Ry · Rx.", effect: "" },
      ],
    },
    lec05: {
      title: "Voxelization",
      subtitle: "Voxel-grid downsampling — drag the leaf size slider.",
      about:
        "Voxel-grid filtering discretizes 3D space into uniform cubic cells and replaces every cell's points with their centroid. A staple preprocessing step — fast, deterministic, and preserves overall shape.",
      params: [
        {
          name: "leaf size",
          desc: "Side length of each voxel cell.",
          effect: "Larger → sparser, faster, less detail.",
        },
      ],
    },
    lec06: {
      title: "PassThrough",
      subtitle: "Crop the cloud with an axis-aligned interval filter.",
      about:
        "PassThrough is the simplest spatial filter: keep points whose value along one axis falls inside an interval (or, with negative=true, drop them).",
      params: [
        { name: "axis", desc: "Spatial axis the filter runs along.", effect: "" },
        {
          name: "min / max",
          desc: "Interval bounds (meters).",
          effect: "Wider → keeps more points.",
        },
        { name: "negative", desc: "Flips the filter to keep points outside the interval.", effect: "" },
      ],
    },
    lec07: {
      title: "Statistical Outlier Removal",
      subtitle: "Remove sparse outliers using neighbor distance statistics.",
      about:
        "For each point, average its distance to the K nearest neighbors. Compute the mean and stddev of those averages over the whole cloud, and drop any point whose value exceeds (mean + mult × stddev).",
      params: [
        {
          name: "mean K",
          desc: "Neighbors used to estimate each point's local density.",
          effect: "Larger → smoother density estimate.",
        },
        {
          name: "stddev mult",
          desc: "Threshold multiplier on top of the global stddev.",
          effect: "Smaller → stricter (removes more).",
        },
      ],
    },
    lec08: {
      title: "Radius Search",
      subtitle: "All neighbors of a query point within a given radius (KdTree).",
      about:
        "KdTree-accelerated query: return every point within a fixed radius of a query position. Click any point in the viewer to set the query.",
      params: [
        {
          name: "query x / y / z",
          desc: "Query position (meters). Click any point to set.",
          effect: "",
        },
        { name: "radius", desc: "Search distance (meters).", effect: "Larger → more neighbors." },
      ],
    },
    lec09: {
      title: "K-Nearest Neighbor",
      subtitle: "K closest neighbors of a query point (KdTree).",
      about:
        "KdTree returns the K closest points to a query, sorted by distance. Click any point in the viewer to set the query.",
      params: [
        {
          name: "query x / y / z",
          desc: "Query position (meters). Click any point to set.",
          effect: "",
        },
        {
          name: "K",
          desc: "Number of nearest neighbors to fetch.",
          effect: "Larger → bigger highlighted region.",
        },
      ],
    },
    lec10: {
      title: "Normal Estimation",
      subtitle: "Per-point normals from KdTree + SVD on local neighborhoods.",
      about:
        "For each point, build the covariance of its K nearest neighbors. The eigenvector of the smallest eigenvalue is the surface normal at that point. Color = (|nx|, |ny|, |nz|).",
      params: [
        {
          name: "preprocess voxel",
          desc: "Voxel size used to thin the cloud first.",
          effect: "Smaller → more detail, slower.",
        },
        {
          name: "K",
          desc: "Neighbors used in each point's local PCA.",
          effect: "Larger → smoother but blurrier normals.",
        },
      ],
    },
    lec11: {
      title: "Iterative Closest Point",
      subtitle: "Step through correspondences and pose updates one iteration at a time.",
      about:
        "ICP alternates two steps: (1) for each src point find the nearest tgt point, (2) solve the rigid (R, t) that best aligns those pairs (Procrustes / SVD). Repeat until convergence.",
      params: [
        {
          name: "max correspondence distance",
          desc: "Pairs farther apart than this are dropped each iteration.",
          effect: "Smaller → robust to outliers but may stall when far from convergence.",
        },
        { name: "Step", desc: "Run one iteration manually.", effect: "" },
        { name: "Play", desc: "Auto-iterate every ~180 ms.", effect: "" },
      ],
    },
    extra01: {
      title: "RANSAC Plane Segmentation",
      subtitle: "Find the dominant plane (e.g. ground) and split inliers / outliers.",
      about:
        "RANSAC samples 3 random points to fit a plane, scores it by the number of inliers (within `threshold`), and keeps the best across many trials. The classic ground / wall / table extractor.",
      params: [
        {
          name: "distance threshold",
          desc: "Maximum distance from a point to the plane to count as inlier.",
          effect: "Larger → admits more (noisier inliers).",
        },
        {
          name: "iterations",
          desc: "Number of random 3-point trials.",
          effect: "Larger → more reliably finds the dominant plane.",
        },
      ],
    },
    extra02: {
      title: "Euclidean Clustering",
      subtitle: "Connected-component grouping by spatial proximity.",
      about:
        "Group points by spatial proximity: BFS that connects any two points within `tolerance`. Cluster colors stay stable across slider changes by matching centroids to the previous frame.",
      params: [
        {
          name: "tolerance",
          desc: "Maximum gap between two points in the same cluster.",
          effect: "Larger → fewer, bigger clusters.",
        },
        {
          name: "min size",
          desc: "Clusters smaller than this are discarded.",
          effect: "Larger → drop more noise.",
        },
        { name: "remove ground", desc: "Run RANSAC plane removal first as preprocessing.", effect: "" },
      ],
    },
  },
};

export type LocaleDict = typeof en;
