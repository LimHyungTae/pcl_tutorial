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
    pcl: "PCL",
    notInPcl: "Not in PCL",
  },
  legend: {
    pcl: "Standard PCL technique",
    notInPcl: "Open-source addition (not part of PCL)",
  },
  home: {
    eyebrow: "PCL Tutorial",
    headline1: "A Point Cloud Library tutorial",
    headlineEm: "you can play with",
    headline2: "",
    intro:
      "Must-to-know techniques for processing 3D point clouds — voxelization, KdTree, RANSAC, ICP and more — reimplemented in TypeScript and running live in your browser. While some of these methods are not officially included in PCL, I added them to the tutorial because they are intuitive, practical, and easy to use.",
    chapterPrefix: "Chapter",
  },
  author: {
    toggle: "How's Hyungtae Lim?",
    title: "Hyungtae Lim, Ph.D.",
    role: "Postdoctoral Associate · MIT SPARK Lab",
    focus: "Focuses on robust perception, simultaneous localization and mapping (SLAM), and state estimation.",
    bio: "Hyungtae Lim is a Postdoctoral Associate at MIT's SPARK Lab, working with Prof. Luca Carlone. He earned his M.S. (2020) and Ph.D. (2023) in Electrical Engineering from KAIST under Prof. Hyun Myung, after receiving his B.S. in Mechanical Engineering from the same institution in 2018. His research focuses on robust robotic perception — robust state estimation and lifelong mapping for mobile robots and autonomous vehicles. He is a recipient of the RSS Pioneers 2024 and 2022 IEEE RA-L Best Paper awards, and an Associate Editor for IEEE RA-L. His team also won 1st place at IEEE ICRA's NSS 2024 Challenge and the 2023 HILTI SLAM Challenge — back-to-back top finishes in LiDAR point cloud-based competitions. He is the author of widely used open-source LiDAR libraries.",
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
    bibtex: "Cite",
    copy: "Copy",
    copied: "Copied",
  },
  caution: {
    title: "Sensor-specific demo",
    patchworkBody:
      "Patchwork's CZM and R-GPF parameters are tuned per LiDAR. This demo only ships configs for VLP-16 and HDL-64. To run on your own data, build the official C++ implementation and tune the YAML for your sensor:",
    travelBody:
      "TRAVEL's range image dimensions and clustering thresholds are sensor-specific. This demo only ships configs for VLP-16 and HDL-64. For your own LiDAR, build the official C++/ROS package and tune the YAML config:",
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
    converged: "✓ Converged",
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
  extra04: {
    found: "{n} clusters found",
  },
  extra05: {
    planar: "planar (point-to-plane)",
    nonPlanar: "non-planar (point-to-point)",
  },
  chapters: {
    transformation: {
      title: "Transformation",
      subtitle: "4×4 rigid transform for 3D motion.",
      about:
        "Applying a 4×4 rigid transform T = [R | t; 0 | 1] to every point is called transformation; the 4×4 matrix itself is the transformation matrix. It's the foundational operation behind point-cloud registration, multi-sensor fusion, and world ↔ body frame conversions — found in essentially every SLAM pipeline.",
      params: [
        { name: "tx / ty / tz", desc: "Translation along each axis (meters).", effect: "" },
        { name: "rx / ry / rz", desc: "Euler rotation in degrees, applied as Rz · Ry · Rx.", effect: "" },
      ],
    },
    voxelization: {
      title: "Voxelization",
      subtitle: "Voxel-grid downsampling.",
      about:
        "Voxel-grid filtering (a.k.a. voxelization) discretizes 3D space into uniform cubic cells and replaces every cell's points with a single centroid. It's the most common first step in any point cloud pipeline — it strips redundant points so downstream algorithms run far faster while the overall shape stays intact.",
      params: [
        {
          name: "leaf size",
          desc: "Side length of each voxel cell.",
          effect: "Larger → sparser, faster, less detail.",
        },
      ],
    },
    "pass-through": {
      title: "PassThrough",
      subtitle: "Crop the cloud with an axis-aligned interval filter.",
      about:
        "PassThrough keeps points whose value along one axis falls inside [min, max] (or, with negative=true, drops them). Common uses: stripping the floor in flat indoor scenes, or trimming away points sitting too high above an autonomous vehicle to matter for driving decisions.",
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
    "statistical-outlier-removal": {
      title: "Statistical Outlier Removal",
      subtitle: "Outlier removal via neighbor-distance statistics.",
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
    "radius-search": {
      title: "Radius Search",
      subtitle: "KdTree-based search for neighbors of a query point within a given radius.",
      about:
        "Use a KdTree to return every point within a fixed radius of a query position. Click any point in the viewer to move the query there.",
      params: [
        {
          name: "query x / y / z",
          desc: "Query position (meters). Click any point to set.",
          effect: "",
        },
        { name: "radius", desc: "Search distance (meters).", effect: "Larger → more neighbors." },
      ],
    },
    "k-nearest-neighbor": {
      title: "K-Nearest Neighbor",
      subtitle: "KdTree-based search for the K closest neighbors of a query point.",
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
    "normal-estimation": {
      title: "Normal Estimation",
      subtitle: "Per-point normal estimation via KdTree + SVD.",
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
    "iterative-closest-point": {
      title: "Iterative Closest Point",
      subtitle: "Watch ICP find correspondences → update pose each iteration with Step / Play.",
      about:
        "Iterative Closest Point (ICP) is a local registration method that, on each iteration, treats the nearest tgt point of every src point as a valid correspondence and optimizes from there. The two-step loop: (1) for each src point find the nearest tgt point, (2) solve the rigid (R, t) that best aligns those pairs (Procrustes / SVD). Repeat until the pose update drops below a threshold.",
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
    "ransac-plane-segmentation": {
      title: "RANSAC Plane Segmentation",
      subtitle: "Find the dominant plane (e.g. ground) and split inliers / outliers.",
      about:
        "RANSAC (Random Sample Consensus) repeatedly picks 3 random points to fit a plane, counts how many other points lie within `threshold` of that plane (inliers), and keeps the plane with the most inliers. The classic, robust extractor for ground, walls, and tabletops.",
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
    "euclidean-clustering": {
      title: "Euclidean Clustering",
      subtitle: "Group spatially adjacent points into clusters.",
      about:
        "Grouping spatially adjacent points into a single object is called clustering. Here, a BFS connects any two points within `tolerance` and assigns them to the same cluster.",
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
    patchwork: {
      title: "Patchwork — Ground Segmentation",
      subtitle: "Concentric Zone Model + region-wise plane fitting for LiDAR ground extraction.",
      about:
        "Patchwork (Lim et al., RA-L, 2021) tackles ground segmentation by splitting the radial space into 4 concentric zones, each subdivided into ring × sector bins so the algorithm operates region-wise. For each bin we run region-wise ground plane fitting (R-GPF); ground likelihood estimation (GLE) then validates the fitted plane through uprightness, elevation, and flatness gates. Ground points are colored by their (zone, ring, sector) so each CZM patch is visually distinct.",
      params: [
        { name: "sensor", desc: "VLP-16 or HDL-64 only (params are baked in).", effect: "" },
        { name: "CZM", desc: "Concentric Zone Model: 4 zones × rings × sectors.", effect: "" },
        { name: "R-GPF + GLE", desc: "Per-bin plane fit, then validate via uprightness, elevation, flatness.", effect: "" },
      ],
    },
    "genz-icp": {
      title: "GenZ-ICP — Generalizable and Degeneracy-Robust ICP",
      subtitle: "Adaptive point-to-plane / point-to-point correspondences are what we need.",
      about:
        "GenZ-ICP (Lee et al., RA-L, 2024) registers a frame against a local map by classifying every correspondence as either planar (point-to-plane) or non-planar (point-to-point) based on the local planarity of the target neighborhood, and solving a single 6-DoF Gauss-Newton step that blends both residuals with an adaptive weight α = #planar / (#planar + #non_planar). Planar pair lines are drawn in blue (RGB 0, 119, 187) and non-planar in magenta (RGB 238, 51, 119).",
      params: [
        { name: "max correspondence distance", desc: "Pairs farther apart are dropped each iteration.", effect: "Smaller → robust to outliers, but stalls when far from convergence." },
        { name: "α (adaptive)", desc: "Computed each iteration as #planar / total pairs.", effect: "1 → all planar surfaces, 0 → all unstructured." },
        { name: "Step / Play", desc: "Watch the α and the two pair streams as they balance.", effect: "" },
      ],
    },
    travel: {
      title: "TRAVEL — Range Image Clustering",
      subtitle: "Project to a range image and flood-fill above-ground objects with the same color in 3D and 2D.",
      about:
        "TRAVEL (Oh et al., RA-L, 2022) performs ground-aware object clustering by exploiting the 2D grid structure of the LiDAR range image. The pipeline runs in two stages: Traversable Ground Segmentation (TGS) removes ground, and Above-ground Object Segmentation (AOS) clusters the remaining points directly on the range image. In this demo, Patchwork is reused instead of TGS; AOS is then ported faithfully, projecting each non-ground point onto one (row, col) pixel of the (rows × cols) range image and merging 4-connected pixels whose depth gap is below sensor-specific horizontal and vertical thresholds. Cluster colors are shared between the 3D viewer and the 2D range image, so any blob in one view maps directly to the other.",
      params: [
        { name: "sensor", desc: "VLP-16 or HDL-64 only (params are baked in).", effect: "" },
        { name: "TGS", desc: "Substituted with Patchwork (upstream uses Travelable Ground Segmentation).", effect: "" },
        { name: "AOS", desc: "4-connected BFS on the range image, split by depth gap.", effect: "" },
      ],
    },
  },
};

export type LocaleDict = typeof en;
