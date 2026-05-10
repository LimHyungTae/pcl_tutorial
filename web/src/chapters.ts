/** Whether the algorithm ships with PCL itself, or is from an open-source
 *  library outside PCL (e.g. Patchwork, TRAVEL, GenZ-ICP). */
export type ChapterStatus = "pcl" | "notInPcl";

export type ChapterMeta = {
  slug: string;
  /** 1-based chapter number shown in UI. */
  number: string;
  /** Path to the .cpp file at the repo root. Empty for web-only extras. */
  source: string;
  /** Full URL to the matching blog post (or "" if none). */
  blog: string;
  status: ChapterStatus;
};

const BLOG = "https://limhyungtae.github.io";

export const chapters: ChapterMeta[] = [
  {
    slug: "voxelization",
    number: "1",
    source: "lec05_voxelization.cpp",
    blog: `${BLOG}/2021-09-12-ROS-Point-Cloud-Library-(PCL)-5.-Voxelization/`,
    status: "pcl",
  },
  {
    slug: "pass-through",
    number: "2",
    source: "lec06_pass_through.cpp",
    blog: `${BLOG}/2021-09-12-ROS-Point-Cloud-Library-(PCL)-6.-PassThrough/`,
    status: "pcl",
  },
  {
    slug: "transformation",
    number: "3",
    source: "lec03_transformation.cpp",
    blog: `${BLOG}/2021-09-10-ROS-Point-Cloud-Library-(PCL)-3.-Transformation/`,
    status: "pcl",
  },
  {
    slug: "statistical-outlier-removal",
    number: "4",
    source: "lec07_sor.cpp",
    blog: `${BLOG}/2021-09-12-ROS-Point-Cloud-Library-(PCL)-7.-Statistical-Outlier-Removal/`,
    status: "pcl",
  },
  {
    slug: "radius-search",
    number: "5",
    source: "lec08_radius_search.cpp",
    blog: `${BLOG}/2021-09-12-ROS-Point-Cloud-Library-(PCL)-8.-KdTree를-활용한-Radius-Search/`,
    status: "pcl",
  },
  {
    slug: "k-nearest-neighbor",
    number: "6",
    source: "lec09_knn.cpp",
    blog: `${BLOG}/2021-09-12-ROS-Point-Cloud-Library-(PCL)-9.-KdTree를-활용한-K-nearest-Neighbor-Search-(KNN)/`,
    status: "pcl",
  },
  {
    slug: "normal-estimation",
    number: "7",
    source: "lec10_1_normal.cpp",
    blog: `${BLOG}/2021-09-13-ROS-Point-Cloud-Library-(PCL)-10.-Normal-Estimation/`,
    status: "pcl",
  },
  {
    slug: "ransac-plane-segmentation",
    number: "8",
    source: "",
    blog: "",
    status: "pcl",
  },
  {
    slug: "patchwork",
    number: "9",
    source: "",
    blog: "",
    status: "notInPcl",
  },
  {
    slug: "euclidean-clustering",
    number: "10",
    source: "",
    blog: "",
    status: "pcl",
  },
  {
    slug: "travel",
    number: "11",
    source: "",
    blog: "",
    status: "notInPcl",
  },
  {
    slug: "iterative-closest-point",
    number: "12",
    source: "lec11_icp.cpp",
    blog: `${BLOG}/2021-09-14-ROS-Point-Cloud-Library-(PCL)-11.-Iterative-Closest-Point-(ICP)/`,
    status: "pcl",
  },
  {
    slug: "genz-icp",
    number: "13",
    source: "",
    blog: "",
    status: "notInPcl",
  },
  {
    slug: "kiss-matcher",
    number: "14",
    source: "",
    blog: "",
    status: "notInPcl",
  },
];

export const findChapter = (slug: string) =>
  chapters.find((c) => c.slug === slug);

export const statusColor: Record<ChapterStatus, string> = {
  pcl: "bg-sky-500/15 text-sky-300 ring-sky-500/30",
  notInPcl: "bg-amber-500/15 text-amber-300 ring-amber-500/30",
};
