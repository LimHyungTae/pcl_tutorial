export type ChapterStatus = "interactive" | "precomputed" | "code" | "stub";

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
    slug: "lec05",
    number: "1",
    source: "lec05_voxelization.cpp",
    blog: `${BLOG}/2021-09-12-ROS-Point-Cloud-Library-(PCL)-5.-Voxelization/`,
    status: "interactive",
  },
  {
    slug: "lec06",
    number: "2",
    source: "lec06_pass_through.cpp",
    blog: `${BLOG}/2021-09-12-ROS-Point-Cloud-Library-(PCL)-6.-PassThrough/`,
    status: "interactive",
  },
  {
    slug: "lec03",
    number: "3",
    source: "lec03_transformation.cpp",
    blog: `${BLOG}/2021-09-10-ROS-Point-Cloud-Library-(PCL)-3.-Transformation/`,
    status: "interactive",
  },
  {
    slug: "lec07",
    number: "4",
    source: "lec07_sor.cpp",
    blog: `${BLOG}/2021-09-12-ROS-Point-Cloud-Library-(PCL)-7.-Statistical-Outlier-Removal/`,
    status: "interactive",
  },
  {
    slug: "lec08",
    number: "5",
    source: "lec08_radius_search.cpp",
    blog: `${BLOG}/2021-09-12-ROS-Point-Cloud-Library-(PCL)-8.-KdTree를-활용한-Radius-Search/`,
    status: "interactive",
  },
  {
    slug: "lec09",
    number: "6",
    source: "lec09_knn.cpp",
    blog: `${BLOG}/2021-09-12-ROS-Point-Cloud-Library-(PCL)-9.-KdTree를-활용한-K-nearest-Neighbor-Search-(KNN)/`,
    status: "interactive",
  },
  {
    slug: "lec10",
    number: "7",
    source: "lec10_1_normal.cpp",
    blog: `${BLOG}/2021-09-13-ROS-Point-Cloud-Library-(PCL)-10.-Normal-Estimation/`,
    status: "interactive",
  },
  {
    slug: "extra01",
    number: "8",
    source: "",
    blog: "",
    status: "interactive",
  },
  {
    slug: "extra03",
    number: "9",
    source: "",
    blog: "",
    status: "interactive",
  },
  {
    slug: "extra02",
    number: "10",
    source: "",
    blog: "",
    status: "interactive",
  },
  {
    slug: "extra04",
    number: "11",
    source: "",
    blog: "",
    status: "interactive",
  },
  {
    slug: "lec11",
    number: "12",
    source: "lec11_icp.cpp",
    blog: `${BLOG}/2021-09-14-ROS-Point-Cloud-Library-(PCL)-11.-Iterative-Closest-Point-(ICP)/`,
    status: "interactive",
  },
  {
    slug: "extra05",
    number: "13",
    source: "",
    blog: "",
    status: "interactive",
  },
];

export const findChapter = (slug: string) =>
  chapters.find((c) => c.slug === slug);

export const statusColor: Record<ChapterStatus, string> = {
  interactive: "bg-emerald-500/15 text-emerald-300 ring-emerald-500/30",
  precomputed: "bg-sky-500/15 text-sky-300 ring-sky-500/30",
  code: "bg-amber-500/15 text-amber-300 ring-amber-500/30",
  stub: "bg-slate-500/15 text-slate-400 ring-slate-500/30",
};
