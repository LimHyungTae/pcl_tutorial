import { findChapter } from "../chapters";
import PrecomputedRegistration from "./PrecomputedRegistration";

const CODE = `pcl::IterativeClosestPoint<
    pcl::PointXYZ, pcl::PointXYZ> icp;
icp.setMaxCorrespondenceDistance(1.0);
icp.setTransformationEpsilon(0.003);
icp.setMaximumIterations(1000);
icp.setInputSource(src);
icp.setInputTarget(tgt);
icp.align(*aligned);`;

export default function Lec11Icp() {
  return (
    <PrecomputedRegistration
      chapter={findChapter("lec11")!}
      prefix="icp"
      codeBlock={CODE}
    />
  );
}
