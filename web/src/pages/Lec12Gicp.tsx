import { findChapter } from "../chapters";
import PrecomputedRegistration from "./PrecomputedRegistration";

const CODE = `pcl::GeneralizedIterativeClosestPoint<
    pcl::PointXYZ, pcl::PointXYZ> gicp;
gicp.setMaxCorrespondenceDistance(1.0);
gicp.setTransformationEpsilon(0.003);
gicp.setMaximumIterations(1000);
gicp.setInputSource(src);
gicp.setInputTarget(tgt);
gicp.align(*aligned);`;

export default function Lec12Gicp() {
  return (
    <PrecomputedRegistration
      chapter={findChapter("lec12")!}
      prefix="gicp"
      codeBlock={CODE}
    />
  );
}
