//
// Tutorial Author: shapelim@kaist.ac.kr (임형태)
#include <iostream>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/point_cloud.h>


void colorize(const pcl::PointCloud<pcl::PointXYZ> &pc,
              pcl::PointCloud<pcl::PointXYZRGB> &pc_colored,
              const std::vector<int> &color) {

    int N = pc.points.size();

    pc_colored.clear();
    pcl::PointXYZRGB pt_tmp;
    for (int i = 0; i < N; ++i) {
        const auto &pt = pc.points[i];
        pt_tmp.x = pt.x;
        pt_tmp.y = pt.y;
        pt_tmp.z = pt.z;
        pt_tmp.r = color[0];
        pt_tmp.g = color[1];
        pt_tmp.b = color[2];
        pc_colored.points.emplace_back(pt_tmp);
    }
}

int main(int argc, char **argv){

    pcl::PointCloud<pcl::PointXYZ>::Ptr src(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(
            "/home/shapelim/git/pcl_tutorial/materials/naverlabs_vel16.pcd", *src) ==
        -1) {
        PCL_ERROR ("Couldn't read source pcd file! \n");
        return (-1);
    }

    /**
     * Main
     */
    pcl::PointCloud<pcl::PointXYZ>::Ptr output(new pcl::PointCloud<pcl::PointXYZ>);

    int num_neigbor_points = 10;
    double std_multiplier = 1.0;

    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud (src);
    sor.setMeanK (num_neigbor_points);
    sor.setStddevMulThresh (std_multiplier);
    sor.filter(*output);
    /*******************************************/

    /**
     * 결과 visualization 하기
     */
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr src_colored(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr out_colored(new pcl::PointCloud<pcl::PointXYZRGB>);

    // Point cloud XYZ에 RGB 칼라 추가하기
    colorize(*src, *src_colored, {255, 0, 0});
    colorize(*output, *out_colored, {0, 255, 0});

    pcl::visualization::PCLVisualizer viewer1("Raw");
    pcl::visualization::PCLVisualizer viewer2("filtered");

    viewer1.addPointCloud<pcl::PointXYZRGB>(src_colored, "src_red");
    viewer2.addPointCloud<pcl::PointXYZRGB>(out_colored, "filtered_green");
    viewer1.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "src_red");
    viewer2.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "filtered_green");

    while (!viewer1.wasStopped() && !viewer2.wasStopped()) {
        viewer1.spinOnce();
        viewer2.spinOnce();
    }

    return 0;
}