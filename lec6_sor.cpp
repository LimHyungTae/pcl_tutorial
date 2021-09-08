//
// Tutorial Author: shapelim@kaist.ac.kr (임형태)
#include <iostream>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>


//Input: pcl::PointCloud source, namely pc_src
//Output: Filtered pcl::PointCloud, namely pc_sor_filtered
int main(int argc, char **argv){

    pcl::PointCloud<pcl::PointXYZ> pc_src;
    int num_pts = 100;
    pcl::PointXYZ point_xyz;
    for (int k=0; k < num_pts ; ++k){
        float x = static_cast <float> (rand()) / static_cast <float> (RAND_MAX) * 10;
        float y = static_cast <float> (rand()) / static_cast <float> (RAND_MAX) * 10;
        float z = static_cast <float> (rand()) / static_cast <float> (RAND_MAX) * 10;
        point_xyz.x = x;     point_xyz.y = y;     point_xyz.z = z;
        pc_src.push_back(point_xyz);
    }


    pcl::PointCloud<pcl::PointXYZ> pc_sor_filtered;
    pcl::PointCloud<pcl::PointXYZ>::Ptr ptr_sor_filtered(new pcl::PointCloud<pcl::PointXYZ>);

    int num_neigbor_points = 10;
    double std_multiplier = 1.0;

    *ptr_sor_filtered = pc_src;
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud (ptr_sor_filtered);
    sor.setMeanK (num_neigbor_points);
    sor.setStddevMulThresh (std_multiplier);
    sor.filter(*ptr_sor_filtered);

    pc_sor_filtered = *ptr_sor_filtered;

    return 0;
}