//
// Tutorial Author: shapelim@kaist.ac.kr (임형태)
#include <iostream>
#include <pcl/filters/passthrough.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

//Input: pcl::PointCloud source, namely pc_src
//Output: Filtered pcl::PointCloud, namely pc_filtered, along X axis, from 0.5m to 100.0m
int main(int argc, char **argv){

    pcl::PointCloud<pcl::PointXYZ> pc_src;
    int num_pts = 100;
    pcl::PointXYZ point_xyz;
    for (int k=0; k < num_pts ; ++k){
        float x = static_cast <float> (rand()) / static_cast <float> (RAND_MAX) * 100;
        float y = static_cast <float> (rand()) / static_cast <float> (RAND_MAX) * 100;
        float z = static_cast <float> (rand()) / static_cast <float> (RAND_MAX) * 100;
        point_xyz.x = x;     point_xyz.y = y;     point_xyz.z = z;
        pc_src.push_back(point_xyz);
    }

    pcl::PointCloud<pcl::PointXYZ> pc_filtered;
    pcl::PointCloud<pcl::PointXYZ>::Ptr ptr_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PassThrough<pcl::PointXYZ> filter;

    double min_range = 0.5;
    double max_range = 100.0;
    *ptr_filtered = pc_src;

    filter.setInputCloud(ptr_filtered);
    filter.setFilterFieldName("x");
    filter.setFilterLimits(min_range, max_range);
// filter.setFilterLimitsNegative(true);
    filter.filter(*ptr_filtered);

    pc_filtered = *ptr_filtered;

    return 0;
}