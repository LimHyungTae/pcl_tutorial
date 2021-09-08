//
// Tutorial Author: shapelim@kaist.ac.kr (임형태)
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>

using namespace std;

#include <iostream>
#include <pcl/common/transforms.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>


// Input: pcl::PointCloud source, namely cloud_src
//Output: Transformed pcl::PointCloud, namely pc_transformed, via 4x4 transformation matrix
int main(int argc, char **argv){

    pcl::PointCloud<pcl::PointXYZ> cloud_src;

    pcl::PointXYZ point_xyz; // pcl::PointXYZ이라는 type에 data를 담는다.
    for (int k=0; k < 5 ; ++k){
        float x = static_cast <float> (rand()) / static_cast <float> (RAND_MAX) * 5;
        float y = static_cast <float> (rand()) / static_cast <float> (RAND_MAX) * 5;
        float z = static_cast <float> (rand()) / static_cast <float> (RAND_MAX) * 5;
        point_xyz.x = x;     point_xyz.y = y;     point_xyz.z = z;
        cloud_src.push_back(point_xyz);
    }

    pcl::PointCloud<pcl::PointXYZ> pc_transformed;
    pcl::PointCloud<pcl::PointXYZ>::Ptr ptr_transformed(new pcl::PointCloud<pcl::PointXYZ>);

    Eigen::Matrix4f trans;
    trans<< 1,   0,  0, 0.165,
            0,   1,  0, 0.000,
            0,   0,  1, 0.320,
            0,   0,  0,     1;
    pcl::transformPointCloud(cloud_src, *ptr_transformed, trans);

    pc_transformed = *ptr_transformed;
    return 0;

}
