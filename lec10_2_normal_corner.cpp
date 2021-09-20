//
// Tutorial Author: shapelim@kaist.ac.kr (임형태)
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>

int main(int argc, char **argv)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointXYZ pt;

    pt.x = 0; pt.y = 0; pt.z = 0;
    cloud->points.push_back(pt);

    pt.x = 0.01; pt.y = 0; pt.z = 0;
    cloud->points.push_back(pt);

    pt.x = 0; pt.y = 0.01; pt.z = 0;
    cloud->points.push_back(pt);

    pt.x = -0.01; pt.y = 0; pt.z = 0;
    cloud->points.push_back(pt);

    pt.x = 0; pt.y = -0.01; pt.z = 0;
    cloud->points.push_back(pt);

    pt.x = 10.0; pt.y = 10.0; pt.z = 0;
    cloud->points.push_back(pt);

    /**
     * Main
     */
    // Create the normal estimation class, and pass the input dataset to it
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    ne.setInputCloud (cloud);

    // Create an empty kdtree representation, and pass it to the normal estimation object.
    // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
    ne.setSearchMethod (tree);

    // Output datasets
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);

    // Use all neighbors in a sphere of radius 3cm
    ne.setRadiusSearch (0.03);

    ne.compute (*cloud_normals);

    /** print */
    for (auto const &pt:cloud_normals->points){
        std::cout<<pt<<std::endl;
    }
}