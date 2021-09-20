//
// Tutorial Author: shapelim@kaist.ac.kr (임형태)

#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <iostream>
#include <vector>
#include <chrono>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>


template<typename T>
void voxelize(const boost::shared_ptr<pcl::PointCloud<T> > srcPtr, pcl::PointCloud<T> &dst, double voxelSize) {
    static pcl::VoxelGrid<T> voxel_filter;
    voxel_filter.setInputCloud(srcPtr);
    voxel_filter.setLeafSize(voxelSize, voxelSize, voxelSize);
    voxel_filter.filter(dst);
}

template<typename KDTree>
void calc_normal(KDTree &kdtree,
                 pcl::PointCloud<pcl::PointXYZ>::Ptr &src,
                 int K,
                 pcl::PointCloud<pcl::Normal>::Ptr &cloud_normal) {

    //Initialization
    /** KdTree 세팅 */
    std::chrono::system_clock::time_point t_start = std::chrono::system_clock::now();
    kdtree.setInputCloud(src);
    std::chrono::system_clock::time_point t_mid = std::chrono::system_clock::now();

    pcl::Normal normal_tmp;
    std::vector<int> idxes(K);
    std::vector<float> sqr_dists(K);
    int num_pc = src->points.size();

    for (int i = 0; i < num_pc; ++i) {
        pcl::PointXYZ &query = src->points[i];
        kdtree.nearestKSearch(query, K, idxes, sqr_dists);
        // reserve() is important for speed up!
        pcl::PointCloud<pcl::PointXYZ>::Ptr NN(new pcl::PointCloud<pcl::PointXYZ>);
        NN->reserve(num_pc);
        for (int tgt_idx: idxes) {
            NN->points.emplace_back(src->points[tgt_idx]);
        }
        Eigen::Matrix3f cov;
        Eigen::Vector4f mean;
        pcl::computeMeanAndCovarianceMatrix(*NN, cov, mean);
        // Singular Value Decomposition: SVD
        Eigen::JacobiSVD<Eigen::MatrixXf> svd(cov, Eigen::DecompositionOptions::ComputeFullU);
        /**
         * SVD에서 가장 작은 eigenvalue와 대응되는 eigenvector가
         * 그 인근 pointcloud에서 추출한 normal vector
         */
        Eigen::MatrixXf normal = (svd.matrixU().col(2));
        normal_tmp.normal_x = normal(0, 0);
        normal_tmp.normal_y = normal(1, 0);
        normal_tmp.normal_z = normal(2, 0);
        cloud_normal->points.emplace_back(normal_tmp);
    }
    std::chrono::system_clock::time_point t_end = std::chrono::system_clock::now();

    std::chrono::duration<double> t_setting = t_mid - t_start;
    std::chrono::duration<double> t_normal = t_end - t_mid;

    cout << "Setting: " << t_setting.count() << " sec..." << endl;
    cout << "Searching: " << t_normal.count() << " sec..." << endl;
}

int main(int argc, char **argv) {
    /*
     * Load toy data
     */
    pcl::PointCloud<pcl::PointXYZ>::Ptr raw_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_voxel(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(
            "/home/shapelim/git/pcl_tutorial/materials/naverlabs_vel16.pcd", *raw_cloud) ==
        -1) {
        PCL_ERROR ("Couldn't read source pcd file! \n");
        return (-1);
    }

    /*
     * Preprocessing
     * Voxelization -> Transformation 순으로 처리하는게 연산 효율적임
     */
    voxelize(raw_cloud, *cloud_voxel, 0.05);
    *raw_cloud = *cloud_voxel;

    Eigen::Matrix4f lidar0ToBody;
    lidar0ToBody << 0, -1, 0, 0,
            -1, 0, -0, 0,
            0, 0, -1, 0.4,
            0, 0, 0, 1;
    pcl::transformPointCloud(*raw_cloud, *cloud, lidar0ToBody);

    /**
     * Main
     */
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    int K = 20;
    int num_pc = cloud->points.size();

    pcl::PointCloud<pcl::Normal>::Ptr cloud_normal(new pcl::PointCloud<pcl::Normal>);
    cloud_normal->points.reserve(num_pc);

    calc_normal(kdtree, cloud, K, cloud_normal);
    std::cout << "Total Num: " << num_pc << std::endl;
    /*******************************************/

    /**
     * 결과 visualization 하기
     */
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_color(new pcl::PointCloud<pcl::PointXYZRGB>);
    std::uint8_t r(255), g(15), b(15);
    for (const auto &basic_point: cloud->points) {
        pcl::PointXYZRGB point;
        point.x = basic_point.x;
        point.y = basic_point.y;
        point.z = basic_point.z;
        std::uint32_t rgb = (static_cast<std::uint32_t>(r) << 16 |
                             static_cast<std::uint32_t>(g) << 8 | static_cast<std::uint32_t>(b));
        point.rgb = *reinterpret_cast<float *>(&rgb);
        cloud_color->points.push_back(point);
    }

    pcl::visualization::PCLVisualizer viewer("Simple Cloud Viewer");
    viewer.addPointCloud<pcl::PointXYZRGB>(cloud_color);

    int level = 7; // the larger, more normal vectors are displayed
    float scale = 0.7; // size of arrow
    viewer.addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal>(cloud_color, cloud_normal, level, scale,
                                                               "pc_w_normal", 0);
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "pc_w_normal");

    while (!viewer.wasStopped()) {
        viewer.spinOnce();
    }
}