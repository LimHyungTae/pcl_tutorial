//
// Tutorial Author: shapelim@kaist.ac.kr (임형태)
#include <limits.h> /* PATH_MAX = 4096 */
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/transforms.h>

using namespace std;

pcl::PointCloud<pcl::PointXYZ>::ConstPtr load_bin(const string &filename) {
    FILE*file                     = fopen(filename.c_str(), "rb");
    if (!file) {
        std::cerr << "Error: failed to load " << filename << std::endl;
        return nullptr;
    }
    std::vector<float> buffer(1000000);
    size_t             num_points =
                               fread(reinterpret_cast<char*>(buffer.data()), sizeof(float), buffer.size(), file) / 4;
    fclose(file);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    cloud->resize(num_points);

    for (int i = 0; i < num_points; i++) {
        auto &pt = cloud->at(i);
        pt.x = buffer[i * 4];
        pt.y = buffer[i * 4 + 1];
        pt.z = buffer[i * 4 + 2];
        // pt.intensity = buffer[i * 4 + 3];
    }

    return cloud;
}

void colorize(const pcl::PointCloud<pcl::PointXYZ> &pc,
              pcl::PointCloud<pcl::PointXYZRGB> &pc_colored,
              const std::vector<int> &color) {

    int N              = pc.points.size();

    pc_colored.clear();
    pcl::PointXYZRGB pt_tmp;
    for (int         i = 0; i < N; ++i) {
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

int main(int argc, char**argv) {
    /*
     * Load toy data
     */
    pcl::PointCloud<pcl::PointXYZ>::Ptr src(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr tgt(new pcl::PointCloud<pcl::PointXYZ>);
    *src = *load_bin("/home/shapelim/git/pcl_tutorial/materials/kitti00_000000.bin");

    Eigen::Matrix4f tf;
    tf << 1, 0, 0, 5.0,
            0, 1, 0, 0.0,
            0, 0, 1, 0.0,
            0, 0, 0, 1.0;
    pcl::transformPointCloud(*src, *tgt, tf);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr src_colored(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr tgt_colored(new pcl::PointCloud<pcl::PointXYZRGB>);

    // Point cloud XYZ에 RGB 칼라 추가하기
    colorize(*src, *src_colored, {255, 0, 0});
    colorize(*tgt, *tgt_colored, {0, 255, 0});

    /*
     * Method 1. PCLVisualizer
     */

    pcl::visualization::PCLVisualizer viewer1("Simple Cloud Viewer");
    viewer1.addPointCloud<pcl::PointXYZRGB>(src_colored, "src_red");
    viewer1.addPointCloud<pcl::PointXYZRGB>(tgt_colored, "tgt_green");

    while (!viewer1.wasStopped()) {
        viewer1.spinOnce();
    }


    /*
     * Method 2. CloudViewer
     * 주의: PCL 버전이 높은 데서만 지원 (TEST: PCL v.1.8)
     */
//    pcl::visualization::CloudViewer viewer2("Cloud Viewer");
//    viewer2.showCloud(src_colored, "src_red");
//    viewer2.showCloud(tgt_colored, "tgt_green");
//
//    int cnt = 0;
//    while (!viewer2.wasStopped()) {
//        cnt++;
//    }

}
