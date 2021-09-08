//
// Tutorial Author: shapelim@kaist.ac.kr (임형태)

#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl/visualization/cloud_viewer.h>
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
    *src = *load_bin("/home/shapelim/catkin_ws/src/pcl_tutorial/materials/000000.bin");

    Eigen::Matrix4f tf;
    tf << 1, 0, 0, 5.0,
            0, 1, 0, 0.0,
            0, 0, 1, 0.0,
            0, 0, 0, 1.0;
    pcl::transformPointCloud(*src, *tgt, tf);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr src_colored(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr tgt_colored(new pcl::PointCloud<pcl::PointXYZRGB>);
    colorize(*src, *src_colored, {255, 0, 0});
    colorize(*tgt, *tgt_colored, {0, 255, 0});

    /*
     * 주의: PCL 버전이 높은 데서만 지원 (TEST: PCL v.1.8)
     * Visualization
     */
    pcl::visualization::CloudViewer viewer("Cloud Viewer");
    viewer.showCloud(src_colored, "src_viz");
    viewer.showCloud(tgt_colored, "tgt_viz");

    int cnt = 0;
    while (!viewer.wasStopped()) {
        //you can also do cool processing here
        //FIXME: Note that this is running in a separate thread from viewerPsycho
        //and you should guard against race conditions yourself...
        cnt++;
    }

}
