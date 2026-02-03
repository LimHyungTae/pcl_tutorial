//
// Tutorial Author: shapelim@kaist.ac.kr (임형태)
#include <iostream>
#include <pcl/filters/passthrough.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <string>
#include <pcl/visualization/pcl_visualizer.h>

using namespace std;

pcl::PointCloud<pcl::PointXYZ>::ConstPtr load_bin(const string &filename) {
    FILE *file = fopen(filename.c_str(), "rb");
    if (!file) {
        std::cerr << "Error: failed to load " << filename << std::endl;
        return nullptr;
    }
    std::vector<float> buffer(1000000);
    size_t num_points =
            fread(reinterpret_cast<char *>(buffer.data()), sizeof(float), buffer.size(), file) / 4;
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

int main(int argc, char **argv) {
    /*
     * Load toy data
     */
    pcl::PointCloud<pcl::PointXYZ>::Ptr src(new pcl::PointCloud<pcl::PointXYZ>);
    *src = *load_bin("/home/shapelim/git/pcl_tutorial/materials/kitti00_000000.bin");

    /**
     * Main
     */
    pcl::PointCloud<pcl::PointXYZ>::Ptr center(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr outskirt(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr output(new pcl::PointCloud<pcl::PointXYZ>);

    float car_size = 3.0; // 3.0m
    // X 축으로 filtering
    // 중앙 부분 추출
    pcl::PassThrough<pcl::PointXYZ> xfilter;
    xfilter.setInputCloud(src);
    xfilter.setFilterFieldName("x");
    xfilter.setFilterLimits(-car_size, car_size);
    xfilter.filter(*center);
    // 중앙 이외의 부분 추출
    // 한번 setInputCloud를 해두면 여러가지로 filter를 계속 할 수 있음
    // Class 내부에 setInputCloud로 설정된 member변수 pointcloud는 변하지 않기 때문
    xfilter.setNegative(true);
    xfilter.filter(*outskirt);

    // 그 후 y축 방향으로 중앙에 있는 부분 제거
    pcl::PassThrough<pcl::PointXYZ> yfilter;
    xfilter.setInputCloud(center);
    xfilter.setFilterFieldName("y");
    xfilter.setFilterLimits(-car_size, car_size);
    xfilter.setNegative(true);
    xfilter.filter(*output);

    *output += *outskirt;
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

    while (!viewer1.wasStopped() && !viewer2.wasStopped()) {
        viewer1.spinOnce();
        viewer2.spinOnce();
    }


    return 0;

}

