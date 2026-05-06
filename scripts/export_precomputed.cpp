//
// Tutorial Author: shapelim@mit.edu (임형태)
//
// Runs ICP/GICP/Normal pipelines once and writes their inputs/outputs as PCD
// files into web/public/precomputed/ for the React tutorial site to load.

#include <filesystem>
#include <iostream>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/gicp.h>

namespace fs = std::filesystem;

static pcl::PointCloud<pcl::PointXYZ>::Ptr load_kitti_bin(const std::string &filename) {
    FILE *file = fopen(filename.c_str(), "rb");
    if (!file) {
        std::cerr << "[export] failed to open " << filename << std::endl;
        return nullptr;
    }
    std::vector<float> buffer(1'000'000);
    size_t num_points =
            fread(reinterpret_cast<char *>(buffer.data()), sizeof(float), buffer.size(), file) / 4;
    fclose(file);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    cloud->resize(num_points);
    for (size_t i = 0; i < num_points; ++i) {
        cloud->points[i].x = buffer[i * 4];
        cloud->points[i].y = buffer[i * 4 + 1];
        cloud->points[i].z = buffer[i * 4 + 2];
    }
    return cloud;
}

template <typename Reg>
static void run_registration(const std::string &name,
                             Reg &reg,
                             pcl::PointCloud<pcl::PointXYZ>::Ptr src,
                             pcl::PointCloud<pcl::PointXYZ>::Ptr tgt,
                             const fs::path &outDir) {
    reg.setMaxCorrespondenceDistance(1.0);
    reg.setTransformationEpsilon(0.003);
    reg.setMaximumIterations(1000);
    reg.setInputSource(src);
    reg.setInputTarget(tgt);

    pcl::PointCloud<pcl::PointXYZ>::Ptr aligned(new pcl::PointCloud<pcl::PointXYZ>);
    reg.align(*aligned);

    pcl::io::savePCDFileBinary((outDir / (name + "_src.pcd")).string(), *src);
    pcl::io::savePCDFileBinary((outDir / (name + "_tgt.pcd")).string(), *tgt);
    pcl::io::savePCDFileBinary((outDir / (name + "_aligned.pcd")).string(), *aligned);

    std::cout << "[export] " << name
              << "  fitness=" << reg.getFitnessScore()
              << "  converged=" << reg.hasConverged() << std::endl;
}

int main(int argc, char **argv) {
    fs::path outDir = (argc > 1) ? fs::path(argv[1])
                                 : fs::path("../web/public/precomputed");
    fs::create_directories(outDir);
    std::cout << "[export] writing to " << outDir << std::endl;

    auto src = load_kitti_bin("./auxiliary/kitti00_000000.bin");
    if (!src) return 1;

    // Synthetic target = src translated +2m along x.
    pcl::PointCloud<pcl::PointXYZ>::Ptr tgt(new pcl::PointCloud<pcl::PointXYZ>);
    Eigen::Matrix4f tf = Eigen::Matrix4f::Identity();
    tf(0, 3) = 2.0f;
    pcl::transformPointCloud(*src, *tgt, tf);

    {
        pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
        run_registration("icp", icp, src, tgt, outDir);
    }
    {
        pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> gicp;
        run_registration("gicp", gicp, src, tgt, outDir);
    }

    std::cout << "[export] done" << std::endl;
    return 0;
}
