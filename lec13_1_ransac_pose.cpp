//
// Tutorial Author: shapelim@kaist.ac.kr (임형태)


#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/common/transforms.h>
#include <pcl/registration/gicp.h>
#include <random>

using namespace std;

Eigen::Matrix3d getRz(const double rad) {
    Eigen::Matrix3d rot = Eigen::Matrix3d::Identity();
    rot(0, 0) = static_cast<double>(cos(rad));
    rot(0, 1) = static_cast<double>(-sin(rad));
    rot(1, 0) = static_cast<double>(sin(rad));
    rot(1, 1) = static_cast<double>(cos(rad));
    return rot;
}

Eigen::Matrix3d getRy(const double rad) {
    Eigen::Matrix3d rot = Eigen::Matrix3d::Identity();
    rot(0, 0) = static_cast<double>(cos(rad));
    rot(0, 2) = static_cast<double>(sin(rad));
    rot(2, 0) = static_cast<double>(-sin(rad));
    rot(2, 2) = static_cast<double>(cos(rad));
    return rot;
}

Eigen::Matrix3d getRx(const double rad) {
    Eigen::Matrix3d rot = Eigen::Matrix3d::Identity();
    rot(1, 1) = static_cast<double>(cos(rad));
    rot(1, 2) = static_cast<double>(-sin(rad));
    rot(2, 1) = static_cast<double>(sin(rad));
    rot(2, 2) = static_cast<double>(cos(rad));
    return rot;
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

    double lower_bound                               = -20.0;
    double upper_bound                               = 20.0;

    std::random_device                     rd;  //Will be used to obtain a seed for the random number engine
    std::mt19937                           gen(rd());
    std::uniform_real_distribution<double> unif(lower_bound, upper_bound);
    std::default_random_engine             re;

    /*
     * Generate dummy source point cloud
     */
    Eigen::Matrix<double, 4, Eigen::Dynamic> src_h;
    int                                      num_pts = 100;
    src_h.resize(4, num_pts);
    for (int i = 0; i < num_pts; ++i) {

        for (int j  = 0; j < 3; ++j) {
            double random_value = unif(re);
            cout << random_value << endl;
            cout << i << " " << j << endl;
            src_h(j, i) = random_value;
        }
        src_h(3, i) = 1;
    }

    double upper_small_angle_bound = 10.0;
    double upper_translation_bound = 5.0;

    std::uniform_real_distribution<double> unif_rot(-upper_small_angle_bound, upper_small_angle_bound);
    std::uniform_real_distribution<double> unif_ts(-upper_translation_bound, upper_translation_bound);

    /*
     * Generate Dummy SE(3)
     */
    const double yaw_gt   = unif_rot(re) * M_PI / 180; // radian
    const double pitch_gt = unif_rot(re) * M_PI / 180; // radian
    const double roll_gt  = unif_rot(re) * M_PI / 180; // radian

    const double x_gt  = unif_ts(re);
    const double y_gt  = unif_ts(re);
    const double z_gt  = unif_ts(re);

    Eigen::Matrix3d rot_gt;
    Eigen::Matrix3d Rz = getRz(yaw_gt);
    Eigen::Matrix3d Ry = getRy(pitch_gt);
    Eigen::Matrix3d Rx = getRx(roll_gt);
    rot_gt = Rz * Ry * Rx;

    Eigen::Matrix4d T_gt = Eigen::Matrix4d::Identity();
    T_gt.block<3, 3>(0, 0) = rot_gt;
    T_gt(0, 3)             = x_gt;
    T_gt(1, 3)             = y_gt;
    T_gt(2, 3)             = z_gt;

    Eigen::Matrix4f tf;
    tf << 1, 0, 0, 2.0,
            0, 1, 0, 0.0,
            0, 0, 1, 0.0,
            0, 0, 0, 1.0;
    pcl::transformPointCloud(*src, *tgt, tf);

    pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setMaxCorrespondenceDistance(1.0);
    icp.setTransformationEpsilon(0.003);
    icp.setMaximumIterations(1000);
    icp.setRANSACIterations(500);
    icp.setEuclideanFitnessEpsilon(1);

    pcl::PointCloud<pcl::PointXYZ>::Ptr align(new pcl::PointCloud<pcl::PointXYZ>);
    icp.setInputSource(src);
    icp.setInputTarget(tgt);
    icp.align(*align);
    // Set outputs
    Eigen::Matrix4f src2tgt   = icp.getFinalTransformation();
    double score     = icp.getFitnessScore();
    bool is_converged = icp.hasConverged();

    cout<<src2tgt<<endl;
    cout<<score<<endl;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr src_colored(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr tgt_colored(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr align_colored(new pcl::PointCloud<pcl::PointXYZRGB>);
    colorize(*src, *src_colored, {255, 0, 0});
    colorize(*tgt, *tgt_colored, {0, 255, 0});
    colorize(*align, *align_colored, {0, 0, 255});

    /*
     * 주의: PCL 버전이 높은 데서만 지원 (TEST: PCL v.1.8)
     * Visualization
     */
    pcl::visualization::CloudViewer viewer("Cloud Viewer");
    viewer.showCloud(src_colored, "src_viz");
    viewer.showCloud(tgt_colored, "tgt_viz");
    viewer.showCloud(align_colored, "align_viz");

    int cnt = 0;
    while (!viewer.wasStopped()) {
        //you can also do cool processing here
        //FIXME: Note that this is running in a separate thread from viewerPsycho
        //and you should guard against race conditions yourself...
        cnt++;
    }

}