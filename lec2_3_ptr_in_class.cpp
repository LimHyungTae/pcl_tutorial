//
// Tutorial Author: shapelim@kaist.ac.kr (임형태)
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>

using namespace std;

class DummyClass {
public:
    // Case 1
    // 선언 단에 new를 하면 error가 남!
//    pcl::PointCloud<pcl::PointXYZ>::Ptr wrong_member(new pcl::PointCloud<pcl::PointXYZ>);

    // Case 2
    pcl::PointCloud<pcl::PointXYZ>::Ptr right_member;

    DummyClass() {
        // *중요* 필수로 생성자 내에서 reset을 해줘야 함!!!!
        right_member.reset(new pcl::PointCloud<pcl::PointXYZ>());
        cout << "Constructor complete" << endl;
    }
    ~DummyClass() {}

};

int main() {
    DummyClass ptr_test = DummyClass();

    pcl::PointCloud<pcl::PointXYZ> cloud;
    cloud.push_back(pcl::PointXYZ(1, 2, 3));
    cloud.push_back(pcl::PointXYZ(4, 5, 6));

    *ptr_test.right_member = cloud;
    cout << ptr_test.right_member->points[0].x << ", " << ptr_test.right_member->points[0].y
         << ", " << ptr_test.right_member->points[0].z << endl;

    return 0;
}