//
// Tutorial Author: shapelim@kaist.ac.kr (임형태)
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>

using namespace std;

class DummyClass {
public:
    // Error Case 1
    // 선언 단에 new를 하면 error가 남!
//    pcl::PointCloud<pcl::PointXYZ>::Ptr wrong_member(new pcl::PointCloud<pcl::PointXYZ>);

    // Error Case 2를 해결하기 위해, 선언만 member 변수로 해 둠
    pcl::PointCloud<pcl::PointXYZ>::Ptr right_member;

    DummyClass() {
        // *중요* 그 후, 생성자에서 초기화를 꼭 해줘야함
        // .reset() 함수 사용
        // 아래 줄이 없으면 에러가 남!!!
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
//     1, 2, 3
    return 0;
}