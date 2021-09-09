//
// Tutorial Author: shapelim@kaist.ac.kr (임형태)
#include <pcl/point_types.h>
#include <pcl/conversions.h>

using namespace std;

template<class T>
void print_pc(pcl::PointCloud<T> &cloud) {
    int             count = 0;
    for (const auto &pt: cloud.points) {
        cout << count++ << ": ";
        cout << pt.x << ", " << pt.y << ", " << pt.z << endl;
    }
}

template<class T>
void print_pc_via_copy(pcl::PointCloud<T> cloud) {
    int             count = 0;
    for (const auto &pt: cloud.points) {
        cout << count++ << ": ";
        cout << pt.x << ", " << pt.y << ", " << pt.z << endl;
    }
}

void print_ptr(pcl::PointCloud<pcl::PointXYZ>::Ptr ptr_cloud) {
    int             count = 0;
    for (const auto &pt: ptr_cloud->points) {
        cout << count++ << ": ";
        cout << pt.x << ", " << pt.y << ", " << pt.z << endl;
    }
}

void print_address(pcl::PointCloud<pcl::PointXYZ>::Ptr ptr_cloud) {
    cout << ptr_cloud << endl;
}

void print_address2(pcl::PointCloud<pcl::PointXYZ>::Ptr &ptr_cloud) {
    cout << ptr_cloud << endl;
}

void add_pc(pcl::PointCloud<pcl::PointXYZ>::Ptr ptr_cloud) {
    ptr_cloud->points.emplace_back(pcl::PointXYZ(0, 0, 1));
}

void add_pc2(pcl::PointCloud<pcl::PointXYZ>::Ptr &ptr_cloud) {
    ptr_cloud->points.emplace_back(pcl::PointXYZ(1, 0, 0));
}

void consptr_test(pcl::PointCloud<pcl::PointXYZ>::ConstPtr ptr_cloud) {
    pcl::PointCloud<pcl::PointXYZ> pc = *ptr_cloud;
    print_pc(pc);
//    print_pc(*ptr_cloud); // Cause error due to the call-by-reference of "print_pc" function
    print_pc_via_copy(*ptr_cloud);
}


int main(int argc, char **argv) {
    pcl::PointCloud<pcl::PointXYZ> cloud2;
    cloud2.push_back(pcl::PointXYZ(1, 2, 3));
    cloud2.push_back(pcl::PointXYZ(4, 5, 6));

    pcl::PointCloud<pcl::PointXYZ> cloud3;
    cloud3.push_back(pcl::PointXYZ(7, 8, 9));
    cloud3.push_back(pcl::PointXYZ(10, 11, 12));

    cloud2 += cloud3;

    pcl::PointCloud<pcl::PointXYZ>::Ptr ptr_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr ptr_cloud2(new pcl::PointCloud<pcl::PointXYZ>);
    *ptr_cloud = cloud2;

    cout << "Before: " << endl;
    print_pc(cloud3);
    // 0: 7, 8, 9
    // 1: 10, 11, 12

    // ptr_cloud가 가리키고 있던 pointcloud를 cloud3에 복사해서 전달
    cloud3 = *ptr_cloud;
    cout << "After: " << endl;
    print_pc(cloud3);
    // 0: 1, 2, 3
    // 1: 4, 5, 6
    // 2: 7, 8, 9
    // 3: 10, 11, 12

    // ptr간의 = 기호는 주솟값을 공유한다는 뜻임. 서로 같은 데이터를 가리킴
    ptr_cloud2 = ptr_cloud;

    std::cout << ptr_cloud << std::endl;
    // 0x55b586287000
    std::cout << ptr_cloud2 << std::endl;
    // 0x55b586287000 (90번 줄과 같은 값)

    // 따라서 print된 것을 확인해보면, ptr_cloud2 뒤에 넣었는데 ptr_cloud의 제일 뒤에도 (-1, -2, -3)이 들어가 있음!
    ptr_cloud2->points.emplace_back(pcl::PointXYZ(-1, -2, -3));
    std::cout << ptr_cloud->points.back().x << ", " << ptr_cloud->points.back().y << ", " << ptr_cloud->points.back().z << std::endl;
    // -1, -2, -3

    cout << " ----- clouds ----- " << endl;
    print_pc(cloud2); // does not change
    // 0: 1, 2, 3
    // 1: 4, 5, 6
    // 2: 7, 8, 9
    // 3: 10, 11, 12
    print_pc(cloud3); // does not change
    // 0: 1, 2, 3
    // 1: 4, 5, 6
    // 2: 7, 8, 9
    // 3: 10, 11, 12
    // 따라서, Ptr에 값을 복사한 이후에 두 객체는 독립적임!
    cout << " ----- Ptrs ----- " << endl;
    print_ptr(ptr_cloud);
    // 0: 1, 2, 3
    // 1: 4, 5, 6
    // 2: 7, 8, 9
    // 3: 10, 11, 12
    // 4: -1, -2, -3
    print_ptr(ptr_cloud2);
    // 0: 1, 2, 3
    // 1: 4, 5, 6
    // 2: 7, 8, 9
    // 3: 10, 11, 12
    // 4: -1, -2, -3
    // 91번째 줄의 결과로, ptr_cloud2와 ptr_cloud가 가리키는 points가 같다!!
    cout << " ---------------- " << endl;

    // 함수 인자로 받을때: 둘다 위에서 출력한 주소와 같은 주소를 출력함
    // 90~92번째 줄과 같은 주소값을 print함
    print_address(ptr_cloud);
    print_address2(ptr_cloud);

    pcl::PointCloud<pcl::PointXYZ>::Ptr ptr_cloud3(new pcl::PointCloud<pcl::PointXYZ>);

    // 88 번줄과는 다르게 *$Ptr$ = *$Ptr$을 하면 메모리에 있던 정보를 복사해서 넘겨준다는 의미임
    // 따라서, ptr_cloud3와 ptr_cloud는 둘이 독립적임!
    *ptr_cloud3 = *ptr_cloud;

    cout << " ----- add_pc ----- " << endl;
    add_pc(ptr_cloud); // call-by-value
    add_pc2(ptr_cloud); // call-by-reference
    print_ptr(ptr_cloud); // value로 받든, reference로 받든, 상관없음
    // 0: 1, 2, 3
    // 1: 4, 5, 6
    // 2: 7, 8, 9
    // 3: 10, 11, 12
    // 4: -1, -2, -3
    // 5: 0, 0, 1
    // 6: 1, 0, 0
    cout << " ------ $Ptr$ = $Ptr$ ------ " << endl;
    // ptr_cloud2는 88번째 줄에서 $Ptr$ = $Ptr$로 할당받았는데,
    // ptr_cloud와 같은 객체를 가리키고 있다보니 add_pc()와 add_pc2() 함수의 영향을 받음
    print_ptr(ptr_cloud2); // changes
    // 0: 1, 2, 3
    // 1: 4, 5, 6
    // 2: 7, 8, 9
    // 3: 10, 11, 12
    // 4: -1, -2, -3
    // 5: 0, 0, 1 <- add_pc()함수로 인해 추가됨
    // 6: 1, 0, 0 <- add_pc2()함수로 인해 추가됨

    cout << " ------ *$Ptr$ = *$Ptr$ ------ " << endl;
    // ptr_cloud3는 137번째 줄에서 *$Ptr$ = *$Ptr$로 할당받았는데,
    // rvalue Ptr이 가리키던 points를 복사했다보니,
    // ptr_cloud3는 add_pc()와 add_pc2() 함수의 영향 x
    print_ptr(ptr_cloud3); // does not change
    // 0: 1, 2, 3
    // 1: 4, 5, 6
    // 2: 7, 8, 9
    // 3: 10, 11, 12
    // 4: -1, -2, -3

    cout << " --- ConstPtr test --- " << endl;
    consptr_test(ptr_cloud);
}