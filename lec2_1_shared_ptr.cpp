//
// Tutorial Author: shapelim@kaist.ac.kr (임형태)
#include <pcl/PCLPointCloud2.h>

using namespace std;

int main(int argc, char**argv) {
    // 선언
    boost::shared_ptr<vector<int> > vec_ptr(new vector<int>());
    vector<int>                     data = {1, 2, 3, 4, 5};

    // 문법: *shared_ptr에 데이터를 복사하겠다는 의미
    *vec_ptr = data;
    for (int i = 0; i < vec_ptr->size(); ++i) {
        cout << (*vec_ptr)[i] << ", ";
    }
    cout << endl;

    // 선언 2
    boost::shared_ptr<vector<int> > vec_ptr2;
    vec_ptr2.reset(new vector<int>(3, 5));
    for (int i = 0; i < vec_ptr2->size(); ++i) {
        cout << (*vec_ptr2)[i] << ", ";
    }
    cout << endl;

    return 0;
}