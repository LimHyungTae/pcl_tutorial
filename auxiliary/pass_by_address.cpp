//
// Created by shapelim on 21. 9. 8..
//

#include "ceres/ceres.h"
#include "glog/logging.h"

using namespace std;

void PassByAddress(const Eigen::Vector3d *b) {
    cout << (*b)(0) << ", " << (*b)(1) << ", " << (*b)(2) << endl;
}

void PassByData(const double* b) {
    // Pointer array
    cout << b[0] << ", " << b[1] << ", " << b[2] << endl;
}


void bar(double *b) {
    cout << b[0] << ", " << b[1] << ", " << b[2] << endl;
}

int main() {
    Eigen::Vector3d a(0.0, 1.1, 1.3);
    PassByAddress(&a);

    Eigen::Vector3d b(-10.0, 1.0, 0.0);
    PassByData(b.data());

    double *ts_est = (double *) malloc(3 * sizeof(double));
    ts_est[0] = 0.0;
    ts_est[1] = 1.0;
    ts_est[2] = 0.0;
    bar(ts_est);

    return 0;
}