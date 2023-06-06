#include <cmath>
#include <iostream>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/geometry>

#include "sophus/se3.hpp"

using namespace Eigen;

using namespace std;

int main(int argc, char **argv) {
    Matrix3d R = AngleAxisd(M_PI / 2, Vector3d(0, 0, 1)).toRotationMatrix();
    Quaterniond q(R);
    Sophus::SO3d SO3_R(R);
    Sophus::SO3d SO3_q(q);
    cout << "SO3 from rotation matrix \n" << SO3_R.matrix() << endl;
    cout << "SO3 from Quaterniond \n" << SO3_q.matrix() << endl;

    Vector3d so3 = SO3_R.log();
    cout << "so3 \n" << so3.transpose() << endl;

    cout << "so3 hat \n" << Sophus::SO3d::hat(so3) << endl;
    cout << "so3 vee\n" << Sophus::SO3d::vee(Sophus::SO3d::hat(so3)) << endl;

    Vector3d update_so3(1e-4, 0, 0);

    Sophus::SO3d SO3_updated = Sophus::SO3d::exp(update_so3) * SO3_R;
    cout << "SO3 updated = \n" << SO3_updated.matrix() << endl;
    Vector3d t(1, 0, 0);

    Sophus::SE3d SE3_R(R, t);
    Sophus::SE3d SE3_q(q, t);

    typedef Matrix<double, 6, 1> Vector6d;
    Vector6d se3 = SE3_R.log();
    cout << "se3 \n" << se3.transpose() << endl;
    cout << "se3 hat \n" << Sophus::SE3d::hat(se3) << endl;
    cout << "se3 vee\n" << Sophus::SE3d::vee(Sophus::SE3d::hat(se3)) << endl;
    Vector6d update_se3;
    update_se3.setZero();
    update_se3(0, 0) = 1e-4;
    Sophus::SE3d SE3_updated = Sophus::SE3d::exp(update_se3) * SE3_R;
    cout << "SE3 updated = " << SE3_updated.matrix() << endl;

    return 0;
}
