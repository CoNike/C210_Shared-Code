#include <iostream>
#include "IKSolver.h"

IKSolver::IKSolver() {
}

IKSolver::IKSolver(double a1, double a2, double a3, double d1, double d4, double d6)
    : _a1(a1), _a2(a2), _a3(a3), _d1(d1), _d4(d4), _d6(d6), _Results(Eigen::MatrixXd::Constant(16, 6, 0.0)) {
}

void IKSolver::setParameters(double a1, double a2, double a3, double d1, double d4, double d6) {
    _a1 = a1;
    _a2 = a2;
    _a3 = a3;
    _d1 = d1;
    _d4 = d4;
    _d6 = d6;
    _Results = Eigen::MatrixXd::Constant(16, 6, 0.0);
}

IKSolver::~IKSolver() {
}

Eigen::Isometry3d IKSolver::getEndPose(const Eigen::VectorXd &jointAngles) {
    if (jointAngles.size() < 6) {
        std::cerr << "You need to input 6 joint angles!\n";
        return Eigen::Isometry3d::Identity();
    }
    Eigen::Matrix4d T01(Eigen::Matrix4d::Zero());
    T01(0, 0) = cos(jointAngles[0]);
    T01(0, 2) = -sin(jointAngles[0]);
    T01(0, 3) = _a1 * cos(jointAngles[0]);
    T01(1, 0) = sin(jointAngles[0]);
    T01(1, 2) = cos(jointAngles[0]);
    T01(1, 3) = _a1 * sin(jointAngles[0]);
    T01(2, 1) = -1;
    T01(2, 3) = _d1;
    T01(3, 3) = 1;
    Eigen::Matrix4d T12(Eigen::Matrix4d::Zero());
    T12(0, 0) = -sin(jointAngles[1]);
    T12(0, 1) = -cos(jointAngles[1]);
    T12(0, 3) = -_a2 * sin(jointAngles[1]);
    T12(1, 0) = cos(jointAngles[1]);
    T12(1, 1) = -sin(jointAngles[1]);
    T12(1, 3) = _a2 * cos(jointAngles[1]);
    T12(2, 2) = 1;
    T12(3, 3) = 1;
    Eigen::Matrix4d T23(Eigen::Matrix4d::Zero());
    T23(0, 0) = sin(jointAngles[2]);
    T23(0, 2) = -cos(jointAngles[2]);
    T23(0, 3) = _a3 * sin(jointAngles[2]);
    T23(1, 0) = -cos(jointAngles[2]);
    T23(1, 2) = -sin(jointAngles[2]);
    T23(1, 3) = -_a3 * cos(jointAngles[2]);
    T23(2, 1) = 1;
    T23(3, 3) = 1;
    Eigen::Matrix4d T34(Eigen::Matrix4d::Zero());
    T34(0, 0) = cos(jointAngles[3]);
    T34(0, 2) = -sin(jointAngles[3]);
    T34(1, 0) = sin(jointAngles[3]);
    T34(1, 2) = cos(jointAngles[3]);
    T34(2, 1) = -1;
    T34(2, 3) = _d4;
    T34(3, 3) = 1;
    Eigen::Matrix4d T45(Eigen::Matrix4d::Zero());
    T45(0, 0) = cos(jointAngles[4]);
    T45(0, 2) = sin(jointAngles[4]);
    T45(1, 0) = sin(jointAngles[4]);
    T45(1, 2) = -cos(jointAngles[4]);
    T45(2, 1) = 1;
    T45(3, 3) = 1;
    Eigen::Matrix4d T56(Eigen::Matrix4d::Zero());
    T56(0, 0) = cos(jointAngles[5]);
    T56(0, 1) = -sin(jointAngles[5]);
    T56(1, 0) = sin(jointAngles[5]);
    T56(1, 1) = cos(jointAngles[5]);
    T56(2, 2) = 1;
    T56(2, 3) = _d6;
    T56(3, 3) = 1;
    //_T06 = T01 * T12 * T23 * T34 * T45 * T56;
    //return Eigen::Isometry3d(_T06);
    return Eigen::Isometry3d(T01 * T12 * T23 * T34 * T45 * T56);
}

void IKSolver::setEndPose(const Eigen::Matrix4d &endPose) {
    _T06 = endPose;
}

void IKSolver::setEndPose(const Eigen::Isometry3d &endPose) {
    Eigen::Matrix4d Pose(endPose.data());
    _T06 = Pose;
}

void IKSolver::setEndPose(double x, double y, double z, double R, double P, double Y) {
    Eigen::AngleAxisd Rot_About_Z(R, Eigen::Vector3d::UnitZ());
    Eigen::AngleAxisd Rot_about_Y(P, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd Rot_About_X(Y, Eigen::Vector3d::UnitX());
    Eigen::Matrix3d Rotation = (Rot_About_Z * Rot_about_Y * Rot_About_X).matrix(); //convert quaternion to matrix
    Eigen::Matrix4d Pose(Eigen::Matrix4d::Identity());
    Pose.block(0, 0, 3, 3) = Rotation;
    Pose.block(0, 3, 3, 1) = Eigen::Vector3d(x, y, z);
    _T06 = Pose;
}

Eigen::MatrixXd IKSolver::solve() {
    Eigen::Matrix4d _T06_modified = _T06; //Modify the Pose of the End-effector
    _T06_modified.block(0, 3, 3, 1) -= _d6 * _T06_modified.block(0, 2, 3, 1);
    double theta1_1 = atan2(_T06_modified(1, 3), _T06_modified(0, 3));
    double theta1_2 = atan2(-_T06_modified(1, 3), -_T06_modified(0, 3));
    _Results.block(0, 0, 8, 1) = Eigen::VectorXd::Constant(8, 1, theta1_1);
    _Results.block(8, 0, 8, 1) = Eigen::VectorXd::Constant(8, 1, theta1_2);
    for (long i = 0; i < 2; ++i) {
        double A3 = 2 * _a2 * _a3;
        double B3 = -2 * _a2 * _d4;
        double D3 = pow(_a1 + (_d6 * _T06(0, 2) - _T06(0, 3)) * cos(_Results(8 * i, 0)) + (_d6 * _T06(1, 2) - _T06(1, 3)) * sin(_Results(8 * i, 0)), 2);
        double E3 = pow(_d1 + _d6 * _T06(2, 2) - _T06(2, 3), 2);
        double C3 = D3 + E3 - _a2 * _a2 - _a3 * _a3 - _d4 * _d4;
        if(A3 * A3 + B3 * B3 - C3 * C3 < 0) {
            _Results.block(8 * i, 2, 8, 1) = Eigen::VectorXd::Constant(8, 1, std::numeric_limits<double>::quiet_NaN());
        } else {
            double theta3_1 = atan2(C3, sqrt(A3 * A3 + B3 * B3 - C3 * C3)) - atan2(B3, A3);
            double theta3_2 = atan2(C3, -sqrt(A3 * A3 + B3 * B3 - C3 * C3)) - atan2(B3, A3);
            _Results.block(8 * i, 2, 4, 1) = Eigen::VectorXd::Constant(4, 1, theta3_1);
            _Results.block(8 * i + 4, 2, 4, 1) = Eigen::VectorXd::Constant(4, 1, theta3_2);
        }
    }
    for (long i = 0; i < 4; ++i) {
        double A2 = _a3 * cos(_Results(4 * i, 2)) + _d4 * sin(_Results(4 * i, 2));
        double B2 = _a2 + _a3 * sin(_Results(4 * i, 2)) - _d4 * cos(_Results(4 * i, 2));
        double C2 = _d1 + _d6 * _T06(2, 2) - _T06(2, 3);
        if (A2 * A2 + B2 * B2 - C2 * C2 < 0) {
            _Results.block(4 * i, 1, 4, 1) = Eigen::VectorXd::Constant(4, 1, std::numeric_limits<double>::quiet_NaN());
        } else {
            double theta2_1 = atan2(C2, sqrt(A2 * A2 + B2 * B2 - C2 * C2)) - atan2(B2, A2);
            double theta2_2 = atan2(C2, -sqrt(A2 * A2 + B2 * B2 - C2 * C2)) - atan2(B2, A2);
            _Results.block(4 * i, 1, 2, 1) = Eigen::VectorXd::Constant(2, 1, theta2_1);
            _Results.block(4 * i + 2, 1, 2, 1) = Eigen::VectorXd::Constant(2, 1, theta2_2);
        }
    }
    for (long i = 0; i < 8; ++i) {
        double A5 = _T06(0, 2) * sin(_Results(2 * i, 1) + _Results(2 * i, 2)) * cos(_Results(2 * i, 0));
        double B5 = _T06(1, 2) * sin(_Results(2 * i, 0)) * sin(_Results(2 * i, 1) + _Results(2 * i, 2));
        double C5 = _T06(2, 2) * cos(_Results(2 * i, 1) + _Results(2 * i, 2));
        double D5 = A5 + B5 + C5;
        if (1 - D5 * D5 < 0) {
            _Results.block(2 * i, 4, 2, 1) = Eigen::VectorXd::Constant(2, 1, std::numeric_limits<double>::quiet_NaN());
        } else {
            double theta5_1 = atan2(sqrt(1 - D5 * D5), D5);
            double theta5_2 = atan2(-sqrt(1 - D5 * D5), D5);
            _Results(2 * i, 4) = theta5_1;
            _Results(2 * i + 1, 4) = theta5_2;
        }
    }
    for (long i = 0; i < 16; ++i) {
        if (std::abs(_Results(i, 4)) < 1e-6 ) {
            _Results(i, 3) = std::numeric_limits<double>::quiet_NaN();
            _Results(i, 5) = std::numeric_limits<double>::quiet_NaN();
        } else {
            double A4 = -_T06(0, 2) * sin(_Results(i, 0)) + _T06(1, 2) * cos(_Results(i, 0));
            double B4 = _T06(0, 2) * cos(_Results(i, 0)) * cos(_Results(i, 1) + _Results(i, 2));
            double C4 = _T06(1, 2) * sin(_Results(i, 0)) * cos(_Results(i, 1) + _Results(i, 2));
            double D4 = _T06(2, 2) * sin(_Results(i, 1) + _Results(i, 2));
            _Results(i, 3) = atan2(A4 / sin(_Results(i, 4)), (B4 + C4 - D4) / sin(_Results(i, 4)));
            double A6 = _T06(0, 1) * sin(_Results(i, 1) + _Results(i, 2)) * cos(_Results(i, 0));
            double B6 = _T06(1, 1) * sin(_Results(i, 0)) * sin(_Results(i, 1) + _Results(i, 2));
            double C6 = _T06(2, 1) * cos(_Results(i, 1) + _Results(i, 2));
            double D6 = _T06(0, 0) * sin(_Results(i, 1) + _Results(i, 2)) * cos(_Results(i, 0));
            double E6 = _T06(1, 0) * sin(_Results(i, 0)) * sin(_Results(i, 1) + _Results(i, 2));
            double F6 = _T06(2, 0) * cos(_Results(i, 1) + _Results(i, 2));
            _Results(i, 5) = atan2((A6 + B6 + C6) / sin(_Results(i, 4)), -(D6 + E6 + F6) / sin(_Results(i, 4)));
        }
    }
    //set invalid theta1 to nan
    long Nr = _Results.rows();
    for (long i = 0; i < Nr; ++i) {
        double A1 = _a1 - _a2 * sin(_Results(i, 1)) + _a3 * cos(_Results(i, 1) + _Results(i, 2)) + _d4 * sin(_Results(i, 1) + _Results(i, 2));
        double theta1 = atan2(_T06_modified(1, 3) / A1, _T06_modified(0, 3) / A1);
        if (std::abs(_Results(i, 0) - theta1) > 1e-6) {
            _Results(i, 0) = std::numeric_limits<double>::quiet_NaN();
        }
    }
    //remove invalid theta1 whose value is nan
    _Results_finally.resize(16, 6);
    int j = 0;
    int count = 16;
    for (long i = 0; i < Nr; ++i) {
        if (std::isnan(_Results(i, 0))) {
            count -= 1;
        } else {
            _Results_finally.block(j, 0, 1, 6) = _Results.block(i, 0, 1, 6);
            j += 1;
        }
    }
    if (count > 0) {
        _Results_finally.conservativeResize(count, 6);
    }
    return _Results;
}

Eigen::MatrixXd IKSolver::getSolvedResults() {
    return _Results_finally;
}
