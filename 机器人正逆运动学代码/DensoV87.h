#ifndef IKSOLVER_H
#define IKSOLVER_H

#include <Eigen/Dense>

class IKSolver {
public:
    IKSolver();
    IKSolver(double a1, double a2, double a3, double d1, double d4, double d6);
    ~IKSolver();

public:
    void setParameters(double a1, double a2, double a3, double d1, double d4, double d6);
    Eigen::Isometry3d getEndPose(const Eigen::VectorXd &jointAngles);
    void setEndPose(const Eigen::Matrix4d &endPose);
    void setEndPose(const Eigen::Isometry3d &endPose);
    /**
    * @brief     This R, P and Y of setEndPose are rotation angles that rotate about its current Frame.
    * @author    Liu Qiang
    * @date      2019-08-23
    */
    void setEndPose(double x, double y, double z, double R, double P, double Y);
    Eigen::MatrixXd solve();
    Eigen::MatrixXd getSolvedResults();
public:
    Eigen::MatrixXd _Results_finally;

private:
    double _a1, _a2, _a3, _d1, _d4, _d6;
    Eigen::MatrixXd _Results;
    Eigen::Matrix4d _T06;
};

#endif // IKSOLVER_H
