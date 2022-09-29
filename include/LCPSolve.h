#ifndef LCPSOLVE_H_
#define LCPSOLVE_H_

#include <Eigen/Dense>

namespace MotionPlanner
{
    // Linear complementarity problem datatype.
    struct LCP {
        int exitCond;
        Eigen::VectorXd w{};
        Eigen::VectorXd z{};
        Eigen::MatrixXd M{};
        Eigen::VectorXd q{};
    };

    // LCP Solver.
    LCP LCPSolve(Eigen::MatrixXd M, Eigen::VectorXd q);
}

#endif