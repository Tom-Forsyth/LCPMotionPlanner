#pragma once

#include <Eigen/Dense>

namespace LCPSolve
{
    /// @brief Linear complementarity problem datatype.
    struct LCP {
        int exitCond;
        Eigen::VectorXd w{};
        Eigen::VectorXd z{};
        Eigen::MatrixXd M{};
        Eigen::VectorXd q{};
    };

    /// @brief Linear complementarity problem solver.
    /// @param M Matrix.
    /// @param q Vector.
    /// @return Solved LCP (z and w vectors).
    LCP LCPSolve(Eigen::MatrixXd M, Eigen::VectorXd q);
}
