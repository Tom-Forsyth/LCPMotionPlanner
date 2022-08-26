#include <cmath>
#include <Eigen/Dense>
#include "LCPSolve.h"

using namespace Eigen;
using namespace std;

// Forward declarations.
bool checkCompatability(const MatrixXd &M, const int &dim);
bool checkTrivialSol(const VectorXd &q);
MatrixXd constructTableau(const MatrixXd &M, const VectorXd &q);
void pivot(MatrixXd &tableau, const int &row, const int &col);
int initializeTableau(MatrixXd &tableau);
VectorXd basicVars(const MatrixXd &tableau);
bool checkSol(const MatrixXd &tableau);
bool checkRayTermination(const MatrixXd &tableu, const int &pivotCol);
int minRatioTest(const MatrixXd &tableau, const int &pivotCol);
MatrixXd extractSolution(const MatrixXd &tableau);

// Solve LCP by Lemke's Method.
LCP LCPSolve(MatrixXd M, VectorXd q) {
    const int dim = q.size();

    // Initialize LCP data structure.
    LCP solution {};
    solution.M = M;
    solution.q = q;

    // Check that inputs are compatable.
    bool compatable = checkCompatability(M, dim);
    if (!compatable) {
        solution.exitCond = 2;
        return solution;
    }

    // Check for trivial solution.
    bool trivial = checkTrivialSol(q);
    if (trivial) {
        solution.z = VectorXd::Zero(dim);
        solution.w = q;
        solution.exitCond = 0;
        return solution;
    }

    // Construct & initialize tableau.
    MatrixXd tableau = constructTableau(M, q);
    VectorXd index = VectorXd::LinSpaced(dim, 0, dim-1);

    int pivotCol = initializeTableau(tableau);
    int pivotRow = pivotCol - dim;
    index(pivotRow) = 2*dim;

    // Now, the tableau is initialized with a feasible basis. Pivot until there is a feasible basis w/o z0.
    const int maxIter = pow(2, dim);
    int iter {0};
    bool solFound = false;
    while ((!solFound) && (iter < maxIter)) {
        // Check for ray termination.
        bool rayTermination = checkRayTermination(tableau, pivotCol);
        if (rayTermination) {
            solFound = true;
        } else {
            // Minimum ratio test to determine the pivot row (blocked/dropped variable).
            pivotRow = minRatioTest(tableau, pivotCol);
            pivot(tableau, pivotRow, pivotCol);
            int drop = index(pivotRow);
            index(pivotRow) = pivotCol;
            
            // Find next entering variable (pivotCol). Dropped variable is pivotRow.
            if (drop > dim-1) {
                pivotCol = drop - dim;
            } else {
                pivotCol = drop + dim;
            }

            // Check for solution.
            solFound = checkSol(tableau);
            iter++;
        }
    }

    // Return solution.
    if (solFound) {
        MatrixXd sols = extractSolution(tableau);
        solution.z = sols.col(0);
        solution.w = sols.col(1);
        solution.exitCond = 0;
    } else {
        MatrixXd sols = extractSolution(tableau);
        solution.z = sols.col(0);
        solution.w = sols.col(1);
        solution.exitCond = 3;
    }
    
    return solution;
}


// Check that the inputs M and q are compatable.
bool checkCompatability(const MatrixXd &M, const int &dim) {
    const int rows = M.rows();
    const int cols = M.cols();

    if (rows != cols)
        return false;
    else if (rows != dim)
        return false;
    else
        return true;
}

// Check for trivial solution where q is positive.
bool checkTrivialSol(const VectorXd &q) {
    const int dim = q.size();
    for (int i = 0; i < dim; i++)
        if (q[i] <= 0)
            return false;
    return true;
}

// Construct Lemke tableau with auxilillary variable.
MatrixXd constructTableau(const MatrixXd &M, const VectorXd &q) {
    const int dim = q.size();
    MatrixXd tableau = MatrixXd::Zero(dim, 2*dim + 2); // Initialize with zeros.
    VectorXd auxVar = -VectorXd::Ones(dim); // Auxillary variable z_0 = {-1, -1, ... -1}.
    tableau.topLeftCorner(dim, dim) = MatrixXd::Identity(dim, dim); // Enter identity matrix on left side (w_1, w_2, etc).
    tableau.middleCols(dim, dim) = -M; // Enter I-M (z_1, z_2, etc).
    tableau.col(2*dim) = auxVar; // Enter auxiliary variable.
    tableau.col(2*dim + 1) = q; // Enter q.
    return tableau;
}

// Pivot function.
void pivot(MatrixXd &tableau, const int &row, const int &col) {
    const int dim = tableau.rows();
    double pivotElement = tableau(row, col);
    VectorXd newPivotRow = (1/pivotElement)*tableau.row(row);
    tableau.row(row) = newPivotRow;
    
    VectorXd newNonPivotRow {};
    for (int i = 0; i < dim; i++) {
        if (i != row) {
            newNonPivotRow = tableau.row(i) - tableau(i, col)*tableau.row(row);
            tableau.row(i) = newNonPivotRow;
        }
    }
}

// Initialize tableau.
int initializeTableau(MatrixXd &tableau) {
    const int dim = tableau.rows();
    const int cols = 2*dim +2;

    // Pivot row of min element of q w.r.t. aux column.
    int minRow {0}; 
    double minVal = tableau.coeff(minRow, cols-1);
    for (int i = 1; i < dim; i++) {
        if (tableau.coeff(i, cols-1) < minVal) {
            minRow = i;
            minVal = tableau(minRow, cols-1);
        }
    }

    pivot(tableau, minRow, cols-2);

    // Return the next entering variable, which is the complement to the non-basic variable.
    int newPivotCol = minRow + dim;
    return newPivotCol;
}

// Return which variables are basic.
VectorXd basicVars(const MatrixXd &tableau) {
    int cols = tableau.cols();
    double varNorm {};
    VectorXd isBasic = VectorXd::Zero(cols - 1);

    double err {1e-8};
    for (int i = 0; i < cols - 1; i++) {
        varNorm = tableau.col(i).norm();
        if (abs(1 - varNorm) < err) {
            isBasic[i] = true;
        } else {
            isBasic[i] = false;
        }
    }

    return isBasic;
}

// Check for solution.
bool checkSol(const MatrixXd &tableau) {

    // Determine basic variables.
    VectorXd isBasic = basicVars(tableau);

    // Size information and initialize q & z0.
    bool solFound {false};
    int dim = tableau.rows();
    int cols = 2*dim + 2;
    VectorXd q = tableau.col(cols-1);
    VectorXd z0 = tableau.col(cols-2);
    
    // No solution if any element of q is negative.
    for (int i = 0; i <= dim-1; i++) {
        if (q[i] < 0) {
            return solFound;
        }
    }
    
    // No solution if auxillary variable z0 is basic.
    if (isBasic[cols-2] == 1) {
        return solFound;
    }
    
    // If q is positive and z0 is non-basic, solution has been found.
    solFound = true;
    return solFound;
}

// Check for secondary ray termination.
bool checkRayTermination(const MatrixXd &tableu, const int &pivotCol) {
    VectorXd column = tableu.col(pivotCol);
    int negTestSum {0};
    for (int i {0}; i < column.size(); i++) {
        if (column(i) <= 0) {
            negTestSum++;
        }
    }

    if (negTestSum == column.size()) {
        return true;
    } else {
        return false;
    }
}

// Minimum ratio test to determine pivot row.
int minRatioTest(const MatrixXd &tableau, const int &pivotCol) {
    const int dim = tableau.rows();

    VectorXd ratioTest = VectorXd::Zero(dim);
    for (int i = 0; i < dim; i++) {
        ratioTest[i] = tableau.coeff(i, 2*dim + 1) / tableau.coeff(i, pivotCol);
        if (ratioTest[i] < 0) {
            ratioTest[i] = 1e10;
        }
    }
    
    int pivotRow {0};
    double minRatio = ratioTest[0];
    for (int i = 1; i < dim; i++) {
        if (ratioTest[i] < minRatio) {
            minRatio = ratioTest[i];
            pivotRow = i;
        }
    }

    return pivotRow;
}

// Extract solution from tableau.
MatrixXd extractSolution(const MatrixXd &tableau) {
    const int dim = tableau.rows();
    MatrixXd A = MatrixXd::Zero(dim, 2*dim);
    VectorXd z = VectorXd::Zero(dim);
    VectorXd w = VectorXd::Zero(dim);
    VectorXd isBasic = basicVars(tableau);
    
    for (int i = 0; i < 2*dim; i++)
        if (isBasic[i] == 1)
            A.col(i) = tableau.col(i);

    for (int i = 0; i < 2*dim; i++) {
        if (i < dim) {
            w[i] = A.col(i).dot(tableau.col(2*dim + 1));
        } else {
            z[i-dim] = A.col(i).dot(tableau.col(2*dim + 1));
        }
    }

    MatrixXd sols = MatrixXd::Zero(dim, 2);
    sols.col(0) = z;
    sols.col(1) = w;
    return sols;
}
