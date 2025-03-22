#ifndef DARE_H
#define DARE_H

#undef __ARM_NEON__
#undef __ARM_NEON

#include "globals.hpp"
#include <Eigen/Eigen/Dense>
#include <Eigen/Eigen/LU>
#include <Eigen/unsupported/Eigen/MatrixFunctions>


template <int Rows, int Cols,
          int Options = Eigen::AutoAlign |
                        ((Rows == 1 && Cols != 1) ? Eigen::RowMajor
                         : (Cols == 1 && Rows != 1)
                             ? Eigen::ColMajor
                             : EIGEN_DEFAULT_MATRIX_STORAGE_ORDER_OPTION),
          int MaxRows = Rows, int MaxCols = Cols>
using Matrixd = Eigen::Matrix<double, Rows, Cols, Options, MaxRows, MaxCols>;

template <int States, int Inputs>
void discretize(const Matrixd<States, States> &contA,
                const Matrixd<States, Inputs> &contB, double dt,
                Matrixd<States, States> *discA,
                Matrixd<States, Inputs> *discB) {
 
  Matrixd<States + Inputs, States + Inputs> M;
  M.template block<States, States>(0, 0) = contA;
  M.template block<States, Inputs>(0, States) = contB;
  M.template block<Inputs, States + Inputs>(States, 0).setZero();

  Matrixd<States + Inputs, States + Inputs> phi = (M * dt).exp();

  *discA = phi.template block<States, States>(0, 0);
  *discB = phi.template block<States, Inputs>(0, States);
}

template <int States, int Inputs>
void DARE(
    const Eigen::Matrix<double, States, States>& A,
    const Eigen::Matrix<double, States, Inputs>& B,
    const Eigen::Matrix<double, States, States>& Q,
    const Eigen::LLT<Eigen::Matrix<double, Inputs, Inputs>>& R_llt, Eigen::Matrix<double, States, States> *S) {
  using StateMatrix = Eigen::Matrix<double, States, States>;

  StateMatrix A_k = A;
  StateMatrix G_k = B * R_llt.solve(B.transpose());
  StateMatrix H_k;
  StateMatrix H_k1 = Q;

  do {
    H_k = H_k1;

    auto W_solver = (StateMatrix::Identity(H_k.rows(), H_k.cols()) + G_k * H_k).lu();

    StateMatrix V_1 = W_solver.solve(A_k);

    G_k += A_k * W_solver.solve(G_k.transpose()).transpose() * A_k.transpose();
    H_k1 += V_1.transpose() * H_k * A_k;
    A_k *= V_1;

  } while ((H_k1 - H_k).norm() > 1e-10 * H_k1.norm());

  *S = H_k1;
}

#endif // DARE_H