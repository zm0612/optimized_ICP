//
// Created by Zhang Zhimeng on 22-8-26.
//

#ifndef OPTIMIZED_ICP_COMMON_H
#define OPTIMIZED_ICP_COMMON_H

#include <Eigen/Dense>

template<typename Derived>
Eigen::Matrix<typename Derived::Scalar, 3, 3> Hat(const Eigen::MatrixBase<Derived> &v) {
    Eigen::Matrix<typename Derived::Scalar, 3, 3> skew_mat;
    skew_mat.setZero();
    skew_mat(0, 1) = -v(2);
    skew_mat(0, 2) = v(1);
    skew_mat(1, 2) = -v(0);
    skew_mat(1, 0) = v(2);
    skew_mat(2, 0) = -v(1);
    skew_mat(2, 1) = v(0);
    return skew_mat;
}

template<typename Derived>
static Eigen::Matrix<typename Derived::Scalar, 3, 3> SO3Exp(const Eigen::MatrixBase<Derived> &v) {
    Eigen::Matrix<typename Derived::Scalar, 3, 3> R;
    typename Derived::Scalar theta = v.norm();
    Eigen::Matrix<typename Derived::Scalar, 3, 1> v_normalized = v.normalized();
    R = std::cos(theta) * Eigen::Matrix<typename Derived::Scalar, 3, 3>::Identity()
        + (typename Derived::Scalar(1.0) - std::cos(theta)) * v_normalized *
          v_normalized.transpose() + std::sin(theta) * Hat(v_normalized);

    return R;
}

#endif //OPTIMIZED_ICP_COMMON_H
