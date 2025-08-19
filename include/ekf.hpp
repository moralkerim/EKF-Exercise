#pragma once

#include<iostream>
#include<memory>
#include <Eigen/Dense>
#include <cmath>
#include <fstream>
#include "robot.hpp"



class EKF {
    public:
        EKF(Eigen::Matrix2f& R_,
         Eigen::Matrix3f& Q_,
         Eigen::Matrix3f& P0_,
         Eigen::Vector3f& X0_,
         std::shared_ptr<Logger>& logger_);

        void predict(Eigen::Vector3f& X_prev, Eigen::Vector2f& U);

        void update(const Measurement& Z, const std::shared_ptr<Landmark>& lm);

    private:
        Eigen::Matrix3f F_;
        Eigen::Matrix<float, 2, 3> H;
        Eigen::Matrix3f Q;
        Eigen::Matrix2f R;
        Eigen::Matrix3f P0;
        Eigen::Matrix<float, 3, 2> Kt;
        Eigen::Matrix2f S_in;
        Eigen::Vector3f X0;

        const float RANGE_LIMIT = 10.0;
        const float BEAR_LIM_DEG = 90.0;
        const float BEAR_LIM_RAD = BEAR_LIM_DEG * M_PI / 180.0;
        std::shared_ptr<Logger> logger;

    public:
        Eigen::Matrix3f P, P_hat;
        Eigen::Vector3f X, X_hat;
        Eigen::Vector2f Z_hat;


};