#include"ekf.hpp"



EKF::EKF(Eigen::Matrix2f& R_,
         Eigen::Matrix3f& Q_,
         Eigen::Matrix3f& P0_,
         Eigen::Vector3f& X0_,
         std::shared_ptr<Logger>& logger_)
    : R(R_), Q(Q_), P0(P0_), X0(X0_), logger(logger_)
{}


void EKF::predict(Eigen::Vector3f& X_prev, Eigen::Vector2f& U) {
    float& x_prev = X_prev(0);
    float& y_prev = X_prev(1);
    float& theta_prev = X_prev(2);

    float& v = U(0);
    float& w = U(1);

    F << 1, 0, -v*sin(theta_prev),
          0, 1,  v*cos(theta_prev),
          0, 0,  1;
    
    X_hat(0) = x_prev + v*cos(theta_prev);
    X_hat(1) = y_prev + v*sin(theta_prev);
    X_hat(2) = theta_prev + w;

    P_hat = F*P*F.transpose() + Q;

    std::cout << "EKF prediction: " << std::endl;
    std::cout << "x: " <<  X_hat(0) << " y: " <<  X_hat(1) << " Thet: " <<  X_hat(2) << std::endl;
    logger->logPosition("Prediction",Position(X_hat(0),X_hat(1),X_hat(2)));
}

void EKF::update(const Measurement& Z, const std::shared_ptr<Landmark>& lm) {

            Eigen::Vector2f Z_vec;
            Z_vec(0) = Z.range;
            Z_vec(1) = Z.bearing;

            float& x = X_hat(0);
            float& y = X_hat(1);
            float& theta = X_hat(2);

            float dx = lm->x - x;
            float dy = lm->y - y;

            float q = dx*dx + dy*dy;

            float range = std::sqrt(q);
            float bearing = std::atan2(dy, dx) - theta;

            // Normalize bearing to [-pi, pi]
            if (bearing > M_PI) bearing -= 2 * M_PI;
            if (bearing < -M_PI) bearing += 2 * M_PI;

            //Compute H matrix
            H <<  -dx/sqrt(q), -dy/sqrt(q), 0,
                    dy/q      , -dx/q, -1;

            S_in = H*P_hat*H.transpose() + R;
            Kt = P_hat * H.transpose() * S_in.inverse();

            Z_hat(0) = range;
            Z_hat(1) = bearing;

            X = X_hat + Kt*(Z_vec - Z_hat);
            P = P_hat - Kt*S_in*Kt.transpose();

            std::cout << "EKF update: " << std::endl;
            std::cout << "x: " <<  X(0) << " y: " <<  X(1) << " Thet: " <<  X(2) << std::endl;
            logger->logPosition("Update",Position(X(0),X(1),X(2)));

            


}