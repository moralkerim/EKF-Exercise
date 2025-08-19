#include <iostream>
#include <vector>
#include <cmath>
#include <random>
#include <memory>
#include "ekf.hpp"

float sigma_d = 0.1;
float sigma_a = 0.001;

float q_x = 0.5;
float q_y = 0.5;
float q_t = 0.017;


Eigen::Matrix2f R({{sigma_d, 0.0f}, {0.0f, sigma_a}}); //Measurement Noise
Eigen::Matrix3f Q({{q_x, 0.0f, 0.0f}, {0.0f, q_y, 0.0f}, {0.0f, 0.0f, q_t}}); //Process Noise

Eigen::Matrix3f P0({{0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f}});
Eigen::Vector3f X0({0.0f, 0.0f, 0.0f});

Eigen::Vector3f X_prev({0.0f, 0.0f, 0.0f});


EKF ekf(R, Q, P0, X0);

struct Position {
    double x;
    double y;
    double theta;

    Position(double x_=0, double y_=0, double theta_=0) : x(x_), y(y_), theta(theta_) {}
};


class Robot {
public:
    Position pos;

    std::default_random_engine gen;
    std::normal_distribution<double> dist_noise;
    std::normal_distribution<double> angle_noise;

    std::normal_distribution<double> range_noise;
    std::normal_distribution<double> bear_noise;

    Robot(double x=0, double y=0, double theta=0)
    : pos(x, y, theta),
      dist_noise(0.0,  q_x),     // 0.1m std dev
      angle_noise(0.0, q_t),   // 1 degree ≈ 0.017 rad
      range_noise(0.0, sigma_d),     // 0.1m std dev
      bear_noise(0.0, sigma_a)   // 1 degree ≈ 0.017 rad
    {}

    void move(double distance, double angle) {
        // Gürültü ekle
        double noisy_distance = distance + dist_noise(gen);
        double noisy_angle = angle + angle_noise(gen);

        pos.theta += noisy_angle;
        pos.x += noisy_distance * cos(pos.theta);
        pos.y += noisy_distance * sin(pos.theta);
    }


    void print() const {
        std::cout << "Robot position: x=" << pos.x << ", y=" << pos.y << ", theta=" << pos.theta << std::endl;
    }

    void senseLandmarks(const std::vector<Landmark>& landmarks,
                        double range_limit = 10.0,
                        double bearing_limit_deg = 90.0)
    {
        double bearing_limit_rad = bearing_limit_deg * M_PI / 180.0;

        std::cout << "  Sensor readings:" << std::endl;
        for (const auto& lm : landmarks) {
            double dx = lm.x - pos.x;
            double dy = lm.y - pos.y;

            double range = std::sqrt(dx * dx + dy * dy) + range_noise(gen);
            double bearing = std::atan2(dy, dx) - pos.theta + bear_noise(gen);

            // Normalize bearing to [-pi, pi]
            if (bearing > M_PI) bearing -= 2 * M_PI;
            if (bearing < -M_PI) bearing += 2 * M_PI;

            // Sadece görüş alanındaki landmark'ları göster
            if (range <= range_limit && std::abs(bearing) <= bearing_limit_rad) {
                std::cout << "    Landmark " << lm.id << ": range=" << range << ", bearing=" << bearing << std::endl;
                Measurement z(range,bearing);
                ekf.update(z, lm);
                X_prev = ekf.X;

            }
        }
    }

};

int main() {
    
    auto robot_ptr = std::make_unique<Robot>(0.0, 0.0, 0.0);

    std::vector<Landmark> landmarks = {
        {1, 3.0, 5.0},
        {2, -1.0, 7.0},
        {3, 8.0, -4.0}
    };

    double total_time = 5.0;
    double dt = 1.0;
    double v = 1.0;    // Linear speed
    double w = 0.1;    // Angular speed

    std::cout << "Başlangıç konumu:\n";
    robot_ptr->print();

    Eigen::Vector2f U({v,w});


    for (double t = 0; t < total_time; t += dt) {
        robot_ptr->move(v, w);
        std::cout << "\nt=" << t + dt << std::endl;
        robot_ptr->print();
        ekf.predict(X_prev, U);
        robot_ptr->senseLandmarks(landmarks);
    }

    std::cout << "\nLandmark konumları:\n";
    for (const auto& lm : landmarks) {
        std::cout << "  Landmark " << lm.id << ": x=" << lm.x << ", y=" << lm.y << std::endl;
    }

    return 0;
}
