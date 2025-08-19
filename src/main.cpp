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

//Create a shared logger file for both Robot and EKF
auto logger = std::make_shared<Logger>("poses.txt");

EKF ekf(R, Q, P0, X0,logger);

class Robot {
public:
    Position pos;

    std::default_random_engine gen;
    std::normal_distribution<double> dist_noise;
    std::normal_distribution<double> angle_noise;

    std::normal_distribution<double> range_noise;
    std::normal_distribution<double> bear_noise;

    Robot(double x=0, double y=0, double theta=0, std::shared_ptr<Logger> logger_ = nullptr)
    : pos(x, y, theta),
      dist_noise(0.0,  q_x),     // 0.1m std dev
      angle_noise(0.0, q_t),   // 1 degree ≈ 0.017 rad
      range_noise(0.0, sigma_d),     // 0.1m std dev
      bear_noise(0.0, sigma_a),   // 1 degree ≈ 0.017 rad
      logger(logger_)
    {}

    void move(double distance, double angle) {
        // Gürültü ekle
        double noisy_distance = distance + dist_noise(gen);
        double noisy_angle = angle + angle_noise(gen);

        pos.theta += noisy_angle;
        pos.x += noisy_distance * cos(pos.theta);
        pos.y += noisy_distance * sin(pos.theta);
        logger->logPosition("Gaussian",pos);
    }



    void print() const {
        std::cout << "Robot position: x=" << pos.x << ", y=" << pos.y << ", theta=" << pos.theta << std::endl;
    }

    std::vector<Measurement> senseLandmarks(const std::unordered_map<int, std::shared_ptr<Landmark>> & landmarks,
                            double range_limit = 10.0,
                            double bearing_limit_deg = 90.0)
    {
        double bearing_limit_rad = bearing_limit_deg * M_PI / 180.0;
        std::vector<Measurement> measurements;

        std::cout << "  Sensor readings:" << std::endl;
        for (const auto& [id, lm] : landmarks) {  
            double dx = lm->x - pos.x;
            double dy = lm->y - pos.y;

            double range = std::sqrt(dx * dx + dy * dy) + range_noise(gen);
            double bearing = std::atan2(dy, dx) - pos.theta + bear_noise(gen);


            // Normalize bearing to [-pi, pi]
            if (bearing > M_PI) bearing -= 2 * M_PI;
            if (bearing < -M_PI) bearing += 2 * M_PI;

            //Check if we have a measurement
            if (range <= range_limit && std::abs(bearing) <= bearing_limit_rad) {
                std::cout << "    Landmark " << id << ": range=" << range 
                        << ", bearing=" << bearing << std::endl;
                Measurement z(id, range, bearing); 
                measurements.emplace_back(z);
            }
        }
        return measurements;
    }
    
private:
    std::shared_ptr<Logger> logger;

};

std::shared_ptr<Landmark> FindAssociation(
    const Measurement& z,
    const std::unordered_map<int, std::shared_ptr<Landmark>>& landmarks)
{
    auto it = landmarks.find(z.id);
    if (it != landmarks.end()) {
        auto lm = it->second;
        std::cout << "Measurement " << z.id
                  << " --> Landmark at (" << lm->x << ", " << lm->y << ")\n";
        return lm; 
    } else {
        std::cout << "Landmark not found for id " << z.id << "\n";
        return nullptr;
    }
}


int main() {


    Robot robot(0.0, 0.0, 0.0,logger);


    // Landmarks as unordered maps
    std::unordered_map<int, std::shared_ptr<Landmark>> landmarks;
    landmarks.emplace(1, std::make_shared<Landmark>(1,  5.0f,  5.0f));
    landmarks.emplace(2, std::make_shared<Landmark>(2, -2.0f,  7.0f));
    landmarks.emplace(3, std::make_shared<Landmark>(3,  8.0f, -4.0f));


    double total_time = 5.0;
    double dt = 1.0;
    double v = 1.0;    // Linear speed
    double w = 0.1;    // Angular speed

    std::cout << "Starting Position:\n";
    robot.print();

    Eigen::Vector2f U({v,w});


    for (double t = 0; t < total_time; t += dt) {
        robot.move(v, w);
        std::cout << "\nt=" << t + dt << std::endl;
        robot.print();
        ekf.predict(X_prev, U);
        auto measurements = robot.senseLandmarks(landmarks);
        //Check if we have a measurement
        if(!measurements.empty()) {
            //Update EKF for all measurements
            for(auto& z : measurements) {
                auto lm = FindAssociation(z,landmarks);
                if(lm) {
                    ekf.update(z,lm);
                    X_prev = ekf.X;
                }
            }

        }
        
    }

    return 0;
}
