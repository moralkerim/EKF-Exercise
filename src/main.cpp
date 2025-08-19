#include <iostream>
#include <vector>
#include <cmath>
#include <memory>
#include "robot.hpp"
#include "ekf.hpp"


//EKF Noise parameters
float sigma_d = 0.1;
float sigma_a = 0.01;

float q_x = 0.5;
float q_y = 0.5;
float q_t = 0.034;


Eigen::Matrix2f R({{sigma_d, 0.0f}, {0.0f, sigma_a}}); //Measurement Noise
Eigen::Matrix3f Q({{q_x, 0.0f, 0.0f}, {0.0f, q_y, 0.0f}, {0.0f, 0.0f, q_t}}); //Process Noise

Eigen::Matrix3f P0({{0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f}}); //Initial Cov. Matrix
Eigen::Vector3f X0({0.0f, 0.0f, 0.0f}); //Initial pose

Eigen::Vector3f X_prev({0.0f, 0.0f, 0.0f}); //Prev update

//Create a shared logger file for both Robot and EKF
auto logger = std::make_shared<Logger>("poses.txt");

//Create EKF object
EKF ekf(R, Q, P0, X0,logger);


//Data association function, which measurement belong to which landmark?
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


    Robot robot(0.0, 0.0, 0.0,logger); //Robot object


    // Landmarks as unordered maps
    std::unordered_map<int, std::shared_ptr<Landmark>> landmarks;
    landmarks.emplace(1, std::make_shared<Landmark>(1,  5.0f,  5.0f));
    landmarks.emplace(2, std::make_shared<Landmark>(2,  6.0f,  8.0f));
    landmarks.emplace(3, std::make_shared<Landmark>(3,  7.0f,  12.0f));


    double total_time = 5.0;
    double dt = 1.0;
    double v = 3.0;    // Linear speed
    double w = 0.3;    // Angular speed

    std::cout << "Starting Position:\n";
    robot.print();

    //Input vector
    Eigen::Vector2f U({v,w});

    //Run for total time with dt steps
    for (double t = 0; t < total_time; t += dt) {
        robot.move(v, w); //Move and take samples
        std::cout << "\nt=" << t + dt << std::endl;
        robot.print();
        ekf.predict(X_prev, U); //EKF Prediction step.
        auto measurements = robot.senseLandmarks(landmarks); //Take measurements
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

    //Put Landmarks to the log
    for(const auto& [id,lm] : landmarks) {
        logger->logPosition("Landmark",Position(lm->x,lm->y,0));
    }

    return 0;
}
