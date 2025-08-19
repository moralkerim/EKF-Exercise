#include "robot.hpp"

void Robot::move(double distance, double angle) {
    // Gürültü ekle
    Position pos_sample;
    for(int i=0;i<SAMPLE_NUMBER-1;i++) {
        double noisy_distance = distance + dist_noise(gen);
        double noisy_angle = angle + angle_noise(gen);

        pos_sample.theta = pos.theta + noisy_angle;
        pos_sample.x = pos.x + noisy_distance * cos(pos_sample.theta);
        pos_sample.y = pos.y + noisy_distance * sin(pos_sample.theta);
        logger->logPosition("Gaussian",pos_sample);
    }

        pos.theta = pos_sample.theta;
        pos.x = pos_sample.x;
        pos.y = pos_sample.y;
        logger->logPosition("Actual",pos);
}

void Robot::print() const {
        std::cout << "Robot position: x=" << pos.x << ", y=" << pos.y << ", theta=" << pos.theta << std::endl;
    }


std::vector<Measurement> Robot::senseLandmarks(const std::unordered_map<int, std::shared_ptr<Landmark>> & landmarks,
                        double range_limit,
                        double bearing_limit_deg)
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