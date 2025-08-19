#pragma once

#include<fstream>
#include<iostream>
#include <random>
#include<memory>
#include<unordered_map>
#include <cmath>
#include <vector>


const int SAMPLE_NUMBER = 50;

struct Landmark {
    int id;
    float x;
    float y;

    Landmark(int id_, float x_, float y_) : id(id_), x(x_), y(y_) {}
};

struct Measurement {
    int id;
    float range;
    float bearing;

    Measurement(int id_, float range_, float bearing_) : id(id_), range(range_), bearing(bearing_) {}
};

struct Position {
    double x;
    double y;
    double theta;

    Position(double x_=0, double y_=0, double theta_=0) : x(x_), y(y_), theta(theta_) {}
};


class Logger {
public:
    Logger(const std::string& filename) 
        : file(filename) 
    {
        if (!file.is_open()) {
            throw std::runtime_error("Could not open log file!");
        }
    }

    void logPosition(const std::string& label, const Position& pos) {
        file << label << ", "
             << pos.x << ", "
             << pos.y << ", "
             << pos.theta << "\n";
    }

private:
    std::ofstream file;

};

class Robot {
public:
    Position pos;



    Robot(double x=0, double y=0, double theta=0, std::shared_ptr<Logger> logger_ = nullptr)
    : pos(x, y, theta),
      dist_noise(0.0,  q_x),     // 0.1m std dev
      angle_noise(0.0, q_t),   // 1 degree ≈ 0.017 rad
      range_noise(0.0, r_d),     // 0.1m std dev
      bear_noise(0.0, r_a),   // 1 degree ≈ 0.017 rad
      logger(logger_)
    {}
    void move(double distance, double angle);

    void print() const;



    std::vector<Measurement> senseLandmarks(const std::unordered_map<int, std::shared_ptr<Landmark>> & landmarks,
                            double range_limit = 10.0,
                            double bearing_limit_deg = 90.0);

private:
    std::shared_ptr<Logger> logger;
    float r_d = 0.01;
    float r_a = 0.001;

    float q_x = 0.5;
    float q_y = 0.5;
    float q_t = 0.034;

    std::default_random_engine gen;
    std::normal_distribution<double> dist_noise;
    std::normal_distribution<double> angle_noise;

    std::normal_distribution<double> range_noise;
    std::normal_distribution<double> bear_noise;
};