#pragma once

#include<fstream>
#include<iostream>
#include <random>
#include<memory>
#include<unordered_map>
#include <cmath>
#include <vector>
#include "definitions.hpp"
#include "logger.hpp"


const int SAMPLE_NUMBER = 50;

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
    void move(double distance, double angle, int t);

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

    std::vector<float> s_r = {q_x,q_y,q_t};

    std::default_random_engine gen;
    std::normal_distribution<double> dist_noise;
    std::normal_distribution<double> angle_noise;

    std::normal_distribution<double> range_noise;
    std::normal_distribution<double> bear_noise;
};