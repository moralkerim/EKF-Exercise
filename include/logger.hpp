#pragma once

#include<iostream>
#include<fstream>
#include<vector>
#include"definitions.hpp"

class Logger {
public:
    Logger(const std::string& filename);

    void logPosition(const std::string& label, const Position& pos, double t, std::vector<float> s);

private:
    std::ofstream file;

};