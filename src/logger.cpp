#include"logger.hpp"
    
    Logger::Logger(const std::string& filename) 
            : file(filename) 
    {
        if (!file.is_open()) {
            throw std::runtime_error("Could not open log file!");
        }
    }

    void Logger::logPosition(const std::string& label, const Position& pos, double t, std::vector<float> s) {
        file << label << ", "
             << pos.x << ", "
             << pos.y << ", "
             << pos.theta << ", "
             << t << ", "
             << s[0] << ", "
             << s[1] << ", "
             << s[2] << "\n ";

    }
