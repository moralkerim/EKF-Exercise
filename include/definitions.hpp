#pragma once

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