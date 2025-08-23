# EKF Exercise

This project demonstrates an implementation of the Extended Kalman Filter (EKF) using modern C++.

## Requirements

- C++17 or later
- CMake 3.10+
- Eigen3

## Build Instructions

```bash
git clone https://github.com/moralkerim/EKF-Exercise.git
cd EKF-Exercise
mkdir build && cd build
cmake ..
make
```

## Usage
Run the code:
```bash
cd build
./ekf_main
```

Move log file next to plotter:
```bash
cp poses.txt ~/EKF-Exercise/plotter
python3 plotter_anim.py
```
You should see robot pose with uncertainty, EKF prediction and EKF update poses. Note that when robot sees a landmark, uncertainty ellipsoids of prediction and update gets smaller.

![Example usage](example.png)

## Playing with the Code
You can tweak robot and EKF parameters to see their effect. Noise parameters available in both EKF and Robot classes:
```bash
float r_d = 0.01;
float r_a = 0.001;

float q_x = 0.5;
float q_y = 0.5;
float q_t = 0.034;
```

You can change robot input to change its position

```bash
double dt = 1.0;    //Time step
double v = 2.0;    // Linear speed
double r = 8.0;    // Robot turn radius
double w = v/r;    // Angular speed
double total_time = 2*M_PI/w;
```

You can change or add new landmarks

```bash
// Landmarks as unordered maps
std::unordered_map<int, std::shared_ptr<Landmark>> landmarks;
landmarks.emplace(0, std::make_shared<Landmark>(0,   5.0f,  5.0f));
landmarks.emplace(1, std::make_shared<Landmark>(1,   6.0f,  8.0f));
landmarks.emplace(2, std::make_shared<Landmark>(2,   7.0f,  12.0f));
landmarks.emplace(3, std::make_shared<Landmark>(3,  -2.0f,  12.0f));
```


## Project Structure

- `src/` - Source code
- `include/` - Header files
- `plotter/` - Python files for animation

## License

This project is licensed under the MIT License.