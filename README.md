
# Kidnapped vehicle project

## Overview
This project uses a particle filter to localize a kidnapped car within a 2D sparse map containing landmarks. The [simulator](https://github.com/udacity/self-driving-car-sim/releases) provided by Udacity is used to generated noisy observation data of the landmark that is fed to particle filter every time step using [uWebSockets](https://github.com/uNetworking/uWebSockets).

 ---
## Dependencies
* cmake >= 3.5
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1 (Linux, Mac), 3.81 (Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools](https://developer.apple.com/xcode/features/)
* [Udacity Simulator](https://github.com/udacity/self-driving-car-sim/releases)

--- 
## Running the Code

This repository includes two files that can be used to set up and install uWebSocketIO for either Linux or Mac systems.
```
install--mac.sh
install-ubuntu.sh
```
Once the install for uWebSocketIO is complete, the main program can be built and run by running scripts below from top directory
```
> ./clean.sh
> ./build.sh
> ./run.sh
```
Below message should appear if succefully built.
```
Listening to port 4567
```

After this, the Udacity simulator can be openened and starting the kidnapped vehicle project should show the ground truth and car's position estimated by partcile filter. The simulator also shows the error in position estimation.


