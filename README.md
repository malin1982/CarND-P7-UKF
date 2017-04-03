# Unscented Kalman Filter Project Starter Code
Self-Driving Car Engineer Nanodegree Program

---

## Dependencies

* cmake >= v3.5
* make >= v4.1
* gcc/g++ >= v5.4

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile on Mac: `cmake -G Xcode ..`
   * On windows, you may need to run: `cmake .. -G "Unix Makefiles" && make`
4. Run it: `open UnscentedKF.xcodeproj/`
5. Edit scheme within Xcode: shortcut: cmdkey+< Fill in arguments passed on Luanch.
   ../../data/sample-laser-radar-measurement-data-2.txt and ./output.txt
## Known Issue

1. It works on sample data1, hangs when using sample data2 as input. Problem seems to be the normalization `while()` loop at the prediction step.
2. Issue fixed by using small values when initializing covariance matrix P.
3. Normalizing function was updated, refer to ukf.h

## Editor Settings

We've purposefully kept editor configuration files out of this repo in order to
keep it as simple and environment agnostic as possible. However, we recommend
using the following settings:

* indent using spaces
* set tab width to 2 spaces (keeps the matrices in source code aligned)

## Code Style

Please stick to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html) as much as possible.

## Results
Data1
Accuracy - RMSE:
0.167695
0.195717
0.723887
0.788733

Data2
Accuracy - RMSE:
0.184855
0.182395
0.42336
0.386014
