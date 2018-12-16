[//]: # (Image References)
[video1]: ./videos/P_1.avi

# A Vehicle driving using PID controller
Udacity's Self-Driving Car Engineer Nanodegree Program, a project for second term  

---

## Project Introduction

This project implements a PID (Proportional, Integral and Derivative) controller in C++ to maneuver a vehicle in a simulated environment on a track. The project involves two steps:

1. Manual step to initialize PID parameters

2. Parameter optimization using _Gradient Decent (Twiddle)_ algorithm

## PID Basics

Using PID controller, for the input signal the controller tries to estimate the output by minimizing the error compared to the desired value by tweaking the three control terms.

* Proportional term: This term influences the output by generating a value in proportion to the error, i.e. if the error is small it outputs a smaller value, however if the error is large then it tries generate larger value to compensate for the larger error.

* Integral term: This term tries to minimize the error by accounting for past errors after the proportional term is applied. As the error in the system is reduced this term tends to reduce as well.

* Differential term: This term provides an estimate to reduce the error in the system by anticipating the future error based on the rate of error change. It exerts a higher value to growing error change vs a smaller value to the reducing error.

**For more technical details visit** [Wikipedia](https://en.wikipedia.org/wiki/PID_controller).

## Project execution

As a first step, I played with each of the PID terms and observed it's impact on the vehicle movement when used indvidually or in combination of them. Here are few trials that I did and capture videos in some cases:

* P modified, I & D set to zero: Executed as `./pid 1.0 0.0 0.0 0.02`

  While the vehicle progresses straight forward and as the road curvature comes into play, the controller is trying to generate steering angle to bring the vehicle back to center position. During this process, the error keeps accumulating and the controller trying to catch up with the error resulting into the vehicle swaying a lot as shown ![here](./videos/P_1.avi).

* I modified, P & D set to zero: Executed as `./pid 0.0 1.0 0.0 0.02`

  In order to handle the systemic bias, when integral controller is applied alone I observed that the steering engaged often and abruptly resulting in an unstable ride as it tries to account various errors from previous runs. ![Here](./videos/I_1.avi) is a video to demonstrate that.

* D modified, P & I set to zero: Executed as `./pid 0.0 0.0. 1.0 0.02`

  To compensate for the error in future steps the D parameter is used.  As we are not applying any P value to correct the course the input D values makes no difference in vehicle maneuver. Following ![video](./videos/D_1.avi) shows the behavior.

* P & D modified, I set to zero: Executed as `./pid 0.5 0.0 2.5 0.02`

  By setting P and D value the vehicle moves within road lanes with few oscillations eventually going it out of control. The ![video](./videos/P_0.5_D_2.5.avi) shows the vehicle going farthest though it can be much more smooth which indicates the parameters needs further tuning.

* Finally, P, I and D all are modified: Executed as `./pid 028 0.00015 2.12 0.02`

  With trial and error in adjusting parameters, the vehicle ran successfully within a track as shown in the following video.

Though it was fun to play and observe the vehicle maneuvering erratically, it was much more fun to implement the gradient descent algorithm covered in the twiddle lessons and optimize these parameters.



## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1(mac, linux), 3.81(Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `./install-mac.sh` or `./install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.
* Simulator. You can download these from the [project intro page](https://github.com/udacity/self-driving-car-sim/releases) in the classroom.

There's an experimental patch for windows in this [PR](https://github.com/udacity/CarND-PID-Control-Project/pull/3)

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./pid`. 

Tips for setting up your environment can be found [here](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/23d376c7-0195-4276-bdf0-e02f1f3c665d)

## Editor Settings

We've purposefully kept editor configuration files out of this repo in order to
keep it as simple and environment agnostic as possible. However, we recommend
using the following settings:

* indent using spaces
* set tab width to 2 spaces (keeps the matrices in source code aligned)

## Code Style

Please (do your best to) stick to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).

## Project Instructions and Rubric

Note: regardless of the changes you make, your project must be buildable using
cmake and make!

More information is only accessible by people who are already enrolled in Term 2
of CarND. If you are enrolled, see [the project page](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/f1820894-8322-4bb3-81aa-b26b3c6dcbaf/lessons/e8235395-22dd-4b87-88e0-d108c5e5bbf4/concepts/6a4d8d42-6a04-4aa6-b284-1697c0fd6562)
for instructions and the project rubric.

## Hints!

* You don't have to follow this directory structure, but if you do, your work
  will span all of the .cpp files here. Keep an eye out for TODOs.

## Call for IDE Profiles Pull Requests

Help your fellow students!

We decided to create Makefiles with cmake to keep this project as platform
agnostic as possible. Similarly, we omitted IDE profiles in order to we ensure
that students don't feel pressured to use one IDE or another.

However! I'd love to help people get up and running with their IDEs of choice.
If you've created a profile for an IDE that you think other students would
appreciate, we'd love to have you add the requisite profile files and
instructions to ide_profiles/. For example if you wanted to add a VS Code
profile, you'd add:

* /ide_profiles/vscode/.vscode
* /ide_profiles/vscode/README.md

The README should explain what the profile does, how to take advantage of it,
and how to install it.

Frankly, I've never been involved in a project with multiple IDE profiles
before. I believe the best way to handle this would be to keep them out of the
repo root to avoid clutter. My expectation is that most profiles will include
instructions to copy files to a new location to get picked up by the IDE, but
that's just a guess.

One last note here: regardless of the IDE used, every submitted project must
still be compilable with cmake and make./

## How to write a README
A well written README file can enhance your project and portfolio.  Develop your abilities to create professional README files by completing [this free course](https://www.udacity.com/course/writing-readmes--ud777).

