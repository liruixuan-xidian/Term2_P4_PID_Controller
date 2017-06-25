# CarND-Controls-PID
Self-Driving Car Engineer Nanodegree Program

---

## Implementation

The PID controller has three major parameters to tune, namely proportional gain (K<sub>p</sub>), integral gain (K<sub>i</sub>) and derivative gain (K<sub></sub>). To tune them for the simulator, I first tried to manually tune the parameters K<sub>p</sub> and K<sub>d</sub>. I increased the proportional gain to a certain extent where I could see the car do almost a constant amplitude oscillations. After achieving this, I tried to increase the derivative gain to reduce the oscillations. This lead to the initial configuration of my parameters for implementing twiddle. Through the twiddle algorithm, I got parameters that were better, but a bit jerky. Finally a manual fine tuning of those parameters lead to the values of K<sub>p</sub> as 0.65, K<sub>i</sub> as 0.008 and K<sub>d</sub> as 22..

## Reflection

The proportaional gain is set as a control to equate steering angle to cross track error, which one gets through the simulator. Since it is directly porportional to the cross track error, it has a tendancy to induce an oscillating effect near the desired position and this is exactly what I noticed after I started tuning this parameter. To find the right parameter value such that there is a constant amplitude of oscillations was a little bit of a challenge nevertheless. 

The derivative gain is set to control the change in the cross track error. Therefore the oscillatory movement of the car resulting in ever changing cross track error was reduced by this parameter. The proportional and dervative gains were tuned in the first instant.

The integral gain is set to control the past values of the cross track error. This accumulates the cross track error since the beginning and when it becomes sufficiently large, the controller has the ability to act according to the desired outcome.

Eventually, a little bit of manual tuning and twiddle helped me get the right parameters for controlling the car. Although, I still feel I can do better with the parameters, the final combination does make the car go round the track with minimal errors. The downside of using twiddle is that it can get stuck in a local minima and that is usually not the most optimum configuration. The future work will entail a better tuning of parameters and controlling the throttle in such a way to accommodate for higher speeds. A [video](https://youtu.be/B85GKgiXIHw) of a single lap of the car maneuvering through the lanes can be viewed and studied. The movement is a little jerky and a better parameter tuning would result in better maneuvering of the car.

## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1
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

