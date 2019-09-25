# CarND-Path-Planning-Project


## Rubric Points

### Compilation

#### The code compiles correctly.

I just added compile options for `Release` and `Debug` build types to `CMakeFiles.txt`.
I also added spline library `spline.h` and path planner code `planner.hpp` to `src` directory.


### Valid Trajectories

#### The car is able to drive at least 4.32 miles without incident.

I ran the simulator for 5 miles without incident.

#### The car drives according to the speed limit.

No speed limit exceed message was seen.

#### Max Acceleration and Jerk are not Exceeded.

No max acceleration limit exceed or max jerk exceed message were seen.

#### Car does not have collisions.

No collisions.

#### The car stays in its lane, except for the time between changing lanes.

The car stays in one of the lanes.

#### The car is able to change lanes.

The car changed its lane if necessary.


### Reflection

In `main.cpp`, I used some structs defined in `planner.hpp` for implementation convenience.
I implemented path planning function on `Planner` class in `planner.hpp`.
The basic part of this class is based on the provided code.

`Planner` class consists of three parts:

#### Prediction: [line 337 to line 371](./src/planner.hpp#L337)

This part predicts other vehicles movement and check ego vehicle's condition.
I implemented as `CheckCondition()` method.
This method does the following process:

* Check whether there will be a close vehicle on each lane
* Check the distance of the closest vehicle ahead on each lane

#### Behavior: [line 373 to line 481](./src/planner.hpp#L373)

This part decides what to do:

* Switch lane when there is a close vehicle ahead on the current lane and there are no close vehicles on a target lane
   * If we on the center lane, we prefer the lane which seems fluent.
* Slow down when there is no chance to switch lane
* Speed up when there is no close vehicle

I used a constant acceleration rate for changing velocity.

#### Trajectory: [line 483 to line 559](./src/planner.hpp#L483)

This part generates the `(x,y)` coordinates trajectory based on target velocity and target lane.

First, sparse points are generated as following:

* Add the last two points of previous path, used to calculate reference point
   * Use the last one when there are not more than one points
* Add three next way points

These points are transformed into local car coordinates.
Then, I initialize spline curves with these points.
These operations are done by `GenerateSparsePoints()` method.

Next, I add some extended points to previous trajectory using spline curve.
Each time generating the point, the velocity is taken into account.
These generated spline points are transformed into global coordinates before added to trajectory, since they are based on local car coordinates.
When there are 50 points in trajectory, path generation has completed.
These operations are done by `GenerateInterporated()` method.

