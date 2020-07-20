## Path Planning
### Path Planning Project

#### The goals / steps of this project are the following:

##### The code compiles correctly.
##### The car is able to drive at least 4.32 miles without incident
##### The car must not come into contact with any of the other cars on the road.
##### The car does not exceed a total acceleration of 10 m/s^2 and a jerk of 10 m/s^3.


##  Implementation
For this project, i referred to the path planning walktrhough by David silver and Aaron Brown as mentioned in the course. It was really helpful to understand more about this project. To determine the next state of the car, the data from the sensor fusion module is used. 

### Process
1. Find the current car lane and check if the the left and right lanes are safe. Chcek if the current car lane is 0, then set the `is_left_lane_free` to false and if current car lane is 2 then `is_left_lane_free` to false
2. Next we need to analyse every other car from the sensor fusion data. If the other car is in the same lane and is too close, slow down to match ahead car speed and set flag  `ahead_close`to true. If the other car is in the left lane and is too close, then set flag `is_left_lane_free` to false. If the other car is in the right lane and is too close, then set the flag `is_left_lane_free` to false
3. After the iteration through the sensor fusion data, the status of the flags is used to determine the next state. If the front car is too close, slow down to match ahead car speed. If the left lane is free then move to left lane.
If left is not safe and right lane is free then move to right lane. If left and right lanes are not safe, then keep the same lane maintaining ahead car speed.
4. I used spline.h described in [spline function] (https://kluge.in-chemnitz.de/opensource/spline/) to generate the spline. To create a smooth tragectory, i have sampled many points as necessary.


