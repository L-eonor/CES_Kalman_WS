# State Estimation and Tracking, Kalman Filter Workshop

For this workshop, the main goals are:
* Understand the importance of State Estimation in ADAS
* Understand the methods used for typical vehicle tracking and localization in ADAS
* Apply the Kalman filter to a driver monitoring feature tracking state estimation problem

## Kalman Filter
The [Kalman Filter](https://en.wikipedia.org/wiki/Kalman_filter) is a widely used tool in perception and general sensorics in the automotive scope. In ADAS 🚗, we can apply it to either the outside world (like cars, bikes and pedestrians' trajectories) but also to the environment inside the vehicle. That's the scope of this workshop: a driver monitoring problem;

## The Dummy Problem

When trying to monitor the driver's dizziness😵‍, the systems inside the car try to gather a set of the face's landmarks. That will give an estimation regarding what the driver is looking at. Here we can see a dummy and the corresponding face landmarks.


![Dummy image](https://myoctocat.com/assets/images/base-octocat.svg)


The point behind the nose is the starting point to understanding the driver's roll, pitch and yaw. With that, we can derive the direction of the face. Then with the eye analysis, the loop is more or less closed.


## Workshop Roadmap

To complete the task, one should:
* Derive the state transition model of our track and system inputs;
* Determine our observation model;
* Model process and measurement noise;
* Implement Prediction and Update steps of the Kalman Filter;
* Visualize Kalman Filter in action 😎
