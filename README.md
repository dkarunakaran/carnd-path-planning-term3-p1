# CarND-Path-Planning-Project
[![Udacity - Self-Driving Car NanoDegree](https://s3.amazonaws.com/udacity-sdc/github/shield-carnd.svg)](http://www.udacity.com/drive)

In this project our goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. You will be provided the car's localization and sensor fusion data, there is also a sparse map list of waypoints around the highway. The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 10 m/s^3.

## Reflection

### Prediction
The prediction component estimates what actions other objects might take in the future. For example, if another vehicle were identified, the prediction component would estimate its future trajectory.

#### Input of the prediction module from sensor fusion example
```
{
    "timestamp" : 34512.21,
    "vehicles" : [
        {
            "id"  : 0,
            "x"   : -10.0,
            "y"   : 8.1,
            "v_x" : 8.0,
            "v_y" : 0.0,
            "sigma_x" : 0.031,
            "sigma_y" : 0.040,
            "sigma_v_x" : 0.12,
            "sigma_v_y" : 0.03,
        },
        {
            "id"  : 1,
            "x"   : 10.0,
            "y"   : 12.1,
            "v_x" : -8.0,
            "v_y" : 0.0,
            "sigma_x" : 0.031,
            "sigma_y" : 0.040,
            "sigma_v_x" : 0.12,
            "sigma_v_y" : 0.03,
        },
    ]
}
```
#### Output of the prediction module example

```
{
    "timestamp" : 34512.21,
    "vehicles" : [
        {
            "id" : 0,
            "length": 3.4,
            "width" : 1.5,
            "predictions" : [
                {
                    "probability" : 0.781,
                    "trajectory"  : [
                        {
                            "x": -10.0,
                            "y": 8.1,
                            "yaw": 0.0,
                            "timestamp": 34512.71
                        },
                        {
                            "x": -6.0,
                            "y": 8.1,
                            "yaw": 0.0,
                            "timestamp": 34513.21
                        },
                        {
                            "x": -2.0,
                            "y": 8.1,
                            "yaw": 0.0,
                            "timestamp": 34513.71
                        },
                        {
                            "x": 2.0,
                            "y": 8.1,
                            "yaw": 0.0,
                            "timestamp": 34514.21
                        },
                        {
                            "x": 6.0,
                            "y": 8.1,
                            "yaw": 0.0,
                            "timestamp": 34514.71
                        },
                        {
                            "x": 10.0,
                            "y": 8.1,
                            "yaw": 0.0,
                            "timestamp": 34515.21
                        },
                    ]
                },
                {
                    "probability" : 0.219,
                    "trajectory"  : [
                        {
                            "x": -10.0,
                            "y": 8.1,
                            "yaw": 0.0,
                            "timestamp": 34512.71
                        },
                        {
                            "x": -7.0,
                            "y": 7.5,
                            "yaw": -5.2,
                            "timestamp": 34513.21
                        },
                        {
                            "x": -4.0,
                            "y": 6.1,
                            "yaw": -32.0,
                            "timestamp": 34513.71
                        },
                        {
                            "x": -3.0,
                            "y": 4.1,
                            "yaw": -73.2,
                            "timestamp": 34514.21
                        },
                        {
                            "x": -2.0,
                            "y": 1.2,
                            "yaw": -90.0,
                            "timestamp": 34514.71
                        },
                        {
                            "x": -2.0,
                            "y":-2.8,
                            "yaw": -90.0,
                            "timestamp": 34515.21
                        },
                    ]

                }
            ]
        },
        {
            "id" : 1,
            "length": 3.4,
            "width" : 1.5,
            "predictions" : [
                {
                    "probability" : 1.0,
                    "trajectory" : [
                        {
                            "x": 10.0,
                            "y": 12.1,
                            "yaw": -180.0,
                            "timestamp": 34512.71
                        },
                        {
                            "x": 6.0,
                            "y": 12.1,
                            "yaw": -180.0,
                            "timestamp": 34513.21
                        },
                        {
                            "x": 2.0,
                            "y": 12.1,
                            "yaw": -180.0,
                            "timestamp": 34513.71
                        },
                        {
                            "x": -2.0,
                            "y": 12.1,
                            "yaw": -180.0,
                            "timestamp": 34514.21
                        },
                        {
                            "x": -6.0,
                            "y": 12.1,
                            "yaw": -180.0,
                            "timestamp": 34514.71
                        },
                        {
                            "x": -10.0,
                            "y": 12.1,
                            "yaw": -180.0,
                            "timestamp": 34515.21
                        }
                    ]
                }
            ]
        }
    ]
}
```
This example only shows vehicles but in reality we would also generate predictions for all dynamic objects in view.

As we see above, actual prediction module gives the possible trajectories from the current timeline to the future of each vehicle. In this highway exmaple, we will have only one possible trajectory for each vehicle and we are using simple approach for predictio here. In complex situation we may need to use model, data, or hybrid approach for prediction module.

Currently we assume that we have only three lanes and each lane has 4 meter width. In actual scenarion, number of lanes, distance between the lanes, and total lanes distance can be detected using computer vision technologies. We slightly touched in advanced lane findings in term1. 

content to be added


### Behaviour planning
The behavioral planning component determines what behavior the vehicle should exhibit at any point in time. For example stopping at a traffic light or intersection, changing lanes, accelerating, or making a left turn onto a new street are all maneuvers that may be issued by this component. Behavior planner takes the input of maps, route and predictions about what other vehicles are likely to do and suggest the trajectory module to create trajectory based on the suggestion from the behavior planner.

image to be added

In actual case, behaviour planner decides the trajectory based on the cost functions. In this highway example, we may no need to worry about cost functions as we are considering only lane change or reduce speed based on the obstacles. 


### Trajectory Generation
Based on the desired immediate behavior, the trajectory planning component will determine which trajectory is best for executing this behavior.
