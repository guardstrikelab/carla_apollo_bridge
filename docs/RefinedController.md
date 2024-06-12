# Remark

This rep is a branch of Apollo 8.0, in which the **controller** is particularly optimized for co-simulation with Carla, for a better co-simulation performance.

[Apollo-Carla official bridge](https://github.com/guardstrikelab/carla_apollo_bridge)


## Demo Video

Followings are the demo videos showing the performance of improved controller.

Demo: turn left - 90-degree sharp left turn, Town01

[![Demo: left turn](https://img.youtube.com/vi/XBq-kF-m2Us/0.jpg)](https://www.youtube.com/watch?v=XBq-kF-m2Us)

Demo: turn right - 90-degree sharp right turn, Town01

[![Demo: right turn](https://img.youtube.com/vi/oOJiw21AAGk/0.jpg)](https://www.youtube.com/watch?v=oOJiw21AAGk)


The above demo is using the tuned MPC controller with turning ratio (in bridge) tuned. This is the default co-simulation control performance that this project aims to provide.


To achieve the above performance, [turning ratio](../carla_bridge/actor/ego_vehicle.py#L57) is set as the following:

```shell
self.right_turn_ratio = 0.7
self.left_turn_ratio = 0.85
```

The MPC controller [parameters](../carla_bridge/refined_controller/control_conf.pb.txt#L9370):

```shell
  matrix_q: 3.0
  matrix_q: 0.0
  matrix_q: 18.45
  matrix_q: 0.0
  matrix_q: 30.0
  matrix_q: 10.0
  matrix_r: 3.25
  matrix_r: 1.75
```


## How to use the improved MPC controller?

First, you should set up the co-simulation environment according to the setup doc: [Getting Started](GettingStarted.md).

Then, replace the following files in [Apollo 8.0](https://github.com/ApolloAuto/apollo/tree/v8.0.0) with [the modified files](../carla_bridge/refined_controller/) provided in this repo:

```shell
modules/control/conf/control_conf.pb.txt

modules/control/controller/mpc_controller.cc

modules/control/controller/mpc_controller.h
```

## Specific Modifications

Compared with original Apollo 8.0 code ([Apollo 8.0](https://github.com/ApolloAuto/apollo/tree/v8.0.0)), the modified control code made the following improvements:


### Optimized controller configuration 

```shell
modules/control/conf/control_conf.pb.txt
```

The above file is the key configuration file for setting up the controller, including the relation of **how the acceleration can be transformed to throttle or brake signal**. However, the [original configuration](https://github.com/ApolloAuto/apollo/blob/v8.0.0/modules/control/conf/control_conf.pb.txt) has several bugs including the too low acceleration, incorrect acc-to-throttle mapping, etc.



### Optimized MPC controller 

```shell
modules/control/controller/mpc_controller.cc

modules/control/controller/mpc_controller.h
```

The above file is the implementation of the Model Predicted Control (MPC) in Apollo, which is an advanced controller. Several bugs have been discovered in the [original MPC controller](https://github.com/ApolloAuto/apollo/blob/v8.0.0/modules/control/controller/mpc_controller.cc), and they were fixed in this repo. These bugs include the inconsistent choice of planning point, lack of look-ahead time for planning query, redundant feedback on final control signal, etc.



### Detailed bug description

If you are interested, the following page can be helpful for understanding the Apollo controller (It is Apollo 9.0, but key ideas are identical to 8.0).

[Apollo Control Module Introduction](https://github.com/ApolloAuto/apollo/blob/master/modules/control/control_component/README_cn.md)


Following are the detailed report of discoverd bugs:

[Bugs](https://github.com/guardstrikelab/carla_apollo_bridge/issues/159)



## How to tune the controller

Due to the complexity of this co-simulation (e.g, complexity of the Apollo ADS, complex dynamics in Carla), the current controller is still far from being perfect. If you find the controller unsatisfying, you are encouraged to tune the controller as well. Following are some suggestions:

1. Directly tune the [percentage](https://github.com/guardstrikelab/carla_apollo_bridge/blob/a54a4b10f6c0aa0c27b0b39f45a40d039f5fdbd0/carla_bridge/actor/ego_vehicle.py#L57) of the steering signal that is actually applied by Carla. By default, the left turn and right turn signal is tuned with 70% and 85% weight, respectively (the default steer signal tends to overshoot at turns).

2. Modify the configuration file ([<em>control_conf.pb.txt</em>](https://github.com/ApolloAuto/apollo/blob/v8.0.0/modules/control/conf/control_conf.pb.txt)) based on your experimental results.

3. Try different controllers: change **active_controllers** in [<em>control_conf.pb.txt</em>](https://github.com/ApolloAuto/apollo/blob/v8.0.0/modules/control/conf/control_conf.pb.txt) from <em>MPC_CONTROLLER</em> to <em>LAT_CONTROLLER</em>, <em>LON_CONTROLLER</em>, then Apollo will apply MPC controller instead.

4. Modify the controller code (e.g., fixing more bugs which you find). These codes include [lat_controller.cc](https://github.com/PhilWallace/apollo-8.0.0_control_opt/blob/main/modules/control/controller/lat_controller.cc), [lon_controller.cc](https://github.com/PhilWallace/apollo-8.0.0_control_opt/blob/main/modules/control/controller/lon_controller.cc), [mpc_controller.cc](https://github.com/PhilWallace/apollo-8.0.0_control_opt/blob/main/modules/control/controller/mpc_controller.cc)


## Challenges and future works

It is very challenging to get a good controller in co-simulation. The Apollo code can be buggy, not to mention the implementation problems and new challenes brought by Carla. It would be very helpful for the community if you can also contribute to this co-simulation project.

