Some basic tools for controlling a simulated quadrotor using [Drake](https://drake.mit.edu/)

## Installation

Install Drake with python bindings per [these instructions](https://drake.mit.edu/installation.html)

Set these environment variables in your `.bashrc` file:
```
export LD_LIBRARY_PATH="/opt/drake/lib${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}"
export PATH="/opt/drake/bin${PATH:+:${PATH}}"
export PYTHONPATH="/opt/drake/lib/python$(python3 -c 'import sys; print("{0}.{1}".format(*sys.version_info))')/site-packages${PYTHONPATH:+:${PYTHONPATH}}"
```

## Usage

Open a terminal and start the drake visualizer:
```
drake-visualizer
```

Then follow one of the following subsections to simulate the quadrotor.

### Waypoint tracking with LQR

Use Drake's built-in LQR controller to stabilize the quadrotor at a pre-defined
waypoint by running
```
python lqr_control.py
```
or just
```
./lqr_control.py
```
in a new terminal window. 

You can change the waypoint position on line 22 of `lqr_control.py`.

### Designing a custom controller

An example of designing a custom controller can be found in `custom_control.py`. Just like the LQR example, you can run this file with either
```
python custom_control.py
```
or just
```
./custom_control.py
```

Currently, this controller is just a trivial PD controller that drives the quadrotor to a desired height, without any horizontal stabilization. 

To write your own controller, you need to change the `CalcOutput` method of `CustomQuadrotorController`. This method gives you access to the state (x) of the quadrotor, and allows you set control inputs (u), which are propeller speeds. Specifically, replace the block startin on line 43 with your own code. 

The simple PD controller only considers the mass of the quadrotor. If you want to use other properties like the inertia matrix to design your controller, you can find those parameters in the [Drake source code](https://github.com/RobotLocomotion/drake/blob/master/examples/quadrotor/quadrotor_plant.cc)

