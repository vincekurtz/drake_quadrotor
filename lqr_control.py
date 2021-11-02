#!/usr/bin/env python

##
#
# A simple example of using Drake's built-in LQR controller
# to stabilize the quadrotor at a pre-defined waypoint.
#
##

from pydrake.all import *
from pydrake.examples.quadrotor import (
        QuadrotorPlant, QuadrotorGeometry, StabilizingLQRController)

# Set up system plant
builder = DiagramBuilder()
quad = builder.AddSystem(QuadrotorPlant())
scene_graph = builder.AddSystem(SceneGraph())
geom = QuadrotorGeometry.AddToBuilder(builder, quad.get_output_port(0), scene_graph)

# Connect controller
target_position = np.array([0, -2, 1.0])  # this is the waypoint [x,y,z]
controller = builder.AddSystem(StabilizingLQRController(quad, target_position))

builder.Connect(
        quad.get_output_port(),
        controller.get_input_port())
builder.Connect(
        controller.get_output_port(),
        quad.get_input_port())

# Connect visualizer
DrakeVisualizer().AddToBuilder(builder, scene_graph)

# Set up simulation
diagram = builder.Build()
context = diagram.CreateDefaultContext()
simulator = Simulator(diagram, context)
simulator.set_target_realtime_rate(1.0)
simulator.AdvanceTo(5.0)   # run for 5 seconds

