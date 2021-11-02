#!/usr/bin/env python

##
#
# A simple example of using a custom controller to send
# low-level commands to a simulated quadrotor.
#
##

from pydrake.all import *
from pydrake.examples.quadrotor import QuadrotorPlant, QuadrotorGeometry

# Define the controller object
class CustomQuadrotorController(LeafSystem):
    """
    This class defines a custom controller for a quadrotor.
    It takes as input the state [q;qd] of the quadrotor and outputs
    propeller forces to be applied on the system. 
    """
    def __init__(self):
        LeafSystem.__init__(self)

        # Set up input and output ports
        self.state_port = self.DeclareVectorInputPort(
                "state",
                BasicVector(12))
        self.DeclareVectorOutputPort(
                "propeller_force",
                BasicVector(4),
                self.CalcOutput)

    def CalcOutput(self, context, output):
        """
        This method is called at each timestep and used to
        set the output of this system (propeller forces). 
        """
        # The state of the quadrotor is [q;qd], where
        # q = [x position; y position; z position; roll angle; pitch angle; yaw angle]
        x = self.state_port.Eval(context)

        ##################################################################
        #            IMPLEMENT YOUR CONTROLLER HERE!                     #
        ##################################################################

        # For now we'll just write a simple PD controller to hover 
        # at a given height (z_des), without any horizontal stabilization.

        z_des = 1.0     # desired height
        zd_des = 0.0    # desired vertical velocity
        z = x[2]        # actual height
        zd = x[8]       # actual vertical velocity

        m = 0.775       # quadrotor mass
        g = 9.81        # acceleration due to gravity

        kp = 2          # proportional gain
        kd = 1          # derivative gain

        # PD control law for total force in the z direction,
        # including gravity compensation
        fz = m*g - kp*(z-z_des) - kd*(zd-zd_des)

        # Divide this force equally between the four propellors
        u = fz/4*np.ones(4)

        ##################################################################

        # The control input u must be a list or numpy array with 4 elements,
        # representing the force applied by each propellor.
        output.SetFromVector(u)

# Set up system plant
builder = DiagramBuilder()
quad = builder.AddSystem(QuadrotorPlant())
scene_graph = builder.AddSystem(SceneGraph())
geom = QuadrotorGeometry.AddToBuilder(builder, quad.get_output_port(0), scene_graph)

# Connect controller
controller = builder.AddSystem(CustomQuadrotorController())

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
simulator.AdvanceTo(5.0)

