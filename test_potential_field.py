import numpy
import os
import inspect
currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
parentdir = os.path.dirname(currentdir)
os.sys.path.insert(0, parentdir)

# from absl import app
# import scipy.interpolate
# import numpy as np

import pybullet
import pybullet_data as pd
from pybullet_utils import bullet_client

import time
#import random

from mpc_controller import a1_sim as robot_sim
from potential_field_planner import PotentialFieldPlanner
from trot_controller import TrotController

''' set the experiment '''
max_time = 100

simulation_time_step = 0.001
ini_pos = [0, 0, 0.24]  # initial position
end_pos = [3, 1, 0.24]  # end position

''' recording video requires ffmpeg in the path'''
record_video = False
if record_video:
    p = pybullet
    p.connect(p.GUI, options="--width=1280 --height=720 --mp4=\"test.mp4\" --mp4fps=100")
    p.configureDebugVisualizer(p.COV_ENABLE_SINGLE_STEP_RENDERING,1)
else:
    p = bullet_client.BulletClient(connection_mode=pybullet.GUI)    
     
''' set pybullet '''
p.configureDebugVisualizer(p.COV_ENABLE_GUI,0)
p.configureDebugVisualizer(p.COV_ENABLE_RENDERING,0)
p.setAdditionalSearchPath(pd.getDataPath())
num_bullet_solver_iterations = 30
p.setPhysicsEngineParameter(numSolverIterations=num_bullet_solver_iterations)  
p.setPhysicsEngineParameter(enableConeFriction=0)
p.setPhysicsEngineParameter(numSolverIterations=30)   
p.setTimeStep(simulation_time_step) 
p.setGravity(0, 0, -9.8)
p.setPhysicsEngineParameter(enableConeFriction=0)
p.setAdditionalSearchPath(pd.getDataPath())
p.loadURDF("plane.urdf")
p.configureDebugVisualizer(p.COV_ENABLE_RENDERING,1)

robot_uid = p.loadURDF(robot_sim.URDF_NAME, robot_sim.START_POS)
robot = robot_sim.SimpleRobot(p, robot_uid, simulation_time_step=simulation_time_step)

''' set the planner '''
time_step = 0.001
k_att     = numpy.array([[1, 0, 0], [0, 1, 0], [0, 0, 0.25]])
k_rep     = 1
vel_max   = 0.5
planner   = PotentialFieldPlanner(end_pos, time_step, k_att, k_rep, vel_max)

''' set the controller '''
controller = TrotController()
controller._setup_controller(robot)
 
current_time = robot.GetTimeSinceReset()
ang_vel = 0.0 

while current_time < max_time:
    pos, orn = p.getBasePositionAndOrientation(robot_uid)
    print("t=", current_time, " pos=",pos, " orn=",orn, " end_pos=", end_pos )
    p.submitProfileTiming("loop")
            
    pos_des, lin_vel =	planner.get_desired_pos_vel(pos)
    print("t=", current_time, " lin_vel=",lin_vel, " ang_vel=",ang_vel)
    hybrid_action, info = controller.update(lin_vel, ang_vel)

    robot.Step(hybrid_action)

    if record_video:
        p.configureDebugVisualizer(p.COV_ENABLE_SINGLE_STEP_RENDERING,1)

    current_time = robot.GetTimeSinceReset()
    p.submitProfileTiming()

