import os
import inspect
current_dir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
parent_dir = os.path.dirname(current_dir)
os.sys.path.insert(0, parent_dir)
world_dir = os.path.abspath("../aws-robomaker-racetrack-world/")
model_dir = os.path.abspath("../aws-robomaker-racetrack-world/models")
os.sys.path.insert(0, world_dir)
os.sys.path.insert(0, model_dir)

world_dir = current_dir + "/worlds"

import numpy
import pybullet
from pybullet_utils import bullet_client
import pybullet_data as pd
import world_parser as gazebo_world_parser

import time
import random

from mpc_controller import a1_sim as robot_sim
from potential_field_planner import PotentialFieldPlanner
from trot_controller import TrotController

max_time = 300

simulation_time_step = 0.001
ini_pos = [0.0, 0.0, 0.27]  # initial position
end_pos = [4.0, 0.8, 0.27]  # end position


''' recording video requires ffmpeg in the path '''
record_video = False
if record_video:
    p = pybullet
    p.connect(p.GUI, options="--width=1280 --height=720 --mp4=\"test.mp4\" --mp4fps=100")
    p.configureDebugVisualizer(p.COV_ENABLE_SINGLE_STEP_RENDERING,1)
else:
    p = bullet_client.BulletClient(connection_mode=pybullet.GUI)    
     

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
gazebo_world_parser.parseWorld( p, filepath = "worlds/bowl.world", model_path=world_dir )
p.configureDebugVisualizer(p.COV_ENABLE_RENDERING,1)

robot_uid = p.loadURDF(robot_sim.URDF_NAME, robot_sim.START_POS)
robot = robot_sim.SimpleRobot(p, robot_uid, simulation_time_step=simulation_time_step)

# set the planner
time_step = 0.001
k_att     = numpy.array([[1, 0, 0], [0, 1, 0], [0, 0, 0.5]])
k_rep     = 1
vel_max   = 0.5
planner   = PotentialFieldPlanner(end_pos, time_step, k_att, k_rep, vel_max)
planner.set_obstacle_distance(1.5)
planner.set_obstacle_position([2, 0, 0.27])


# set the controller
controller = TrotController()
controller._setup_controller(robot)
 

current_time = robot.GetTimeSinceReset()
ang_vel = 0.0 

while current_time < max_time:
    pos, orn = p.getBasePositionAndOrientation(robot_uid)
    p.submitProfileTiming("loop")
            
    pos_des, lin_vel =	planner.get_avoidance_force (pos)
#    print("t=", current_time, " lin_vel=",lin_vel, " ang_vel=",ang_vel)
    ##print("t=", current_time, " pos=", pos, ",lin_vel", lin_vel)
    hybrid_action, info = controller.update(lin_vel, ang_vel)

    robot.Step(hybrid_action)

    if record_video:
        p.configureDebugVisualizer(p.COV_ENABLE_SINGLE_STEP_RENDERING,1)

    #time.sleep(0.003)
    current_time = robot.GetTimeSinceReset()
    p.submitProfileTiming()

