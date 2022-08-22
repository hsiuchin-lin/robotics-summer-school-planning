import pybullet as p
import pybullet_data
import math
import numpy
from potential_field_planner import PotentialFieldPlanner


# setup simulation
p.connect(p.GUI) # connect to GUI
p.setAdditionalSearchPath(pybullet_data.getDataPath()) # add python path
p.loadURDF("plane.urdf", [0, 0, -0.3]) # load floor
p.setGravity(0, 0, 0)
p.setRealTimeSimulation(0)
useSimulation = 1
#If we set useSimulation=0, it sets the arm pose to be the IK result directly without using dynamic control.
#This can be used to test the IK result accuracy.

#p.resetBasePositionAndOrientation(robot_id, [0, 0, 0], [0, 0, 0, 1])

# setup kuka
robot_id = p.loadURDF("kuka_iiwa/model.urdf", [0, 0, 0]) # load Kuka arm
kukaEndEffectorIndex = 6
numJoints = p.getNumJoints(robot_id)
if (numJoints != 7):
  exit()

ll = [-.967, -2, -2.96, 0.19, -2.96, -2.09, -3.05]         # lower limits for null space
ul = [.967, 2, 2.96, 2.29, 2.96, 2.09, 3.05]               # upper limits for null space
jr = [5.8, 4, 5.8, 4, 5.8, 4, 6]                           # joint ranges for null space
rp = [0, 0, 0, 0.5 * math.pi, 0, -math.pi * 0.5 * 0.66, 0] # restposes for null space
jd = [0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1]                   # joint damping coefficents
for i in range(numJoints):
    p.resetJointState(robot_id, i, rp[i])


t = 0.
prevPose = [0, 0, 0]
prevPose1 = [0, 0, 0]
hasPrevPose = 0
useNullSpace = 1


#use 0 for no-removal
trailDuration = 15

i = 0
time_step = 0.005 
orn = p.getQuaternionFromEuler([0, -math.pi, 0])


end_pos = [0.3, -0.7, 1.0]
time_step = 0.001
k_att     = 50*numpy.array([[1, 0, 0], [0, 1, 0], [0, 0, 1]])
k_rep     = 50
vel_max   = 50
planner   = PotentialFieldPlanner(end_pos, time_step, k_att, k_rep, vel_max)

planner = PotentialFieldPlanner(end_pos, time_step, k_att, k_rep, vel_max)

while 1:
    i += 1
 
    t = t + time_step
    p.stepSimulation()

    for i in range(1):    
	     
        ls = p.getLinkState(robot_id, kukaEndEffectorIndex)
        robot_pos   = ls[0]
     		
        desired_pos, desired_vel =	planner.get_desired_pos_vel(robot_pos)
     		     		
        if (useNullSpace == 1):     
            jointPoses = p.calculateInverseKinematics(robot_id, kukaEndEffectorIndex, desired_pos, orn, ll, ul, jr, rp)      
        else:
            jointPoses = p.calculateInverseKinematics(robot_id, kukaEndEffectorIndex, desired_pos)

        for i in range(numJoints):
            p.setJointMotorControl2(bodyIndex=robot_id,
                                jointIndex=i,
                                controlMode=p.POSITION_CONTROL,
                                targetPosition=jointPoses[i],
                                targetVelocity=0,
                                force=500,
                                positionGain=0.03,
                                velocityGain=1)
 

    ls = p.getLinkState(robot_id, kukaEndEffectorIndex)
    if (hasPrevPose):
        p.addUserDebugLine(prevPose, desired_pos, [0, 0, 0.3], 1, trailDuration)
        p.addUserDebugLine(prevPose1, ls[4], [1, 0, 0], 1, trailDuration)
    prevPose = desired_pos
    prevPose1 = ls[4]
    hasPrevPose = 1

p.disconnect()
