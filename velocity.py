"""
This file takes care of giving the tobot velocities.
It has the folling functionality:
    - Take a trajectory and generate the necessary velocities.
    - Generate velocities based on joystick input. 
    - Generate velocities based on a desired end position.
"""
import numpy as np

import pybullet as p
from pybullet_utils import bullet_client

from mpc_controller import a1_sim as robot_sim

#from joystick import Controller
from optimizer import Optimizer


class VelocityMapper:
    """ This class generates velocities for desired motions. It is designed for the a1 robot simulation running in pybullet.

            Attributes: 

                robot_id (int): The uid of the robot as given by pybullet.
                simulation_time_step (float): The time interval between simulation frames.
                mode (str):Possible modes are: 
                    joystick: for joystick control, make sure to pass in a joystick object.
                    potential: for simple potential velocity control. Pass in a target position.
                    trajectory: for following a pre-computed trajectory which has position and/or velocity.

                joystick (Controller): A joystick object to be used only if mode is 'joystick'.
                target (sequence[float]): A target vector to be used only for the 'potential' mode in [x, y, z] format.
                potential_constant (float): The proportionality constant for the potential function.
                trajectory (sequence[Sequence[float]]): A trajectory for the robot to follow for the 'trajectory' mode.
                    It should be an array of 4-vectors or 8-vectors, where each entry represents 
                    [x, y, z, theta, vx, vy, vz, vtheta], where theta is measured from the positive x-axis.
                    Shape is (4, N) or (8, N) where N is the number of points in the trajectory.
                total_time (float): The total time the trajectory takes to complete. Used for collocation.
                inference_mode (str): one of 'collocation' or 'nearest'. 
                    It specifies how to use the trajectory to control the robot.
                alpha (float): a constant for the 'nearest' inference mode. 

                bullet (BulletClient): the pybullet client in which the simulation is running.

            Methods:

                get_velocity(t): 
                    Gets the current desired velocity for the robot given the class input parameters.

        """

    def __init__(self, robot_id, simulation_time_step,
                 mode="trajectory",
                 joystick=None,
                 target=None, potential_constant=1,
                 trajectory=None, total_time=5, inference_mode="nearest", alpha=0.2,
                 bullet=None):
        """ Set up the velocity mapper.

            Parameters
            ----------
            robot_id : int 
                The uid of the robot as given by pybullet.
            simulation_time_step : float 
                The time interval between simulation frames.
            mode : str 
                Possible modes are: 
                    joystick: for joystick control, make sure to pass in a joystick object.
                    potential: for simple potential velocity control. Pass in a target position.
                    trajectory: for following a pre-computed trajectory which has position and/or velocity.

            joystick : Controller 
                The joystick object to be used only if mode is 'joystick'.
            target : array-like
                A target vector to be used only for the 'potential' mode in [x, y, z] format.
            potential_constant : float
                The proportionality constant for the potential function.
            trajectory : array-like
                A trajectory for the robot to follow for the 'trajectory' mode.
                It should be an array of 4-vectors or 8-vectors, where each entry represents 
                [x, y, z, theta, vx, vy, vz, vtheta], where theta is measured from the positive x-axis.
                Shape is (4, N) or (8, N) where N is the number of points in the trajectory.
            total_time : float
                The total time the trajectory takes to complete. Used for collocation.
            inference_mode : str
                One of 'collocation' or 'nearest'. 
                It specifies how to use the trajectory to control the robot.
            alpha : float
            A constant for the 'nearest' inference mode. 

            bullet : BulletClient
                The pybullet client in which the simulation is running.

            Returns
            -------
            self : VelocityMapper
                The velocity mapper object which computes the control variables to give to the robot at 
                the appropriate instant. 
        """
        self.robot_id = robot_id
        self.simulation_time_step = simulation_time_step
        self.mode = mode
        if mode == "joystick":
            assert joystick is not None, "Make sure to pass a joystick object."
            self.joystick = joystick
        elif mode == "potential":
            assert target is not None, "Make sure to give a target position."
            self.target = target
            self.potential_constant = potential_constant
        elif mode == "trajectory":
            assert trajectory is not None, "Make sure to give a trajectory to follow."
            if trajectory.shape[0] == 4:
                self.trajectory_positions = trajectory
            elif trajectory.shape[0] == 8:
                self.trajectory_positions = trajectory[:4]
                self.trajectory_velocities = trajectory[4:]
            else:
                raise Exception("Can't understand trajectory data.")

            self.inference_mode = inference_mode
            if inference_mode == "nearest":
                assert trajectory.shape[0] == 8, "Need velocity for 'nearest' inference."
            elif inference_mode != "collocation":
                raise Exception(
                    "Unknown inference mode, pass in one of ('nearest', 'collocation').")
            self.total_time = total_time
        else:
            raise Exception(
                "Unknown mode, please pass in one of (joystick, potential, trajectory).")

        self._alpha = alpha
        self.bullet_client = bullet

    # Define getters and setters
    def set_target(self, target):
        """ Change the target end position of the robot. """
        self.target = target

    def set_joystick(self, joystick):
        '''Set the joystick'''
        self.joystick = joystick

    def potential_map(self, k):
        """
        This function tells the robot how to move.
        It will function based a simple potential which just pushes the robot to the destination.
        Simple linear force -> v = k(x_goal - x)

        Determines its position using on-board sensors. 

        Parameters:
            position (array, (2,), float): the current (x, y) position of the robot.
            goal (array, (3,), float): the desired end (x, y) position of the robot.
            k (float): the proportionality constant for the motion.

        Returns:
            speed (array, (3,), float): the required linear velocity for the robot.
            rot (float): the required rotation velocity in the z direction.
        """
        goal = np.array(goal)
        # Parse the position into location and rotation components.
        location, rotation = self.bullet_client.getBasePositionAndOrientation(
            self.robot_id)
        location = np.array(location)
        rotation = np.array(rotation)

        speed = np.zeros(3)
        rot = 0

        # First the robot rotates until it faces the goal.
        def unit(v):
            return v / np.linalg.norm(v)

        delta = goal - location
        dist = np.linalg.norm(delta[0:2])

        # Need the signed angle between the direction vector of the robot and the goal.
        direction = np.array(self.bullet_client.multiplyTransforms([0, 0, 0], rotation,
                                                                   [1, 0, 0],
                                                                   self.bullet_client.getQuaternionFromEuler(
                                                                       [0, 0, 0])
                                                                   )[0])

        location_inversed, orientation_inversed = self.bullet_client.invertTransform(
            location, rotation)
        xyspeed, _ = self.bullet_client.multiplyTransforms(
            location_inversed, orientation_inversed,
            goal, self.bullet_client.getQuaternionFromEuler([0, 0, 0]))

        speed[0:2] = xyspeed[0:2]
        speed = k * speed  # speed = [v_X, v_y, v_z]
        # Need to cap the speed, since the robot can't handle everything.
        cap = np.array([1, 0.6, 0]) * robot_sim.MPC_VELOCITY_MULTIPLIER
        speed = np.minimum(speed, cap)
        speed = np.maximum(speed, -cap)

        speed[np.abs(xyspeed) < 0.08] = 0
        return speed, rot

    def joystick_map(self, joystick):
        return joystick.get_velocity()

    def trajectory_map(self, t):
        if self.inference_mode == "nearest":
            pos, orn = self.bullet_client.getBasePositionAndOrientation(
                self.robot_id)
            angle = self.bullet_client.getEulerFromQuaternion(orn)[2]
            pv = np.append(pos, angle)
            d = np.linalg.norm(self.trajectory_positions -
                               pv.reshape(-1, 1), axis=0)
            k = np.argmin(d) + 1
            k = min(k, 100)
            velocity = self._alpha * (self.trajectory_positions[:, k] - pv) + \
                (1 - self._alpha) * self.trajectory_velocities[:, k]
        elif self.inference_mode == "collocation":
            velocity = self.interpolate(
                self.trajectory_velocities, self.total_time, t)

        return velocity[:3], velocity[3]

    def get_velocity(self, t=0):
        """ This method gets the velocity based on the modes speficied for the mapper.
        Note that to use the trajectory 'collocation' method, you must give the current time.

        Parameters:
            t (float): The current time in the simulation. Required for the 'collocation' inference of the 'trajectory' mode. 

        Returns:
            lin_speed (float, (3,)): The velocity of the robot. The entries are [vx, vy, vz] respectively, and the units are metres per second.
            ang_speed (float): the angular velocity of the robot in radians per second.
        """
        if self.mode == "joystick":
            lin_speed, ang_speed = self.joystick_map()
        elif self.mode == "potential":
            lin_speed, ang_speed = self.potential_map()
        elif self.mode == "trajectory":
            lin_speed, ang_speed = self.trajectory_map(t)

        # Convert body_height change to absolute height change.
        lin_speed[2] *= self.simulation_time_step
        return lin_speed, ang_speed

    @staticmethod
    def interpolate(velocities, T, t):
        """This method infers the current position based on the elapsed time.
        It then interpolates between the closest trajectory points to determine the velocity.
        """
        N = velocities.shape[1]
        position = t / T * N
        if position + 1 >= N:
            return velocities[:, -1]
        index = int(position)
        next_i = index + 1
        frac = position - index

        return (1 - frac) * velocities[:, index] + frac * velocities[:, next_i]
