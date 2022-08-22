from pyPS4Controller.controller import Controller
import numpy as np

class JoyStick(Controller):
    """
    This class initializes use of a PS4 controller to control the robot.
    For the time being, all that is implemented is the use of the joysticks.
    As is convention, the left joystick controls the speed of the robot in
    different directions, while the right joystick makes the robot turn.
    """

    def __init__(self, **kwargs):
        Controller.__init__(self, **kwargs)

    def on_L3_up(self, value):
        print("on_L3_up", value)
    
    def on_L3_down(self, value):
        print("on_L3_down", value)

    def on_L3_left(self, value):
        print("on_L3_left", value)
    
    def on_L3_right(self, value):
        print("on_L3_right", value)
    
    def on_R3_up(self, value):
        print("on_R3_up", value)
    
    def on_R3_down(self, value):
        print("on_R3_down", value)
    
    def on_R3_left(self, value):
        print("on_R3_left", value)
    
    def on_R3_right(self, value):
        print("on_R3_right", value)

    def on_x_press(self, value):
        print("x pressed")

from pygame import joystick 
import pygame
"""
pygame.init()

joystick.init()

done = False 

while not done:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            done = True

    j = joystick.Joystick(0)
    j.init()

    ax = j.get_numaxes()

    for i in range(ax):
        val = j.get_axis(i)
        print("Axis %d: %f" % (i, val), end=', ')
    print("")

"""
class Controller:


    def __init__(self, id, speed_limit, axes):
        """
        The id variable specifies the joystick to be used.
        If you need to determine it, use the pygame.joystick.get_count() method to determine the number, 
        then use the Joystick.get_name() method to determine the name of each. 

        The speed_limit variable sets the limit on the speed to be given to the robot. 

        The axes variable gives the axes to be used from the controller as input.
        The order should be left joystick left/right, left joystick up/down, right joystick left/right.
        """
        pygame.init()
        joystick.init()
        self._jstick = joystick.Joystick(id)
        self._jstick.init()

        self._speed_limit = np.array(speed_limit)
        self._axes = axes
    
    def get_velocity(self):
        """
        This method returns the robot velocity based on the max velocity given joystick inputs.
        Returns:
            velocity (float (3, ), float): the 3-velocity and the rotational velocity in a tuple.
        """
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                joystick.quit()

        velocity = [0, 0, 0, 0]
        for i in range(3):
            if i == 2:
                velocity[3] = self._jstick.get_axis(self._axes[i])
            else:
                velocity[i] = self._jstick.get_axis(self._axes[i])
        
        velocity = np.minimum(velocity, self._speed_limit)
        velocity = np.maximum(velocity, -self._speed_limit)
        velocity *= -1  # Weird quirk with controller input
        return velocity[:3], velocity[3]




