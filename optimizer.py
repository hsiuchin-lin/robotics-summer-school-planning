"""
This file is used to generate optimal paths for the robot using casadi.
Control is being changed from being done globally and doing conversions 
to implementing the dynamics as having the control converted to world coordinates, 
which makes more sense. 
"""

import casadi
import numpy as np
#import a1_sim as robot_sim
from mpc_controller import a1_sim as robot_sim
import matplotlib.pyplot as plt

# First let's do just move to a location with no obstacles.

#N = 100  # Number of time steps


class Optimizer():

    def __init__(self, integrator="Euler", N=100, n_variables=4, ini_pos=[0, 0, 0.24, 0], end_pos= [3, 0, 0.24, 0], ini_vel=[0, 0, 0, 0], end_vel=[0, 0, 0, 0]):
        self.integrator = integrator
        self.N = N
        self.n_variables = n_variables
        self.ini_pos = ini_pos
        self.end_pos = end_pos        
        self.ini_vel = ini_vel
        self.end_vel = end_vel
	    
	    # Initialize the optimizer
        self.opti = casadi.Opti()       
        
        # Declaration of decision variables
        self.n_states = 2 * self.n_variables
        self.x = self.opti.variable(self.n_states, N+1)  	# Position/velocity
        self.u = self.opti.variable(self.n_variables, N+1)  # Acceleration / throttle
                
        # Hermite-Simpson variables
        self.x_half = self.opti.variable(self.n_states, N+1)
        self.u_half = self.opti.variable(self.n_variables, N+1)

        # Time optimization
        self.T = self.opti.variable()
        self.opti.minimize(self.T)  # The control objective is to min time

        # Specifying gap closing constraints
        self.dt = self.T / self.N
        self.inference_mode = "collocation"
        self.simulation_time_step = 0.001     
        
    # Let's define the dynamics of the problem, as well as the methods of integration.
    def f(self, x, u):
        """
        Control is performed relative to the robot, but
        the position change of the robot depends on its orientation.
        This function converts the local velocity to a change of position
        in global coordinates.

        Single x, u
        """

        dx0 = x[4, :] * casadi.cos(x[3, :]) + x[5, :] * casadi.sin(x[3, :])
        dx1 = x[4, :] * casadi.sin(x[3, :]) - x[5, :] * casadi.cos(x[3, :])
        dx2 = x[6, :]
        dx3 = x[7, :]

        dx = casadi.vertcat(dx0, dx1, dx2, dx3)
        dv = u
        return casadi.vertcat(dx, dv)

    # Constraints for Hermite-Simpson
    def hermite_spline(self, x1, x12, x2, f1, f2, h):
        return x12 == (x1 + x2) / 2 + (h / 8) * (f1 - f2)

    def simpson_integration(self, f1, f12, f2, x1, x2, h):
        return x2 == x1 + (h / 6) * (f1 + 4 * f12 + f2)

    def herm_simp_constraint(self, x, u, x_half, u_half):
        constraints = []
        for i in range(self.N):
            f1 = self.f(x[:, i], u[:, i])
            f12 = self.f(x_half[:, i], u_half[:, i])
            f2 = self.f(x[:, i+1], u[:, i+1])

            constraints.append(
                self.hermite_spline(
                    x[:, i], x_half[:, i], x[:, i+1], f1, f2, self.dt
                )
            )

            constraints.append(
                self.simpson_integration(
                    f1, f12, f2, x[:, i], x[:, i+1], self.dt
                )
            )

        return constraints

    def euler_constraint(self, x, u):
        constraints = []
        for k in range(self.N):
            x_next = x[:, k] + self.dt * self.f(x[:, k], u[:, k])
            constraints.append(x[:, k+1] == x_next)

        return constraints

    # Specify the boundary constraints
    def boundary_constraints(self, x, v, x_start, x_end, v_start, v_end):
        constraints = []
        for i in range(self.n_variables):
            constraints.extend([
                x[i, 0] == x_start[i], x[i, -1] == x_end[i],
                v[i, 0] == v_start[i], v[i, -1] == v_end[i]
            ])

        return constraints

    # Specify the robot's limits
    def system_constraints(self, x, v, u, x_limits, v_limits, u_limits):
        # limits are index by coordinate, then lower/upper
        constraints = []
        for i in range(self.n_variables):
            constraints.extend([
                self.opti.bounded(x_limits[i, 0], x[i, :], x_limits[i, 1]),
                self.opti.bounded(v_limits[i, 0], v[i, :], v_limits[i, 1]),
                self.opti.bounded(u_limits[i, 0], u[i, :], u_limits[i, 1])
            ])
        return constraints

    def solve(self):
        # Do all the things and run the program.
        self.opti.subject_to(self.T >= 0)
        self.opti.subject_to(self.T <= 50)
        self.opti.set_initial(self.T, 5)

        self.opti.set_initial(self.x[4, :], 0)
        xs = np.linspace(self.ini_pos[0], self.end_pos[0], self.N+1)
        ys = 8 / 9 * xs * (3 - xs)
        self.opti.set_initial(self.x[0, :], xs)
        self.opti.set_initial(self.x[1, :], ys)

        # Use the integrator constraint
        if self.integrator == "Euler":
            self.opti.subject_to(self.euler_constraint(self.x, self.u))
        else:
            self.opti.subject_to(self.herm_simp_constraint(
                self.x, self.u, self.x_half, self.u_half))

        # Limits of the robot
        BOUNDING_BOX = np.array([
            [-10, 10],
            [-10, 10],
            [0.22, 0.32],
            [-1e4, 1e4]
        ])

        SPEED_LIMIT = np.array([1, 0.3, 0.02, 1]) * \
            robot_sim.MPC_VELOCITY_MULTIPLIER
        SPEED_LIMIT = np.vstack((-SPEED_LIMIT, SPEED_LIMIT)).T
        THROTTLE_LIM = np.array([[-1, 1],
                                 [-1, 1],
                                 [-1, 1],
                                 [-2, 2]])

        self.opti.subject_to(
            self.system_constraints(
                self.x[:self.n_variables,
                       :], self.x[self.n_variables:, :], self.u,
                BOUNDING_BOX, SPEED_LIMIT, THROTTLE_LIM
            )
        )

        # Boundary Constraints
        self.opti.subject_to(
            self.boundary_constraints(
                self.x[:self.n_variables, :], self.x[self.n_variables:, :],
                self.ini_pos, self.end_pos, self.ini_vel, self.end_vel
            )
        )

        # Table condition
        self.opti.subject_to((self.x[0, :] - 1.5)
                             ** 2 + self.x[1, :]**2 >= (1)**2)

        p_opts = {}
        s_opts = {"max_iter": 1000}
        self.opti.solver('ipopt', p_opts, s_opts)

        self.sol = self.opti.solve()
        return self.sol

    def plot_solution(self):
        ax = plt.axes(projection='3d')
        points = self.sol.value(self.x)
        colours = points[3] * 2 / np.pi
        ax.scatter(points[0], points[1], points[2])  # c=colours
        # ax.scatter(points[4], points[5], points[6])  # c=points[7]

        plt.show()

        plt.plot(np.linspace(0, self.sol.value(self.T),
                 self.N+1), points[3] * 2 / np.pi)
        plt.show()

    def save_solution(self):
        points = self.sol.value(self.x)
        
        self.pos_traj = points[:4] ;
        self.vel_traj = points[4:] ;
        self.total_time = self.sol.value(self.T) ;
        
        np.save("opt_sol/pos.npy", points[:self.n_variables])        
        np.save("opt_sol/speed.npy", points[self.n_variables:])
        np.save("opt_sol/time.npy", self.sol.value(self.T))
        np.save("opt_sol/pos_vel.npy", points)

    """ #def interpolate(self, velocities, T, N, dt):
        Take the velocity trajectory and interpolate linearly.
        Interpolation is linear, so the velocity transforms from one
        to the next gradually. 
        dt is given by the solution to the optimization problem.
        Goal is to create a lookup table to get a velocity for any time t.
        Parameters: 
            dt (float): the time step size (T / N)
            velocities ((4, N), float): The velocty trajoectory to be followed.

        Returns:
            interpolated_v
        """


    def interpolate(self, velocities, T, t):
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
            velocity = self.interpolate(self.vel_traj, self.total_time, t)

        

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
               
        lin_speed, ang_speed = self.trajectory_map(t)

        # Convert body_height change to absolute height change.
        lin_speed[2] *= self.simulation_time_step
        

        
        return lin_speed, ang_speed



if __name__ == '__main__':
    solver = Optimizer(integrator="Euler", N=100)
    solver.solve()
        
   # solver.plot_solution()
    solver.save_solution()
    
    


"""
    TODO: Implement boundary parameters.

    TODO: Implement variable dynamic function.
"""
