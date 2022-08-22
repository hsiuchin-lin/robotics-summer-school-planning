from mpc_controller import com_velocity_estimator
from mpc_controller import gait_generator as gait_generator_lib
from mpc_controller import locomotion_controller
from mpc_controller import openloop_gait_generator
from mpc_controller import raibert_swing_leg_controller
from mpc_controller import torque_stance_leg_controller
from mpc_controller import a1_sim as robot_sim

#FLAGS = flags.FLAGS

_NUM_SIMULATION_ITERATION_STEPS = 300


_STANCE_DURATION_SECONDS = [
    0.25
] * 4  # For faster trotting (v > 1.5 ms reduce this to 0.13s).

# Trotting
_DUTY_FACTOR = [0.6] * 4
_INIT_PHASE_FULL_CYCLE = [0.9, 0, 0, 0.9]
_MAX_TIME_SECONDS = 50

_INIT_LEG_STATE = (
    gait_generator_lib.LegState.SWING,
    gait_generator_lib.LegState.STANCE,
    gait_generator_lib.LegState.STANCE,
    gait_generator_lib.LegState.SWING,
)



class TrotController():
  
  # def __init__(self):
    
    def _setup_controller(self, robot):
    
        """Demonstrates how to create a locomotion controller."""
        desired_speed = (0, 0)
        desired_twisting_speed = 0

        gait_generator = openloop_gait_generator.OpenloopGaitGenerator(
            robot,
            stance_duration   = _STANCE_DURATION_SECONDS,
            duty_factor       = _DUTY_FACTOR,
            initial_leg_phase = _INIT_PHASE_FULL_CYCLE,
            initial_leg_state = _INIT_LEG_STATE)
            
        state_estimator = com_velocity_estimator.COMVelocityEstimator(robot, window_size=20)

        sw_controller = raibert_swing_leg_controller.RaibertSwingLegController(
            robot,
            gait_generator,
            state_estimator,
            desired_speed = desired_speed,
            desired_twisting_speed = desired_twisting_speed,
            desired_height = robot_sim.MPC_BODY_HEIGHT,
            foot_clearance = 0.01)

        st_controller = torque_stance_leg_controller.TorqueStanceLegController(
            robot,
            gait_generator,
            state_estimator,
            desired_speed = desired_speed,
            desired_twisting_speed = desired_twisting_speed,
            desired_body_height = robot_sim.MPC_BODY_HEIGHT,
            body_mass = robot_sim.MPC_BODY_MASS,
            body_inertia = robot_sim.MPC_BODY_INERTIA)

        self.locomotion_controller = locomotion_controller.LocomotionController(
            robot = robot,
            gait_generator = gait_generator,
            state_estimator = state_estimator,
            swing_leg_controller = sw_controller,
            stance_leg_controller = st_controller,
            clock = robot.GetTimeSinceReset)
            
        self.locomotion_controller.reset()
                
    def update (self, lin_speed, ang_speed):
        self.locomotion_controller.swing_leg_controller.desired_speed          = lin_speed
        self.locomotion_controller.swing_leg_controller.desired_twisting_speed = ang_speed
        self.locomotion_controller.stance_leg_controller.desired_speed         = lin_speed
        self.locomotion_controller.stance_leg_controller.desired_twisting_speed= ang_speed
        self.locomotion_controller.update()
        
        return self.locomotion_controller.get_action()
