from __future__ import annotations

"""
Crazyflie Hardware-In-The-Loop Wrapper that uses the firmware Python bindings.

    2024 - Christian Hickstein (TU Berlin)
    
    inspired by crazyflie_sil.py, by Wolfgang Hönig (TU Berlin)
"""

"""
Create a thread that constantly updates the state to the hardware
"""

# NOTE: I GOT PWM VALUES. but when i turned the flie 90 degree (so motors were facing 90 degree to floor) it stoped and returned 0

# NEXT STEP: compile the firmware and flash it with defined CONFIG_ESTIMATOR_HIL_ENABLE

# Simulation Imports
import cffirmware as firm
import numpy as np
from numpy import sin, cos, tan
import rowan

from . import sim_data_types

# Logger Imports
import logging
import time
import random

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.syncLogger import SyncLogger

from threading import Timer
from cflib.utils import uri_helper
import threading

# Only output errors from the logging framework
logging.basicConfig(level=logging.ERROR)


class TrajectoryPolynomialPiece:

    def __init__(self, poly_x, poly_y, poly_z, poly_yaw, duration):
        self.poly_x = poly_x
        self.poly_y = poly_y
        self.poly_z = poly_z
        self.poly_yaw = poly_yaw
        self.duration = duration

def copy_svec(v):
    return firm.mkvec(v.x, v.y, v.z)    

class CrazyflieHILLogger: # TODO: change to work like the basicparam.py example (https://github.com/bitcraze/crazyflie-lib-python/blob/master/examples/parameters/basicparam.py) to also write the state
    def __init__(self, link_uri="usb://0", state=None):
        """ Initialize and run the example with the specified link_uri """

        # Software state of the crazyflie
        self.cf_state = state

        self._cf = Crazyflie(rw_cache='./cache')

        self.data = {"m1":0, "m2":0, "m3":0, "m4":0}

        # Connect some callbacks from the Crazyflie API
        self._cf.connected.add_callback(self._connected)
        self._cf.fully_connected.add_callback(self._fully_connected)
        self._cf.disconnected.add_callback(self._disconnected)
        self._cf.connection_failed.add_callback(self._connection_failed)
        self._cf.connection_lost.add_callback(self._connection_lost)

        print('Connecting to %s' % link_uri)

        # Try to connect to the Crazyflie
        self._cf.open_link(link_uri)

        # Variable used to keep main loop occupied until disconnect
        self.is_connected = True

        self._param_check_list = []
        self._param_groups = []

    def _connected(self, link_uri):
        """ This callback is called form the Crazyflie API when a Crazyflie
        has been connected and the TOCs have been downloaded. Parameter values are not downloaded yet."""
        print('Connected to %s' % link_uri)

        # Print the param TOC when using the print statements
        p_toc = self._cf.param.toc.toc
        for group in sorted(p_toc.keys()):
            #print('{}'.format(group))
            for param in sorted(p_toc[group].keys()):
                #print('\t{}'.format(param))
                self._param_check_list.append('{0}.{1}'.format(group, param))
            self._param_groups.append('{}'.format(group))
            # For every group, register the callback
            self._cf.param.add_update_callback(group=group, name=None, cb=self._param_callback)

    def _fully_connected(self, link_uri):
        """This callback is called when the Crazyflie has been connected and all parameters have been
        downloaded. It is now OK to set and get parameters."""
        print(f'Parameters downloaded to {link_uri}')
    
        print('Setting the estimator of the Crazyflie to HIL')
        self._cf.param.set_value('stabilizer.estimator', '2')

        print('Starting Logger for PWM values!')
        # Start logger
        self._lg_pwm = LogConfig(name='PWM', period_in_ms=10)
        self._lg_pwm.add_variable('pwm.m1_pwm', 'uint16_t')
        self._lg_pwm.add_variable('pwm.m2_pwm', 'uint16_t')
        self._lg_pwm.add_variable('pwm.m3_pwm', 'uint16_t')
        self._lg_pwm.add_variable('pwm.m4_pwm', 'uint16_t')
        #self._lg_pwm.add_variable('kalman.initialX', 'float')

        try:
            self._cf.log.add_config(self._lg_pwm)
            # This callback will receive the data
            self._lg_pwm.data_received_cb.add_callback(self._pwm_log_data)
            # This callback will be called on errors
            self._lg_pwm.error_cb.add_callback(self._pwm_log_error)
            # Start the logging
            self._lg_pwm.start()
        except KeyError as e:
            print('Could not start log configuration,'
                  '{} not found in TOC'.format(str(e)))
        except AttributeError:
            print('Could not add Stabilizer log config, bad configuration.')


        """ # We can get a parameter value directly without using a callback
        value = self._cf.param.get_value('pid_attitude.pitch_kd')
        print(f'Value read with get() is {value}')

        # When a parameter is set, the callback is called with the new value
        self._cf.param.add_update_callback(group='pid_attitude', name='pitch_kd', cb=self._a_pitch_kd_callback)
        # When setting a value the parameter is automatically read back
        # and the registered callbacks will get the updated value
        self._cf.param.set_value('kalman.initialX', 0.1234) """

    def _param_callback(self, name, value):
        """Generic callback registered for all the groups"""
        print('{0}: {1}'.format(name, value))

        # Remove each parameter from the list when fetched
        self._param_check_list.remove(name)
        if len(self._param_check_list) == 0:
            print('Have fetched all parameter values.')

            # Remove all the group callbacks
            for g in self._param_groups:
                self._cf.param.remove_update_callback(group=g, cb=self._param_callback)

    def _connection_failed(self, link_uri, msg):
        """Callback when connection initial connection fails (i.e no Crazyflie
        at the specified address)"""
        print('Connection to %s failed: %s' % (link_uri, msg))
        self.is_connected = False

    def _connection_lost(self, link_uri, msg):
        """Callback when disconnected after a connection has been made (i.e
        Crazyflie moves out of range)"""
        print('Connection to %s lost: %s' % (link_uri, msg))

    def _disconnected(self, link_uri):
        """Callback when the Crazyflie is disconnected (called in all cases)"""
        print('Disconnected from %s' % link_uri)
        self.is_connected = False


    def _pwm_log_error(self, logconf, msg):
        """Callback from the log API when an error occurs"""
        print('Error when logging %s: %s' % (logconf.name, msg))

    def _pwm_log_data(self, timestamp, data, logconf):
        """Callback from a the log API when data arrives"""
        # Save the generated PWM values
        self.data["m1"] = data["pwm.m1_pwm"]
        self.data["m2"] = data["pwm.m2_pwm"]
        self.data["m3"] = data["pwm.m3_pwm"]
        self.data["m4"] = data["pwm.m4_pwm"]
        
        # Send new position/rotation to Hardware
        self._cf.param.set_value('hil.simPosX', self.cf_state.position.x)
        self._cf.param.set_value('hil.simPosY', self.cf_state.position.y)
        self._cf.param.set_value('hil.simPosZ', self.cf_state.position.z)

        self._cf.param.set_value('hil.simRotPitch', self.cf_state.attitude.pitch)
        self._cf.param.set_value('hil.simRotRoll', self.cf_state.attitude.roll)
        self._cf.param.set_value('hil.simRotYaw', self.cf_state.attitude.yaw)

class CrazyflieHIL:

    # Flight modes.
    MODE_IDLE = 0
    MODE_HIGH_POLY = 1
    MODE_LOW_FULLSTATE = 2
    MODE_LOW_POSITION = 3
    MODE_LOW_VELOCITY = 4

    def __init__(self, name, initialPosition, controller_name, time_func):
        # INIT HW CRAZYFLIE and USB connection
        # Initialize the low-level drivers
        cflib.crtp.init_drivers()
        self.cf_hil_logger = CrazyflieHILLogger(state=self.state)

        self.busy = False # busy flag to determine if the action was finished

        # INIT Simulation
        # Core.
        self.name = name
        self.groupMask = 0 # Group mask is currently not used (only 1 USB connection to a crazyflie is supported)
        self.initialPosition = np.array(initialPosition)
        self.time_func = time_func

        # Commander.
        self.mode = CrazyflieHIL.MODE_IDLE
        self.planner = firm.planner()
        firm.plan_init(self.planner)
        self.trajectories = {}

        # previous state for HL commander
        self.cmdHl_pos = firm.mkvec(*initialPosition)
        self.cmdHl_vel = firm.vzero()
        self.cmdHl_yaw = 0

        # current setpoint
        self.setpoint = firm.setpoint_t()

        # latest sensor values.
        self.state = firm.state_t()
        self.state.position.x = self.initialPosition[0]
        self.state.position.y = self.initialPosition[1]
        self.state.position.z = self.initialPosition[2]
        self.state.velocity.x = 0
        self.state.velocity.y = 0
        self.state.velocity.z = 0
        self.state.attitude.roll = 0
        self.state.attitude.pitch = -0  # WARNING: this is in the legacy coordinate system
        self.state.attitude.yaw = 0

        self.cf_hil_logger.cf_state = self.state # set the state accessable for the hil logger

        self.sensors = firm.sensorData_t()
        self.sensors.gyro.x = 0
        self.sensors.gyro.y = 0
        self.sensors.gyro.z = 0

        self.control = firm.control_t()
        self.motors_thrust_uncapped = firm.motors_thrust_uncapped_t()
        self.motors_thrust_pwm = firm.motors_thrust_pwm_t()

        self.controller_name = controller_name

        # set up controller
        if controller_name == 'none':
            self.controller = None
        elif controller_name == 'pid':
            firm.controllerPidInit()
            self.controller = firm.controllerPid
        elif controller_name == 'mellinger':
            self.mellinger_control = firm.controllerMellinger_t()
            firm.controllerMellingerInit(self.mellinger_control)
            self.controller = firm.controllerMellinger
        elif controller_name == 'brescianini':
            firm.controllerBrescianiniInit()
            self.controller = firm.controllerBrescianini
        else:
            raise ValueError('Unknown controller {}'.format(controller_name))

        self.busy = False # Variable to track if the last command was finished

    def set_motor_pwm(self):
        for i in range(50000, 60000, 1):
            time.sleep(0.001)
            print(i)
            self.motors_thrust_pwm.motors.m1 = i
            self.motors_thrust_pwm.motors.m2 = i
            self.motors_thrust_pwm.motors.m3 = i
            self.motors_thrust_pwm.motors.m4 = i
        self.motors_thrust_pwm.motors.m1 = 0
        self.motors_thrust_pwm.motors.m2 = 0
        self.motors_thrust_pwm.motors.m3 = 0
        self.motors_thrust_pwm.motors.m4 = 0

    def takeoff(self, targetHeight, duration, groupMask=0):
        self.mode = CrazyflieHIL.MODE_HIGH_POLY
        thread = threading.Thread(target=self._takeoff_thread, args=(targetHeight, duration, groupMask))
        thread.start()
            
    def land(self, targetHeight, duration, groupMask=0):
        thread = threading.Thread(target=self._goTo_thread, args=(targetHeight, duration, groupMask))
        thread.start()

    def goTo(self, goal, yaw, duration, relative=False, groupMask=0): #Call in crazyflie_server.py L:254
        """ # TODO: maybe add a delay between write and read
        
        cf = Crazyflie(rw_cache='./cache')
        with SyncCrazyflie(self.available_cfs[0][0], cf=cf) as scf:
            cf.high_level_commander.go_to(goal[0], goal[1], goal[2], yaw, duration, relative) #Send goTo via cfLib to Crazyflie
        
        print("sent goTo command")
        # update simulation until crazyflie reaches destination
        while (self.state.position.x != goal[0]) or (self.state.position.y != goal[1]) or (self.state.position.z != goal[2]):
            # Nicht threaden sondern immer nur in funktionen nutzen
            data = self._get_motor_data()
            print(data) """
        if self.mode != CrazyflieHIL.MODE_HIGH_POLY:
            # We need to update to the latest firmware that has go_to_from.
            raise ValueError('goTo from low-level modes not yet supported.')
        thread = threading.Thread(target=self._goTo_thread, args=(goal, yaw, duration, groupMask))
        thread.start()

    def setState(self, state: sim_data_types.State): # Updates the state (called after a step by the crazyflie_server in _timer_callback)
        self.state.position.x = state.pos[0]
        self.state.position.y = state.pos[1]
        self.state.position.z = state.pos[2]

        self.state.velocity.x 
        self.state.velocity.y = state.vel[1]
        self.state.velocity.z = state.vel[2]

        rpy = np.degrees(rowan.to_euler(state.quat, convention='xyz'))
        # Note, legacy coordinate system, so invert pitch
        self.state.attitude.roll = rpy[0]
        self.state.attitude.pitch = -rpy[1]
        self.state.attitude.yaw = rpy[2]

        self.state.attitudeQuaternion.w = state.quat[0]
        self.state.attitudeQuaternion.x = state.quat[1]
        self.state.attitudeQuaternion.y = state.quat[2]
        self.state.attitudeQuaternion.z = state.quat[3]

        # omega is part of sensors, not of the state
        self.sensors.gyro.x = np.degrees(state.omega[0])
        self.sensors.gyro.y = np.degrees(state.omega[1])
        self.sensors.gyro.z = np.degrees(state.omega[2])

    def executeController(self):
        if self.controller is None:
            return None

        if self.mode == CrazyflieHIL.MODE_IDLE:
            return sim_data_types.Action([0, 0, 0, 0])

        time_in_seconds = self.time_func()
        # ticks is essentially the time in milliseconds as an integer
        tick = int(time_in_seconds * 1000)
        if self.controller_name != 'mellinger':
            self.controller(self.control, self.setpoint, self.sensors, self.state, tick)
        else:
            self.controller(
                self.mellinger_control,
                self.control,
                self.setpoint,
                self.sensors,
                self.state,
                tick)
        return self._fwcontrol_to_sim_data_types_action()

    # private functions
    def _takeoff_thread(self, targetHeight, duration, groupMask=0):
        # TODO: 
        # 1. send via cflib the takeoff command (done)
        # 2. read motor data (done)
        # 3. set the self.motors_thrust_pwm.motors.mX variables to the read values (done)
        # 4. calculate new position (todo)
        # 5. feed back position to crazyflie (todo)
        # 6. if targetheight is reached end else go to 2.
        while (self.busy == True): # replace with mutex
            time.sleep(0.1)
        
        self.busy = True

        self.cf_hil_logger._cf.high_level_commander.takeoff(targetHeight, duration)
        time.sleep(0.1)
        
        while (self.state.position.z != targetHeight): #NOTE: maybe add a threshold
            data = self._get_motor_data()
            self._set_sim_motors(data)
            print(data) # debug print
        
        self.busy = False

    def _land_thread(self, targetHeight, duration, groupMask=0):
        self.cf_hil_logger._cf.high_level_commander.land(targetHeight, duration)
        while (self.state.position.z != targetHeight): #NOTE: maybe add a threshold
            data = self._get_motor_data()
            self._set_sim_motors(data)
            print(data) # debug print
        

    def _goTo_thread(self, goal, yaw, duration, relative=False, groupMask=0):
        self.cf_hil_logger._cf.high_level_commander.go_to(goal[0], goal[1], goal[2], yaw, duration, relative)
        while (self.state.position.x != goal[0] and self.state.position.y != goal[1] and self.state.position.z != goal[2]):
            data = self._get_motor_data()
            self._set_sim_motors(data)
            print(data) # debug print

    def _fwcontrol_to_sim_data_types_action(self):

        """ firm.powerDistribution(self.control, self.motors_thrust_uncapped)
        firm.powerDistributionCap(self.motors_thrust_uncapped, self.motors_thrust_pwm)"""
    
        #print(self.motors_thrust_pwm.motors.m1)

        # self.motors_thrust_pwm.motors.m{1,4} contain the PWM
        # convert PWM -> RPM
        def pwm_to_rpm(pwm):
            # polyfit using data and scripts from https://github.com/IMRCLab/crazyflie-system-id
            if pwm < 10000:
                return 0
            p = [3.26535711e-01, 3.37495115e+03]
            return np.polyval(p, pwm)

        def pwm_to_force(pwm):
            # polyfit using data and scripts from https://github.com/IMRCLab/crazyflie-system-id
            p = [1.71479058e-09,  8.80284482e-05, -2.21152097e-01]
            force_in_grams = np.polyval(p, pwm)
            force_in_newton = force_in_grams * 9.81 / 1000.0
            return np.maximum(force_in_newton, 0)

        return sim_data_types.Action(
            [pwm_to_rpm(self.motors_thrust_pwm.motors.m1),
             pwm_to_rpm(self.motors_thrust_pwm.motors.m2),
             pwm_to_rpm(self.motors_thrust_pwm.motors.m3),
             pwm_to_rpm(self.motors_thrust_pwm.motors.m4)])

    def getSetpoint(self):
        if self.mode == CrazyflieHIL.MODE_HIGH_POLY:
            # See logic in crtp_commander_high_level.c
            ev = firm.plan_current_goal(self.planner, self.time_func())
            if firm.is_traj_eval_valid(ev):
                self.setpoint.position.x = ev.pos.x
                self.setpoint.position.y = ev.pos.y
                self.setpoint.position.z = ev.pos.z
                self.setpoint.velocity.x = ev.vel.x
                self.setpoint.velocity.y = ev.vel.y
                self.setpoint.velocity.z = ev.vel.z
                self.setpoint.attitude.yaw = np.degrees(ev.yaw)
                self.setpoint.attitudeRate.roll = np.degrees(ev.omega.x)
                self.setpoint.attitudeRate.pitch = np.degrees(ev.omega.y)
                self.setpoint.attitudeRate.yaw = np.degrees(ev.omega.z)
                self.setpoint.mode.x = firm.modeAbs
                self.setpoint.mode.y = firm.modeAbs
                self.setpoint.mode.z = firm.modeAbs
                self.setpoint.mode.roll = firm.modeDisable
                self.setpoint.mode.pitch = firm.modeDisable
                self.setpoint.mode.yaw = firm.modeAbs
                self.setpoint.mode.quat = firm.modeDisable
                self.setpoint.acceleration.x = ev.acc.x
                self.setpoint.acceleration.y = ev.acc.y
                self.setpoint.acceleration.z = ev.acc.z

                self.cmdHl_pos = copy_svec(ev.pos)
                self.cmdHl_vel = copy_svec(ev.vel)
                self.cmdHl_yaw = ev.yaw

        return self._fwsetpoint_to_sim_data_types_state(self.setpoint)

    @staticmethod
    def _fwsetpoint_to_sim_data_types_state(fwsetpoint):
        pos = np.array([fwsetpoint.position.x, fwsetpoint.position.y, fwsetpoint.position.z])
        vel = np.array([fwsetpoint.velocity.x, fwsetpoint.velocity.y, fwsetpoint.velocity.z])
        acc = np.array([
            fwsetpoint.acceleration.x,
            fwsetpoint.acceleration.y,
            fwsetpoint.acceleration.z])
        omega = np.radians(np.array([
            fwsetpoint.attitudeRate.roll,
            fwsetpoint.attitudeRate.pitch,
            fwsetpoint.attitudeRate.yaw]))

        if fwsetpoint.mode.quat == firm.modeDisable:
            # compute rotation based on differential flatness
            thrust = acc + np.array([0, 0, 9.81])
            z_body = thrust / np.linalg.norm(thrust)
            yaw = np.radians(fwsetpoint.attitude.yaw)
            x_world = np.array([np.cos(yaw), np.sin(yaw), 0])
            y_body = np.cross(z_body, x_world)
            # Mathematically not needed. This addresses numerical issues to ensure R is orthogonal
            y_body /= np.linalg.norm(y_body)
            x_body = np.cross(y_body, z_body)
            # Mathematically not needed. This addresses numerical issues to ensure R is orthogonal
            x_body /= np.linalg.norm(x_body)
            R = np.column_stack([x_body, y_body, z_body])
            quat = rowan.from_matrix(R)
        else:
            quat = fwsetpoint.attitudeQuaternion

        return sim_data_types.State(pos, vel, quat, omega)

    def _get_motor_data(self):
        return self.cf_hil_logger.data

    def _set_sim_motors(self, data):
        self.motors_thrust_pwm.motors.m1 = data["m1"]
        self.motors_thrust_pwm.motors.m2 = data["m2"]
        self.motors_thrust_pwm.motors.m3 = data["m3"]
        self.motors_thrust_pwm.motors.m4 = data["m4"]

    def _set_sim_crazyflie_pos(self, position: list, velocity: list, rotation: list):
        self.state.position.x = position[0]
        self.state.position.y = position[1]
        self.state.position.z = position[2]

        self.state.velocity.x = velocity[0]
        self.state.velocity.y = velocity[1]
        self.state.velocity.z = velocity[2]

        self.state.attitude.roll    = rotation[0]
        self.state.attitude.pitch   = rotation[1]
        self.state.attitude.yaw     = rotation[2]

    def pwm_to_force(self, pwm):
        # polyfit using data and scripts from https://github.com/IMRCLab/crazyflie-system-id
        p = [1.71479058e-09,  8.80284482e-05, -2.21152097e-01]
        force_in_grams = np.polyval(p, pwm)
        force_in_newton = force_in_grams * 9.81 / 1000.0
        return np.maximum(force_in_newton, 0)