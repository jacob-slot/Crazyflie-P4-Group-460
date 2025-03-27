# -*- coding: utf-8 -*-
#
# ,---------,       ____  _ __
# |  ,-^-,  |      / __ )(_) /_______________ _____  ___
# | (  O  ) |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
# | / ,--'  |    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
#    +------`   /_____/_/\__/\___/_/   \__,_/ /___/\___/
#
# Copyright (C) 2023 Bitcraze AB
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, in version 3.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program. If not, see <http://www.gnu.org/licenses/>.
"""
Example of how to connect to a motion capture system and feed the position to a
Crazyflie, using the motioncapture library. The motioncapture library supports all major mocap systems and provides
a generalized API regardless of system type.
The script uses the high level commander to upload a trajectory to fly a figure 8.

Set the uri to the radio settings of the Crazyflie and modify the
mocap setting matching your system.
"""
import time
from threading import Thread
import numpy as np

import motioncapture

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.positioning.motion_commander import MotionCommander
from cflib.crazyflie.mem import MemoryElement
from cflib.crazyflie.mem import Poly4D
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.utils import uri_helper
from cflib.utils.reset_estimator import reset_estimator

# URI to the Crazyflie to connect to
uri = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E7E7')

# The host name or ip address of the mocap system
host_name = '192.168.1.33'

# The type of the mocap system
# Valid options are: 'vicon', 'optitrack', 'optitrack_closed_source', 'qualisys', 'nokov', 'vrpn', 'motionanalysis'
mocap_system_type = 'vicon'

# The name of the rigid body that represents the Crazyflie
rigid_body_name = 'Crazyflie'

# True: send position and orientation; False: send position only
send_full_pose = True

# When using full pose, the estimator can be sensitive to noise in the orientation data when yaw is close to +/- 90
# degrees. If this is a problem, increase orientation_std_dev a bit. The default value in the firmware is 4.5e-3.
orientation_std_dev = 8.0e-3

# The trajectory to fly
# See https://github.com/whoenig/uav_trajectories for a tool to generate
# trajectories


class MocapWrapper(Thread):
    def __init__(self, body_name):
        Thread.__init__(self)

        self.body_name = body_name
        self.on_pose = None
        self._stay_open = True

        self.start()

    def close(self):
        self._stay_open = False

    def run(self):
        mc = motioncapture.connect(mocap_system_type, {'hostname': host_name})
        while self._stay_open:
            mc.waitForNextFrame()
            for name, obj in mc.rigidBodies.items():
                if name == self.body_name:
                    if self.on_pose:
                        pos = obj.position
                        quat = obj.rotation
                        global latest_pose
                        latest_pose = [pos[0], pos[1], pos[2], quat.x, quat.y, quat.z, quat.w]
                        self.on_pose([pos[0], pos[1], pos[2], obj.rotation])


def log_data_callback(timestamp, data, x):

    # print("logging")
    # Extract the motor values
    log_data = np.zeros(13)
    log_data[0] = data["motor.m1"]
    log_data[1] = data["motor.m2"]
    log_data[2] = data["motor.m3"]
    log_data[3] = data["motor.m4"]
    log_data[4] = data["pm.vbatMV"]
    log_data[5] = latest_pose[0] # x
    log_data[6] = latest_pose[1] # y
    log_data[7] = latest_pose[2] # z
    log_data[8] = latest_pose[3] # quaternion x
    log_data[9] = latest_pose[4] # quaternion y
    log_data[10] = latest_pose[5] # quaternion z
    log_data[11] = latest_pose[6] # quaternion w
    log_data[12] = timestamp

    print(log_data)

    # save data as a new line in a csv file
    with open('data.csv', 'a') as f:
        for i in range(0, len(log_data)):
            f.write(str(log_data[i]))
            if i < len(log_data) - 1:
                f.write(',')
        f.write('\n')



def send_extpose_quat(cf, x, y, z, quat):
    """
    Send the current Crazyflie X, Y, Z position and attitude as a quaternion.
    This is going to be forwarded to the Crazyflie's position estimator.
    """
    if send_full_pose:
        cf.extpos.send_extpose(x, y, z, quat.x, quat.y, quat.z, quat.w)
    else:
        cf.extpos.send_extpos(x, y, z)


def adjust_orientation_sensitivity(cf):
    cf.param.set_value('locSrv.extQuatStdDev', orientation_std_dev)


def activate_kalman_estimator(cf):
    cf.param.set_value('stabilizer.estimator', '2')

    # Set the std deviation for the quaternion data pushed into the
    # kalman filter. The default value seems to be a bit too low.
    cf.param.set_value('locSrv.extQuatStdDev', 0.06)


def activate_mellinger_controller(cf):
    cf.param.set_value('stabilizer.controller', '2')


def upload_trajectory(cf, trajectory_id, trajectory):
    trajectory_mem = cf.mem.get_mems(MemoryElement.TYPE_TRAJ)[0]
    trajectory_mem.trajectory = []

    total_duration = 0
    for row in trajectory:
        duration = row[0]
        x = Poly4D.Poly(row[1:9])
        y = Poly4D.Poly(row[9:17])
        z = Poly4D.Poly(row[17:25])
        yaw = Poly4D.Poly(row[25:33])
        trajectory_mem.trajectory.append(Poly4D(duration, x, y, z, yaw))
        total_duration += duration

    trajectory_mem.write_data_sync()
    cf.high_level_commander.define_trajectory(trajectory_id, 0, len(trajectory_mem.trajectory))
    return total_duration

def controll_commands(cf):
    commander = cf.high_level_commander
    commander.takeoff(1.0, 2.0)
    time.sleep(3.0)

    for i in range(0, 3):
        commander.go_to(0, 0, 1, 0.0, 1.0)
        time.sleep(11.0)
        commander.go_to(0, 0, 1.5, 0.0, 1.0)
        time.sleep(11.0)
        
    commander.go_to(0, 0, 1, 0.0, 1.0)
    time.sleep(11.0)
    commander.land(0.0, 4.0)
    time.sleep(5.0)


if __name__ == '__main__':
    cflib.crtp.init_drivers()

    # Connect to the mocap system
    mocap_wrapper = MocapWrapper(rigid_body_name)

    with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf:
        #cf = scf.cf
        trajectory_id = 1

        # Set up a callback to handle data from the mocap system
        mocap_wrapper.on_pose = lambda pose: send_extpose_quat(scf.cf, pose[0], pose[1], pose[2], pose[3])

        adjust_orientation_sensitivity(scf.cf)
        activate_kalman_estimator(scf.cf)
        # activate_mellinger_controller(cf)

        reset_estimator(scf.cf)

        # Arm the Crazyflie
        scf.cf.platform.send_arming_request(True)
        time.sleep(1.0)

        # Set up logging
        logconf = LogConfig(name='Motors', period_in_ms=20)
        logconf.add_variable('motor.m1', 'uint16_t')
        logconf.add_variable('motor.m2', 'uint16_t')
        logconf.add_variable('motor.m3', 'uint16_t')
        logconf.add_variable('motor.m4', 'uint16_t')
        logconf.add_variable('pm.vbatMV', 'uint16_t')
        scf.cf.log.add_config(logconf)
        logconf.data_received_cb.add_callback(log_data_callback)

        logconf.start()
        controll_commands(scf.cf)

    mocap_wrapper.close()
