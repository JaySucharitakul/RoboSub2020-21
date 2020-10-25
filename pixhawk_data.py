#!python3
# Author: Theodor Giles
# Created: 7/15/20
# Last Edited 8/12/20
# Description:
# gets data from the simulated/non-simulated pixhawk for attitude, positioning, and maybe some
# other cool tasks it can do

from dronekit import connect, VehicleMode
import time
import math

# sitl is basically a simulation, can be "ran" from any computer kinda? I will figure out a way to make an incorporated
# 3d python sim for managing all this at some point
# sitl = dronekit_sitl.start_default()  # (sitl.start)
# connection_string = sitl.connection_string()
GYRO: int = 0
POSITION: int = 1
YAW: int = 0
PITCH: int = 1
ROLL: int = 2
NORTH: int = 0
EAST: int = 1
DOWN: int = 2


def MapToAngle(x):
    in_min = -100.0
    in_max = 100.0
    out_min = 0.0
    out_max = 180.0
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min


class PixHawk:
    Gyro = [0.0, 0.0, 0.0]
    Position = [0.0, 0.0, 0.0]
    Angular_Motions = [[0.0, 0.0, 0.0], [0.0, 0.0, 0.0]]
    Measures = [[0.0, 0.0, 0.0], [0.0, 0.0, 0.0]]
    Error = [[0.0, 0.0, 0.0], [0.0, 0.0, 0.0]]
    Previous_Error = [[0.0, 0.0, 0.0], [0.0, 0.0, 0.0]]
    Error_Sum = [[0.0, 0.0, 0.0], [0.0, 0.0, 0.0]]
    Error_Delta = [[0.0, 0.0, 0.0], [0.0, 0.0, 0.0]]
        # gyro              position
    Kp = [[0.7, 0.5, 0.5], [0.3, 0.4, 0.4]]
    Ki = [[0.0, 0.00, 0.00], [0.1, 0.1, 0.1]]
    Kd = [[0.3, 0.3, 0.3], [0.1, 0.1, 0.1]]

    North_PID = 0.0
    North_P = 0.0
    North_I = 0.0
    North_D = 0.0

    East_PID = 0.0
    East_P = 0.0
    East_I = 0.0
    East_D = 0.0

    Down_PID = 0.0
    Down_P = 0.0
    Down_I = 0.0
    Down_D = 0.0

    Yaw_PID = 0.0
    Yaw_P = 0.0
    Yaw_I = 0.0
    Yaw_D = 0.0

    Pitch_PID = 0.0
    Pitch_P = 0.0
    Pitch_I = 0.0
    Pitch_D = 0.0

    Roll_PID = 0.0
    Roll_P = 0.0
    Roll_I = 0.0
    Roll_D = 0.0

    def __init__(self):
        # simulation started, connect vehicle object to fake vehicle ip
        # print("Connecting to vehicle on: %s" % (connection_string,))
        # self.vehicle = connect(connection_string,
        #                        wait_ready=True)
        self.vehicle = connect('/dev/serial/by-id/usb-3D_Robotics_PX4_FMU_v2.x_0-if00', wait_ready=True, baud=921600)
        # self.vehicle.reboot()
        # print("Wait 10 for rebooting PixHawk...")
        # time.sleep(10)
        # self.vehicle.wait_ready(True, timeout=300)
        # - wait_ready flag hold the program until all the parameters are been read (=, not .)
        # read info from vehicle
        self.vehicle.wait_ready('autopilot_version')
        print('Autopilot version: %s' % self.vehicle.version)

        # arm vehicle to see position
        self.vehicle.armed = True
        print('PixHawk armed...')
        # print("Wait 10 for GPS and Gyro initialization. ")
        # self.vehicle.send_calibrate_gyro()
        # time.sleep(10)

        # - Read the actual position North, East, and Down
        self.vehicle.add_attribute_listener('position', self.position_callback)  # -- message type, callback function
        self.UpdatePosition()
        self.StartingPosition = self.Position

        # - Read the actual attitude: Roll, Pitch, and Yaw
        self.vehicle.add_attribute_listener('attitude', self.gyro_callback)  # -- message type, callback function
        self.UpdateGyro()
        self.SubtractYaw = self.Gyro[YAW]
        self.StartingGyro = self.Gyro

        # - Read the actual depth:
        time.sleep(3)
        print("Starting gyro: ", self.Gyro)
        print("Starting position: ", self.Position)

    # updating gyro data from the pixhawk
    def gyro_callback(self, attr_name, value, extraval):
        self.UpdateGyro()

    # updating position data from the pixhawk
    def position_callback(self, attr_name, value, extraval):
        self.UpdatePosition()

    # parse gyro object data from pixhawk, can then pass to other programs
    def UpdateGyro(self):
        i = 0
        for CommaParse in str(self.vehicle.attitude).split(','):
            if CommaParse is not None:
                for EqualParse in CommaParse.split('='):
                    try:
                        if i == 1:
                            # (180 / math.pi)
                            # "angular motion" is basically just the difference between the
                            # last recorded gyro data and the current gyro data. The algorithm
                            # I looked at for this had an IMU that gave the raw gyro data, but
                            # the pixhawk does not return raw gyro data, so I have to improvise.
                            # basically the same value as self.Error, but I have to set it here
                            # so I can get the new gyro data right after.
                            self.Angular_Motions[GYRO][PITCH] = (
                                    self.Gyro[PITCH] - (float(EqualParse) * (180 / math.pi)))
                            self.Gyro[PITCH] = round(float(EqualParse) * (180 / math.pi), 5)
                        if i == 3:
                            self.Angular_Motions[GYRO][YAW] = (
                                    self.Gyro[YAW] - (float(EqualParse) * (180 / math.pi)))
                            self.Gyro[YAW] = round((float(EqualParse) * (180 / math.pi)), 5)
                        if i == 5:
                            self.Angular_Motions[GYRO][ROLL] = (
                                    self.Gyro[ROLL] - (float(EqualParse) * (180 / math.pi)))
                            self.Gyro[ROLL] = round(float(EqualParse) * (180 / math.pi), 5)
                        i += 1
                    except:
                        pass

    # parse position object data from pixhawk, can then pass to other programs
    def UpdatePosition(self):
        i = 0
        for CommaParse in str(self.vehicle.location.local_frame).split(','):
            if CommaParse is not None:
                for EqualParse in CommaParse.split('='):
                    try:
                        if i == 1:
                            self.Position[NORTH] = float(EqualParse)
                        if i == 3:
                            self.Position[EAST] = float(EqualParse)
                        if i == 5:
                            self.Position[DOWN] = float(EqualParse)
                        i += 1
                    except:
                        pass
                        # print("error reading position")
        # print(self.Position)

    # position read when starting the RoboSub
    def getStartingPosition(self):
        return self.StartingPosition

    # current position read
    def getPosition(self):
        return self.Position

    def getNorth(self):
        return self.Position[NORTH]

    def getEast(self):
        return self.Position[EAST]

    def getDown(self):
        return self.Position[DOWN]

    # gyro read when starting the RoboSub
    def getStartingGyro(self):
        return self.StartingGyro

    # current gyro read
    def getGyro(self):
        return self.Gyro

    def getPitch(self):
        return self.Gyro[PITCH]

    def getRoll(self):
        return self.Gyro[ROLL]

    def getYaw(self):
        return self.Gyro[YAW]

    # req for PID calculation
    def CalculateError(self, yawoffset, pitchoffset, rolloffset, northoffset, eastoffset, downoffset):

        # previous error for error delta
        # gyro
        self.Previous_Error[GYRO][YAW] = self.Error[GYRO][YAW]
        self.Previous_Error[GYRO][PITCH] = self.Error[GYRO][PITCH]
        self.Previous_Error[GYRO][ROLL] = self.Error[GYRO][ROLL]

        # position
        self.Previous_Error[POSITION][NORTH] = self.Error[POSITION][NORTH]
        self.Previous_Error[POSITION][EAST] = self.Error[POSITION][EAST]
        self.Previous_Error[POSITION][DOWN] = self.Error[POSITION][DOWN]

        # error for proportional control
        # gyro
        self.Error[GYRO][YAW] = self.Gyro[YAW] - yawoffset
        self.Error[GYRO][PITCH] = self.Gyro[PITCH] - pitchoffset
        self.Error[GYRO][ROLL] = self.Gyro[ROLL] - rolloffset

        # position
        self.Error[POSITION][NORTH] = self.Position[NORTH] - northoffset
        self.Error[POSITION][EAST] = self.Position[EAST] - eastoffset
        self.Error[POSITION][DOWN] = self.Position[DOWN] - downoffset

        # sum of error for integral
        # gyro
        self.Error_Sum[GYRO][YAW] = self.Error_Sum[GYRO][YAW] + self.Error[GYRO][YAW]
        self.Error_Sum[GYRO][PITCH] = self.Error_Sum[GYRO][PITCH] + self.Error[GYRO][PITCH]
        self.Error_Sum[GYRO][ROLL] = self.Error_Sum[GYRO][ROLL] + self.Error[GYRO][ROLL]

        # position
        self.Error_Sum[POSITION][NORTH] = self.Error_Sum[POSITION][NORTH] + self.Error[POSITION][NORTH]
        self.Error_Sum[POSITION][EAST] = self.Error_Sum[POSITION][EAST] + self.Error[POSITION][EAST]
        self.Error_Sum[POSITION][DOWN] = self.Error_Sum[POSITION][DOWN] + self.Error[POSITION][DOWN]

        # math for change in error to do derivative
        # gyro
        self.Error_Delta[GYRO][YAW] = self.Error[GYRO][YAW] - self.Previous_Error[GYRO][YAW]
        self.Error_Delta[GYRO][PITCH] = self.Error[GYRO][PITCH] - self.Previous_Error[GYRO][PITCH]
        self.Error_Delta[GYRO][ROLL] = self.Error[GYRO][ROLL] - self.Previous_Error[GYRO][ROLL]

        # position
        self.Error_Delta[POSITION][NORTH] = self.Error[POSITION][NORTH] - self.Previous_Error[POSITION][NORTH]
        self.Error_Delta[POSITION][EAST] = self.Error[POSITION][EAST] - self.Previous_Error[POSITION][EAST]
        self.Error_Delta[POSITION][DOWN] = self.Error[POSITION][DOWN] - self.Previous_Error[POSITION][DOWN]

    # pid calculation
    def PID(self):
        # Yaw PID variable setting
        self.Yaw_P = (self.Error[GYRO][YAW] * self.Kp[GYRO][YAW])
        self.Yaw_I = (self.Error_Sum[GYRO][YAW] * self.Ki[GYRO][YAW])
        self.Yaw_D = (self.Error_Delta[GYRO][YAW] * self.Kd[GYRO][YAW])
        self.Yaw_PID = self.Yaw_P + self.Yaw_I + self.Yaw_D

        # Pitch PID variable setting
        self.Pitch_P = (self.Error[GYRO][PITCH] * self.Kp[GYRO][PITCH])
        self.Pitch_I = (self.Error_Sum[GYRO][PITCH] * self.Ki[GYRO][PITCH])
        self.Pitch_D = (self.Error_Delta[GYRO][PITCH] * self.Kd[GYRO][PITCH])
        self.Pitch_PID = self.Pitch_P + self.Pitch_I + self.Pitch_D

        # Roll PID variable setting
        self.Roll_P = (self.Error[GYRO][ROLL] * self.Kp[GYRO][ROLL])
        self.Roll_I = (self.Error_Sum[GYRO][ROLL] * self.Ki[GYRO][ROLL])
        self.Roll_D = (self.Error_Delta[GYRO][ROLL] * self.Kd[GYRO][ROLL])
        self.Roll_PID = self.Roll_P + self.Roll_I + self.Roll_D

        # North PID variable setting
        self.North_P = (self.Error[POSITION][NORTH] * self.Kp[POSITION][NORTH])
        self.North_I = (self.Error_Sum[POSITION][NORTH] * self.Ki[POSITION][NORTH])
        self.North_D = (self.Error_Delta[POSITION][NORTH] * self.Kd[POSITION][NORTH])
        self.North_PID = self.North_P  # + self.North_I + self.North_D

        # East PID variable setting
        self.East_P = (self.Error[POSITION][EAST] * self.Kp[POSITION][EAST])
        self.East_I = (self.Error_Sum[POSITION][EAST] * self.Ki[POSITION][EAST])
        self.East_D = (self.Error_Delta[POSITION][EAST] * self.Kd[POSITION][EAST])
        self.East_PID = self.East_P  # + self.East_I + self.East_D

        # Down PID variable setting
        self.Down_P = (self.Error[POSITION][DOWN] * self.Kp[POSITION][DOWN])
        self.Down_I = (self.Error_Sum[POSITION][DOWN] * self.Ki[POSITION][DOWN])
        self.Down_D = (self.Error_Delta[POSITION][DOWN] * self.Kd[POSITION][DOWN])
        self.Down_PID = self.Down_P  # + self.Down_I + self.Down_D

    def getYawPID(self):
        return self.Yaw_PID

    def getPitchPID(self):
        return self.Pitch_PID

    def getRollPID(self):
        return self.Roll_PID

    def getNorthPID(self):
        return self.Yaw_PID

    def getEastPID(self):
        return self.Pitch_PID

    def getDownPID(self):
        return self.Roll_PID

    # end command/vehicle running
    def Terminate(self):
        self.vehicle.close()
