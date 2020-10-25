#!python3
# Author: Theodor Giles
# Created: 8/7/20
# Last Edited 8/12/20
# Description:
# This program manages the commands/movement/physical control of the RoboSub V1
#
import time
import random
import pyfirmata
import math
from threading import Thread

# ROBOSUB
A_TARGET = 1
A_POSITION = 2
A_GYRO = 3

MAX_THROTTLE = 35

GENERAL_THROTTLE = 17.5


class MovementCommander:

    # initialize everything to supposed starting position
    def __init__(self, usingvision, usingpixhawk, usingsim):
        # setting up board serial port
        print("Communicating with Arduino...")
        # something so the serial buffer doesn't overflow
        self.board = pyfirmata.ArduinoMega(
            '/dev/serial/by-id/usb-Arduino__www.arduino.cc__0042_55839313438351417131-if00')
        self.iterator = pyfirmata.util.Iterator(self.board)
        self.iterator.start()

        print("Communication with Arduino started...")

        self.YawOffset = 0
        self.PitchOffset = 0
        self.RollOffset = 0

        self.EastOffset = 0
        self.NorthOffset = 0
        self.DownOffset = 0

        self.UsingVision = usingvision
        self.UsingPixHawk = usingpixhawk
        self.UsingSim = usingsim
        if self.UsingVision:
            import Theos_Really_Good_Detection_Script as obj_det
            self.VisionAI = obj_det.Detector("TensorFlow_Graph/Tflite", False)
            print("MovementCommander is using Vision AI...")
        else:
            print("MovementCommander is not using Vision AI...")

        if self.UsingPixHawk:
            from pixhawk_data import PixHawk
            self.PixHawk = PixHawk()
            print("MovementCommander is using PixHawk...")
        else:
            print("MovementCommander is not using PixHawk...")
        if self.UsingSim:
            from advancedtelemetry import Telemetry
            self.TelemetrySim = Telemetry()
            print("MovementCommander is using Telemetry...")
        else:
            print("MovementCommander is not using Telemetry...")
        # thruster hardpoint classes
        self.ThrusterLB = ThrusterDriver(2, self.board)  # left back
        self.ThrusterLF = ThrusterDriver(4, self.board)  # left front
        self.ThrusterRB = ThrusterDriver(3, self.board)  # right back
        self.ThrusterRF = ThrusterDriver(5, self.board)  # right front
        self.ThrusterBL = ThrusterDriver(6, self.board)  # back left
        self.ThrusterBR = ThrusterDriver(7, self.board)  # back right
        self.ThrusterFL = ThrusterDriver(8, self.board)  # front left
        self.ThrusterFR = ThrusterDriver(9, self.board)  # front right
        print("Wait 3 to arm thrusters...")
        time.sleep(3)

        # power values to set to the thruster hardpoints
        # horizontally oriented
        self.PowerLB = 0
        self.PowerLF = 0
        self.PowerRB = 0
        self.PowerRF = 0
        # vertically oriented
        self.PowerBL = 0
        self.PowerBR = 0
        self.PowerFR = 0
        self.PowerFL = 0

        # initialize thruster values to brake (self.PowerXX set to 0^)
        self.ThrusterLB.SetSpeed(self.PowerLB)
        self.ThrusterLF.SetSpeed(self.PowerLF)
        self.ThrusterRB.SetSpeed(self.PowerRB)
        self.ThrusterRF.SetSpeed(self.PowerRF)
        self.ThrusterBL.SetSpeed(self.PowerBL)
        self.ThrusterBR.SetSpeed(self.PowerBR)
        self.ThrusterFL.SetSpeed(self.PowerFL)
        self.ThrusterFR.SetSpeed(self.PowerFR)

        # string list of movement commands, because I thought I'd make
        # the index number of each command streamlined with other
        # functions, but it seems a bit detrimental the more I work with
        # it.

        # basic: these commands are just normal up, down, turn, etc.
        self.BASIC_MOVEMENT_COMMANDS = [
            "FORWARD",
            "REVERSE",
            "LEFT",
            "RIGHT",
            "CLOCKWISE TURN",
            "COUNTERCLOCKWISE TURN",
            "DIVE",
            "SURFACE",
            "IDLE"]

        # advanced: these commands are much more complicated, will need to
        # develop pathing and a lot of vision/gyro/position integration
        self.ADVANCED_MOVEMENT_COMMANDS = [
            "LOG START POINT",
            "RETURN TO START",
        ]
        self.TARGET_MOVEMENT_COMMANDS = [
            "MOVE TO TARGET",
            "RAM TARGET",
            "FIRE AT TARGET"
        ]
        self.GYRO_TO = "GYRO TO"
        self.POSITION_TO = "POSITION TO"
        # currently only for firing torpedoes, maybe a claw action later on?
        self.SUPPLEMENTARY_COMMANDS = [
            "FIRE TORPEDO"
        ]
        # name of object to target sent to TF/openCV AI
        self.TO_TARGET = ""

        # possible targets, matches up with labelmap.txt to make easier
        self.POSSIBLE_TARGETS = [
            "red_buoy",
            "blue_buoy",
            "green_buoy",
            "orange_buoy",
            "gate"]
        self.TargetList = []
        print("MovementCommander initialized...")

    # Concept code, basically for checking if the Sub has already seen the detected object.
    def IsTargetInMemory(self, label, x, y, z):
        NewTarget = [label, x, y, z]
        InMemory = False
        for target in self.TargetList:
            # Determining how far something could be next to the said target,
            DistanceConfidence = math.sqrt(target[4]) * 1.5
            WithinX = abs(NewTarget[1] - target[1]) > DistanceConfidence
            WithinY = abs(NewTarget[2] - target[2]) > DistanceConfidence
            WithinZ = abs(NewTarget[3] - target[3]) > DistanceConfidence
            if (target[0] != NewTarget[0]) and WithinX and WithinY and WithinZ:
                InMemory = True
        return InMemory

    # Concept code, puts target into memory
    def SaveTargetToMemory(self, label, x, y, z, area):
        TargetInfo = [label, x, y, z, area]
        self.TargetList.append(TargetInfo)

    # handles list of commands
    def receiveCommands(self, commandlist):
        self.WaitForSupplementary = False
        self.TargetOrPosition = 0
        self.CommandIndex = None
        for command in commandlist:
            print("Command data read: ", command)
            self.InitialTime = time.perf_counter()
            self.ElapsedTime = 0.0
            self.UpdateThrustersPID()
            self.IsMoveCommand = False
            self.Basic = False
            self.Advanced = False
            if self.WaitForSupplementary:
                trupleindex = 0
                targetorval = False
                for posstarget in self.POSSIBLE_TARGETS:
                    if command == posstarget:
                        targetorval = True
                if not targetorval:
                    for value in command.split('x'):
                        if trupleindex == 0:
                            self.YawOffset = int(value)
                        elif trupleindex == 1:
                            self.PitchOffset = int(value)
                        elif trupleindex == 2:
                            self.RollOffset = int(value)
                        trupleindex += 1
                self.IsMoveCommand = True
            for i in range(len(self.BASIC_MOVEMENT_COMMANDS)):
                if command == self.BASIC_MOVEMENT_COMMANDS[i]:
                    self.IsMoveCommand = True
                    self.Basic = True
                    self.CommandIndex = i
            for i in range(len(self.ADVANCED_MOVEMENT_COMMANDS)):
                if command == self.ADVANCED_MOVEMENT_COMMANDS[i]:
                    self.IsMoveCommand = True
                    self.Advanced = True
                    self.CommandIndex = i
            for i in range(len(self.TARGET_MOVEMENT_COMMANDS)):
                if command == self.TARGET_MOVEMENT_COMMANDS[i]:
                    self.WaitForSupplementary = True
                    self.TargetOrPosition = A_TARGET
                    self.CommandIndex = i
            if command == self.GYRO_TO:
                self.WaitForSupplementary = True
                self.TargetOrPosition = A_GYRO
            if command == self.POSITION_TO:
                self.WaitForSupplementary = True
                self.TargetOrPosition = A_POSITION
            if self.IsMoveCommand:
                self.Running = True
                self.GyroRunning = True
                self.PositionRunning = True
                try:
                    while self.Running:
                        if self.UsingSim:
                            self.TelemetrySim.update(self.PixHawk.getYaw())
                        self.WaitForSupplementary = False
                        print("Yaw: ", self.PixHawk.getYaw())
                        print("Pitch: ", self.PixHawk.getPitch())
                        print("Down: ", self.PixHawk.getDown())
                        # print("Roll: ", self.PixHawk.getRoll())
                        # print("Right Back speed: ", MapToSpeed(self.ThrusterRB.GetSpeed()))
                        if self.TargetOrPosition == A_GYRO:
                            self.CheckIfGyroDone()
                            self.Running = self.GyroRunning
                        elif self.TargetOrPosition == A_POSITION:
                            self.CheckIfPositionDone()
                            self.Running = self.PositionRunning
                        elif self.TargetOrPosition == A_TARGET:
                            self.TargetCommand(self.CommandIndex, command)
                        elif self.Basic:
                            self.BasicCommand(self.CommandIndex)
                        elif self.Advanced:
                            self.AdvancedCommand(self.CommandIndex)
                        self.UpdateThrustersPID()
                    self.TargetOrPosition = 0
                except:
                    self.BrakeAllThrusters()
        self.BrakeAllThrusters()

    def CheckIfGyroDone(self, threshold=3, timethreshold=5):
        self.PowerBR = -10
        self.PowerBL = -10
        self.PitchOffset = 0
        if (abs(self.PixHawk.getYaw() - self.YawOffset) < threshold) and (
                abs(self.PixHawk.getPitch() - self.PitchOffset) < threshold) and (
                abs(self.PixHawk.getRoll() - self.RollOffset) < threshold):
            self.ElapsedTime = time.perf_counter() - self.InitialTime
            print("Within threshold. Waiting ", timethreshold, "...")
            if self.ElapsedTime >= timethreshold:
                self.GyroRunning = False
        else:
            self.InitialTime = time.perf_counter()

    def CheckIfPositionDone(self, threshold=3, timethreshold=5):
        if (abs(self.PixHawk.getNorth() - self.NorthOffset) < threshold) and (
                abs(self.PixHawk.getEast() - self.EastOffset) < threshold) and (
                abs(self.PixHawk.getDown() - self.DownOffset) < threshold):
            self.ElapsedTime = time.perf_counter() - self.InitialTime
            print("Within threshold. Waiting ", timethreshold, "...")
            if self.ElapsedTime >= timethreshold:
                self.GyroRunning = False
        else:
            self.InitialTime = time.perf_counter()

    def UpdateThrustersPID(self):
        self.PixHawk.UpdateGyro()
        self.PixHawk.CalculateError(self.YawOffset,
                                    self.PitchOffset,
                                    self.RollOffset,
                                    self.NorthOffset,
                                    self.EastOffset,
                                    self.DownOffset)
        self.PixHawk.PID()
        self.ThrusterLB.SetSpeedPID(self.PowerLB, yawpid=self.PixHawk.getYawPID())
        self.ThrusterLF.SetSpeedPID(self.PowerLF, yawpid=self.PixHawk.getYawPID())
        self.ThrusterRB.SetSpeedPID(self.PowerRB, yawpid=-self.PixHawk.getYawPID())
        self.ThrusterRF.SetSpeedPID(self.PowerRF, yawpid=-self.PixHawk.getYawPID())

        self.ThrusterBL.SetSpeedPID(self.PowerBL,
                                    rollpid=self.PixHawk.getRollPID(),
                                    pitchpid=-self.PixHawk.getPitchPID())
        self.ThrusterBR.SetSpeedPID(self.PowerBR,
                                    rollpid=-self.PixHawk.getRollPID(),
                                    pitchpid=-self.PixHawk.getPitchPID())
        self.ThrusterFL.SetSpeedPID(self.PowerFL,
                                    rollpid=-self.PixHawk.getRollPID(),
                                    pitchpid=-self.PixHawk.getPitchPID())
        self.ThrusterFR.SetSpeedPID(self.PowerFR,
                                    rollpid=self.PixHawk.getRollPID(),
                                    pitchpid=-self.PixHawk.getPitchPID())

    def UpdateThrusters(self):
        self.ThrusterLB.SetSpeed(self.PowerLB)
        self.ThrusterLF.SetSpeed(self.PowerLF)
        self.ThrusterRB.SetSpeed(self.PowerRB)
        self.ThrusterRF.SetSpeed(self.PowerRF)
        self.ThrusterBL.SetSpeed(self.PowerBL)
        self.ThrusterBR.SetSpeed(self.PowerBR)
        self.ThrusterFR.SetSpeed(self.PowerFR)
        self.ThrusterFL.SetSpeed(self.PowerFL)

    def BrakeAllThrusters(self):
        # horizontal
        self.PowerLB = 0
        self.PowerLF = 0
        self.PowerRB = 0
        self.PowerRF = 0
        # vert
        self.PowerBL = 0
        self.PowerBR = 0
        self.PowerFR = 0
        self.PowerFL = 0

        self.UpdateThrusters()

    # self.BASIC_MOVEMENT_COMMANDS = [
    #     "FORWARD",
    #     "REVERSE",
    #     "LEFT",
    #     "RIGHT",
    #     "CLOCKWISE TURN",
    #     "COUNTERCLOCKWISE TURN",
    #     "DIVE",
    #     "SURFACE",
    #     "IDLE"]

    def BasicCommand(self, speed=GENERAL_THROTTLE):
        if self.UsingPixHawk:
            self.PixHawk.UpdateGyro()
            self.PixHawk.CalculateError(self.YawOffset,
                                        self.PitchOffset,
                                        self.RollOffset,
                                        self.NorthOffset,
                                        self.EastOffset,
                                        self.DownOffset)
            self.PixHawk.PID()
        DownConst = -5.0
        # 0 = FORWARD
        if self.CommandIndex == 0:
            # horizontal
            self.PowerLB = speed
            self.PowerLF = speed
            self.PowerRB = speed
            self.PowerRF = speed
            # vert
            self.PowerBL = DownConst
            self.PowerBR = DownConst
            self.PowerFR = DownConst
            self.PowerFL = DownConst

        # 1 = REVERSE
        if self.CommandIndex == 1:
            # horizontal
            self.PowerLB = -speed
            self.PowerLF = -speed
            self.PowerRB = -speed
            self.PowerRF = -speed
            # vert
            self.PowerBL = DownConst
            self.PowerBR = DownConst
            self.PowerFR = DownConst
            self.PowerFL = DownConst

        # 2 = LEFT
        if self.CommandIndex == 2:
            # horizontal
            self.PowerLB = speed
            self.PowerLF = -speed
            self.PowerRB = -speed
            self.PowerRF = speed
            # vert
            self.PowerBL = DownConst
            self.PowerBR = DownConst
            self.PowerFR = DownConst
            self.PowerFL = DownConst

        # 3 = RIGHT
        if self.CommandIndex == 3:
            # horizontal
            self.PowerLB = -speed
            self.PowerLF = speed
            self.PowerRB = speed
            self.PowerRF = -speed
            # vert
            self.PowerBL = DownConst
            self.PowerBR = DownConst
            self.PowerFR = DownConst
            self.PowerFL = DownConst

        # 4 = CLOCKWISE
        if self.CommandIndex == 4:
            self.PowerLB = speed
            self.PowerLF = -speed
            self.PowerRB = -speed
            self.PowerRF = speed

            self.PowerBL = DownConst
            self.PowerBR = DownConst
            self.PowerFR = DownConst
            self.PowerFL = DownConst

        # 5 = COUNTERCLOCKWISE
        if self.CommandIndex == 5:
            self.PowerLB = -speed
            self.PowerLF = speed
            self.PowerRB = speed
            self.PowerRF = -speed
            self.PowerBL = DownConst
            self.PowerBR = DownConst
            self.PowerFR = DownConst
            self.PowerFL = DownConst

        # 6 = DIVE
        if self.CommandIndex == 6:
            self.PowerLB = 0.0
            self.PowerLF = 0.0
            self.PowerRB = 0.0
            self.PowerRF = 0.0

            self.PowerBL = speed
            self.PowerBR = speed
            self.PowerFR = -speed
            self.PowerFL = -speed

        # 7 = SURFACE
        if self.CommandIndex == 7:
            self.PowerLB = 0.0
            self.PowerLF = 0.0
            self.PowerRB = 0.0
            self.PowerRF = 0.0

            self.PowerBL = -speed
            self.PowerBR = -speed
            self.PowerFR = speed
            self.PowerFL = speed

        # 8 = IDLE
        if self.CommandIndex == 8:
            self.PowerLB = 0.0
            self.PowerLF = 0.0
            self.PowerRB = 0.0
            self.PowerRF = 0.0

            self.PowerBL = -DownConst
            self.PowerBR = -DownConst
            self.PowerFR = DownConst
            self.PowerFL = DownConst
        if self.UsingPixHawk:
            self.ThrusterLB.SetSpeedPID(self.PowerLB)
            self.ThrusterLF.SetSpeedPID(self.PowerLF)
            self.ThrusterRB.SetSpeedPID(self.PowerRB)
            self.ThrusterRF.SetSpeedPID(self.PowerRF)

            self.ThrusterBL.SetSpeedPID(self.PowerBL,
                                        rollpid=self.PixHawk.getRollPID(),
                                        pitchpid=-self.PixHawk.getPitchPID())
            self.ThrusterBR.SetSpeedPID(self.PowerBR,
                                        rollpid=-self.PixHawk.getRollPID(),
                                        pitchpid=-self.PixHawk.getPitchPID())
            self.ThrusterFL.SetSpeedPID(self.PowerFL,
                                        rollpid=-self.PixHawk.getRollPID(),
                                        pitchpid=-self.PixHawk.getPitchPID())
            self.ThrusterFR.SetSpeedPID(self.PowerFR,
                                        rollpid=self.PixHawk.getRollPID(),
                                        pitchpid=-self.PixHawk.getPitchPID())
        else:
            self.UpdateThrusters()

    # self.ADVANCED_MOVEMENT_COMMANDS = [
    #     "LOG START POINT",
    #     "RETURN TO START",
    # ]

    def AdvancedCommand(self, commandnum, supplementarycmd=None):
        if self.UsingPixHawk:
            pass
        if commandnum == 0:
            pass
        if self.UsingPixHawk:
            self.PixHawk.UpdateGyro()
            self.PixHawk.CalculateError(self.YawOffset,
                                        self.PitchOffset,
                                        self.RollOffset,
                                        self.NorthOffset,
                                        self.EastOffset,
                                        self.DownOffset)
            self.PixHawk.PID()
            if commandnum == 5:
                # horizontal
                self.PowerLB = 0
                self.PowerLF = 0
                self.PowerRB = 0
                self.PowerRF = 0
                # vert
                self.PowerBL = 0
                self.PowerBR = 0
                self.PowerFR = 0
                self.PowerFL = 0
        else:
            print("Command requires the PixHawk")
            self.Running = False

    # self.TARGET_MOVEMENT_COMMANDS = [
    #     "MOVE TO TARGET",
    #     "RAM TARGET",
    #     "FIRE AT TARGET"
    # ]

    def TargetCommand(self, commandnum, target):
        self.Running = True
        targettaskdone = self.SearchForTarget(target)
        targettaskdone = not targettaskdone
        if targettaskdone:
            print("Target found. Wait 1...")
            time.sleep(1)
        else:
            print("No target found. Wait 1...")
            time.sleep(1)
        while targettaskdone:
            self.VisionAI.process_image(target)
            self.PitchOffset = self.PixHawk.getPitch()
            self.YawOffset = self.PixHawk.getYaw()
            self.PitchOffset += self.VisionAI.getObjectVector()[0]
            self.YawOffset += self.VisionAI.getObjectVector()[1]
            self.DistanceOffset = self.VisionAI.getLatDistanceMM()
            speed = self.DistanceOffset / 100
            if commandnum == 0:
                if self.DistanceOffset < 100:
                    targettaskdone = True
            elif commandnum == 1:
                if self.DistanceOffset < 100:
                    speed = 80
                elif self.DistanceOffset < 30:
                    targettaskdone = True
                elif (self.DistanceOffset < 30) or not FoundTarget:
                    time.sleep(3)
                    targettaskdone = True
            elif commandnum == 2:
                if self.DistanceOffset < 200:
                    # firer torperdor
                    pass
            if speed >= 0:
                self.PowerLB = speed + 15
                self.PowerLF = speed + 15
                self.PowerRB = speed + 15
                self.PowerRF = speed + 15
            else:
                self.PowerLB = 0
                self.PowerLF = 0
                self.PowerRB = 0
                self.PowerRF = 0

            # self.TargetVector

    # searches for target if cannot find it
    def SearchForTarget(self, target, repositioning=False, distancethreshold=300):
        FoundTarget = False
        if repositioning:
            for i in range(5):
                self.EastOffset = random.randrange(-distancethreshold, distancethreshold, 50)
                self.NorthOffset = random.randrange(-distancethreshold, distancethreshold, 50)
                self.DownOffset = random.randrange(-distancethreshold, distancethreshold, 50)
                self.PositionRunning = True
                while self.PositionRunning:
                    self.CheckIfPositionDone(threshold=6, timethreshold=3)
                    self.UpdateThrustersPID()
                for j in range(7):
                    self.YawOffset = j * 55 - 165
                    for k in range(5):
                        self.PitchOffset = k * 35 - 70
                        self.GyroRunning = True
                        print("Scanning at rotation matrix: ", "Yaw: ", self.YawOffset, ", Pitch: ", self.PitchOffset)
                        while self.GyroRunning:
                            LateralDistanceMM, DistanceMM, OffCenterX, OffCenterY, \
                            FoundTarget = self.VisionAI.process_image(target)
                            self.CheckIfGyroDone(threshold=5, timethreshold=2)
                            self.UpdateThrustersPID()
                        if FoundTarget:
                            break
                    if FoundTarget:
                        break
                if FoundTarget:
                    break
        else:
            FoundTarget = False
            for i in range(7):
                self.YawOffset = i * 55 - 165
                for j in range(5):
                    self.PitchOffset = j * 35 - 70
                    self.GyroRunning = True
                    print("Scanning at rotation matrix: ")
                    while self.GyroRunning and not FoundTarget:
                        LateralDistanceMM, DistanceMM, OffCenterX, OffCenterY, \
                        FoundTarget = self.VisionAI.process_image(target)
                        self.CheckIfGyroDone(threshold=5, timethreshold=2)
                        self.UpdateThrustersPID()
                    if FoundTarget:
                        break
                if FoundTarget:
                    break
        return FoundTarget

    # ending vehicle connection and AI processing after mission completion/fatal error
    def Terminate(self):
        self.ThrusterLB.SetSpeed(0)
        self.ThrusterLF.SetSpeed(0)
        self.ThrusterRB.SetSpeed(0)
        self.ThrusterRF.SetSpeed(0)
        self.ThrusterBL.SetSpeed(0)
        self.ThrusterBR.SetSpeed(0)
        self.ThrusterFR.SetSpeed(0)
        self.ThrusterFL.SetSpeed(0)
        time.sleep(1)
        if self.UsingPixHawk:
            print("Killing Pixhawk. Wait 1...")
            time.sleep(1)
            self.PixHawk.Terminate()
        if self.UsingVision:
            print("Killing Vision. Wait 1...")
            time.sleep(1)
            self.VisionAI.terminate()
        print("Killing board. Wait 1...")
        time.sleep(1)
        self.board.exit()


def MapToPWM(x) -> float:
    in_min = -100.0
    in_max = 100.0
    out_min = 46.5
    out_max = 139.5
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min


def MapToSpeed(x) -> float:
    out_min = -100.0
    out_max = 100.0
    in_min = 46.5
    in_max = 139.5
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min


# dedicated class to driving a specific thruster
# has own PID, thruster, speed
class ThrusterDriver:
    def __init__(self, pin, board):
        connectstring = "d:" + str(pin) + ":s"
        self.thruster = board.get_pin(connectstring)
        print("Initializing Thruster: ", connectstring)
        self.speed = 93
        self.thruster.write(self.speed)

    # sets speed of thruster
    def SetSpeed(self, speed):  # speed is a value between -100 and 100
        if speed > MAX_THROTTLE:
            speed = MAX_THROTTLE
        elif speed < -MAX_THROTTLE:
            speed = -MAX_THROTTLE
        self.speed = MapToPWM(speed)
        self.thruster.write(self.speed)

    #  sets speed of thruster and incorporates the addition of pwm variables
    def SetSpeedPID(self, speed, rollpid=0.0, pitchpid=0.0, yawpid=0.0):
        self.speed = float(float(speed) + float(rollpid) + float(pitchpid) + float(yawpid))
        if self.speed > MAX_THROTTLE:
            self.speed = MAX_THROTTLE
        elif self.speed < -MAX_THROTTLE:
            self.speed = -MAX_THROTTLE
        self.speed = MapToPWM(self.speed)
        self.thruster.write(self.speed)

    # returns speed
    def GetSpeed(self):
        return self.speed
