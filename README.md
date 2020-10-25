Welcome to the README for the Oregon Tech RoboSub Software
Written By: Theodor Giles

I will go into detail on how to: 

1. **Recreate the hardware we work with**

2. **The software requirements for the said environment**

3. **The programming structure of how I control the sub** 


I am trying to structure the control system in such a way that the application and code are easily manageable through separate systems that can operate independently of one another to particular extents, so whoever uses the code and system can test and run the sub with ease!

Of course, this is all subject to change. ***I am writing this sentence on August 5th, 2020***. 

***I. Hardware Environment:***

Our RoboSub hardware currently consists of-

**Boards/Electronics** - 

**T200 Thrusters** - (Newest from BlueRobotics, we have 8)

**Basic ESCs** - (From BlueRobotics) 

**Raspberry Pi 3B+/4B+** - (Most new-ish Pis will work, as long as it is capable enough to run certain vision code and varying processor-heavy programs)

   The Raspberry Pi doesn't require too much configuration, but there still is some. First, run "sudo raspi-config" in the terminal, and enable VNC, serial communication under interfacing options (but disable the "shell" whatever option"), and whatever other settings you'd like. I like to have the Pi boot to the desktop. See if you can figure that out. If you would like to interface with Pi from your own computer without having to have it connected to a monitor, keyboard, and mouse, then install VNC-viewer. There are some very easy instructions online. 
"Installing VNC viewer on Rpi", or something like that.

**PixHawk** - (For data/gyro/positioning only currently, but can be implemented to use other functionality if someone so chooses)

   The PixHawk is an annoying little board, whereas it requires some extra setup that is somewhat intuitive. On your own computer, you must install a program called QGroundControl in order to upload the necessary firmware to it via USB between your computer and the PixHawk. It should walk you through it, but the button to mess with the settings is accessible through the top left icons, where a gear should show up after some finagling. From there, the firmware tab opens, and you should be able to upload the ArduSub firmware. After it is done uploading, it will ask you to do some calibration. Follow the instructions, and orient the board in the ways shown. After everything is calibrated, you're good to go. 

**Arduino Mega** - (For thruster PWM control)

   The Arduino requires the ServoFirmata.ino uploaded to it, either through the Rpi or another computer. This lets it communicate via serial with the Rpi so we can send commands for it to drive the PWM pins to the thrusters. The ServoFirmata.ino is in the examples/firmata folder that comes with the basic Arduino IDE. Upload the program to the Arduino. I do this occasionally just in case as well, so if there are any problems with communicating with the Arduino, try reuploading the ServoFirmata file. Yes, I know we're driving ESCs with our Arduino, but Servos work on the same PWM code, so it lines up very nicely. Don't worry. 

**Wiring** - 

   The Raspberry Pi is the brains of the entire operation, so most everything has a line to it. The Arduino is connected by USB to the Raspberry Pi and doesn't take any external voltage because all it is doing is simple PWM signals that don't require any extra power. The PixHawk is connected to the Raspberry Pi via USB as well, but if you would like to use the different wiring setups it uses, feel welcome to try. I did not have the best of luck with it. 


***II. Software Environment***

   Pretty much all of the code is written in Python, so it is pretty proficient at handling lots of advanced computation, organization, and interfacing with other things, even hardware. I have the program set up in a sort of tree that flows upward from a starting program to another program, to another program, and branches out to other hardware and software based on what is required/asked of the program.
   
   The libraries we use are specified in the get_pi_requirements.sh, but I will also list them here.
   PyFirmata - (arduino communication
   Dronekit - (pixhawk communication)
   Tensorflow - (AI development and interpretation)
   OpenCV - (vision processing)
   PyGame - (simulation)
   PyOpenGL - (simulation)
   
   The top-most program of our custom application is technically "*START_SUB.py*", but can also be initialized by "*button_listener.py*", a script that is supposed to listen for a button press on the Raspberry Pi. This would be used when we don't have access to a wired connection from the Sub to a computer, but I mainly implemented it because I thought it would be cool to have a button and buzzer.

   **START_SUB** - 

   This is the most simple program in the entire application, but you can also control the use of the Pixhawk and/or implemented vision processing. 

```
#!python

    Mission = TaskIO("mission.txt", False, True)
    Mission.get_tasks()
    Mission.terminate()
```

*TaskIO* is the 2nd-highest program, which is initialized with *mission.txt*, a basic text file consisting of the commands the Sub will run, and they are formatted to a certain standard, which I will go into detail later. The first boolean argument determines if we are using our vision processing algorithms/AIs, the second determines the use of the PixHawk and Gyro, but I may omit the ability to not use the PixHawk soon because of how important the gyro data is to the navigation of the RoboSub. 
*get_tasks()* manages the *mission.txt* file and sends it to the corresponding classes and subsystems.

   **task_io_lib_v2** -

   This program is the first level at which commands are taken in, and controls the peripherals such as the Pixhawk and Vision Processing. It also initializes the *MovementCommander*.

```
#!python
class TaskIO:
    # init
    def __init__(self, filename, usingvision, usingpixhawk):
        self.UsingVision = usingvision
        self.UsingPixhawk = usingpixhawk
        self.Filename = filename
        self.UsingVision = usingvision
        self.Active = False
        self.Movement = MovementCommander(self.UsingVision, self.UsingPixhawk)
        self.CommandList = []
```

   This is the init, and as you can see, it takes two boolean arguments if the user wants to use the vision processing and/or the PixHawk. 

```
#!python
# get tasks from the .txt and completes them
    def get_tasks(self):
        # Testing
        self.Commands = open(self.Filename)
        for CommandLine in self.Commands:
            for ParsedCommand in CommandLine.strip().split(','):
                self.CommandList.append(ParsedCommand)
        print("Commands read...")
        self.Commands.close()
        print("Commands: ", self.CommandList)
        self.Movement.receiveCommands(self.CommandList)
        self.active = False
```

   This function of TaskIO is basically a run() function, but it also puts all the "commands" from the *mission.txt* into a list so we don't have to parse the text file in the MovementCommander(*self.Movement*) as well.
   
   **movement_commander_v4** - 

   This program is the main meat and bones of the navigation, the command interpreting, thruster driving, data collecting, and I think it's the longest program in the application by lines of code. 
   I feel like I don't need to explain the *__init__()* of MovementCommander, because it is self-explanatory, initializing each class and hardware we want it to.

```
#!python
# handles list of commands
    def receiveCommands(self, commandlist):
        self.WaitForSupplementary = False
        self.TargetOrPosition = 0
        for command in commandlist:
            print("Command data read: ", command)
            self.InitialTime = time.perf_counter()
            self.ElapsedTime = 0.0
            self.UpdateThrustersPID()
            self.IsMoveCommand = False
            self.Basic = False
            self.Advanced = False
```

   This initial bit of code is iterating through each command that was saved from the *mission.txt* and starts the process of determining what the command means/what it is supposed to accomplish. 
    *WaitForSupplementary* is a variable dedicated to commands that require an argument after it, and *TargetOrPosition* is a variable meant for determining the sub-command of WaitForSupplementary, whether it be a target(*A_TARGET*), a gyro matrix(*A_GYRO*), or a position matrix(*A_POSITION*). 
    The *InitialTime* and *ElapsedTime* variables are meant for basic time-based commands, or for when we get to the desired matrix, it waits a certain amount of time before starting the next command after reaching the desired position. 

```
#!python
if self.WaitForSupplementary:
    trupleindex = 0
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
```


This set of code checks the value of each command in the command list and sets the variables from before to their respective values for running the specific commands.


```
#!python
if self.IsMoveCommand:
    self.Running = True
    self.GyroRunning = True
    self.PositionRunning = True
    while self.Running:
        self.WaitForSupplementary = False
        self.UpdateThrustersPID()
        # print("Yaw: ", self.PixHawk.getYaw())
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
    self.TargetOrPosition = 0
```


This code here is for running the specific commands because once a "move command" is interpreted, it then knows to run the moving code in that command loop. It sets all the *self.Running* variables to True, so that it will run until one of them becomes false by the command ends, such as the *self.CheckIfGyroDone* that sets *self.GyroRunning* to False and also sets it's value to *self.Running* for a basic turn command, or such as the *TargetCommand()* successfully rams a target and sets *self.Running* to false. 

   **pixhawk_data** - 

   This program is dedicated to receiving and parsing Mavlink data from the Pixhawk over serial, and then allowing it's data to be accessed, as well as taking in offset values for PID calculation. 
    **!! Currently, only Proportional(the P in PID) control has been tested.  !!** 


```
#!python
GYRO: int = 0
POSITION: int = 1
YAW: int = 0
PITCH: int = 1
ROLL: int = 2
NORTH: int = 0
EAST: int = 1
DOWN: int = 2

class PixHawk:

    Gyro = [0.0, 0.0, 0.0]
    Position = [0.0, 0.0, 0.0]
    Angular_Motions = [[0.0, 0.0, 0.0], [0.0, 0.0, 0.0]]
    Measures = [[0.0, 0.0, 0.0], [0.0, 0.0, 0.0]]
    Error = [[0.0, 0.0, 0.0], [0.0, 0.0, 0.0]]
    Previous_Error = [[0.0, 0.0, 0.0], [0.0, 0.0, 0.0]]
    Error_Sum = [[0.0, 0.0, 0.0], [0.0, 0.0, 0.0]]
    Error_Delta = [[0.0, 0.0, 0.0], [0.0, 0.0, 0.0]]

    Kp = [[0.3, 0.4, 0.4], [0.3, 0.4, 0.4]]
    Ki = [[0.1, 0.1, 0.1], [0.1, 0.1, 0.1]]
    Kd = [[0.1, 0.1, 0.1], [0.1, 0.1, 0.1]]

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
```

All these variables are for very specific calculations, so I will walk through how it all comes together. First of all, the constants such as *GYRO: int = 0* is for accessing the Gyroscopic data from the arrays that are 2-dimensional. Gyro data is at 1st index 0, 2nd index 0->2, and position data is at 1st index 0, 2nd index 0->2. The *YAW*/*PITCH*/*ROLL* variables are for accessing the 2nd index, so I can ensure the right data goes into the right index at any given time through readability. The *Error* variables are for PID calculations, which you can read in the program. It is pretty straight forward. The other variables are easy to interpret cause their use is in their name. 


```
#!python
i = 0
for CommaParse in str(self.vehicle.attitude).split(','):
    if CommaParse is not None:
        for EqualParse in CommaParse.split('='):
            try:
                if i == 1:
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
```

This is the code to parse the string data read from the Dronekit library that the Raspberry Pi uses to receive serial data from the Pixhawk, and because the data read back is more like readable telemetry, there is some string splitting that must occur, and the data is at certain points in the string, which is why the values are set to the converted strings when "i" is compared to a certain value in the for loop. I plan to improve this process at some point, making it faster, and I will update this Wiki accordingly.
