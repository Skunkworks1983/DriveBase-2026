# Guide to using simulation
## Step 1: Pick your controller type
In the file Constants.java, located at \src\main\java\frc\robot, change the variable "controlScheme" to either ControlScheme.OI or ControlScheme.XBOX to change the type of controller you would like to use

## Step 2: Start Simulation
In the command palate (open via ctrl+shift+p), search for the command WPILib: Simulate Robot Code

Navigate to it using arrow keys, and press enter to run the command

Eventually, a pop up will appear asking what tools you would like to run. Ensure "SIM GUI" is checked. If you would like to use the actual driver station, select that as well.

## Step 3: Register Joysticks (If you would like to simulate TeleOp)
In the simulation GUI that will automatically open, there is a dropdown labeled "System Joysticks" and a dropdown labeled "Joysticks"

Drag the joysticks you would like to use from the system joysticks dropdown into the same port you would be the joysticks into on driver station (for an OI this should be ports 0 and 1, and for an Xbox controller this should be port 0)

IMPORTANT: If you are using an Xbox controller, select the box "Map Gamepad". This maps the Xbox axis into WPILib

## Step 4: Open simulation
From the Windows start menu, open the app AdvantageScope.

For a pre-configured version of the app, hover over the file dropdown in the top left and select "Import Layout". In this project there is a file called "AdvantageScope Simulation Layout" located in \layouts, import this.

Next, from the same file dropdown as earlier select the option "Connect to Simulator". On connection, you should see a timeline of numbers moving forward towards the right of the screen.

If you are using the simulation layout, you can then select the SimView tab in the upper right, and a 3D field with the robot on it will load.

## Step 5: Run Match
First, select the alliance station you would like to use from the FMS dropdown in the SIM GUI

If you would like to run only Auto: Select the Auto to run through the Auto Choices dropdown in SIM GUI. In the Robot State dropdown in SIM GUI, select autonomous. This will instantly start the auto
If you would like to run only TeleOp: In the Robot State dropdown on SIM GUI, select Teleoperated. The robot will now drive following TeleOp joysticks.

If you would like to simulate a match: First, queue an auto and select an alliance station as described above. Then, in the FMS dropdown on SIM GUI press the "Auto" button located next to the text "Match Time"