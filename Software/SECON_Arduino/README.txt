File: 					SECON_Arduino

Author(s): 				Chase Garner, Sawyer Hall, Daniel Summers

Target Hardware: 		Arduino MEGA 2560

Purpose: 				This software is responsible for the motor control and sensor readings for Tennessee Tech's SoutheastCon 2022 Robot. 
						This serves as the main gameplay control for the robot.

Software Dependencies:	Ardunio IDE

External Connections:	This software is designed to be ran in conjunction with a Raspberry Pi (see SECON2022/Software/RaspberryPi for software) via UART.
						Reference SECON2022/Documentation/ for all external hardware connections. 
						
Installation:			Connect the Arduino MEGA 2560 to a computer via USB. Using the Arduino IDE, flash the given code to the Ardunio.

Running/Use:			After installation, the code will automatically begin running when the Arduino is powered on. Immediately after powering on, the 
						robot will begin calibrating. At this time, the line following sensors (See SECON2022/Documentation/) should be calibrated by providing
						them with a reference for both black and white (This implementation expects a white line with a black background). After calibrating,
						the arduino will entire a standby loop until the start switch is flipped (the GPIO input pin connected to the start switch will have its
						pull up resistor enabled, driving its signal high. Connecting the other side of the switch to Ground will pull the GPIO pin down when the 
						switch is flipped, starting the gameplay routine. Any input of 0/Ground on this pin will achieve the same result. Power Cycling the robot 
						is required to restart the routine.
			