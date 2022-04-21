File(s): 				detect_shapes.py, shapedetector.py, colorlabeler.py, shapeLength.py

Author(s): 				Daniel Summers, Lexi Sheeler

Target Hardware: 		Raspberry Pi 4 (4 GB) Running Ubuntu 21.10

Purpose: 				This software is responsible for the Vision System of Tennessee Tech's SoutheastCon 2022 Robot. This software will detect the presence
						of nets on the game board and communicate this to an Arduino MEGA 2560 over serial communication. Additionally, this software will output
						an image to a TFT Screen. This serves as a method of achieving points as well as indicating that the Pi is running correctly.

Software Dependencies:	Python 3 (Necessary Libraries: imutils, cv2, serial, digitalio, board, PIL, adafruit_rgb_display), the file Tech.bmp (this is the bitmap
						image to be displayed on the TFT screen)

External Connections:	This software is designed to be ran in conjunction with an Arduino MEGA 2560 (see SECON2022/Software/SECON_Arduino for software) via UART.
						Reference SECON2022/Documentation/DesignDocuments for all external hardware connections. 
						
Installation:			Download and Install Python3 and its necessary libraries to the Raspberry Pi using apt and PIP (each where necessary)

Running/Use:			These files are designed to be ran with sudo priviledges automatically on boot up. To accomplish this, the default user of the Pi must be 
						given the necessary sudo priviledges, and the console command 'sudo python3 detect_shapes.py' must be added to the list of commands to be ran
						on startup. The team implemented this by adding the command to the default user's .profile script (Note: for this workaround, requiring the 
						default user's password when executing sudo commands must be disabled. This is a security risk that should not be implemented if the Pi is to
						be used for other applications.)
			