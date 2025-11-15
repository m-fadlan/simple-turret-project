This is a simple turret project that tracks red objects using OpenCV on a laptop. The laptop is then connnected to an Arduino UNO that controls two servos (one for pan, and one for tilt).

The way this works is that the OpenCV finds the centroid of the biggest red object, and then the rest of the python code relates the center of the camera to that centroid by using a closed-loop control scheme.

There are two control schemes : One that uses Fuzzy Logic (found in turretFLC.py) and one that uses PID (found in turretPID.py).

After the control scheme decides the input, the python code that is connected to the laptop then sends the input value to Arduino UNO through UART using raw bytes. (You can check the code yourself for more details.) 

Stuff used:
- 1x Arduino UNO
- 1x Webcam
- 1x Laptop (for the .py code)
- 2x Positional MG996R servos

You can check the demonstration videos and for more details at https://89labs.wordpress.com/2025/11/15/simple-turret-project/
