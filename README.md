<h1>Description</h1>
This is a simple turret project that tracks red objects using OpenCV on a laptop. The laptop is then connnected to an Arduino UNO that controls two servos (one for pan, and one for tilt).

The way this works is that the OpenCV finds the centroid of the biggest red object, and then the rest of the python code relates the center of the camera to that centroid by using a closed-loop control scheme.

There are two control schemes : One that uses Fuzzy Logic (found in turretFLC.py) and one that uses PID (found in turretPID.py).

After the control scheme decides the input, the python code that is connected to the laptop then sends the input value to Arduino UNO through UART using raw bytes. (You can check the code yourself for more details.) 

Stuff used:
- 1x Arduino UNO
- 1x Webcam
- 1x Laptop (for the .py code)
- 2x Positional MG996R servos

<h1>Demonstration Videos</h1>
<br>
<div style="width:300px; height:300px; overflow:hidden;">
  <a href="https://www.youtube.com/shorts/SxH6Nk7yzXg">
    <img src="https://img.youtube.com/vi/SxH6Nk7yzXg/maxresdefault.jpg"
         style="width:80%; height:auto;">
  </a>
</div>
<br>
<div style="width:300px; height:300px; overflow:hidden;">
  <a href="https://www.youtube.com/shorts/_KzThU-pn_o">
    <img src="https://img.youtube.com/vi/_KzThU-pn_o/maxresdefault.jpg"
         style="width:80%; height:auto;">
  </a>
</div>
<br>
<h3>
For more details, you can check at https://89labs.wordpress.com/2025/11/15/simple-turret-project/</h3>
