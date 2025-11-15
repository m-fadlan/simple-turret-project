import cv2
import numpy as np
import serial
import time
import threading
import struct
import matplotlib.pyplot as plt

# Start camera capture
fps = 30
class CamGrabber(threading.Thread):
    def __init__(self, src=0):
        """
        Constructor: runs once when you write `grabber = CamGrabber(src)`
        """
        super().__init__(daemon=True)    # initialize the Thread base class
        self.cap = cv2.VideoCapture(src)  # open the camera
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        self.cap.set(cv2.CAP_PROP_EXPOSURE, -5)
        
        # grab one frame so self.frame always exists
        ret, frame = self.cap.read()      
        self.frame = frame if ret else None
        
        self.lock = threading.Lock()      # for safe concurrent access
        self.running = True               # flag to keep the thread alive

    def run(self):
        """
        This is the thread’s “main” loop. When you call `grabber.start()`,
        Python runs this method in the background automatically.
        """
        while self.running:
            ret, f = self.cap.read()      # fetch the latest camera frame
            if not ret:
                break                     # stop if camera read fails
            # safely overwrite the shared frame
            with self.lock:
                self.frame = f

    def get_frame(self):
        """
        Called from your main thread to grab a copy of the latest image.
        The lock makes sure the background thread isn’t writing at the same time.
        """
        with self.lock:
            return None if self.frame is None else self.frame.copy()

    def stop(self):
        """
        Call this when you want to cleanly shut down the thread and camera.
        """
        self.running = False       # causes run()’s loop to exit
        self.cap.release()         # free the camera resource


webcam = CamGrabber(src=1)
webcam.start()
# BOXES
shootbox_radius = 35
deadbox_radius = 20


# RED RANGE
redlow1 = np.array([0, 80, 50])
redup1 = np.array([10, 255, 255])
redlow2 = np.array([170, 80, 50])
redup2 = np.array([180, 255, 255])

#ARDUINO ACTUATION MODULE
ser = serial.Serial('COM8', 500000, timeout=1)


######## PID CONTROL #########
I_max_x, I_max_y = 20, 20
Kp_x, Ti_x, Td_x = 0.045, 10, 0.065
Kp_y, Ti_y, Td_y = 0.04, 10, 0.065

#Initial conditions
int_error_x = int_error_y = 0
error_x = error_y = 0
error_x_m = error_y_m = 0

#TIME KEEPER
t0 = time.perf_counter()
timinusone = 0


# LISTZ
time_list = []
error_x_list = []
error_y_list = []

def error(u,w):
    err = u-w
    if abs(err) < deadbox_radius:
        err = 0
    elif err < 0:
        err += deadbox_radius
    elif err > 0:
        err -= deadbox_radius
    return err

def on_key(event):
    if event.key in ('q','Q'):
        plt.close('all')

while True:
    ti = time.perf_counter() - t0
    dt = ti - timinusone
    timinusone = ti
    if dt == 0 or dt < 0:
        dt = 1/fps
# Check for exit command ('q' key)
#    if exit requested:
    if cv2.waitKey(1) in (ord('q'), ord('Q')):
        break
#        break
    # block until the next frame arrives
    time.sleep(max(0, (1/fps) - dt))
    # Get latest camera frame
    frame = webcam.get_frame()
    if frame is None:
        continue
    # Detect red objects
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask1 = cv2.inRange(hsv, redlow1, redup1)
    mask2 = cv2.inRange(hsv, redlow2, redup2)
    mask = mask1 | mask2

    # create kernel
    kernel = np.ones((5, 5), np.uint8)

    # morphological opening (removes noise)
    mask = cv2.erode(mask, kernel, iterations=2)
    mask = cv2.dilate(mask, kernel, iterations=2)
    # morphological closing (closes  holes)
    mask = cv2.dilate(mask, kernel, iterations=2)
    mask = cv2.erode(mask, kernel, iterations=2)

    # Contours
    target_x = None
    target_y = None
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    target_contour = None
    target_area = 1000 # Minimum object size (adjustable)
    for contour in contours:
        area = cv2.contourArea(contour)
        if area > target_area:
            target_area = area
            target_contour = contour

    # Detect target position (target_x, target_y)
    if target_contour is not None:
        (x, y, w, h) = cv2.boundingRect(target_contour)
        # Draw a rectangle around the red object
        cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 200, 0), 2)
        M = cv2.moments(target_contour)
        if M["m00"] != 0:
            target_x = int(M["m10"] / M["m00"])
            target_y = int(M["m01"] / M["m00"])
            cv2.putText(frame, f'({target_x},{target_y})', (x + w - 100, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 200, 0), 2) #Write target coordinates
            cv2.circle(frame,(target_x,target_y), 5, (0,200,0), -1)

    # Calculate frame center
    height, width = frame.shape[:2]
    center_x = width // 2
    center_y = height // 2
    # Draw shootbox
    cv2.rectangle(frame, (center_x - shootbox_radius, center_y - shootbox_radius),
              (center_x + shootbox_radius, center_y + shootbox_radius), (0, 255, 0), 1)

    # Draw deadbox (smaller)
    cv2.rectangle(frame, (center_x - deadbox_radius, center_y - deadbox_radius),
              (center_x + deadbox_radius, center_y + deadbox_radius), (255, 0, 0), 1)

    # Check target position compared to center
    pan = 0
    tilt = 0

    if target_x is not None and target_y is not None:
        error_x = error(center_x,target_x)
        error_y = error(center_y,target_y)
        int_error_x += error_x*dt
        int_error_y += error_y*dt
        int_error_x = np.clip(int_error_x, -I_max_x, I_max_x)
        int_error_y = np.clip(int_error_y, -I_max_y, I_max_y)
        diff_error_x = (error_x - error_x_m)/dt
        diff_error_y = (error_y - error_y_m)/dt

        throttle_x = Kp_x*(error_x + (1/Ti_x)*int_error_x + Td_x*diff_error_x)
        throttle_y = Kp_y*(error_y + (1/Ti_y)*int_error_y + Td_y*diff_error_y)

        pan = throttle_x
        tilt = -throttle_y

        if error_x == error_y == 0:
            lock = ", FULLY LOCKED!"
        else:
            lock = ""

        pan = round(np.clip(pan,-128,127))
        tilt = round(np.clip(tilt,-128,127))
        #print(f'{round(ti, 3)} | pan={pan} tilt={tilt} | err=({round(error_x,1)} , {round(error_y,1)}) | {lock}')
        error_x_m = error_x
        error_y_m = error_y


    else:
        #print(f'{round(ti, 3)} | No target detected.')
        int_error_x = int_error_y = error_x = error_y = 0
        diff_error_x = diff_error_y = error_x_m = error_y_m = 0


    # Display video feed (optional)
    cv2.imshow("polar (press q to terminate)",mask)
    cv2.imshow("PID controller (press q to terminate)",frame)

    # SEND TO ARDUINO
    packet = struct.pack('<Bbb', 0xFF, int(pan), int(tilt))
    ser.write(packet)
    time_list.append(ti)
    error_x_list.append(error_x)
    error_y_list.append(error_y)
# Stop camera and clean up motors
webcam.stop()
cv2.destroyAllWindows()

#plot error v time
figure = plt.figure(figsize=(8, 6))
plt.suptitle('Press Q to exit')
plt.subplot(2,1,1)
plt.plot(time_list,error_x_list,'-')
plt.title('Error in X vs. Time')
plt.subplot(2,1,2)
plt.plot(time_list,error_y_list,'-')
plt.title('Error in Y vs. Time')
plt.tight_layout()
plt.show()
figure.canvas.mpl_connect('key_press_event', on_key)