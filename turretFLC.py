import cv2
import numpy as np
import serial
import time
import threading
import struct
import matplotlib.pyplot as plt


def error(u,w):
    err = u-w
    if abs(err) <= deadbox_radius: 
        return 0
    return err - np.sign(err)*deadbox_radius


###### FUZZY LOGIC CONTROL (FLC) #######
# ——— Trapezoidal membership function —————————————————————————
def trapmf(x, a, b, c, d):
    """
    Trapezoidal membership:
      μ = 0                for x ≤ a or x ≥ d
      μ = (x−a)/(b−a)      for a < x < b
      μ = 1                for b ≤ x ≤ c
      μ = (d−x)/(d−c)      for c < x < d
    """
    if x <= a or x >= d:
        return 0.0
    elif a < x < b:
        return (x - a) / (b - a)
    elif b <= x <= c:
        return 1.0
    else:  # c < x < d
        return (d - x) / (d - c)

# ——— Fuzzy membership functions (shared for X and Y) —————————————————

error_params = {
    'hard_neg': (-300, -300, -300, -180),
    'soft_neg': (-300, -140, -100,  -20),
    'zero':     ( -70,    0,    0,   70),               ### POSITIONAL TUNE
    'soft_pos': (  20,  100,  140,  300),
    'hard_pos': ( 180,  300,  300,  300),
}
delta_params = {
    'hard_neg': (-10000, -10000, -3000,  -1500),
    'soft_neg': ( -3000,  -1500, -1000,      0),
    'zero':     (  -600,   -100,   100,    600),        ### CHASE TUNE
    'soft_pos': (     0,   1000,  1500,   3000),
    'hard_pos': (  1500,   3000, 10000,  10000),
}

# WEIGHTS
w_error, w_delta = 1, 0.6


# Error sets
def mu_error(label, e):
    a, b, c, d = error_params[label]
    return trapmf(e, a, b, c, d)
def mu_error_hard_neg(e):   return mu_error('hard_neg', e)
def mu_error_soft_neg(e):   return mu_error('soft_neg', e)
def mu_error_zero(e):       return mu_error('zero', e)
def mu_error_soft_pos(e):   return mu_error('soft_pos', e)
def mu_error_hard_pos(e):   return mu_error('hard_pos', e)

# Delta sets
def mu_delta(label, delta):
    a, b, c, d = delta_params[label]
    return trapmf(delta, a, b, c, d)
def mu_delta_hard_neg(d):   return mu_delta('hard_neg', d)
def mu_delta_soft_neg(d):   return mu_delta('soft_neg', d)
def mu_delta_zero(d):       return mu_delta('zero', d)
def mu_delta_soft_pos(d):   return mu_delta('soft_pos', d)
def mu_delta_hard_pos(d):   return mu_delta('hard_pos', d)

error_labels = list(error_params.keys())
delta_labels = list(delta_params.keys())

# PLOTTING MU FUNCTIONS
x_e = np.linspace(-320, 320, 641)
figure1 = plt.figure(figsize=(8, 6))
plt.suptitle("Press Q to close")
plt.subplot(2,1,1)
for lbl in error_labels:
    a,b,c,d = error_params[lbl]
    y = [trapmf(v, a,b,c,d) for v in x_e]
    plt.plot(x_e, y, label=lbl)
plt.title("Error MFs"); plt.legend(); plt.grid()

x_d = np.linspace(-4000, 4000, 800)
plt.subplot(2,1,2)
for lbl in delta_labels:
    a,b,c,d = delta_params[lbl]
    y = [trapmf(v, a,b,c,d) for v in x_d]
    plt.plot(x_d, y, label=lbl)
plt.title("Delta MFs"); plt.legend(); plt.grid()

def on_key(event):
    if event.key in ('q','Q'):
        plt.close('all')

figure1.canvas.mpl_connect('key_press_event', on_key)
plt.show()

# Output singletons
output_values = {
    'hard_left':   -50,
    'soft_left':   -10,                                                    # TUNE THESE TOO
    'none':          0,
    'soft_right':  +10,
    'hard_right':  +50
}
error_to_output = {
    'hard_neg': 'hard_left',
    'soft_neg': 'soft_left',
    'zero':     'none',
    'soft_pos': 'soft_right',
    'hard_pos': 'hard_right'
}

# ——— Inference engine for 5×5 rules —————————————————————————
def infer_fuzzy_5x5(error, delta):
    rules = []
    for e_lbl in error_labels:
        mu_e = mu_error(e_lbl, error)
        out_lbl = error_to_output[e_lbl]
        for d_lbl in delta_labels:
            mu_d = mu_delta(d_lbl, delta)
            rules.append((min(mu_e*w_error, mu_d*w_delta), out_lbl))
    return rules

# ——— Defuzzification ———————————————————————————————
def defuzzify_fuzzy(rules):
    num = sum(mu * output_values[out] for mu, out in rules)
    den = sum(mu for mu, out in rules)
    return num/den if den else 0.0

# INITIAL CONDITIONS
error_x_m = error_x = error_y_m = error_y = 0





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


#TIME KEEPER
t0 = time.perf_counter()
timinusone = 0


# LISTZ
time_list = []
error_x_list = []
error_y_list = []

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
    dt = max(dt, 1e-3)
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
        delta_x = (error_x - error_x_m) / dt
        delta_y = (error_y - error_y_m) / dt

        # PAN INFERENCE AND DEFUZZIFICATION
        rules_x = infer_fuzzy_5x5(error_x, delta_x)
        throttle_x = defuzzify_fuzzy(rules_x)
        # TILT INFERENCE AND DEFUZZIFICATION
        rules_y = infer_fuzzy_5x5(error_y, delta_y)
        throttle_y = defuzzify_fuzzy(rules_y)

        pan = throttle_x
        tilt = -throttle_y

        if error_x == error_y == 0:
            lock = "FULLY LOCKED!"
        else:
            lock = ""

        pan = round(np.clip(pan,-128,127))
        tilt = round(np.clip(tilt,-128,127))
        print(f'{round(ti, 3)} | pan={pan} tilt={tilt} | err=({round(error_x,1)} , {round(error_y,1)}) | del=({round(delta_x,1)} , {round(delta_y,1)}) | {lock}')
        error_x_m, error_y_m = error_x, error_y


    else:
        #print(f'{round(ti, 3)} | No target detected.')
        int_error_x = int_error_y = error_x = error_y = 0
        diff_error_x = diff_error_y = error_x_m = error_y_m = 0


    # Display video feed (optional)
    cv2.imshow("polar (press q to terminate)",mask)
    cv2.imshow("FLC Controller (press q to terminate)",frame)

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
figure2 = plt.figure(figsize=(8, 6))
plt.suptitle('Press Q to exit')
plt.subplot(2,1,1)
plt.plot(time_list,error_x_list,'-')
plt.title('Error in X vs. Time')
plt.subplot(2,1,2)
plt.plot(time_list,error_y_list,'-')
plt.title('Error in Y vs. Time')
plt.tight_layout()
plt.show()
figure2.canvas.mpl_connect('key_press_event', on_key)