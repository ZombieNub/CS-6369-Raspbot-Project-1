import time
from Raspbot_Lib import Raspbot
import sys
sys.path.append('/home/pi/project_demo/lib')
from McLumk_Wheel_Sports import *
import math
import cv2
import numpy as np
from PIL import Image
from IPython.display import display

bot = Raspbot()

SPIN_SPEED = 100
TIME_LIMIT = 20

TOLERANCE =0.25
TRACK_MIN_SPEED = 30
TRACK_GAIN =40
SPIN_360_TIME = 2.5

FL,FR,RL,RR = 0,1,2,3

COLORS = {
    "red": [
        (np.array([0, 43, 46]), np.array([10, 255, 255])),
        (np.array([156, 43, 46]), np.array([180, 255, 255]))
    ],
    "yellow": [
        (np.array([26, 43, 46]), np.array([34, 255, 255]))
    ],
    "green": [
        (np.array([35, 43, 46]), np.array([77, 255, 255]))
    ],
    "blue": [
        (np.array([100, 43, 46]), np.array([124, 255, 255]))
    ]
}

MIN_PIXELS = 500

def set_wheels(fl,fr,rl,rr):
	bot.Ctrl_Muto(FL,fl)
	bot.Ctrl_Muto(FR,fr)
	bot.Ctrl_Muto(RL,rl)
	bot.Ctrl_Muto(RR,rr)

def stop_all():
	set_wheels(0,0,0,0)

def StartupAction():
	s = SPIN_SPEED
	set_wheels(-s,s,-s,s)
	time.sleep(2.5)
	stop_all()

def TimerCheck(start_time, limit=TIME_LIMIT):
	return(time.monotonic() -start_time) < limit

def cameraCapture(cap):
    """
    Capture one frame from the camera.
    """

    ret, frame = cap.read()

    if not ret:
        raise RuntimeError("Could not capture camera frame")

    return frame

# An admittedly slapdash way to do round robin scheduling, but python doesn't have a interrupt thing
ACTION_MAX_TIME = 10

def ColorCounter(image):
    """
    Counts the number of pixels belonging to each color.

    Returns the pixel counts of red, green, blue, and yellow. I added yellow as an extra case.
    """

    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    color_counts = {}

    for color, ranges in COLORS.items():
        total = 0
        for lower, upper in ranges:
            mask = cv2.inRange(hsv, lower, upper)
            
            total += cv2.countNonZero(mask)

        color_counts[color] = total

    return color_counts

def ColorLocator(image, color_counts):
    """
    Locates the color with the largest pixel count.

    Returns (x, y, color), or None if no color in COLORS is detected.
    """

    # Return if colorCounter didn't work
    if not color_counts:
        return None
        
    # Find the color with the most pixels
    color = max(color_counts, key=color_counts.get)

    # Not enough color pixels
    if color_counts[color] < MIN_PIXELS:
        return None

    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    mask = np.zeros(hsv.shape[:2], dtype=np.uint8)

    for lower, upper in COLORS[color]:
        color_mask = cv2.inRange(hsv, lower, upper)
        mask = cv2.bitwise_or(mask, color_mask)

    # Get coordinates of matching pixels
    ys, xs = np.where(mask > 0)

    if len(xs) == 0:
        return None

    # Average position of the object
    center_x = np.mean(xs)
    center_y = np.mean(ys)

    # Image dimensions
    height, width = image.shape[:2]

    # Convert to [-1, 1]
    x = (center_x - width / 2) / (width / 2)
    y = (center_y - height / 2) / (height / 2)

    return (x, y, color)
    
def RedAction(p_vector, red_end_time):
    if red_end_time is None:
        if p_vector is None or p_vector[2] != "red":
            return None

        s = SPIN_SPEED
        set_wheels(-s, s, -s, s)
        return time.monotonic() + SPIN_360_TIME / 2

    if time.monotonic() >= red_end_time:
        stop_all()
        return None

    return red_end_time

def IdleAction(p_vector, red_end_time):
    if red_end_time is not None or p_vector is not None:
        return 
    stop_all()


def GreenAction(p_vector, red_end_time):
    if red_end_time is not None:
        return

    if p_vector is None or p_vector[2] != "green":
        stop_all()
        return

    x = p_vector[0]

    if abs(x) <= TOLERANCE:
        stop_all()
        return

    #speed = min(TRACK_MIN_SPEED + int(TRACK_GAIN * abs(x)), 80)
    speed = SPIN_SPEED

    try:
        if x > 0:
            set_wheels(speed, -speed, speed, -speed)
            #set_wheels(-speed, speed, -speed, speed)
        else:
            set_wheels(-speed, speed, -speed, speed)
            #set_wheels(speed, -speed, speed, -speed)

        time.sleep(0.05)
    finally:
        stop_all()

def BlueAction(p_vector, red_end_time):

    # Let RedAction control the motors while its rotation is active
    if red_end_time is not None:
        return

    # No blue object detected
    if p_vector is None or p_vector[2] != "blue":
        # Check if stop_all is needed
        stop_all()
        return

    x = p_vector[0]

    # Object is centered
    if abs(x) <= TOLERANCE:
        stop_all()
        return

    # Calculate movement speed based on distance from center
    speed = min(TRACK_MIN_SPEED + int(TRACK_GAIN * abs(x)), 80)

    try:
        if x > 0:
            # Object is on the right
            move_left(speed)
        else:
            # Object is on the left
            move_right(speed)

        # Short movement before returning to scheduler
        time.sleep(0.05)

    finally:
        stop_all()
        
#Round Robin Task Table
TASK_TABLE = [
	(TimerCheck,	("start_time",),	"running"),
	(cameraCapture,	("cap",), 	"image"),
	(ColorCounter, ("image",),	"color_counts"),
	(ColorLocator,	("image", "color_counts"),	"p_vector"),
	(RedAction, 	("p_vector", "red_end_time"), 	"red_end_time"),
	(IdleAction, 	("p_vector", "red_end_time"), 	None),
	(GreenAction, 	("p_vector", "red_end_time"), 	None),
	(BlueAction, 	("p_vector", "red_end_time"), 	None),
]

def roundRobinScheduler(task_table, variables):
	while True:
		for task, inputs, output in task_table:
			result = task(*[variables[name] for name in inputs])
			
			if output is not None:
				variables[output] = result
				
			if not variables["running"]:
				return
	

def main():
    start_time = time.monotonic()

    cap = cv2.VideoCapture(0)

    if not cap.isOpened():
        raise RuntimeError("Could not open RASPBOT camera")
        
    try:
        StartupAction()
		
        variables = {
		"start_time": start_time,
		"cap": cap,
		"running": True,
		"red_end_time": None,
		}
        
        roundRobinScheduler(TASK_TABLE, variables)
    
    finally:
        stop_all()
        cap.release()

if __name__ == "__main__":
	main()