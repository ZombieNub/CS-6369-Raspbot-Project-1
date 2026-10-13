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

def cameraCapture():
    '''
    Use this for RedAction and GreenAction too.
    Returns 'frame', a snapshot that the camera takes.
    '''
    cap = cv2.VideoCapture(0)

    try:
        if not cap.isOpened():
            raise RuntimeError("Could not open RASPBOT camera")

        ret, frame = cap.read()

        if not ret:
            raise RuntimeError("Could not capture camera frame")

    finally:
        cap.release()
        return frame

# An admittedly slapdash way to do round robin scheduling, but python doesn't have a interrupt thing
ACTION_MAX_TIME = 10

def IdleAction():
    # The bot needs to do nothing and wait. For safety, I'm going to zero out the wheels in case a previous function set the wheels to some speed.
    stop_all()
    time.sleep(ACTION_MAX_TIME)

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

def GreenAction(p_vector):
    """
    This is going to be a bit intricate so listen up. I'm using a multiline comment in case you're confused.
    p_vector represents the object's location in the 'camera space' irrelevant of the 3D scene. So I'm lacking data to simply do an atan2
    Furthermore, even if I could, rotation is entirely done by setting wheel speed. I cannot rotate to a given angle
    Which means in order to rotate the raspbot to face the object, I need to keep rotating until the object is detected to be in the center

    Also, I don't know what p_vector is. I'm going to assume it's either an array or tuple, so either [x, y] or (x, y).
    Furthermore, if x is positive then the object is to the right, and if x is negative then the object is to the left
    """
    begin_time = time.monotonic()
    
    tolerance = 0.25
    v = p_vector
    while math.abs(v[0])/2 < tolerance:
        if time.monotonic() - begin_time < ACTION_MAX_TIME:
            break
        if v[0] > 0:
            rotate_right(20 * math.abs(v[0])/2) # Rotate faster if the object is far away. Adjust if needed. Might not be needed at all
        else:
            rotate_left(20 * math.abs(v[0])/2)
        v = GetPVector() # TODO: REPLACE WITH ColorLocator ONCE ITS IMPLEMENTED

def BlueAction(p_vector):
    """
    Move until the command object is centered.
    NOTE: I reversed the move_left() and move_right() functions. I'm not sure why the controls are reversed?
    I also kept a lot of the speeds relatively slow and I'm assuming we loop until it's centered.
    """
    if p_vector is None:
        stop_all()
        return

    tolerance = 0.25

    try:
        while True:
            # Check if the camera lost the object
            if p_vector is None:
                #print("Lost target object. Stopping.")
                break

            x = p_vector[0]

            # Check if object is within tolerance of the center
            if abs(x) <= tolerance:
                #print("Target centered.")
                break

            if p_vector[2] != "blue":
                #print("Lost target object. Stopping.")
                break

            # Speed proportional to how far the object is from center
            speed = max(int(0.5 * abs(x)), 5)
            #speed = min(max(speed, 10), 60)
            #speed = 10

            if x > 0:
                #move_right(speed)
                move_left(speed)
            else:
                #move_left(speed)
                move_right(speed)

            time.sleep(0.05)

            # Capture new frame and update position vector
            frame = cameraCapture()
            p_vector = ColorLocator(frame, color_counts)

    finally:
        stop_all()

def main():
	start_time=time.monotonic()
	try:
		StartupAction()
		while True:
			if not TimerCheck(start_time):	
				break
				
			# Implement other functions here
            image = get_camera_image()
            color_counts = ColorCounter(image)
            p_vector = ColorLocator(image, color_counts)

            # Comment out functions if needed for testing
            if p_vector is None:
                IdleAction()
            elif p_vector[2] == "green":
                GreenAction(p_vector)
            elif p_vector[2] == "blue":
                BlueAction(p_vector)
            elif p_vector[2] == "red":
                RedAction()

	finally:
		stop_all()

'''
if __name__ == "__main__":
	main()

'''
