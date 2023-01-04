import glob
import os
import sys
import carla
import random
import time
import cv2
import time
import numpy as np
import matplotlib.pyplot as plt
from PIL import Image


CAM_WIDTH = 640
CAM_HEIGHT = 480

def detect_lanes(image):
    height = image.shape[0]
    width = image.shape[1]
    rect_left = np.array([[0, height], [0, int(height * 0.7)], [int(width / 2), int(height / 2)], [int(width / 2), height]], dtype=np.int32)
    mask_left = np.zeros(image.shape ,dtype=np.uint8)
    cv2.fillConvexPoly(mask_left, rect_left, 255)

    rect_right = np.array([[int(width / 2), height], [int(width / 2), int(height / 2)], [width, int(height * 0.7)], [width, height]], dtype=np.int32)
    mask_right = np.zeros(image.shape ,dtype=np.uint8)
    cv2.fillConvexPoly(mask_right, rect_right, 255)

    cropped_left = cv2.bitwise_and(image, mask_left)
    cropped_right = cv2.bitwise_and(image, mask_right)

    lines_left = cv2.HoughLinesP(cropped_left, 2, np.pi/180, 100, np.array([]), 40, 5)
    lines_right = cv2.HoughLinesP(cropped_right, 2, np.pi/180, 100, np.array([]), 40, 5)

    line_image = np.zeros(image.shape)
    if lines_left is not None:
        for line in lines_left:
            left_x1, left_y1, left_x2, left_y2 = line.reshape(4)

    line_image = np.zeros(image.shape)
    if lines_right is not None:
        for line in lines_right:
            right_x1, right_y1, right_x2, right_y2 = line.reshape(4)

    left_line_x = [left_x1, left_x2]
    left_line_y = [left_y1, left_y2]
    right_line_x = [right_x1, right_x2]
    right_line_y = [right_y1, right_y2]

    return left_line_x, left_line_y, right_line_x, right_line_y

def process_frame(image):
    # format image
    i = np.array(image.raw_data, dtype=np.uint8)
    shaped = i.reshape((CAM_HEIGHT, CAM_WIDTH, 4))
    rgb = shaped[:, :, :3]

    # preprocessing
    gray = cv2.cvtColor(rgb, cv2.COLOR_RGB2GRAY)
    blur = cv2.GaussianBlur(gray, (7, 7), 0)
    edges = cv2.Canny(blur, 30, 90)

    left_line_x, left_line_y, right_line_x, right_line_y = detect_lanes(edges)
    left_slope = (left_line_y[0] - left_line_y[1]) / (left_line_x[0] - left_line_x[1])
    right_slope = (right_line_y[0] - right_line_y[1]) / (right_line_x[0] - right_line_x[1])
    print(left_slope, right_slope)

    control(left_slope, right_slope)

    plt.plot(left_line_x, left_line_y, color="blue", linewidth=1)
    plt.plot(right_line_x, right_line_y, color="blue", linewidth=1)
    plt.imshow(rgb)
    plt.show()

def control(left_slope, right_slope):
    throttle_val = 0.2
    steering_val = 0

    vehicle.apply_control(carla.VehicleControl(throttle=throttle_val, steer=steering_val))

    print("here")

actor_list = []


# boilerplate for server connection
try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])

except IndexError:
    pass

try:
    client = carla.Client("localhost", 2000)
    client.set_timeout(5.0)
    
    world = client.load_world('Town05')
    
    blueprint_library = world.get_blueprint_library()

    vehicle_bp = blueprint_library.filter("model3")[0]
    print(vehicle_bp)
    
    spawn_point = random.choice(world.get_map().get_spawn_points())

    vehicle = world.spawn_actor(vehicle_bp, spawn_point)
    actor_list.append(vehicle)
    # vehicle.set_autopilot(True)

    cam_bp = blueprint_library.find('sensor.camera.rgb')
    cam_bp.set_attribute('image_size_x', str(CAM_WIDTH))
    cam_bp.set_attribute('image_size_y', str(CAM_HEIGHT))
    cam_bp.set_attribute('fov', '110')
    
    spawn_point = carla.Transform(carla.Location(x=2.5, z=0.7))

    hood_cam = world.spawn_actor(cam_bp, spawn_point, attach_to=vehicle)

    actor_list.append(hood_cam)

    hood_cam.listen(lambda data: process_frame(data))

    time.sleep(100)

finally:
    for actor in actor_list:
        actor.destroy()
        
        print("destroyed all actors")