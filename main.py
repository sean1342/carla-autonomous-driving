from pipeline import *
import glob
import os
import sys
import carla
import time
import random
import time
import numpy as np
import matplotlib.pyplot as plt


path_to_install = "/home/sean/CARLA_0.9.8"

CAM_WIDTH = 640
CAM_HEIGHT = 480

def process_frame(image):
    # format image
    i = np.array(image.raw_data, dtype=np.uint8)
    shaped = i.reshape((CAM_HEIGHT, CAM_WIDTH, 4))
    rgb = shaped[:,:,:3]
    blur = cv2.GaussianBlur(rgb, (5,5), 0)

    processed = detect_lanes(blur)

    plt.imshow(processed)
    plt.show()

def control():
    pass

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
    os.system("DISPLAY= " + path_to_install + "/./CarlaUE4.sh -quality-level=low -opengl &")

    client = carla.Client("localhost", 2000)
    client.set_timeout(10.0)
    
    world = client.load_world('Town05')
    
    blueprint_library = world.get_blueprint_library()

    vehicle_bp = blueprint_library.filter("model3")[0]
    print(vehicle_bp)
    
    spawn_point = random.choice(world.get_map().get_spawn_points())

    vehicle = world.spawn_actor(vehicle_bp, spawn_point)
    actor_list.append(vehicle)
    vehicle.set_autopilot(True)

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
