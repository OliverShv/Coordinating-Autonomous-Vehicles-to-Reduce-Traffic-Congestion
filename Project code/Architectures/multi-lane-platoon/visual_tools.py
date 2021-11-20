import glob
import os
import sys
import random
import time
import numpy as np
import math
import os

try:
    sys.path.append(glob.glob('../../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

try:
    sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))) + '/../carla')
except IndexError:
    pass

import carla

#Get visual links between members of a platoon and place colorued boxes aorund them depending on their role
class Platoon_debug():

    def __init__(self, world, vehicles, fps):
        self.world = world
        self.vehicles = vehicles
        self.fps = fps

    def draw_links(self):
        for key, veh in self.vehicles.items():
            #Platoon links
            if veh.role == "captain":
                color = carla.Color(255,0,0)
                #Create new location objects to increase height by 1 
                for val in range(len(veh.platoon.members)):
                    if val +1 < len(veh.platoon.members):
                        mem_1 = veh.platoon.members[val]
                        mem_2 = veh.platoon.members[val+1]
                        start_point = carla.Location(x=mem_1.location.x, y=mem_1.location.y, z=mem_1.location.z+1)
                        end_point = carla.Location(x=mem_2.location.x, y=mem_2.location.y, z=mem_2.location.z+1)
                        self.world.debug.draw_line(start_point, end_point, thickness=0.1, life_time=(1.3/self.fps))
            
            elif veh.role == "member":
                color = carla.Color(0,255,0)
            
            elif veh.role =="roaming":
                color = carla.Color(0,0,255)
            #Role box
            box = carla.BoundingBox(veh.location,carla.Vector3D(1,1,1))
            rotation = carla.Rotation(0,0,0)
            self.world.debug.draw_box(box,rotation, life_time=(1.3/self.fps), color=color)