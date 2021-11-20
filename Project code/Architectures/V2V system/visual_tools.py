import glob
import os
import sys
import random
import time
import numpy as np
import math
import os

try:
    sys.path.append(glob.glob('../../../carla/dist/carla-*%d.%d-%s.egg' % (
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

class V2V_debug():

    def __init__(self, world, vehicles, fps):
        self.world = world
        self.vehicles = vehicles
        self.fps = fps
    
    #Links draw vehicle member to platoon leader
    def draw_links(self):
        for key, veh in self.vehicles.items():
            if veh.role == "captain":
                start_point = carla.Location(x=veh.location.x, y=veh.location.y, z=veh.location.z+1)
                for key, val in veh.v2v.CLV_pointer.items():
                    x = self.vehicles[key]
                    end_point = carla.Location(x=x.location.x, y=x.location.y, z=x.location.z+1)
                    self.world.debug.draw_line(start_point, end_point, thickness=0.1, life_time=(1.3/self.fps))