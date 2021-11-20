import glob
import os
import sys
import random
import time
import numpy as np
import math

try:
    sys.path.append(glob.glob('../../../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

import carla


client = carla.Client("localhost",2000)
#Wait for client to run
client.set_timeout(10.0)
world = client.load_world('Town02_Opt')
# Toggle all buildings off
world.unload_map_layer(carla.MapLayer.All)

time.sleep(5)
