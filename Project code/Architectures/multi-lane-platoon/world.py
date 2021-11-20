import glob
import os
import sys
import time

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
#world to be loaded in
world = client.load_world('Town05_Opt')
# Toggle all buildings off
world.unload_map_layer(carla.MapLayer.All)
map = world.get_map()

time.sleep(5)

#Visual debug for spawn point locations
def check_spawn_points():
    spawn_points = world.get_map().get_spawn_points()

    for n, transform in enumerate(spawn_points):
        text = [map.get_waypoint(transform.location).road_id, map.get_waypoint(transform.location).lane_id]
        world.debug.draw_string(transform.location, str(text), color = carla.Color(0, 0, 0), life_time=100)
#check_spawn_points()