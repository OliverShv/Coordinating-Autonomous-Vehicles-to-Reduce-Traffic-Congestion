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
    sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))) + '/../../carla')
except IndexError:
    pass

import pygame
import carla

from agents.navigation.basic_agent import BasicAgent  # pylint: disable=import-error

from agents.navigation.local_planner import LocalPlanner

from visual_tools import V2V_debug
from V2V import V2V

#________EARLY VERSION OF THE VEHICLE CLASS________#
client = carla.Client("localhost",2000)
world = client.get_world()
spawn_points = world.get_map().get_spawn_points()

map = world.get_map()

waypoints = map.generate_waypoints(2.0)

class Vehicle():

    def __init__(self, transform):

        self.spawn(transform)
        #ID
        self.id = self.vehicle.id

        self.velocity = 0
        #self.min_velocity = 0
        self.desired_velocity = 0
        #self.max_velocity = 0

        self.desired_acceleration = 0
        #self.max_acceleration = 0
        self.accelerating = False

        self.driver_mode = 0 #Searching, in platoon, splitting, merging
        self.role = "roaming" #Captain, follower, null if searching

        self.roaming_agent()
        self.random_destination()
        self.set_throttle(1.0)

        #Vehicles In Range
        self.bat_to_coms = 10000
        
        self.v2v = None

        self.action_queue = []
    
    def spawn(self, transform):
        #transform = random.choice(spawn_points)

        blueprint_library = world.get_blueprint_library()
        model_3 = blueprint_library.filter("model3")[0]
        self.vehicle = world.spawn_actor(model_3,transform)

    #Attribute control
    def set_speed(self, speed, fps = None, time = None):
        if time is None:
            self.agent._local_planner.set_speed(speed)
        else:
            self.update_attributes()
            self.desired_velocity = speed
            self.desired_acceleration = (self.desired_velocity - self.velocity)/time

            start = self.velocity+(self.desired_acceleration/fps)
            speeds = [start]

            for x in range(fps*time-2):
                contin = speeds[-1]+(self.desired_acceleration/fps)
                speeds.append(contin)
            speeds.append(self.desired_velocity)

            self.speeds = iter(speeds)
            self.accelerating = True

    def set_throttle(self, throttle):
        self.agent._local_planner._vehicle_controller.max_throt = throttle
    
    def update_attributes(self):
        vector = self.vehicle.get_velocity()
        self.velocity = math.sqrt(pow(vector.x,2)+pow(vector.y,2)+pow(vector.z,2))

        vector = self.vehicle.get_acceleration()
        self.acceleration = math.sqrt(pow(vector.x,2)+pow(vector.y,2)+pow(vector.z,2))

        self.location = self.vehicle.get_transform().location

        self.rotation = self.vehicle.get_transform().rotation

    #Roaming control
    def random_destination(self):
        self.destination = random.choice(spawn_points)
        self.agent.set_destination((self.destination.location.x,
                                    self.destination.location.y,
                                    self.destination.location.z))

    def roaming_agent(self):
        self.agent = BasicAgent(self.vehicle, target_speed=self.desired_velocity)

    def roam(self):
        control = self.agent.run_step()
        control.manual_gear_shift = False
        self.vehicle.apply_control(control)

        if self.agent.done():
            self.random_destination()
 
if __name__ == '__main__':
    fps = 60
    clock = time.time()
    vehicles = {}

    spawn_amount = 10
    print(spawn_amount)
    for n, transform in enumerate(spawn_points):
        if n >= spawn_amount:
            break
        veh = Vehicle(transform)
        vehicles[veh.id] = veh
        vehicles[veh.id].set_speed(30, fps, 10)    

    time.sleep(3)
    fps_clock = pygame.time.Clock()

    #When the stages should start
    start_1_start = time.time()
    stage_2_repeater = time.time()
    #Stages
    stage_1 = True
    stage_2 = False
    stage_3 = False

    v2v_checker = False
    V2V_network = False

    v2v_links = V2V_debug(world, vehicles, fps)

    send_msg = time.time()
    send = True

    test = time.time()

    while True:

        fps_clock.tick_busy_loop(fps)
        if stage_2 or stage_3:  
            v2v_links.draw_links()

        #Start V2V
        if start_1_start + 1 < time.time() and stage_1:
            for key, veh in vehicles.items():
                veh.v2v = V2V(veh, vehicles, map)
            stage_1 = False
            stage_2 = True

        if stage_2_repeater +1 < time.time() and stage_3:
            stage_2_repeater = time.time()
            v2v_checker = True
        
        if stage_2 and start_1_start + 3 < time.time():
            V2V_network = True

        #Drive
        for key, veh in vehicles.items():

            veh.update_attributes()

            if  veh.role == "roaming" and V2V_network:
                veh.v2v.introduce()
                veh.v2v.joining_event()
                stage_3 = True
            
            if veh.role == "member" and v2v_checker:
                veh.v2v.update_info()
                #Test message sent
                if send_msg + 8 < time.time() and send:
                    #print("Start with vehicle", veh.id, "sent to captain", veh.v2v.zone_leader)
                    m = {
                        "id": veh.id,
                        "location": {"x": veh.destination.location.x, "y": veh.destination.location.y},
                        "message": 1
                    }
                    #veh.v2v.send_message(False, veh.v2v.zone_leader, m)
                    send = False

            #List to check
            if veh.role == "captain" and v2v_checker:
                veh.v2v.update_info()
                veh.v2v.splitting_check()
                veh.v2v.near_captions()
                veh.v2v.merge_check()
                veh.v2v.position_check()

            if veh.accelerating:
                next_speed = next(veh.speeds, False)
                
                if next_speed:
                    veh.set_speed(next_speed)   
                else:
                    veh.accelerating = False
            veh.roam()

        if stage_3:
            V2V_network = False
            stage_2 = False
            
        v2v_checker = False