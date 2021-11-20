import glob
import os
import sys
import random
import time
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
from data_collection import Volume_to_Capacity

from agents.navigation.basic_agent import BasicAgent


client = carla.Client("localhost",2000)
world = client.get_world()
spawn_points = world.get_map().get_spawn_points()

map = world.get_map()

waypoints = map.generate_waypoints(2.0)

class Vehicle():

    def __init__(self, transform):
        #Spawn vehicle
        self.spawn(transform)
        self.id = self.vehicle.id

        self.velocity = 0
        self.desired_velocity = 0

        self.desired_acceleration = 0
        self.accelerating = False

        self.driver_mode = 0 #Searching, in platoon, splitting, merging
        self.platoon = None
        self.role = "roaming" #Captain, member, roaming

        self.roaming_agent()
        self.random_destination()
        self.set_throttle(1.0)
        self.next_locations()

        self.action_queue = []
        self.manoeuvres_clock = time.time()

    #Spawn the vehicle
    def spawn(self, transform):
        blueprint_library = world.get_blueprint_library()
        model_3 = blueprint_library.filter("model3")[0]
        self.vehicle = world.spawn_actor(model_3,transform)

    #Set the speed of vehicle and whether acceleration is set too
    def set_speed(self, speed, fps = None, time = None):
        if time is None:
            self.desired_velocity = speed
            self.agent._local_planner.set_speed(speed)
            self.desired_acceleration = 0
            self.accelerating = False

            self.update_attributes()
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

    #Set the throttle of vehicle
    def set_throttle(self, throttle):
        self.agent._local_planner._vehicle_controller.max_throt = throttle
    #Update the vehicles attriutes such as speed, velocity, location, etc.
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

    #Roaming control
    def set_destination(self):
        self.destination = random.choice(spawn_points)
        self.agent.set_destination((self.destination.location.x,
                                    self.destination.location.y,
                                    self.destination.location.z))

    #Set up the agent to drive the vehicle
    def roaming_agent(self):
        self.agent = BasicAgent(self.vehicle, target_speed=self.desired_velocity)

    #Make the vehicle adjust direction and throttle based on route
    def roam(self):
        control = self.agent.run_step()
        control.manual_gear_shift = False
        self.vehicle.apply_control(control)

        if self.agent.done():
            self.random_destination()
    
    #Get the next roads and lanes the agent will take
    def next_locations(self):
        a = self.agent._local_planner._waypoints_queue
        queue = []
        for x in range(len(a)):
            b = a[x][0].road_id
            c = a[x][0].lane_id
            if len(queue)== 0 or queue[-1][0] != b or queue[-1][1] != c:
                queue.append([b, c])
            if len(queue) == 4:
                break
        
        self.location_queue = queue
 
if __name__ == '__main__':
    #Parameters to adjust
    FPS = 9
    CHECK_INTERVALS = 0.25
    SPAWN_AMOUNT = 40
    DEFAULT_SPEED = 10
    clock = time.time()
    vehicles = {}

    x = 0
    #Load in vehicles with random spawn points
    random.shuffle(spawn_points)
    for n, transform in enumerate(spawn_points):
        
        if x >= SPAWN_AMOUNT:
            break
        x+=1
        veh =  Vehicle(transform)
        vehicles[veh.id] = veh
        vehicles[veh.id].set_speed(DEFAULT_SPEED)

    v2c = Volume_to_Capacity(world, vehicles, map)
    time.sleep(3)

    fps_clock = pygame.time.Clock()

    v2c.timer = time.time()
    loops = []

    #Main loop of the simulation
    while True:
        fps_clock.tick_busy_loop(FPS)

        #Record an instance of data every second
        if v2c.timer + 1 < time.time() and False:
            v2c.record_data(120)
            v2c.timer = time.time()

        #Updates next speed to achieve if a set scceleration was used
        for key, veh in vehicles.items():

            veh.update_attributes()

            if veh.accelerating:
                next_speed = next(veh.speeds, False)
                
                if next_speed:
                    veh.set_speed(next_speed)   
                else:
                    veh.accelerating = False
            veh.roam()
