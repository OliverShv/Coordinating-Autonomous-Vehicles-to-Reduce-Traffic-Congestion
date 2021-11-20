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

from Platoon import Platoon
from visual_tools import Platoon_debug 

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

        self.LPS = None

        self.action_queue = []
        self.length = 4.79178
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
        self.velocity = math.sqrt(pow(vector.x,2)+pow(vector.y,2)+pow(vector.z,2))*3.6

        vector = self.vehicle.get_acceleration()
        self.acceleration = math.sqrt(pow(vector.x,2)+pow(vector.y,2)+pow(vector.z,2))

        self.location = self.vehicle.get_transform().location

        self.rotation = self.vehicle.get_transform().rotation

        front = carla.Location(self.location.x,self.location.y,self.location.z)
        back = carla.Location(self.location.x,self.location.y,self.location.z)
        vec = self.rotation.get_forward_vector()
        total = ((vec.x)**2 + (vec.y)**2 + (vec.z)**2)**0.5

        front.x = front.x + (self.length/2)*(vec.x/total)
        front.y = front.y + (self.length/2)*(vec.y/total)
        front.z = front.z + (self.length/2)*(vec.z/total)

        back.x = back.x - (self.length/2)*(vec.x/total)
        back.y = back.y - (self.length/2)*(vec.y/total)
        back.z = back.z - (self.length/2)*(vec.z/total)

        self.front_location = front
        self.rear_location = back

    #Roaming control
    def random_destination(self):
        road = map.get_waypoint(self.vehicle.get_transform().location)
        for x in range(8):
            loc = road.next_until_lane_end(2)[-1]
            road = random.choice(loc.next(2))

        self.destination = road.transform
        
        self.agent.set_destination((self.destination.location.x,
                                    self.destination.location.y,
                                    self.destination.location.z))

    #Set a distance that is far away from the vehicle
    def set_destination(self):
        while True:
            self.destination = random.choice(spawn_points)
            loc1 = self.destination.location
            loc2 = self.location
            road_id = map.get_waypoint(self.destination.location).road_id
            #Get a new location thta is not on the same road
            if road_id != map.get_waypoint(self.location).road_id:
                x = math.pow((loc1.x - loc2.x),2)
                y = math.pow((loc1.y - loc2.y),2)
                z = math.pow((loc1.z - loc2.z),2)

                loc = math.sqrt((x+y+z))
                if loc <30:
                    break

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
    LPS = 9
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
        vehicles[veh.id].LPS = LPS
        veh.location
        vec = veh.rotation.get_forward_vector()

    v2c = Volume_to_Capacity(world, vehicles, map)
    time.sleep(3)
    lps_clock = pygame.time.Clock()

    for key, veh in vehicles.items():
        veh.platoon = Platoon(veh, vehicles, map)

    debug_links = Platoon_debug(world, vehicles, LPS)
    v2c.timer = time.time()
    loops = []

    #Main loop of the simulation
    while True:
        lps_clock.tick_busy_loop(LPS)
        #Drive
        debug_links.draw_links()

        #Record an instance of data every second
        if v2c.timer + 1 < time.time() and False:
            v2c.record_data(120)
            v2c.timer = time.time()

        for key, veh in vehicles.items():

            veh.update_attributes()
            #Run manoeuvre
            if veh.platoon.manoeuvres is not None:
                func = veh.platoon.manoeuvres
                func[0](func[1])

            if veh.role == "captain" and veh.manoeuvres_clock + CHECK_INTERVALS < time.time():
                veh.platoon.spacing_maintenance()
                veh.platoon.split_check()
                #check for merges
                if veh.platoon.action_state is None:
                    veh.platoon.merge_search()
                #If vehicle merges then it is no longer captain
                if veh.role == "captain":
                    veh.platoon.leaving_check()
                    
                veh.manoeuvres_clock = time.time()
            #Updates next speed to achieve if a set scceleration was used
            if veh.accelerating:
                next_speed = next(veh.speeds, False)
                
                if next_speed:
                    veh.set_speed(next_speed)   
                else:
                    veh.accelerating = False
            veh.roam()