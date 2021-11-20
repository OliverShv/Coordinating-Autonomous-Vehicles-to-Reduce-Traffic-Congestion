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
from data_collection import Speeds_and_distances, Speeds

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
        self.test_waypoint()
        self.set_throttle(1.0)
        self.next_locations()

        self.action_queue = []
        self.length = 4.79178 #m

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
    def set_throttle(self, throttle: float):
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
    
    #Athorises whether a vehicle cna change lanes or not
    def can_change(self, statement: bool):
        self.agent._local_planner._can_change = statement

    #Waypoint used for location to drive too for multi-lane merging
    def test_waypoint(self):
        self.agent.set_destination((10.336160,
                                    201.616669,
                                    0.000000))

    #Random destination is selected.
    def random_destination(self):
        self.destination = random.choice(spawn_points)
        self.agent.set_destination((self.destination.location.x,
                                    self.destination.location.y,
                                    self.destination.location.z))

    #Set a destination in the distance
    def set_destination(self):
        while True:
            self.destination = random.choice(spawn_points)
            loc1 = self.destination.location
            loc2 = self.location
            road_id = self.map.get_waypoint(self.destination.location).road_id
            #Get a new location thta is not on the same road
            if road_id != self.map.get_waypoint(self.location).road_id:
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
    LPS = 10
    CHECK_INTERVALS = 0.25
    SPAWN_AMOUNT = 30
    DEFAULT_SPEED = 10

    clock = time.time()
    vehicles = {}

    spawns = []

    #lane 1 spawn points
    a1 = carla.Transform(carla.Location(x=-0.908655, y=-200.451691, z=0.3), carla.Rotation(pitch=2.701025, yaw=179.940445, roll=0.000000))
    b1 = carla.Transform(carla.Location(x=-8.904019, y=-200.460953, z=0.8), carla.Rotation(pitch=4.633666, yaw=-179.923553, roll=0.000000))
    c1 = carla.Transform(carla.Location(x=-16.906158, y=-200.471619, z=1.6), carla.Rotation(pitch=6.566940, yaw=-179.923553, roll=0.000000))
    d1 = carla.Transform(carla.Location(x=-24.910124, y=-200.482300, z=2.6), carla.Rotation(pitch=7.247833, yaw=-179.923553, roll=0.000000))
    e1 = carla.Transform(carla.Location(x=-32.910118, y=-200.492966, z=3.6), carla.Rotation(pitch=7.247833, yaw=-179.923553, roll=0.000000))
    f1 = carla.Transform(carla.Location(x=-40.910110, y=-200.503647, z=4.6), carla.Rotation(pitch=7.247833, yaw=-179.923553, roll=0.000000))
    g1 = carla.Transform(carla.Location(x=-48.910103, y=-200.514313, z=5.7), carla.Rotation(pitch=7.247833, yaw=-179.923553, roll=0.000000))
    h1 = carla.Transform(carla.Location(x=-56.910095, y=-200.524994, z=6.7), carla.Rotation(pitch=7.247833, yaw=-179.923553, roll=0.000000))
    i1 = carla.Transform(carla.Location(x=-64.910088, y=-200.535660, z=7.7), carla.Rotation(pitch=7.247833, yaw=-179.923553, roll=0.000000))
    j1 = carla.Transform(carla.Location(x=-72.910080, y=-200.546326, z=8.7), carla.Rotation(pitch=6.302886, yaw=-179.923553, roll=0.000000))
    lane_1 = [a1,b1,c1,d1,e1,f1,g1,h1,i1,j1]
    #lane 2 spawn points
    a2 = carla.Transform(carla.Location(x=-0.912292, y=-203.951691, z=0.3), carla.Rotation(pitch=2.701025, yaw=179.940445, roll=0.000000)) 
    b2 = carla.Transform(carla.Location(x=-8.899314, y=-203.960953, z=0.8), carla.Rotation(pitch=4.633657, yaw=-179.923553, roll=0.000000))
    c2 = carla.Transform(carla.Location(x=-16.900478, y=-203.971619, z=1.6), carla.Rotation(pitch=6.566696, yaw=-179.923553, roll=0.000000))
    d2 = carla.Transform(carla.Location(x=-24.900469, y=-203.982300, z=2.6), carla.Rotation(pitch=7.247833, yaw=-179.923553, roll=0.000000))
    e2 = carla.Transform(carla.Location(x=-32.905750, y=-203.992966, z=3.6), carla.Rotation(pitch=7.247833, yaw=-179.923553, roll=0.000000))
    f2 = carla.Transform(carla.Location(x=-40.909454, y=-204.003647, z=4.6), carla.Rotation(pitch=7.247833, yaw=-179.923553, roll=0.000000))
    g2 = carla.Transform(carla.Location(x=-48.909447, y=-204.014328, z=5.7), carla.Rotation(pitch=7.247833, yaw=-179.923553, roll=0.000000))
    h2 = carla.Transform(carla.Location(x=-56.909439, y=-204.024994, z=6.7), carla.Rotation(pitch=7.247833, yaw=-179.923553, roll=0.000000))
    i2 = carla.Transform(carla.Location(x=-64.909431, y=-204.035660, z=7.7), carla.Rotation(pitch=7.247833, yaw=-179.923553, roll=0.000000))
    j2 = carla.Transform(carla.Location(x=-72.909424, y=-204.046341, z=8.7), carla.Rotation(pitch=6.301915, yaw=-179.923553, roll=0.000000))
    lane_2 = [a2,b2,c2,d2,e2,f2,g2,h2,i2,j2]

    #new_spawns = [j1,i1,h1,g1,f1,e1,d1,c1,b1,a1] #Single lane
    new_spawns = [e2,d2,c2,b2,a2,e1,d1,c1,b1,a1] #Scenario 1
    #new_spawns = [g1,f1,e1,d1,c1,e2,d2,c2,b2,a2] #Scenario 2
    #new_spawns = [g2,f2,e2,d2,c2,e1,d1,c1,b1,a1] #Scenario 3
    #new_spawns = [g1,f1,e1,d1,c1,b1,a1,e2,d2,c2] #Scenario 4
    #new_spawns = [g2,f2,e2,d2,c2,b2,a2,e1,d1,c1] #Scenario 5

    #Add new spawn points
    for x in new_spawns:
        spawn_points.append(x)
        spawns.append(len(spawn_points)-1)

    x = 0

    #Load in vehicles with spawn points created
    for n, transform in enumerate(spawn_points):
        if n in spawns:
            veh =  Vehicle(transform)
            vehicles[veh.id] = veh
            vehicles[veh.id].set_speed(DEFAULT_SPEED)
            if x >= SPAWN_AMOUNT:
                break
            x+=1

    time.sleep(3)
    lps_clock = pygame.time.Clock()

    for key, veh in vehicles.items():
        veh.platoon = Platoon(veh, vehicles, map)

    for key, veh in vehicles.items():
        veh.platoon.search_for_platoons()

    speeds = Speeds_and_distances(vehicles)
    debug_links = Platoon_debug(world, vehicles, LPS)
    speeds.timer = time.time()

    #Main loop of the simulation
    while True:
        lps_clock.tick_busy_loop(LPS)
        #Show links between vehicles
        debug_links.draw_links()

        #Records data for analysis
        if speeds.timer + 1 < time.time() and False:
            speeds.record_data(90)
            speeds.timer = time.time()

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
