import glob
import os
import sys
import random
import time
import math
import os
import pandas as pd
from matplotlib import pyplot as plt

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

import pygame
import carla

#Collect volume-to-capacity data
class Volume_to_Capacity():
    def __init__(self, world, vehicles, map):
        self.vehicles = vehicles
        self.map = map
        self.world = world
        self.waypoint_list = map.generate_waypoints(2.0)
        self.timer = time.time()

        #Length of vehicle tesla model 3
        self.vehicle_size = 4.79178 #m
        self.lanes = pd.DataFrame(columns = ["start_x","start_y","start_z","end_x","end_y","end_z"])
        self.generate_roads()
        self.table = pd.DataFrame(columns = self.columns)

    #Get distance between two waypoints
    def length(self,wp1,wp2):
        loc1 = wp1.transform.location
        loc2 = wp2.transform.location

        dst = ((loc1.x-loc2.x)**2+(loc1.y-loc2.y)**2+(loc1.z-loc2.z)**2)**0.5
        return dst 
    
    def half_way(self,wp1,wp2):
        loc1 = wp1.transform.location
        loc2 = wp2.transform.location

        x = (loc1.x+loc2.x)/2
        y = (loc1.y+loc2.y)/2
        z = ((loc1.z+loc2.z)/2)+2

        loc = carla.Location(x=x,y=y,z=z)
        return loc

    def generate_roads(self):
        #sort waypoints
        sorted_waypoints = {}
        for wp in self.waypoint_list:
            key = str(wp.road_id)+" "+str(wp.lane_id)
            if key in sorted_waypoints:
                sorted_waypoints[key].append(wp)
            else:
                sorted_waypoints[key] = [wp]

        #Start and end of lanes and length
        self.lengths = {}
        self.columns = []

        for k, v in sorted_waypoints.items():

            self.lanes.at[k, "start_x"] = v[0].transform.location.x
            self.lanes.at[k, "start_y"] = v[0].transform.location.y
            self.lanes.at[k, "start_z"] = v[0].transform.location.z
            self.lanes.at[k, "end_x"] = v[-1].transform.location.x
            self.lanes.at[k, "end_y"] = v[-1].transform.location.y
            self.lanes.at[k, "end_z"] = v[-1].transform.location.z

            self.lengths[k] = self.length(v[0], v[-1])
            self.columns.append(k)
        
        #visual debugging
        for k, v in self.lanes.items():
            break
            color = carla.Color(random.randint(0,255),random.randint(0,255),random.randint(0,255))
            start_point = v[0].transform.location
            start_point.z += 1
            end_point = v[1].transform.location
            end_point.z += 1
            self.world.debug.draw_line(start_point, end_point, color = color, thickness=0.1, life_time=-1)
            loc = self.half_way(v[0],v[1])
            print(loc)
            self.world.debug.draw_string(loc, str(self.lengths[k]), color = carla.Color(0, 0, 0), life_time=100)
    #Record an instance of the vehicles
    def record_data(self, seconds):
        i = len(self.table.index)
        if i <= seconds:
            self.table.loc[i] = [0 for x in range(len(self.columns))]

            for k, veh in self.vehicles.items():
                #Theoretical space needed by each each
                if veh.role =="captain" or veh.role == "roaming":
                    length = self.vehicle_size + 4
                else:
                    length = self.vehicle_size + 0.5

                road_id = self.map.get_waypoint(veh.location).road_id
                lane_id = self.map.get_waypoint(veh.location).lane_id
                key = str(road_id)+" "+str(lane_id)
                self.table.at[i,key] += length

            if i == seconds:
                self.save_table_to_csv()
    #Save all instances recorded
    def save_table_to_csv(self):
        #Enter location of folder to save csv to
        parent = ""
        os.chdir(parent)
        folder = str(time.time())
        os.makedirs(folder)
        #Enter location of folder to save csv to
        path = "" + folder
        self.table.to_csv(path+"/data.csv")
        self.lanes.to_csv(path+"/lanes.csv")
        print("Done")