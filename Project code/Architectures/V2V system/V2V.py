#Sort captain vehicle list and queue timing, change hopping from nearest distance to interception point
import glob
import os
import sys
import random
import time
import numpy as np
import math
import copy

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
from CLV_tree import BPlusTree
from agents.navigation.global_route_planner import GlobalRoutePlanner
from agents.navigation.global_route_planner_dao import GlobalRoutePlannerDAO

class V2V():

    def __init__(self, attached_to, vehicles, world_map):
        #Environment and vehicle
        self.vehicle = attached_to
        self.vehicles = vehicles
        self.map = world_map

        #Radio technology
        self.receiver_sensitivity = -92
        self.frequency = 27.9 #GHz
        self.wave_speed = 3*math.pow(10,8)
        self.wave_length = self.wave_speed/(self.frequency*math.pow(10,9))
        self.Pt = 17 #dBm - Power applied to transmitting antenna
        self.com_range = 100

        #Member
        self.captains = []
        self.zone_leader = None

        #Captain
        self.CLV_tree = None
        self.CLV_pointer = None
        self.LEQueue = None

        self.merge_count = {}
        self.near_captains = {}

        if self.vehicle.role == "captain":
            self.select_captain_role()
    
    #Power recieved by avehicle
    def recieved_power(self, veh):
        loc = veh.location

        x = self.vehicle.location.x - loc.x
        y = self.vehicle.location.y - loc.y
        z = self.vehicle.location.z - loc.z

        d = math.sqrt(pow(x,2)+pow(y,2)+pow(z,2))
        pi = math.pi

        #Power received in dm
        self.Pr = self.Pt + 20*math.log(self.wave_length/(4*pi*d),10)

    #return the route between 2 locaitons with a greedy method
    def trace_route(self, start_waypoint, loc):

        end_waypoint = carla.Location(loc["x"], loc["y"], 0)

        # Setting up global router
        dao = GlobalRoutePlannerDAO(self.map, 2.0)
        grp = GlobalRoutePlanner(dao)
        grp.setup()

        # Obtain route plan
        route = grp.trace_route(
            start_waypoint,
            end_waypoint)

        return route   
    
    #Receive message
    def receive(self, sender, title, msg = None):
        my_info = self.details()

        if self.vehicle.role is "captain":
            #Roaming vehicle introduction
            if title == "introduce":
                if  msg["lane_id"] == my_info["lane_id"]: # and msg["road_id"] == my_info["road_id"] :
                    message = {
                        "location": self.vehicle.location,
                        "speed": self.vehicle.velocity,
                        "intersection": self.map.get_waypoint(self.vehicle.location).next_until_lane_end(2)[-1]
                    }
                    self.transmit(sender, "intro_response", message)

            #Receive join request from vehicle
            if title == "join_request":

                self.insert_vehicle(msg,sender)
                #Send join confirmation
                self.transmit(sender, "join_confirmed")

            # update vehicles details
            if title == "update_info":

                self.delete_vehicle(sender)
                self.insert_vehicle(msg,sender)

            #Confirm the transfer was successful
            if title == "captain_transfer_confirmed":
                self.select_member_role(sender)

            if title == "vehicles_in_zone":
                #Get the distance of all members to that point
                count = 0
                for k, v in self.CLV_pointer.items():
                    values = copy.copy(self.CLV_tree.query(v))
                    for i, j in values.items():
                        dst = self.dist_between_locations(msg["location"], j["location"])
                        if dst <= 50:
                            count += 1
                msg = {
                    "count": count
                }
                self.transmit(sender, "vehicle_count",msg)
            
            #Send the total number of vehicles from this captains zone that are in another captaisn zone
            if title == "vehicle_count":
                self.merge_count[msg["count"]] = sender
            
            #Send zone information to another captain
            if title == "zone_information":
                msg = {
                    "CLV_tree": copy.copy(self.CLV_tree),
                    "CLV_pointer": copy.copy(self.CLV_pointer),
                    "LEQueue": copy.copy(self.LEQueue)
                }

                self.transmit(sender, "initiate_merge", msg)

            #Start the merge
            if title == "initiate_merge":                  
                #Merge CLV_pointer
                self.CLV_pointer.update(msg["CLV_pointer"])

                #Merge LEQueue
                for k, v in msg["LEQueue"].items():
                    if k in self.LEQueue:
                        for i in v:
                            self.LEQueue[k].append(v)
                    else:
                        self.LEQueue[k] = v
                
                #Merge CLV_tree
                for k, v in msg["CLV_pointer"].items():
                    if msg["CLV_tree"].query(v) is not None:
                        values = copy.copy(msg["CLV_tree"].query(v))
                        self.CLV_tree.insert(v, values)

                #Tell new vehicles about new zone leader
                for k, v in msg["CLV_pointer"].items():
                    self.transmit(k, "new_zone_leader", self.vehicle.id)

                #Change previous zone leader into captain
                    self.transmit(sender, "captain_transfer_confirmed")

            #Message received to be sent to destination
            if title == "message":
                veh_id = msg["id"]
                loc = msg["location"]
                message = msg["message"]

                x = loc["x"]
                y = loc["y"] 

                #Check whether the message location is the the cluster zone
                #Cluster zone is captains commuication range
                if (x-self.vehicle.location.x)**2 + (y-self.vehicle.location.y)**2 <= self.com_range**2:
                    in_zone = True
                else:
                    in_zone = False

                if in_zone:
                    for k, v in self.CLV_pointer.items():
                        if k != self.vehicle.id and k != sender:
                            self.transmit(k, "destination_message", msg)
                    print("Messaged delivered")
                else:
                    route = self.trace_route(self.vehicle.location, msg["location"])
                    locations = {}

                    for v in route:
                        loc = v[0].transform.location
                        #Pick node that is the closest to the communication range
                        dist = (loc.x-self.vehicle.location.x)**2 +  (loc.y-self.vehicle.location.y)**2
                        if dist <= self.com_range**2:
                            locations[math.sqrt(dist)] = loc
                        else:
                            break

                    #Get the intersection point of the message route and comm range
                    keys = sorted(locations)
                    intersection_point =  locations[keys[0]]
                    id_location = {}

                    #Get the distance of all members to that point
                    for k, v in self.CLV_pointer.items():
                        values = copy.copy(self.CLV_tree.query(v))
                        
                        for i, j in values.items():
                            dst = self.dist_between_locations(intersection_point, j["location"])
                            id_location[dst] = i
                    #Get the member with the shortest distance
                    keys = sorted(id_location)
                    member_id =  id_location[keys[0]]
                    msg["ids"] = keys

                    if self.vehicle.id == member_id:
                        print("Captain will hop message")
                    else:
                        print("Captain", self.vehicle.id, "sent message to member", member_id, "to hop")
                    self.send_message(True, member_id, msg)

            #Find one-hop neighbour
            if title == "message_hop":
                self.message_hop(msg)
                    
        if self.vehicle.role is "roaming":
            #All possible captains found
            if title == "intro_response":
                self.captains.append(sender)

            if title == "join_confirmed":
                self.captains.clear()
                self.zone_leader = sender
                self.vehicle.role = "member"

        if self.vehicle.role is "member":

            #Find one-hop neighbour
            if title == "message_hop":
                self.message_hop(msg)

            if title == "message":
                self.send_message(False, self.zone_leader, msg)

            #Have the member become the zone captain
            if title == "captain_transfer":
                self.select_captain_role(ct = msg["CLV_tree"], cp = msg["CLV_pointer"], leq = msg["LEQueue"])

                for k, v in self.CLV_pointer.items():
                    if k != self.vehicle.id and k != sender:
                        self.transmit(k, "new_zone_leader", self.vehicle.id)

                self.transmit(sender, "captain_transfer_confirmed")

            if title == "new_zone_leader":
                self.zone_leader = msg
            
            #Find a new zone
            if title == "find_new_zone":
                self.near_captains = {}
                self.merge_count = {}
                self.vehicle.role = "roaming"
                self.zone_leader = None
                self.introduce()
                self.joining_event()

    #Send message
    def transmit(self, veh_id, title, msg = None):
        self.vehicles[veh_id].v2v.receive(self.vehicle.id,title,msg)

    def details(self):
        wp = self.map.get_waypoint(self.vehicle.location)
        info = {"road_id": wp.road_id, 
                "lane_id": wp.lane_id*-1}
        return info

    def dist_between_locations(self, loc1, loc2):
        x = math.pow((loc1.x - loc2.x),2)
        y = math.pow((loc1.y - loc2.y),2)
        z = math.pow((loc1.z - loc2.z),2)

        loc = math.sqrt((x+y+z))

        return loc

    def predict_location(self, veh_id, time):
        veh = self.vehicles[veh_id]
        x = veh.location.x + veh.vehicle.get_velocity().x*time
        y = veh.location.y + veh.vehicle.get_velocity().y*time
        z = veh.location.z + veh.vehicle.get_velocity().z*time
        loc = carla.Vector3D(x,y,z)
        return loc
    
    def predict_time(self, veh_id, loc):
        veh = self.vehicles[veh_id]
        dst = self.dist_between_locations(loc, veh.location)

        if veh.velocity!= 0:
            return dst/veh.velocity
        else:
            return None
    
    def distance_to_end(self):
        end = self.map.get_waypoint(self.vehicle.location).next_until_lane_end(2)[-1].transform.location
        start = self.vehicle.location

        dst = self.dist_between_locations(start, end)

        return dst

    #Timestamps between vehicle and potential captain
    def timestamps(self, veh_id):
        times = []
        veh = self.vehicles[veh_id]
        #Time till vehicle reaches junction
        loc1 = self.map.get_waypoint(self.vehicle.location).next_until_lane_end(2)[-1].transform.location
        t1 = self.predict_time(veh_id,loc1)

        #Time till zone captain reaches junction
        loc2 = self.map.get_waypoint(veh.location).next_until_lane_end(2)[-1].transform.location
        t2 = self.predict_time(veh_id,loc2)

        #Time till vehicles exceed communication range
        self_loc = self.vehicle.location
        self_vec = self.vehicle.vehicle.get_velocity()

        cap_loc = veh.location
        cap_vec = veh.vehicle.get_velocity()

        #If both vehicles have the same velocity then they will never be out of comm distance
        x  = self_vec.x - cap_vec.x
        y  = self_vec.y - cap_vec.y
        z  = self_vec.z - cap_vec.z

        vec = math.sqrt(pow(x,2)+pow(y,2)+pow(z,2))

        if vec!= 0:
            #Solve for time using position and velocity
            a = (self_loc.x - cap_loc.x)
            b = x
            c = (self_loc.y - cap_loc.y)
            d = y
            e = (self_loc.z - cap_loc.z)
            f = z
            g = self.com_range

            sub_part = (2*a*b + 2*c*d + 2*e*f)**2 - 4*(b**2 + d**2 + f**2)*(a**2 + c**2 - g**2 + e**2)

            if sub_part >= 0:
                part_1 = (sub_part)**0.5 -2*a*b -2*c*d -2*e*f
                part_2 = 2*(b**2 + d**2 + f**2)
                time = part_1/part_2

                #If time is negative it means they are out of communication so t3 is 0 
                if time < 0:
                    t3 = 0 
                else: 
                    t3 = time
            else:
                t3 = None
        else:
            t3 = None 

        #return times
        return [t1, t2, t3]
        
    #Roaming
    def introduce(self):
        #Get all captains between a distance and send a hello message
        
        for key, veh in self.vehicles.items():
            if key != self.vehicle.id and veh.role == "captain":
                loc = veh.location

                distance = self.dist_between_locations(loc, self.vehicle.location)

                if distance <=self.com_range:
                    self.transmit(veh.id,"introduce",self.details())
    
    def simularity_score(self):
        scores = []
        for cap in self.captains:
            times = self.timestamps(cap)
            #Check if all times are none 
            valid = all(t is None for t in times)
            #If all times are None then no vehicle is moving so min time is 0
            if valid is not True:
                min_time = min(x for x in times if x is not None)
            else:
                min_time = 0
            #Get distance bertween vehicles at start, middle, and end of time
            veh = self.vehicle
            cap_start = self.vehicles[cap].location
            self_vector = veh.vehicle.get_velocity()
            captain_vector = self.vehicles[cap].vehicle.get_velocity()

            x  = cap_start.x - veh.location.x
            y  = cap_start.y - veh.location.y
            z  = cap_start.z - veh.location.z 

            lc = self.dist_between_locations(cap_start, veh.location)

            t = min_time/2
            x = (cap_start.x + (captain_vector.x*t)) - (veh.location.x + (self_vector.x*t))
            y = (cap_start.y + (captain_vector.y*t)) - (veh.location.y + (self_vector.y*t))
            z = (cap_start.z + (captain_vector.z*t)) - (veh.location.z + (self_vector.z*t))

            lm = math.sqrt(pow(x,2)+pow(y,2)+pow(z,2))
            x = (cap_start.x + (captain_vector.x*min_time)) - (veh.location.x + (self_vector.x*min_time))
            y = (cap_start.y + (captain_vector.y*min_time)) - (veh.location.y + (self_vector.y*min_time))
            z = (cap_start.z + (captain_vector.z*min_time)) - (veh.location.z + (self_vector.z*min_time))
            
            lf = math.sqrt(pow(x,2)+pow(y,2)+pow(z,2))
            #Weights
            w = [0.5,0.3,0.2]
            #simularity score
            s = min_time/(w[0]*lc + w[1]*lm + w[2]*lf)
            scores.append(s)
        return scores

    def joining_event(self):
        #If the vehicle finds possible captains, send a join request to the one with the best simularity score
        if len(self.captains)!=0:

            #Get scores of all captains
            scores = self.simularity_score()
            
            #Get index of the best score as it directly relates to captain
            i = scores.index(max(scores))

            #Get when the 2 vehicles will be out of communication range
            timestamps = self.timestamps(self.captains[i])

            if timestamps[2] is None:
                leave_time = None
            else:
                leave_time = time.time()+timestamps[2]

            msg = {
                "location": self.vehicle.location,
                "speed": self.vehicle.velocity,
                "update_timestamp": time.time(),
                "leave_timestamp": leave_time,
                "dist_till_end": self.distance_to_end()
            }

            self.transmit(self.captains[i], "join_request" ,msg)
        #Else become a captain    
        else:
            self.select_captain_role()
    
    def update_info(self):
        if self.vehicle.role == "captain":
            timestamps = self.timestamps(self.vehicle.id)
        else:
            timestamps = self.timestamps(self.zone_leader)

        if timestamps[2] is None:
            leave_time = None
        else:
            leave_time = time.time()+timestamps[2]

        msg = {
            "location": self.vehicle.location,
            "speed": self.vehicle.velocity,
            "update_timestamp": time.time(),
            "leave_timestamp": leave_time,
            "dist_till_end": self.distance_to_end()
        }

        if self.vehicle.role == "captain":
            self.delete_vehicle(self.vehicle.id)
            self.insert_vehicle(msg, self.vehicle.id)
        else:
            self.transmit(self.zone_leader, "update_info" ,msg)

    def send_message(self, hop, veh_id, message):
        #Have the message sent to the captain or one-hop neighbour
        if hop is True:
            self.transmit(veh_id, "message_hop", message)
        else:
            self.transmit(veh_id, "message", message)

    def select_member_role(self, zone_leader):
        self.CLV_tree = None
        self.CLV_pointer = None
        self.LEQueue = None

        self.merge_count = {}
        self.near_captains = {}

        self.vehicle.role = "member"
        self.zone_leader = zone_leader

    #Captain
    def select_captain_role(self, ct = None, cp = None, leq = None):
        self.zone_leader = self.vehicle.id
        self.vehicle.role = "captain"
        self.CLV_tree = ct if ct != None else BPlusTree()
        self.CLV_pointer = cp if cp != None else {}
        self.LEQueue = leq if leq != None else {}

        #Insert self into the data storage first
        msg = {
            "location": self.vehicle.location,
            "speed": self.vehicle.velocity,
            "update_timestamp": time.time(),
            "leave_timestamp": None,
            "dist_till_end": self.distance_to_end()
        }

        self.insert_vehicle(msg, self.vehicle.id)

    def insert_vehicle(self, msg, veh_id):
        key = msg["dist_till_end"]
        value = msg.pop("dist_till_end")

        #Add entry to CLV Tree
        if self.CLV_tree.query(key) is None:
            self.CLV_tree.insert(key, {veh_id : msg})
        else:
            values = copy.copy(self.CLV_tree.query(key))
            values[veh_id] = msg
            self.CLV_tree.insert(key, values)

        #Create pointer to entry
        self.CLV_pointer[veh_id] = key

        #Add leave timestamp entry
        if msg["leave_timestamp"] is not None:
            if msg["leave_timestamp"] in self.LEQueue:
                self.LEQueue[msg["leave_timestamp"]].append(key)
            else:
                self.LEQueue[msg["leave_timestamp"]] = [key]

        #Sort LEQueue by key (leave timestamp)
        if len(self.LEQueue)!=0:
            dic = {}
            for k in sorted(self.LEQueue):
	            dic[k] = self.LEQueue[k]
            self.LEQueue = dic.copy()
        #Sort CLV_pointer by value (Distance till end of the road)
        self.CLV_pointer = dict(sorted(self.CLV_pointer.items(), key=lambda item: item[1]))

    def delete_vehicle(self, veh_id):
        key = self.CLV_pointer[veh_id]
        values = copy.copy(self.CLV_tree.query(key))
        LEQueue_key = values[veh_id]["leave_timestamp"]

        #Remove CLV_Tree entry
        if len(values) == 1:
            self.CLV_tree.delete(key)
        else:
            values.pop(veh_id)
            self.CLV_tree.insert(values)
        
        #Remove LEQueue entry
        if LEQueue_key is not None:
            LEQueue_values = self.LEQueue[LEQueue_key]
            if len(LEQueue_values) == 1:
                self.LEQueue.pop(LEQueue_key)
            else:
                self.LEQueue[LEQueue_key].remove(key)

        #Remove CLV_Pointer entry
        self.CLV_pointer.pop(veh_id)

    def check_position(self):
       total = len(self.CLV_pointer)
       
       #for x, y in self.LEQueue.items()

       if total <=1:
           pos = 0.5
           return pos
       else:
           position = list(self.CLV_pointer.keys()).index(self.vehicle.id)+1
           return position/total

    def position_check(self):
        pos = self.check_position()
        if pos <= 0.3 or pos>=0.7:
            vehs = self.CLV_tree.middlemost_leaf()
            for k, v in vehs.items():
                new_cap = k
                break
            if new_cap != self.vehicle.id:
                self.delete_vehicle(new_cap)

                msg = {
                    "CLV_tree": copy.copy(self.CLV_tree),
                    "CLV_pointer": copy.copy(self.CLV_pointer),
                    "LEQueue": copy.copy(self.LEQueue)
                }             

                self.transmit(new_cap, "captain_transfer", msg)

    def splitting_check(self):
        veh_ids = []
        #Get CLV Tree keys of soon to leave vehicles
        for leave_time, positions in self.LEQueue.copy().items():
            if leave_time - time.time() <=3:
                for pos in positions:
                    #Get Vehicle ids using key
                    vehs = self.CLV_tree.query(pos)
                    for veh_id, details in vehs.copy().items():
                        if details["leave_timestamp"] - time.time() <= 3:

                            #Send message to find new zone
                            self.delete_vehicle(veh_id)
                            self.transmit(veh_id, "find_new_zone")
    
    def message_hop(self, msg):
        loc = msg["location"]
        message = msg["message"]
        #To prevent back and forth loop between zones, the next neighbour will ignore the previous zone
        ids = msg["ids"]

        x = loc["x"]
        y = loc["y"]
        
        hop_neighbours = {}

        #Get every vehicle in radius
        for key, veh in self.vehicles.items():
            #if key != self.vehicle.id:
            loc = veh.location

            #Check that neighbour is within senders comm distance
            distance = self.dist_between_locations(loc, self.vehicle.location)

            if distance <=self.com_range:
                #Find distance between neighbours and message location
                dist = (x-veh.location.x)**2 +  (y-veh.location.x)**2
                hop_neighbours[math.sqrt(dist)] = key

            #Find neighbour with the shortest distance to the location
        if len(hop_neighbours)!= 0:                     
            keys = sorted(hop_neighbours)
            neighbour_id =  hop_neighbours[keys[0]]
            if self.vehicle.id == neighbour_id:
                print(self.vehicle.role, self.vehicle.id, "Hopped message to", neighbour_id, "who is a", self.vehicles[neighbour_id].role, "of", veh.v2v.zone_leader)
                self.send_message(False, neighbour_id, msg)
            else:
                print("Couldn't find a member to continue")
        
        else:
            print("Couldn't find a member to continue")#Queue attempt in u seconds 
            pass
    
    def near_captions(self):
        #Find all the captains that are within half the current captains comm range
        for k, v in self.vehicles.items():
            if k != self.vehicle.id and v.role == "captain":
                dst = self.dist_between_locations(self.vehicle.location, v.location)
                if dst <= (self.com_range/2):
                    if k in self.near_captains:
                        self.near_captains[k] += 1
                else:
                        self.near_captains[k] = 0

    #Check all captains that have been in half the current captains comm range for atleast 3 cycles
    def merge_check(self):
        merge_options = []
        for k, v in self.near_captains.items():
           if v >= 3:
               merge_options.append(k)
               msg = {
                   "location": self.vehicle.location
               }
               self.transmit(k, "vehicles_in_zone", msg)

        #Start merge process if true
        if len(merge_options)>0:
            key = sorted(self.merge_count)
            cap_id = self.merge_count[key[0]]
            self.transmit(cap_id, "zone_information")
                
                
            