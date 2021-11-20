#Fix exiting
import glob
import os
import sys
import math

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
from agents.navigation.global_route_planner import GlobalRoutePlanner
from agents.navigation.global_route_planner_dao import GlobalRoutePlannerDAO

#Platooning archiecture for single-lanes
class Platoon():
    def __init__(self, attached_to, vehicles, world_map):
        self.vehicle = attached_to
        self.vehicles = vehicles
        self.map = world_map
        
        self.members = []
        self.platoon_leader = None
        #Stores available platoons to join
        self.platoon_options = []

        self.manoeuvres = None
        self.action_state = "Searching"
        self.DISTANCE_TILL_JOIN = 20
        self.PLATOON_PROXIMITY = 10
        self.platoon_distance = 1
        self.PLATOON_SPACING = 6
        self.search_for_platoons()

    #__________USEFUL FUNCTIONS__________________
    def dist_between_locations(self, loc1, loc2):
        x = math.pow((loc1.x - loc2.x),2)
        y = math.pow((loc1.y - loc2.y),2)
        z = math.pow((loc1.z - loc2.z),2)

        loc = math.sqrt((x+y+z))

        return loc
    #distance between vehicle and end of the road
    def distance_till_intersection(self, veh):
        intersection = self.map.get_waypoint(veh.location).next_until_lane_end(2)[-1].transform.location
        dst = self.dist_between_locations(veh.location, intersection)
        return dst
    #Checks for obstacles between 2 vehicles
    def obstacle_check(self, veh1, veh2):
        dst_veh1 = self.distance_till_intersection(veh2)
        dst_veh2 = self.distance_till_intersection(veh1)
        
        #If captain distance is smaller then the vehicle is in behind it
        if dst_veh2 < dst_veh1: 
            behind =  True
        else:
            behind =  False

        #Check if any vehicles are between vehicle and end of platoon
        obstacle = False
        a = self.map.get_waypoint(veh1.location).road_id
        b = self.map.get_waypoint(veh1.location).lane_id
        
        for key, veh in self.vehicles.items():
            if key != veh1.id and key != veh2.id:
                c = self.map.get_waypoint(veh.location).road_id
                d = self.map.get_waypoint(veh.location).lane_id

                #Vehicles are on the same lane and road
                if a == c and b == d:
                    dst_veh = self.distance_till_intersection(veh)
                    if dst_veh2 < dst_veh and dst_veh1 > dst_veh and behind:
                        obstacle = True
                    elif dst_veh2 > dst_veh and dst_veh1 < dst_veh and behind is False:
                        obstacle = True

        return obstacle
    #__________SEARCH FOR PLATOONS__________________
    #Search for platoons to join
    def search_for_platoons(self):
        self.action_state = "Searching"
        a = self.map.get_waypoint(self.vehicle.location).road_id
        b = self.map.get_waypoint(self.vehicle.location).lane_id

        for key, veh in self.vehicles.items():
            if key != self.vehicle.id and veh.role == "captain" and veh.platoon.action_state == None and len(veh.platoon.members) < veh.platoon.platoon_size:

                #Check that rear platoon member and vehicel are on same road
                rear_veh = veh.platoon.members[-1]

                c = self.map.get_waypoint(rear_veh.location).road_id
                d = self.map.get_waypoint(rear_veh.location).lane_id
                
                if a == c and b == d:

                    mem_dst = self.distance_till_intersection(rear_veh)
                    self_dst = self.distance_till_intersection(self.vehicle)
                    
                    #Check if vehicle is behind platoon
                    behind = False
                    if mem_dst <= self_dst:
                        behind = True

                    if behind:
                        obstacle = self.obstacle_check(self.vehicle, rear_veh)
                        
                        #If there is no obstacle then vehicle could join platoon
                        if obstacle is False and veh.platoon.action_state is None:
                            #Check that no obstacles are between vehicle and platoon
                            dist = self.dist_between_locations(self.vehicle.location, veh.location)
                            self.platoon_options.append([veh, dist])
        
        if len(self.platoon_options) == 0:
            self.form_platoon()
        else:
            self.join_search()
    #__________FORMATION__________________
    #Forms a platoon
    def form_platoon(self):
        self.action_state = "Forming"
        #self.vehicle.agent._proximity_vehicle_threshold = 2
        self.vehicle.role = "captain"
        self.platoon_leader = self.vehicle
        self.members.append(self.vehicle)
        self.platoon_speed = self.vehicle.desired_velocity
        self.vehicle.agent._proximity_vehicle_threshold = self.PLATOON_PROXIMITY
        self.action_state = None
        self.platoon_size = 10

    #__________JOINING__________________

    #Select the best platoon to join from ones previously found
    def join_search(self):
        captain_obj = None
        dst = 1000000
        self.action_state = "Joining"
        #Join platoon with shortest distance
        for v in self.platoon_options:
            if v[1] < dst and v[0].platoon.action_state is None:
                captain_obj = v[0]
                dst = v[1]
        self.join_position([captain_obj])
    #Attempt to join a selected platoon
    def join_position(self, parameters):
        captain_obj = parameters[0]
        member = captain_obj.platoon.members[-1]

        a = self.map.get_waypoint(self.vehicle.location).road_id
        b = self.map.get_waypoint(self.vehicle.location).lane_id
        c = self.map.get_waypoint(member.location).road_id
        d = self.map.get_waypoint(member.location).lane_id

        #Make sure vehicles are still on the same road
        if a == c and b == d:
            captain_obj.platoon.action_state = "Vehicle_Joining"
            platoon_speed = captain_obj.platoon.platoon_speed
            self.platoon_options = []

            self.vehicle.set_speed(platoon_speed)

            #Get last member of platoon and distance
            dst_member = self.distance_till_intersection(member)
            dst_self = self.distance_till_intersection(self.vehicle)

            dst = math.sqrt(math.pow((dst_self - dst_member), 2))

            if dst <= self.DISTANCE_TILL_JOIN:
                self.join_manoeuvre([captain_obj])

            else:      
                self.vehicle.set_speed(platoon_speed*2)
                self.manoeuvres = [self.join_position, [captain_obj]]
        else:
            self.cancel_join(captain_obj)

    #Finish the join manoeuvre
    def join_manoeuvre(self, parameters):
        self.platoon_leader = parameters[0]
        self.vehicle.role = "member"
        self.action_state = None
        platoon_speed =  self.platoon_leader.platoon.platoon_speed
        self.vehicle.agent._proximity_vehicle_threshold = self.PLATOON_SPACING
        self.vehicle.set_speed(platoon_speed)

        self.platoon_leader.platoon.action_state = None
        self.manoeuvres = None
        platoon_members =  self.platoon_leader.platoon.members
        platoon_members.append(self.vehicle)
    #Cancel the join attempt    
    def cancel_join(self, captain_obj):
        self.action_state = None
        captain_obj.platoon.action_state = None
        self.vehicle.set_speed(self.vehicle.desired_velocity)
        self.manoeuvres = None
        self.search_for_platoons()

    #__________________MERGING________________

    #Search for a platoon to merge with
    def merge_search(self):
        a = self.map.get_waypoint(self.vehicle.location).road_id
        b = self.map.get_waypoint(self.vehicle.location).lane_id

        for key, veh in self.vehicles.items():

            if key != self.vehicle.id and veh.role == "captain" and veh.platoon.action_state is None and len(self.members)+len(veh.platoon.members) < veh.platoon.platoon_size:

                c = self.map.get_waypoint(veh.location).road_id
                d = self.map.get_waypoint(veh.location).lane_id
                
                #platoon captains are on the same lane and road
                if a == c and b == d:
                    c_dst = self.distance_till_intersection(veh)
                    self_dst = self.distance_till_intersection(self.vehicle)
                    
                    #Vehicle must be behind captain to do merge
                    if c_dst <= self_dst:
                        #Check for obstcles between the first vehicle of this platoon and the last vehicle of potential merging platoon

                        veh1 = veh.platoon.members[-1]
                        veh2 = self.members[0]
                        obstacle = self.obstacle_check(veh1,veh2)

                        if obstacle is False:

                            self.merge_position([veh, True])
                            break
    
    #Attempt to join platoon found
    def merge_position(self, parameters):
        cap = parameters[0]
        first_run = parameters[1]

        a = self.map.get_waypoint(self.vehicle.location).road_id
        b = self.map.get_waypoint(self.vehicle.location).lane_id

        c = self.map.get_waypoint(cap.location).road_id
        d = self.map.get_waypoint(cap.location).lane_id

        #Check if merge is still possible
        if a == c and b == d:

            self.action_state = "Merging"

            cap.platoon.action_state = "Merging"
            veh1 = cap.platoon.members[-1]
            veh2 = self.members[0]

            dst = self.dist_between_locations(veh1.location,veh2.location)
            platoon_speed = cap.platoon.platoon_speed

            if dst <= self.DISTANCE_TILL_JOIN:

                self.vehicle.agent._proximity_vehicle_threshold = self.PLATOON_SPACING
                self.set_platoon_speed(platoon_speed)

                self.manoeuvres = None
                self.merge_manoeuvre(cap)   

            else:
                if first_run:

                    self.set_platoon_speed(15)
                    self.manoeuvres = [self.merge_position, [cap, False]]
        else:
            self.cancel_merge(cap)

    #Merge into platoon
    def merge_manoeuvre(self, cap):

        for veh in self.members:
            veh.platoon.platoon_leader = cap
            cap.platoon.members.append(veh)

        self.members.clear()
        self.vehicle.role = "member"
        self.action_state = None
        cap.platoon.action_state = None
    #Cancel the merge attempt
    def cancel_merge(self, cap):
        self.set_platoon_speed(10)
        self.action_state = None
        cap.platoon.action_state = None
        self.manoeuvres = None

    #__________Leaving___________

    #Check for vehicles that are leaving a platoon
    def leaving_check(self):
        #If when 20 metres from the intersection, check whether vehicles are leaving platoon
        if len(self.members)!=1:
            self.get_next_locations()

            cap_loc = self.vehicle.location_queue
            for mem in self.members:
                
                #location_queue is empty as vehicle is close to destination
                if len(mem.location_queue) == 0:
                    mem.location_queue.append([self.map.get_waypoint(mem.location).road_id, self.map.get_waypoint(mem.location).lane_id])

                #Check if the vehicle will be on the same road of captain
                if mem.location_queue[-1] not in cap_loc and mem is not self.vehicle:
                    mem.platoon.leaving_position([])
    
    #Set up leaving position 
    def leaving_position(self, parameters):
        self.platoon_leader.platoon.get_next_locations()

        #location_queue is   empty as vehicle is close to destination
        if len(self.vehicle.location_queue) == 0:
            self.vehicle.location_queue.append([self.map.get_waypoint(self.vehicle.location).road_id, self.map.get_waypoint(self.vehicle.location).lane_id])

        local_route = self.platoon_leader.location_queue
        my_route = self.vehicle.location_queue

        #Get the point at which vehicle leaves
        exit_point = my_route[0]
        for i in my_route:
            if i in local_route:
                exit_point = i

        #Make sure previous member is not stuck behind
        members = self.platoon_leader.platoon.members
        i = members.index(self.vehicle)

        leave = True
        if self.vehicle is not members[-1]:
            #Locations of self and vehicle behind
            self_location = [self.map.get_waypoint(self.vehicle.location).road_id, self.map.get_waypoint(self.vehicle.location).lane_id]
            mem_location = [self.map.get_waypoint(members[i+1].location).road_id, self.map.get_waypoint(members[i+1].location).lane_id]

            if self_location == mem_location:
                leave = False

        if my_route[0] == exit_point and leave:
            self.platoon_leader.platoon.leaving_manoeuvre(self.vehicle)
        else:     
            self.manoeuvres = [self.leaving_position, []]
    #Finish leaving manoeuvre
    def leaving_manoeuvre(self, mem):
        self.members.remove(mem)

        mem.platoon.manoeuvres = None
        mem.platoon.platoon_leader = None
        mem.platoon.platoon_options = []
        mem.platoon.manoeuvres = None
        mem.role = "roaming"
        mem.platoon.search_for_platoons()
    
    #___________Splitting____________

    #Check for a platoon split
    def split_check(self):
        split_index = None
        cap_state = self.vehicle.agent._state

        for key, value in enumerate(self.members):
            mem_state = value.agent._state
            if value is not self.vehicle and str(mem_state) == "AgentState.BLOCKED_RED_LIGHT" and str(cap_state) != "AgentState.BLOCKED_RED_LIGHT":
                split_index = key
                break
        
        if split_index is not None:

            self.split_manoeuvre(key)

    #Finish the platoon split
    def split_manoeuvre(self, key):
        new_members = self.members[key:].copy()
        new_captain = new_members[0]
        new_captain.platoon.members = new_members
        new_captain.platoon.platoon_speed = new_captain.desired_velocity

        new_captain.role = "captain"
        new_captain.platoon.platoon_size = 10

        for mem in new_members:
            mem.platoon.platoon_leader = new_captain

        self.members = self.members[:key].copy()
    
    #___________CAPTAIN CONTROLS________
    #Set the platoon speed
    def set_platoon_speed(self, speed):
        for veh in self.members:
            veh.set_speed(speed)
            veh.platoon.platoon_speed = speed

    #Mantian the gap distances between vehicels in the platoon
    def spacing_maintenance(self):
        if len(self.members) != 1:

            #update attributes and check if a vehicel is stuff at a red light
            for key, value in enumerate(self.members):
                value.update_attributes()

            for x in range(len(self.members)):

                veh_1 = self.members[x]
                veh_2 = self.members[x+1]
                #Increase speed for vehicle to catch ups
                dst = self.dist_between_locations(veh_1.rear_location, veh_2.front_location)

                #Make sure vehicle has time to slow down if vehicle in front has slowed down below desired speed
                if dst < veh_2.platoon.platoon_distance:
                    c = ((dst-1-veh_2.platoon.platoon_distance)**2)**0.5
                    b = 1/(c**(c*0.2))

                    #Limit the mmultiplier to prevent the vehicle going too slow
                    if b < 0.75:
                        b = 0.75

                else:
                    b = (dst+1-veh_2.platoon.platoon_distance)**0.135
                    
                #Limits the speed to 1.5 times the platoon speed
                if b > 1.5:
                    b = 1.5

                #Accounts for vehicle stoppage
                if dst < self.vehicle.length:
                    speed = veh_1.velocity/2 + self.platoon_speed*0.5
                    veh_2.set_speed(speed*b)
                else:
                    veh_2.set_speed(self.platoon_speed*b)

                #End
                if x+2 == len(self.members):
                    break

    #Get the next locations of the platoon members
    def get_next_locations(self):
        for veh in self.members:
            veh.next_locations()
    #Get the route of the captain
    def get_local_route(self):
        self.get_next_locations()
        locs = self.vehicle.location_queue

        return locs[0]