# resort platoon after merge
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

#Platooning archiecture for multi-lane merging testing
class Platoon():
    def __init__(self, attached_to, vehicles, world_map):
        self.vehicle = attached_to
        self.vehicles = vehicles
        self.map = world_map
        self.speed_multiplier = 1
        
        self.members = []
        self.platoon_leader = None
        #Stores available platoons to join
        self.platoon_options = []

        self.manoeuvres = None
        self.action_state = "Searching"
        self.DISTANCE_TILL_JOIN = 20
        self.PLATOON_SPACING = 0
        self.PLATOON_PROXIMITY = 10
        self.platoon_distance = 0.5

    #__________USEFUL FUNCTIONS__________________
    def dist_between_locations(self, loc1, loc2):
        x = math.pow((loc1.x - loc2.x),2)
        y = math.pow((loc1.y - loc2.y),2)
        z = math.pow((loc1.z - loc2.z),2)

        loc = math.sqrt((x+y+z))

        return loc

    #distance between vehicle and end of the road
    def distance_till_intersection(self, veh):
        dst = 0
        waypoints = self.map.get_waypoint(veh.location).next_until_lane_end(2)
        for i in range(len(waypoints)):
            if i < len(waypoints)-1:
                dst += self.dist_between_locations(waypoints[i].transform.location, waypoints[i+1].transform.location)
        return dst

    #Distance from vehicle to start of the road it is on
    def distance_from_start(self, veh):
        dst = 0
        waypoints = self.map.get_waypoint(veh.location).previous_until_lane_start(2)
        for i in range(len(waypoints)):
            if i < len(waypoints)-1:
                dst += self.dist_between_locations(waypoints[i].transform.location, waypoints[i+1].transform.location)
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

    #Distance from a vehicle to the mid point between two other vehicles
    def distance_to_midpoint(self, mem, veh1, veh2):
        mp_x = (veh1.location.x + veh2.location.x)/2
        mp_y = (veh1.location.y + veh2.location.y)/2
        mp_z = (veh1.location.z + veh2.location.z)/2

        loc = carla.Location(x = mp_x, y = mp_y, z = mp_z)

        dst = self.dist_between_locations(mem.location, loc)
        return dst

    #Checks a vehicle positioning in relation to two vehicles
    def between_vehicles(self, veh1, veh2, veh3):

        front = veh1.location
        rear = veh2.location
        veh = veh3

        lane_start = self.map.get_waypoint(veh.location).previous_until_lane_start(0.5)[-2].transform.location
        #Get waypoints of entire road
        waypoints =  self.map.get_waypoint(lane_start).next_until_lane_end(0.5)

        #Get where start and rear would be in relation to road
        rear_dst = 1000
        front_dst = 1000
        for i in range(len(waypoints)):
            if i < len(waypoints):
                dst_1 = self.dist_between_locations(rear, waypoints[i].transform.location)
                dst_2 = self.dist_between_locations(front, waypoints[i].transform.location)
                #Closest to rear
                if dst_1 < rear_dst:
                    rear_dst = dst_1
                    rear_index = i
                #Closest to front
                if dst_2 < front_dst:
                    front_dst = dst_2
                    front_index = i
        
        rear_dst_to_intersection = self.distance_till_intersection(waypoints[rear_index].transform)
        front_dst_to_intersection = self.distance_till_intersection(waypoints[front_index].transform)
        veh_dst_to_interscetion = self.distance_till_intersection(veh)

        #print(rear_dst_to_intersection,veh_dst_to_interscetion,front_dst_to_intersection)
        #Added for licency
        if rear_dst_to_intersection + 1 >= veh_dst_to_interscetion and veh_dst_to_interscetion >=front_dst_to_intersection:
            is_between = "middle"
        elif rear_dst_to_intersection + 1 < veh_dst_to_interscetion and veh_dst_to_interscetion >front_dst_to_intersection:
            is_between = "behind"
        else:
            is_between = "infront"
        
        return is_between

    #__________SEARCH FOR PLATOONS__________________\
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

                mem_dst = self.distance_till_intersection(rear_veh)
                cap_dst = self.distance_till_intersection(veh)
                self_dst = self.distance_till_intersection(self.vehicle)
                
                if a == c :
                    #Check if vehicle is behind platoon and ther ear eno obstcles
                    if b == d:
                        
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
        veh = None

        for v in self.platoon_options:

            if v[1] < dst and v[0].platoon.action_state is None:
                captain_obj = v[0]
                dst = v[1]
                if len(v) == 3:
                    veh = v[2]
                else:
                    veh = None

        self.join_position([captain_obj, veh])

    #Attempt to join a selected platoon
    def join_position(self, parameters):
        captain_obj = parameters[0]

        if parameters[1] is not None:
            member = parameters[1]
            index = captain_obj.platoon.members.index(member)

            for i, mem in enumerate(captain_obj.platoon.members):
                if i <= index:
                    mem.set_speed(captain_obj.platoon.platoon_speed/2)

        else:
            #Single-lane platoon
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
                    self.manoeuvres = [self.join_position, [captain_obj, None]]
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
        #print(self.platoon_leader.platoon)
        self.platoon_leader.platoon.set_platoon_speed(self.platoon_leader.platoon.platoon_speed)

    #Cancel the join attempt
    def cancel_join(self, captain_obj):
        self.action_state = None
        captain_obj.platoon.action_state = None
        self.vehicle.set_speed(self.vehicle.desired_velocity)
        self.manoeuvres = None
        captain_obj.platoon.set_platoon_speed(captain_obj.platoon.platoon_speed)
        self.search_for_platoons()

    #__________________MERGING________________

    #Search for a platoon to merge with
    def merge_search(self):
        self.merge_stage = 1

        a = self.map.get_waypoint(self.vehicle.location).road_id
        b = self.map.get_waypoint(self.vehicle.location).lane_id

        for key, veh in self.vehicles.items():

            if key != self.vehicle.id and veh.role == "captain" and veh.platoon.action_state is None and len(self.members)+len(veh.platoon.members) <= veh.platoon.platoon_size:

                c = self.map.get_waypoint(veh.location).road_id
                d = self.map.get_waypoint(veh.location).lane_id

                veh1 = veh.platoon.members[-1]
                veh2 = self.members[0]
                mem_dst = self.distance_from_start(veh1)
                c_dst = self.distance_till_intersection(veh)
                self_dst = self.distance_till_intersection(self.vehicle)
                
                #platoon captains are on the same lane and road
                if a == c and b == d:

                    #Vehicle must be behind captain to do merge
                    if c_dst <= self_dst:
                        #Check for obstcles between the first vehicle of this platoon and the last vehicle of potential merging platoon

                        obstacle = self.obstacle_check(veh1,veh2)

                        if obstacle is False:
                            self.merge_position([veh, True, None, False])
                            break

                #Merge two platoon in different lanes        
                elif c_dst > 20 and mem_dst > 5:
                    self.vehicle.next_locations()

                    #Vehile wants to change lane onto captains lane
                    if a == c and ((b + 1 == d) or (b - 1 == d)):
                        if [a, b] != self.vehicle.location_queue[0]:
                            if [c, d] == self.vehicle.location_queue[0]:
                                rear = veh.platoon.members[-1]
                                front = veh
                                within_platoon = False
                                for mem in self.members:
                                    if self.between_vehicles(front, rear, mem) == "middle":
                                        within_platoon = True
                                        break
                                if within_platoon:
                                    captain_members = veh.platoon.members
                                    #Get the shortest disance to midpoint of two members of the platoon and add to options
                                    if len(captain_members) > 1:
                                        score = 10000
                                        member = None

                                        for x in range(len(captain_members)):
                                            if x < len(captain_members)-1:
                                                s =self.distance_to_midpoint(self.vehicle, captain_members[x], captain_members[x+1])
                                                if score > s:
                                                    score = s
                                                    member = veh.platoon.members[x+1]
                                                
                                        self.platoon_options.append([veh, score, member])

                                        self.merge_position([veh, True, member, True])
                                    else:
                                        self.merge_position([veh, True, None, True])
    
    #Attempt to join platoon found
    def merge_position(self, parameters):
        cap = parameters[0]
        first_run = parameters[1]
        mutli_lane = parameters[3]

        a = self.map.get_waypoint(self.vehicle.location).road_id
        b = self.map.get_waypoint(self.vehicle.location).lane_id

        c = self.map.get_waypoint(cap.location).road_id
        d = self.map.get_waypoint(cap.location).lane_id

        #Multi_lane merging
        if mutli_lane is True:
            self.PLATOON_PROXIMITY = 0
            cap.platoon.PLATOON_PROXIMITY = 0
            self.vehicle.agent._proximity_vehicle_threshold = self.PLATOON_PROXIMITY
            cap.agent._proximity_vehicle_threshold = cap.platoon.PLATOON_PROXIMITY
            #Check that all members of the platoon are in the same lane and road of the new captain
            cap_lane_road = [c, d]
            same_lane = True
            
            for mem in self.members:
                
                road = self.map.get_waypoint(mem.location).road_id
                lane = self.map.get_waypoint(mem.location).lane_id

                if [road,lane] != cap_lane_road:
                    same_lane = False
                    mem.can_change(False)

            #If vehicles are in different lanes
            if same_lane is False:
                
                #Size of gap to make to allow platoon to slide in
                member = parameters[2]
                size = 15
                #Split the size of vehicles

                if self.merge_stage == 1:

                    self.vehicles_to_influence = [[],[]]
                    self.infront = []
                    self.behind = []
                
                    #Get the vehicles in platoon A that need to space out

                    for mem in self.members:
                        pos = self.between_vehicles(cap, cap.platoon.members[-1], mem)
                        if pos == "infront":
                            self.infront.append(mem)
                        elif pos == "middle":
                            self.vehicles_to_influence[0].append(mem)
                        elif pos == "behind":
                            self.behind.append(mem)
                    
                    #If paltoon is further up road, change gaps selected
                    if len(self.infront)!=0: 
                        selected = 1
                    else:
                        selected = 0

                    #Get the start vehicle ot influence in captain platoon
                    max_dist = 1000
                    for i, mem in enumerate(cap.platoon.members):
                        dst = self.dist_between_locations(self.vehicles_to_influence[0][0].location, mem.location)
                        if dst < max_dist:
                            max_dist = dst
                            index = i + selected
                    
                    #Get the remaining vehicles
                    for i in range(len(self.vehicles_to_influence[0])):
                        if index+i < len(cap.platoon.members):
                            self.vehicles_to_influence[1].append(cap.platoon.members[index+i])
                    
                    #Space the vehicles
                    for lst in self.vehicles_to_influence:
                        for mem in lst:
                            mem.platoon.platoon_distance = size
                    
                    for x in range(len(self.infront)):
                            self.infront[x].platoon.platoon_distance = size/2
                        
                    for x in range(len(self.behind)):
                        if x == 0 and len(self.vehicles_to_influence[0]) <= len(self.vehicles_to_influence[1]):
                            self.behind[x].platoon.platoon_distance = size
                        else:
                            self.behind[x].platoon.platoon_distance = size/2

                    #Add next vehicle if not at front
                    i = self.members.index(self.vehicles_to_influence[0][0])

                    if i !=0:
                        self.vehicles_to_influence[0].insert(0,self.members[i-1])

                    for mem in self.vehicles_to_influence[1]:
                        if mem.platoon.platoon_leader == cap:
                            i = cap.platoon.members.index(mem)
                            break

                    if i !=0:
                        self.vehicles_to_influence[1].insert(0,cap.platoon.members[i-1])

                    self.merge_stage = 2

                if self.merge_stage == 2:

                    next_stage = True

                    #Find when the space is finished
                    for i in range(len(self.vehicles_to_influence)):
                        for j in range(len(self.vehicles_to_influence[i])):
                            if j < len(self.vehicles_to_influence[i])-1:
                                dst = self.dist_between_locations(self.vehicles_to_influence[i][j].location, self.vehicles_to_influence[i][j+1].location)
                                if dst < size:
                                    next_stage = False
                                    break

                    if next_stage:
                        #If there are equal number of vehicles to influence for both platoons, then the platoons are identicial 
                        if len(self.vehicles_to_influence[0]) == len(self.vehicles_to_influence[1]):
                            self.vehicles_to_influence[1].insert(0,None)
                            
                            
                        elif len(self.vehicles_to_influence[0]) > len(self.vehicles_to_influence[1]):
                            self.vehicles_to_influence[0].pop(0)
                            #If platoon is behind then a placeholder of None
                            if len(self.infront)==0:
                                self.vehicles_to_influence[1].insert(0,None)
                            
                            if len(self.behind)!=0:
                                self.vehicles_to_influence[0].pop(-1)

                        self.merge_stage = 3
                        self.vehicles_to_influence = [self.vehicles_to_influence[0].copy(),self.vehicles_to_influence[1].copy()]

                #Arrange the positions of merging paltoon
                if self.merge_stage == 3:
                    
                    in_position_counter = 0

                    #Speed adjustions to check vehicles in center of their gap
                    for i in range(len(self.vehicles_to_influence[0])):
                            mem = self.vehicles_to_influence[0][i]
                            mem.update_attributes()
                            if self.vehicles_to_influence[1][i] is None:
                                wp = self.map.get_waypoint(self.vehicles_to_influence[1][i+1].location).next(20)[0].transform
                                dst_1 = self.dist_between_locations(mem.location, wp.location)
                            else:
                                self.vehicles_to_influence[1][i].update_attributes()
                                dst_1 = self.dist_between_locations(mem.location, self.vehicles_to_influence[1][i].location)

                            dst_2 = self.dist_between_locations(mem.location, self.vehicles_to_influence[1][i+1].location)
  
                            if dst_1 >= dst_2:
                                mem.platoon.speed_multiplier = 1.1
                            else:
                                mem.platoon.speed_multiplier = 0.9
                            
                            if (dst_1 - dst_2)**2 < 3:
                                in_position_counter +=1
                    
                    for mem in self.infront:
                        r_l = [self.map.get_waypoint(mem.location).road_id, self.map.get_waypoint(mem.location).lane_id]
                        if r_l != cap_lane_road:
                            mem.platoon.speed_multiplier = self.vehicles_to_influence[0][0].platoon.speed_multiplier+0.1
                        else:
                            mem.platoon.speed_multiplier = 1

                    #If vehicle is behind     
                    for mem in self.behind:
                        r_l = [self.map.get_waypoint(mem.location).road_id, self.map.get_waypoint(mem.location).lane_id]
                        if r_l != cap_lane_road:
                            mem.platoon.speed_multiplier = self.vehicles_to_influence[0][-1].platoon.speed_multiplier
                        else:
                            mem.platoon.speed_multiplier = 1
                    
                    #If all vehicles are in place then a merge can happen
                    if len(self.vehicles_to_influence[0]) == in_position_counter:
                        for mem in self.members:
                            mem.can_change(True)
                    
                if first_run:
                    self.action_state = "Merging"
                    cap.platoon.action_state = "Merging"
                    self.manoeuvres = [self.merge_position, [cap, False, member, True]]

            else:
                #when all vehicles in same lane then finalise the merge
                platoon_speed = cap.platoon.platoon_speed
                member = parameters[2]
                member.platoon.platoon_distance = 0.5
                self.vehicle.agent._proximity_vehicle_threshold = self.PLATOON_SPACING
                self.set_platoon_speed(platoon_speed)

                self.manoeuvres = None
                self.merge_manoeuvre(cap)  
        #Single_lane merging
        else:

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

                        self.set_platoon_speed(10)
                        self.manoeuvres = [self.merge_position, [cap, False, None, False]]
            else:
                self.cancel_merge(cap)

    #Merge into platoon
    def merge_manoeuvre(self, cap):
        for veh in self.members:
            veh.platoon.platoon_leader = cap
            cap.platoon.members.append(veh)
        
        for mem in cap.platoon.members:
            mem.platoon.platoon_distance = 0.5
        self.members.clear()
        self.vehicle.role = "member"
        self.action_state = None
        cap.platoon.action_state = None
        cap.platoon.set_speed_multiplier(1)
        #Rearrange platoon
        cap.platoon.rearrange_platoon()
    
    #Cancel the merge attempt
    def cancel_merge(self, cap):
        self.set_platoon_speed(10)
        self.action_state = None
        cap.platoon.set_speed_multiplier(1)
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
            veh.set_speed(speed*veh.platoon.speed_multiplier)
            veh.platoon.platoon_speed = speed
    
    #Set the platoon speed multiplier
    def set_speed_multiplier(self, mult):
        for veh in self.members:
            veh.platoon.speed_multiplier = mult

    #Mantian the gap distances between vehicels in the platoon
    def spacing_maintenance(self):
        if len(self.members) != 1:
            for mem in self.members:
                mem.update_attributes()

            previous_b = 1

            for x in range(len(self.members)):
                if x == 0:
                    self.vehicle.set_speed(self.platoon_speed*self.speed_multiplier)

                veh_1 = self.members[x]
                veh_2 = self.members[x+1]

                if veh_1.platoon.platoon_distance > veh_2.platoon.platoon_distance:
                    veh_2.platoon.speed_multiplier = previous_b
                
                else:
                    veh_2.platoon.speed_multiplier =  veh_1.platoon.speed_multiplier

                #Increase speed for vehicle to catch ups
                dst = self.dist_between_locations(veh_1.rear_location, veh_2.front_location)

                if dst < veh_2.platoon.platoon_distance:
                    c = ((dst-1-veh_2.platoon.platoon_distance)**2)**0.5
                    b = 1/(c**(c*0.1))

                    #Limit the mmultiplier to prevent the vehicle going too slow
                    if b < 0.75:
                        b = 0.75

                else:
                    b = (dst+1-veh_2.platoon.platoon_distance)**0.135
                #Limits the speed to 1.5 times the platoon speed
                if b > 1.5:
                    b = 1.5
                
                previous_b = b
                    
                veh_2.set_speed(self.platoon_speed*b*veh_2.platoon.speed_multiplier)

                #End
                if x+2 == len(self.members):
                    break
        else:
            self.vehicle.set_speed(self.platoon_speed*self.speed_multiplier)
    
    #Get the next locations of the platoon members
    def get_next_locations(self):
        for veh in self.members:
            veh.next_locations()
    
    #Get the route of the captain
    def get_local_route(self):
        self.get_next_locations()
        locs = self.vehicle.location_queue

        return locs[0]
    
    #Rearrange the platoon
    def rearrange_platoon(self):
        #Members sort
        distances = {}
        for mem in self.members:
            dist = self.distance_till_intersection(mem)
            distances[dist] = mem
        
        new_mem = []
        for key in sorted(distances):
            new_mem.append(distances[key])

        self.members = new_mem.copy()

        #captian reselect
        self.members[0].role = "captain"
        self.members[0].platoon.members = self.members.copy()
        self.members[0].agent._proximity_vehicle_threshold = self.PLATOON_PROXIMITY
        self.members[0].platoon.platoon_size = 10

        if self.members[0] is not self.vehicle:
            self.vehicle.role = "member"
            self.vehicle.agent._proximity_vehicle_threshold = self.PLATOON_SPACING
            self.platoon_leader = self.members[0]
            self.members.clear()

        for mem in self.platoon_leader.platoon.members:
            mem.platoon.platoon_leader = self.platoon_leader
            if mem is not self.vehicle:
                self.vehicle.agent._proximity_vehicle_threshold = self.PLATOON_SPACING