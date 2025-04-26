from traceback import print_tb

from cbs import run_cbs_for_attached_tugs
from single_agent_planner import simple_single_agent_astar
import math

def find_closest_node(position, nodes_dict):
    """
    Find the closest node to a given (x, y) position.
    """
    min_distance = float('inf')
    closest_node = None
    for node_id, node_data in nodes_dict.items():
        node_pos = node_data["xy_pos"]
        distance = (position[0] - node_pos[0]) ** 2 + (position[1] - node_pos[1]) ** 2
        if distance < min_distance:
            min_distance = distance
            closest_node = node_id
    return closest_node

class Tug(object):
    """Tug class, should be used in the creation of new tug."""
    def __init__(self, tug_id, start_node, starting_energy, nodes_dict):
        """
        Initalisation of tug object.
        INPUT:
            
        """
         #Fixed parameters
        self.speed = 1         #how much tug moves per unit of t
        self.charge_per_distance = 1 #how much percent is consumed per distance unit
        self.energy_threshold = 30 #energy threshold to consider bid (charging forced at 10% higher to make sure tugs aren't blocked off)
        self.charge_speed = 5 #how much percent is charged per unit of t
        self.id = tug_id       #flight_id
        self.start = start_node   #start_node_id
        self.goal = None     #goal_node_id
        self.charging_nodes = [] #list of nodes where the tug can charge
        self.nodes_dict = nodes_dict #keep copy of nodes dict
        for node in nodes_dict.keys():
            if nodes_dict[node]["type"] == "charging":
                self.charging_nodes.append(node)
        
        #Route related
        self.status = 'ready' # ready, assigned, moving_free, moving_tugging, charging
        self.assigned_ac = None #aircraft to which the tug is assigned
        self.attached_ac = None
        self.secondary_assigned_ac = None #aircraft to which the tug is assigned after the first one
        self.path_to_goal = [] #planned path left from current location
        self.from_to = [start_node, start_node]  # Initialize with the starting node
        self.last_node = start_node #last node id

        #State related
        self.heading = 0
        self.position = nodes_dict[start_node]["xy_pos"] #xy position on map
        self.moving_time = 0 ############## -
        self.idle_time = 0
        self.last_position = self.position #########-
        self.energy = starting_energy #energy of the tug
        self.operation_start_time = None ############# 
        self.assignment_log = [] #to track aircraft assignments ############### 
        self.charging_time = 0 ###### time spent charging
        if starting_energy < self.energy_threshold+20:
            self.status = "low_energy"
        
    def get_heading(self, xy_start, xy_next):
        """
        Determines heading of an aircraft based on a start and end xy position.
        INPUT:
            - xy_start = tuple with (x,y) position of start node
            - xy_next = typle with (x,y) position of next node
        RETURNS:
            - heading = heading of aircraft in degrees
        """

        if xy_start[0] == xy_next[0]: #moving up or down
            if xy_start[1] > xy_next[1]: #moving down
                heading = 180
            elif xy_start[1] < xy_next[1]: #moving up
                heading = 0
            else:
                heading=self.heading

        elif xy_start[1] == xy_next[1]: #moving right or left
            if xy_start[0] > xy_next[0]: #moving left
                heading = 270
            elif xy_start[0] < xy_next[0]: #moving right
                heading = 90
            else:
                heading=self.heading
        else: 
            raise Exception("Invalid movement")
    
        self.heading = heading

    def move(self, dt, t):
        # Determine nodes between which the tug is moving
        from_node = self.from_to[0]
        to_node = self.from_to[1]
        xy_from = self.nodes_dict[from_node]["xy_pos"]  # xy position of from node
        xy_to = self.nodes_dict[to_node]["xy_pos"]  # xy position of to node
        distance_to_move = self.speed * dt  # distance to move in this timestep
        # Calculate the remaining distance to the next node
        remaining_distance = round(math.sqrt((xy_to[0] - self.position[0]) ** 2 + (xy_to[1] - self.position[1]) ** 2),2)
        #Make sure no divisions by 0 happen
        if remaining_distance!=0:
            # Move towards the next node
            direction_x = (xy_to[0] - self.position[0]) / remaining_distance
            direction_y = (xy_to[1] - self.position[1]) / remaining_distance

            # Calculate actual movement (don't overshoot)
            distance_moved = min(distance_to_move, remaining_distance)
            posx = round(self.position[0] + direction_x * distance_moved, 2)
            posy = round(self.position[1] + direction_y * distance_moved, 2)
            self.position = (posx, posy)
            remaining_distance = round(math.sqrt((xy_to[0] - self.position[0]) ** 2 + (xy_to[1] - self.position[1]) ** 2),2)
        # If we're very close to the node, just snap to it
        if remaining_distance < 0.01:
            self.position = xy_to
            distance_moved = remaining_distance

        self.get_heading(xy_from, xy_to)

        # Check if goal is reached or if to_node is reached
        if self.position == xy_to:  # If the tug has reached the `to_node`
            if self.position == self.nodes_dict[self.goal]["xy_pos"]:  # If the final goal is reached
                if self.status=="moving_charging":  # If tug moves while charging
                    self.status = "charging"
                elif self.position == self.assigned_ac.position and self.attached_ac==None:
                    self.status = "moving_tugging"
                    self.attach_to_ac(t+dt)
                    self.assigned_ac.acknowledge_attach(self.id)
                elif self.position == self.assigned_ac.goal:
                    self.assigned_ac.end_time = t #if aircraft reached goal (since ac moves with tug, mark it as done)
                    self.assigned_ac.status = "done"
            else:  # Update the path and move to the next step
                if self.from_to[0] != self.from_to[1]:
                    self.last_node =self.from_to[1]
                    self.path_to_goal = self.path_to_goal[1:]  # Remove the first step from the path
                    if self.path_to_goal:  # If there are more steps in the path
                        new_from_id = self.from_to[1]  # Current `to_node` becomes the new `from_node`
                        new_next_id = self.path_to_goal[0][0]  # Next step in the path
                        self.from_to = [new_from_id, new_next_id]  # Update `from_to`
                else:
                    if t >= self.path_to_goal[0][1] - 0.1:
                        self.path_to_goal = self.path_to_goal[1:]  # Remove the first step from the path
                        if self.path_to_goal:  # If there are more steps in the path
                            new_from_id = self.from_to[1]  # Current `to_node` becomes the new `from_node`
                            new_next_id = self.path_to_goal[0][0]  # Next step in the path
                            self.from_to = [new_from_id, new_next_id]  # Update `from_to`

        self.consume_energy(distance_moved)

        if self.position != self.last_position: #####-
            self.moving_time += dt
        else:
            self.idle_time += dt
        self.last_position = self.position #####-

    def consume_energy(self, distance):
        """
        Consumes energy based on the distance moved.
        INPUT:
            - distance: The distance moved by the tug
        """
        if self.energy > 0:
            self.energy = round(self.energy - distance * self.charge_per_distance, 2)
            self.energy = max(0, self.energy)  # Ensure energy doesn't go below 0
            
        else:
            raise Exception(f"Tug {self.id} has no energy left.")
        pass
    
    def assign_ac(self, ac):
        """
        Assigns the tug to an aircraft.
        INPUT:
            - ac: The aircraft to which the tug is assigned
        """
        if self.status == "charging" or self.status == "moving_charging":
            # If currently charging, set as secondary assignment
            if self.assigned_ac is None:
                self.assigned_ac = ac
            else:
                raise Exception(f"Tug {self.id} already has secondary assignment")
        elif self.status == "ready":
            self.assigned_ac = ac
            self.status = "assigned"
            self.goal = ac.start  # Set goal to the aircraft's start node
            self.path_to_goal = []  # Clear any existing path
            self.operation_start_time = None #############
        elif self.assigned_ac is not None and self.secondary_assigned_ac is None:
            self.secondary_assigned_ac = ac
        else:
            # can't be assigned
            raise Exception(f"Tug {self.id} cannot be assigned to aircraft {ac.id} as it is already assigned to another aircraft.")
            
    def attach_to_ac(self,t):
        """
        Attaches the tug to an aircraft.
        INPUT:
            - ac_id: id of the aircraft to which the tug is attached
        """
        self.attachment_time=t
        if self.assigned_ac != None and self.attached_ac == None:
            self.attached_ac = self.assigned_ac
            self.path_to_goal = []  # Clear the path to goal
            self.status = "moving_tugging"
        else:
            raise Exception(f"Tug {self.id} cannot attach to aircraft {self.assigned_ac.id} as it is already attached to another aircraft. Current situation: assigned to {self.assigned_ac} and attached to {self.attached_ac}")
     
    def plan_free_path(self, heuristics, time_start):
        if self.from_to[0] not in self.nodes_dict or self.goal not in self.nodes_dict:
            raise ValueError(f"Invalid from_node ({self.from_to[0]}) or goal_node ({self.goal}) for Tug {self.id}")
        
        # Check to see if the aircraft spawned on top of the tug
        if self.assigned_ac != None:
            if self.position == self.assigned_ac.position:
                self.status = "moving_tugging"
                self.attach_to_ac(time_start)
                self.assigned_ac.acknowledge_attach(self.id)
                return
        success, path = simple_single_agent_astar(self.nodes_dict, self.from_to[0], self.goal, heuristics, time_start, [], f"tug{self.id}")
        if success:
            self.path_to_goal = path
            self.from_to = [self.from_to[1], path[0][0]]
            if self.operation_start_time is None and self.assigned_ac:######
                self.operation_start_time = time_start######
            if self.status == "low_energy":
                self.status = "moving_charging"
            else:
                self.status = "moving_free"
        else:
            raise Exception(f"Tug {self.id} could not find a free path to goal {self.goal}")

    def plan_tugging_path(self, tug_list, aircraft_list, nodes_dict, edges_dict, heuristics, current_time,
                          max_static_block=100):
        """
        Plan a tugging path using Conflict-Based Search (CBS) for all currently tugging tugs.
        Only needs to be called once per planning instance (e.g., after attachment).
        """
        import networkx as nx

        if not self.attached_ac:
            print(f"Tug {self.id} is not attached to any aircraft. Skipping CBS planning.")
            return

        # Build the graph
        graph = nx.DiGraph()
        for edge, edge_data in edges_dict.items():
            weight = edge_data.get("weight", 1) if isinstance(edge_data, dict) else edge_data
            if edge[0] in nodes_dict and edge[1] in nodes_dict:
                graph.add_edge(edge[0], edge[1], weight=weight)

        # --- 1. Collect moving tugs and their goals ---
        starts, goals, moving_tugs,personal_obstacles = [], [], [], []

        for idx, tug in enumerate(tug_list):
            if tug.attached_ac:
                ac = tug.attached_ac
                current_location = find_closest_node(tug.position,nodes_dict)
                start_node = current_location
                goal_node = ac.goal
                starts.append(start_node)
                goals.append(goal_node)
                agent_id = len(goals)-1  # Index in active_aircrafts
                moving_tugs.append(tug)
                last_node = tug.assigned_ac.last_surely_visited_node

                if last_node != ac.start:  # make sure only for nodes not representing starting points
                    personal_obstacles.append((last_node, current_time, current_time + 100, agent_id))
        # --- 2. Temporarily static aircraft (waiting for tug) ---
        # add chokepoints for static blocking (10 seconds)
        chokepoints = {
            37: [11, 101, 37],
            38: [12, 102, 38],
            1: [4, 95, 1],
            2: [5, 96, 2],
            97: [29, 99, 97],
            34: [30, 92, 34],
            35: [31, 93, 35],
            36: [32, 94, 36],
            98: [33, 100,98]
        }

        static_blocks = []
        for ac in aircraft_list:
            current_node = find_closest_node(ac.position,nodes_dict)
            current_time=round(current_time * 2) / 2
            if ac.status == "requested" and current_node in chokepoints:
                block_path = chokepoints[current_node]
                for node in block_path:
                    static_blocks.append((node, current_time, current_time + max_static_block))
            elif ac.status == "requested":
                static_blocks.append((current_node, current_time, current_time + max_static_block))
        # --- 3. Run CBS ---
        from cbs import CBSSolver  # ensure CBSSolver is imported correctly
        cbs_solver = CBSSolver(graph, nodes_dict, starts, goals, current_time, heuristics,
                               static_obstacles=static_blocks,personal_obstacles=personal_obstacles)


        try:
            paths = cbs_solver.find_solution()
        except Exception as e:
            print(f"[CBS ERROR] Failed to find tugging paths: {e}")
            return

        # --- 4. Assign paths back to tugs and aircraft ---
        for tug, path in zip(moving_tugs, paths):
            tug.path_to_goal = path
            print(tug.id,tug.path_to_goal)
            tug.status = "moving_tugging"
            tug.attached_ac.path = path  # optional, for syncing visualization
            if len(path) > 1:
                tug.from_to = [path[0][0], path[1][0]]
            tug.path_to_goal = tug.path_to_goal[1:] # Make sure that we don't read the first transit node twice
        print(static_blocks, personal_obstacles)
    
    def calculate_free_path(self, from_node, to_node, heuristics, time_start):
        """
        Calculates the path to the goal without executing it.
        INPUT:
            - heuristics: Heuristic values for the nodes
            - time_start: Starting timestep
        """
        success, path = simple_single_agent_astar(self.nodes_dict, from_node, to_node, heuristics, time_start, [], f"tug{self.id}")
        if success:
            return path
        else:
            return None  # No path found
            
    def detach_ac(self, ac_id, t): ##### ,t 
        """
        Detaches the tug from an aircraft.
        INPUT:
            - ac_id: id of the aircraft to which the tug is detached
        """
        if self.operation_start_time is not None: ###########-
            self.assignment_log.append({
                "aircraft_id": ac_id,
                "start_time": self.operation_start_time,
                "end_time": t,
                "duration": round(t - self.operation_start_time, 2),
                "moving_time": round(self.moving_time, 2),
                "idle_time": round(self.idle_time, 2),
                "charging_time": round(self.charging_time, 2)
            })  

            self.operation_start_time = None
            self.operation_end_time = None
            self.moving_time = 0
            self.idle_time = 0
            self.charging_time = 0 ##############-
        
        if self.assigned_ac != None and self.attached_ac != None:
            self.attached_ac = None
            self.path_to_goal = []
            self.goal = None
            self.assigned_ac = None
            self.status = "ready"
        else:
            raise Exception(f"Tug {self.id} cannot detach from aircraft {ac_id} as it is not attached to any aircraft.")
        
        if self.secondary_assigned_ac != None:
            self.assigned_ac = self.secondary_assigned_ac
            self.secondary_assigned_ac = None
            self.status = 'assigned'
            self.goal = self.assigned_ac.start
            
        if self.energy < self.energy_threshold + 20 and self.assigned_ac==None: #Make sure tugs don't stay blocked (can actually bid)
            # go to charging station
            self.status = "low_energy"
            self.attached_ac = None
            self.path_to_goal = []
            self.goal = None
            
        return
    
    def charge(self, dt):
        """
        Charges the tug at a charging station.
        INPUT:
            - dt: time step
        """
        self.energy = min(self.energy + self.charge_speed * dt, 100)  # Ensure energy doesn't exceed 100%
        self.charging_time += dt ##########
        
        if self.energy >= 100:
            if self.assigned_ac==None:
                self.status = "ready"
                self.goal = None
                self.path_to_goal = []  # Clear the path to avoid stale data
            else:
                self.status = "assigned"
                self.goal = self.assigned_ac.start  # Set goal to the aircraft's start node
                self.path_to_goal = []  # Clear any existing path
            
        return self.energy
    
    def make_bid(self, ac, heuristics, bid_time):
        """
        Makes a bid for the time of arrival at the aircraft (Free path).
        INPUT:
            - ac: The aircraft for which the bid is made
        """
        if self.status == "ready":
            calculated_path = self.calculate_free_path(self.from_to[0], ac.start, heuristics, bid_time)
            charge_needed_to_ac = self.charge_per_distance * (calculated_path[-1][1] - bid_time)
             
            if calculated_path != None and self.energy - charge_needed_to_ac > self.energy_threshold:
                return calculated_path[-1][1]
            
        elif self.status == "moving_free":
            if self.secondary_assigned_ac != None:
                return float('inf')
            if self.goal == ac.start:
                return float('inf')
            # calculate the time to reach the aircraft after delivering the current one
            time_to_arrive_current = self.path_to_goal[-1][1]
            time_to_deliver_current = self.calculate_free_path(self.assigned_ac.start, self.assigned_ac.goal, heuristics, time_to_arrive_current)[-1][1]
            time_to_reach_ac = self.calculate_free_path(self.assigned_ac.goal, ac.start, heuristics, time_to_deliver_current)[-1][1]
            charge_needed_to_ac = self.charge_per_distance * (time_to_reach_ac - bid_time)
            if self.energy - charge_needed_to_ac > self.energy_threshold:
                return time_to_reach_ac
             
        
        elif self.status == "moving_tugging":
            # calculate the time to reach the aircraft after delivering the current one
            if self.secondary_assigned_ac != None:
                return float('inf')
            if self.assigned_ac.goal == ac.start: #avoid conflict generation
                return float('inf')
            
            time_to_deliver_current = self.calculate_free_path(self.from_to[0], self.assigned_ac.goal, heuristics, bid_time)[-1][1]
            time_to_reach_ac = self.calculate_free_path(self.assigned_ac.goal, ac.start, heuristics, time_to_deliver_current)[-1][1]
            charge_needed_to_ac = self.charge_per_distance * (time_to_reach_ac - bid_time)
            
            if self.energy - charge_needed_to_ac > self.energy_threshold and self.assigned_ac.goal!=ac.start:
                return time_to_reach_ac
        elif self.status == "charging" and self.assigned_ac==None:
            #calculate time left to charge
            time_to_charge = ((100-self.energy)/self.charge_speed)
            time_to_reach_ac=self.calculate_free_path(self.from_to[0],ac.start,heuristics,bid_time+time_to_charge)[-1][1]
            return time_to_reach_ac
        elif self.status == "moving_charging" and self.assigned_ac == None:
            # calculate time left to charge
            time_to_arrive_current = self.path_to_goal[-1][1]
            time_to_charge = ((100 - self.energy) / self.charge_speed)
            time_to_reach_ac = self.calculate_free_path(self.path_to_goal[-1][0], ac.start, heuristics, time_to_charge+time_to_arrive_current)[-1][1]
            return time_to_reach_ac
        
        return float('inf')

    def find_closest_charging_node(self, heuristics):
        '''finds the closest charging node to the tug'''
        distances = {}
        
        for node in self.charging_nodes:
            path = self.calculate_free_path(self.from_to[0], node, heuristics, 0)
            distances[node] = len(path) if path else float('inf')
        
        closest_charging = min(distances, key=distances.get)
        return closest_charging
    
    def __str__(self):
        return f"Tug {self.id} at {self.position} heading {self.heading} with energy {self.energy} assigned to {self.assigned_ac} with status {self.status}"
    
    def __repr__(self):
        return self.__str__()