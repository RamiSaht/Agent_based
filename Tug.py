from single_agent_planner import simple_single_agent_astar
import math

class Tug(object):
    """Tug class, should be used in the creation of new tug."""
    def __init__(self, tug_id, start_node, starting_energy, nodes_dict):
        """
        Initalisation of tug object.
        INPUT:
            
        """
         #Fixed parameters
        self.speed = 1         #how much a/c moves per unit of t
        self.id = tug_id       #flight_id
        self.start = start_node   #start_node_id
        self.goal = None     #goal_node_id
        self.charging_nodes = [] #list of nodes where the tug can charge
        self.nodes_dict = nodes_dict #keep copy of nodes dict
        for node in nodes_dict.keys():
            if nodes_dict[node]["type"] == "charging":
                self.charging_nodes.append(node)
        
        #Route related
        self.status = 'ready' # ready, moving_free, moving_tugging, charging
        self.assigned_ac = None #aircraft to which the tug is assigned
        self.attached_ac = None
        self.path_to_goal = [] #planned path left from current location
        self.from_to = [start_node, start_node]  # Initialize with the starting node
        self.last_node = start_node #last node id

        #State related
        self.heading = 0
        self.position = nodes_dict[start_node]["xy_pos"] #xy position on map
        self.energy = starting_energy #energy of the tug
        
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
                heading = 90
            elif xy_start[0] < xy_next[0]: #moving right
                heading = 270
            else:
                heading=self.heading
        else: 
            raise Exception("Invalid movement")
    
        self.heading = heading
        
    def move(self, dt, t):   
        # Determine nodes between which the tug is moving
        # print(f'tug {self.id} moving according to path {self.path_to_goal}')
        from_node = self.from_to[0]
        to_node = self.from_to[1]
        xy_from = self.nodes_dict[from_node]["xy_pos"]  # xy position of from node
        xy_to = self.nodes_dict[to_node]["xy_pos"]  # xy position of to node
        distance_to_move = self.speed * dt  # distance to move in this timestep

        # Update position with rounded values
        x = xy_to[0] - xy_from[0]
        y = xy_to[1] - xy_from[1]
        if x != 0 or y != 0:
            x_normalized = x / math.sqrt(x**2 + y**2)
            y_normalized = y / math.sqrt(x**2 + y**2)
        else:
            x_normalized = 0
            y_normalized = 0
        posx = round(self.position[0] + x_normalized * distance_to_move, 2)  # round to prevent errors
        posy = round(self.position[1] + y_normalized * distance_to_move, 2)  # round to prevent errors
        self.position = (posx, posy)
        self.get_heading(xy_from, xy_to)

        # Check if goal is reached or if to_node is reached
        if self.position == xy_to:  # If the tug has reached the `to_node`
            if self.position == self.nodes_dict[self.goal]["xy_pos"]:  # If the final goal is reached
                print(f"Tug {self.id} reached its goal at {self.position}")
                if self.position == self.assigned_ac.position:
                    self.status = "moving_tugging"
                    self.attach_to_ac()
                    self.assigned_ac.acknowledge_attach(self.id)
            else:  # Update the path and move to the next step
                self.path_to_goal = self.path_to_goal[1:]  # Remove the first step from the path
                if self.path_to_goal:  # If there are more steps in the path
                    new_from_id = self.from_to[1]  # Current `to_node` becomes the new `from_node`
                    new_next_id = self.path_to_goal[0][0]  # Next step in the path
                    self.from_to = [new_from_id, new_next_id]  # Update `from_to`
        self.consume_energy()
                    
    def consume_energy(self):
        pass
    
    def assign_ac(self, ac):
        """
        Assigns the tug to an aircraft.
        INPUT:
            - ac: The aircraft to which the tug is assigned
        """
        if self.status == "ready":
            self.assigned_ac = ac
            self.status = "assigned"
            self.goal = ac.start
    
    def attach_to_ac(self):
        """
        Attaches the tug to an aircraft.
        INPUT:
            - ac_id: id of the aircraft to which the tug is attached
        """
        if self.assigned_ac != None and self.attached_ac == None:
            self.attached_ac = self.assigned_ac
            self.path_to_goal = []  # Clear the path to goal
            self.status = "moving_tugging"
            print(f"Tug {self.id} attached to aircraft {self.assigned_ac.id}")
        else:
            raise Exception(f"Tug {self.id} cannot attach to aircraft {self.assigned_ac.id} as it is already attached to another aircraft. Current situation: assigned to {self.assigned_ac} and attached to {self.attached_ac}")
     
    def plan_free_path(self, heuristics, time_start):
        if self.from_to[0] not in self.nodes_dict or self.goal not in self.nodes_dict:
            raise ValueError(f"Invalid from_node ({self.from_to[0]}) or goal_node ({self.goal}) for Tug {self.id}")
        
        success, path = simple_single_agent_astar(self.nodes_dict, self.from_to[0], self.goal, heuristics, time_start, [], f"tug{self.id}")
        if success:
            self.path_to_goal = path
            self.from_to = [self.from_to[0], path[1][0]]
            print(f'tug {self.id} planned free path to goal {self.goal} with path {self.path_to_goal}')
            self.status = "moving_free"
        else:
            raise Exception(f"Tug {self.id} could not find a free path to goal {self.goal}")
    
    def plan_tugging_path(self, heuristics, time_start):
        if self.assigned_ac == None:
            raise Exception(f"Tug {self.id} cannot plan a tugging path as it is not assigned to any aircraft.")
        
        success, path = simple_single_agent_astar(self.nodes_dict, self.from_to[0], self.assigned_ac.goal, heuristics, time_start, [], f"tug{self.id}")
        if success:
            self.path_to_goal = path
            self.from_to = [self.from_to[0], path[1][0]]
            print(f'tug {self.id} planned tugging path to goal {self.assigned_ac.goal} with path {self.path_to_goal}')
            self.status = "moving_tugging"
        else:
            raise Exception(f"Tug {self.id} could not find a tugging path to goal {self.assigned_ac.goal}")
    def detach_ac(self, ac_id):
        """
        Detaches the tug from an aircraft.
        INPUT:
            - ac_id: id of the aircraft to which the tug is detached
        """
        print(f"Tug {self.id} detaching from aircraft {ac_id}")
        if self.assigned_ac != None and self.attached_ac != None:
            self.attached_ac = None
            self.path_to_goal = []
            self.status = "ready"
            print(f"Tug {self.id} detached from aircraft {ac_id}")
        else:
            raise Exception(f"Tug {self.id} cannot detach from aircraft {ac_id} as it is not attached to any aircraft.")
            
    def __str__(self):
        return f"Tug {self.id} at {self.position} heading {self.heading} with energy {self.energy} assigned to {self.assigned_ac} with status {self.status}"