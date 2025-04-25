from single_agent_planner import simple_single_agent_astar
import math

###### test commmit tim

class Aircraft(object):
    """Aircraft class, should be used in the creation of new aircraft."""

    def __init__(self, flight_id, a_d, start_node, goal_node, spawn_time, nodes_dict):
        """
        Initalisation of aircraft object.
        INPUT:
            - flight_id: [int] unique id for this aircraft
            - a_d: [str] "a" for arrival flight and "d" for departure flight
            - start_node: node_id of start node
            - goal_node: node_id of goal node
            - spawn_time: spawn_time of a/c 
            - nodes_dict: copy of the nodes_dict
        """
        
        #Fixed parameters
        self.speed = 1 #how much ac moves per unit of time
        self.id = flight_id       #flight_id
        self.type = a_d           #arrival or departure (A/D)
        self.spawntime = spawn_time #spawntime
        self.start = start_node   #start_node_id
        self.goal = goal_node     #goal_node_id
        self.nodes_dict = nodes_dict #keep copy of nodes dict
        # self.nodes_dict = nodes_dict #keep copy of nodes dict
        self.start_time = None ########### 
        self.end_time = None ##############
        tugs_modes=1
        #Route related
        if tugs_modes==1:
            self.status = 'waiting' # waiting, requested, attached, done
        else:
            self.status=None
        self.path_to_goal = [] #planned path left from current location
        self.from_to = [0,0]
        self.assigned_tug = None #tug to which the aircraft is attached

        #State related
        self.heading = 0
        self.position = self.nodes_dict[start_node]["xy_pos"] #xy position on map

        #Travelled path
        self.visited_nodes = [self.start]  # start with the initial node

        self.moving_time = 0 ##############- 
        self.idle_time = 0
        self.last_position = self.position #######-

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

    def move(self, tugs_mode, dt, t):
        # Tugs movement
        if tugs_mode == 1:
            if self.status == 'attached' and self.assigned_tug is not None:
                if self.position == self.nodes_dict[self.goal]["xy_pos"]:
                    self.detach_tug(t)
                    self.status = "done"
                    if self.end_time is None:  # Track end time
                        self.end_time = t + dt
                    return
                self.move_with_tug()

                if self.position != self.last_position:
                    self.moving_time += dt
                else:
                    self.idle_time += dt
                self.last_position = self.position

        # No tugs movement
        else:
            if self.start_time is None:
                self.start_time = t

            from_node = self.from_to[0]
            to_node = self.from_to[1]
            xy_from = self.nodes_dict[from_node]["xy_pos"]
            xy_to = self.nodes_dict[to_node]["xy_pos"]

            # Case 1: Waiting at a node
            if from_node == to_node:
                scheduled_time = self.path_to_goal[0][1]
                if t >= scheduled_time:
                    self.path_to_goal = self.path_to_goal[1:]
                    if self.path_to_goal:
                        next_node = self.path_to_goal[0][0]
                        self.from_to = [to_node, next_node]

            # Case 2: Moving between nodes
            else:
                distance_to_move = self.speed * dt
                x = xy_to[0] - xy_from[0]
                y = xy_to[1] - xy_from[1]
                if x != 0 or y != 0:
                    x_normalized = x / math.sqrt(x**2 + y**2)
                    y_normalized = y / math.sqrt(x**2 + y**2)
                else:
                    x_normalized = 0
                    y_normalized = 0

                posx = round(self.position[0] + x_normalized * distance_to_move, 2)
                posy = round(self.position[1] + y_normalized * distance_to_move, 2)
                self.position = (posx, posy)
                self.get_heading(xy_from, xy_to)

                # Track movement time
                if self.position != self.last_position:
                    self.moving_time += dt
                else:
                    self.idle_time += dt
                self.last_position = self.position

                # Check if goal is reached
                if self.position == xy_to:
                    if self.position == self.nodes_dict[self.goal]["xy_pos"]:
                        self.status = "done"
                        if self.end_time is None:  # Critical for output analysis
                            self.end_time = t + dt
                    else:
                        remaining_path = self.path_to_goal[1:]
                        self.path_to_goal = remaining_path
                        new_from_id = self.from_to[1]
                        new_next_id = self.path_to_goal[0][0] if self.path_to_goal else new_from_id
                        if new_from_id != self.from_to[0]:
                            self.last_node = self.from_to[0]
                        self.from_to = [new_from_id, new_next_id]

    def acknowledge_attach(self, tug_id):
        """
        Attaches the aircraft to a tug.
        INPUT:
            - tug_id: id of the tug to which the aircraft is attached
        """
        if self.status == "requested" and tug_id == self.assigned_tug.id:
            self.status = 'attached'
            
    def move_with_tug(self):
        """
        Moves the aircraft with the tug.
        """
        if self.assigned_tug != None and self.status == "attached":
            self.position = self.assigned_tug.position #move with the tug
            self.heading = self.assigned_tug.heading #move with the tug
        else:
            raise Exception(f"Moving Aircraft {self.id} which is not attached to a tug or no tug assigned.")
            
    def assign_tug(self, tug):
        """
        Assigns a tug to the aircraft.
        INPUT:
            - tug_id: id of the tug to which the aircraft is assigned
        """
        if self.status == "waiting":
            self.assigned_tug = tug
            self.status = 'requested'
     
    def detach_tug(self, t): ###### t
        """
        Detaches the aircraft from the tug.
        """
        if self.status == "attached":
            if self.end_time is None: ############# 
                self.end_time = t ############# 
            self.assigned_tug.position = self.position #update the position of the tug
            self.assigned_tug.detach_ac(self.id, t) ###### t
            self.assigned_tug = None
            self.status = 'done'
             
    def __str__(self):
        return f"Aircraft {self.id} ({self.type}) at {self.position} with heading {self.heading} and status {self.status}"
    
    def __repr__(self):
        return self.__str__()
    
    def get_time_to_destination(self): #########-
        if self.end_time is not None:
            return round(self.end_time - self.spawntime, 2)
        return None #####-