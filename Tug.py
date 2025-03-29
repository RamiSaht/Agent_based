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
        for node in nodes_dict.keys():
            if nodes_dict[node]["type"] == "charging":
                self.charging_nodes.append(node)
        
        #Route related
        self.status = 'ready' # ready, movingf, movingt, charging
        self.path_to_goal = [] #planned path left from current location
        self.from_to = [nodes_dict[start_node]["id"], nodes_dict[start_node]["id"]] #from and to node id
        self.last_node = start_node #last node id

        #State related
        self.heading = 0
        self.position = nodes_dict[start_node]["xy_pos"] #xy position on map
        self.energy = starting_energy #energy of the tug
        
        
    def move(self, dt, t):   
        # to be implemented look at the aircraft move function
        pass