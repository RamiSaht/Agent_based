"""
Run-me.py is the main file of the simulation. Run this file to run the simulation.
"""

import os
from matplotlib.style import available
import pandas as pd
import networkx as nx
import matplotlib.pyplot as plt
import time as timer
import pygame as pg
from single_agent_planner import calc_heuristics
from visualization import map_initialization, map_running
from Aircraft import Aircraft
from independent import run_independent_planner
from prioritized import run_prioritized_planner
from cbs import run_CBS
from Tug import Tug
import random

#%% SET SIMULATION PARAMETERS
#Input file names (used in import_layout) -> Do not change those unless you want to specify a new layout.
nodes_file = "nodes.xlsx" #xlsx file with for each node: id, x_pos, y_pos, type
edges_file = "edges.xlsx" #xlsx file with for each edge: from  (node), to (node), length

#Parameters that can be changed:
simulation_time = 100
random_schedule = True #True if you want to generate a random schedule, False if you want to use the schedule.csv file
random_generation_time = 50 # time after which no random aircraft are generated anymore example 30 means all aircraft are generated in the first 30 seconds of the simulation
num_aircraft = 5 #number of aircraft to be generated
planner = "CBS" #choose which planner to use (currently only Independent is implemented)
#Visualization (can also be changed)
plot_graph = False    #show graph representation in NetworkX
visualization = True        #pygame visualization
visualization_speed = 0.01 #set at 0.1 as default

#%%Function definitions
def import_layout(nodes_file, edges_file):
    """
    Imports layout information from xlsx files and converts this into dictionaries.
    INPUT:
        - nodes_file = xlsx file with node input data
        - edges_file = xlsx file with edge input data
    RETURNS:
        - nodes_dict = dictionary with nodes and node properties
        - edges_dict = dictionary with edges annd edge properties
        - start_and_goal_locations = dictionary with node ids for arrival runways, departure runways and gates 
    """
    gates_xy = []   #lst with (x,y) positions of gates
    rwy_dep_xy = [] #lst with (x,y) positions of entry points of departure runways
    rwy_arr_xy = [] #lst with (x,y) positions of exit points of arrival runways
    charging_nodes = [] #lst with (x,y) positions of charging nodes
    
    df_nodes = pd.read_excel(os.getcwd() + "/" + nodes_file)
    df_edges = pd.read_excel(os.getcwd() + "/" + edges_file)
    
    #Create nodes_dict from df_nodes
    nodes_dict = {}
    for i, row in df_nodes.iterrows():
        node_properties = {"id": row["id"],
                           "x_pos": row["x_pos"],
                           "y_pos": row["y_pos"],
                           "xy_pos": (row["x_pos"],row["y_pos"]),
                           "type": row["type"],
                           "neighbors": set()
                           }
        node_id = row["id"]
        nodes_dict[node_id] = node_properties
        
        #Add node type
        if row["type"] == "rwy_d":
            rwy_dep_xy.append((row["x_pos"],row["y_pos"]))
        elif row["type"] == "rwy_a":
            rwy_arr_xy.append((row["x_pos"],row["y_pos"]))
        elif row["type"] == "gate":
            gates_xy.append((row["x_pos"],row["y_pos"]))
        elif row["type"] == "charging":
            charging_nodes.append((row["x_pos"],row["y_pos"]))

    #Specify node ids of gates, departure runways and arrival runways in a dict
    start_and_goal_locations = {"gates": gates_xy, 
                                "dep_rwy": rwy_dep_xy,
                                "arr_rwy": rwy_arr_xy,
                                "charging_nodes": charging_nodes}
    
    #Create edges_dict from df_edges
    edges_dict = {}
    for i, row in df_edges.iterrows():
        edge_id = (row["from"],row["to"])
        from_node =  edge_id[0]
        to_node = edge_id[1]
        start_end_pos = (nodes_dict[from_node]["xy_pos"], nodes_dict[to_node]["xy_pos"])
        edge_properties = {"id": edge_id,
                           "from": row["from"],
                           "to": row["to"],
                           "length": row["length"],
                           "weight": row["length"],
                           "start_end_pos": start_end_pos
                           }
        edges_dict[edge_id] = edge_properties
   
    #Add neighbor nodes to nodes_dict based on edges between nodes
    for edge in edges_dict:
        from_node = edge[0]
        to_node = edge[1]
        nodes_dict[from_node]["neighbors"].add(to_node)  
    
    return nodes_dict, edges_dict, start_and_goal_locations

def create_graph(nodes_dict, edges_dict, plot_graph = True):
    """
    Creates networkX graph based on nodes and edges and plots 
    INPUT:
        - nodes_dict = dictionary with nodes and node properties
        - edges_dict = dictionary with edges annd edge properties
        - plot_graph = boolean (True/False) If True, function plots NetworkX graph. True by default.
    RETURNS:
        - graph = networkX graph object
    """
    
    graph = nx.DiGraph() #create directed graph in NetworkX
    
    #Add nodes and edges to networkX graph
    for node in nodes_dict.keys():
        graph.add_node(node, 
                       node_id = nodes_dict[node]["id"],
                       xy_pos = nodes_dict[node]["xy_pos"],
                       node_type = nodes_dict[node]["type"])
        
    for edge in edges_dict.keys():
        graph.add_edge(edge[0], edge[1], 
                       edge_id = edge,
                       from_node =  edges_dict[edge]["from"],
                       to_node = edges_dict[edge]["to"],
                       weight = edges_dict[edge]["length"])
    
    #Plot networkX graph
    if plot_graph:
        plt.figure()
        node_locations = nx.get_node_attributes(graph, 'xy_pos')
        nx.draw(graph, node_locations, with_labels=True, node_size=100, font_size=10)
        
    return graph

def parse_schedule(file_path, nodes_dict):
    df = pd.read_csv(file_path)
    aircraft_lst = []
    # df.sort_values(by="t", inplace=True)
    for i, row in df.iterrows():
        if row.t < 0:
            raise Exception("Error: Schedule contains negative time values")
        
        aircraft_lst.append(Aircraft(row.ac_id, row["arrival_departure"], row.start_node, row.end_node, row.t, nodes_dict))
    spawn_times = df.t.unique()
    return aircraft_lst, spawn_times

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

def parse_tugs(file_path, nodes_dict):
    
    df = pd.read_csv(file_path)
    lst = []
    
    for i, row in df.iterrows():
        if row.starting_node not in nodes_dict.keys():
            raise Exception("Error: Start node of tug does not exist in nodes_dict")
        lst.append(Tug(row.tug_id, row.starting_node, row.starting_energy, nodes_dict))
    
    return lst

def assign_tug_to_aircraft(aircraft_to_assign_l, available_tugs_l, t_l):
    """
    Assigns a tug to an aircraft based on the bids made by the tugs.
    The tug with the minimum bid is assigned to the aircraft.
    Input:
        - aircraft_queue_l: list of aircraft waiting for a tug
        - available_tugs_l: list of available tugs
        - t_l: current time
    """
    bids = {}
    for tug in available_tugs_l:
        bids[tug.id] = tug.make_bid(aircraft_to_assign_l, heuristics, t_l)
    
    # Find the tug with the minimum bid
    min_bid_tug_id = min(bids, key=bids.get)
    if bids[min_bid_tug_id] == float('inf'):
        return False
    
    print(f"For aircraft {aircraft_to_assign_l.id}, bids are: \n{bids}")
    # Assign the tug to the aircraft
    for tug in available_tugs_l:
        if tug.id == min_bid_tug_id:
            print(f"Assigning tug {tug.id} to aircraft {aircraft_to_assign_l.id}")
            tug.assign_ac(aircraft_to_assign_l)
            aircraft_to_assign_l.assign_tug(tug)
    
    
    return True

def continuous_random_generation(t):
    # Test if it's time to generate a new aircraft
    
    random_generation_interval = random_generation_time // num_aircraft
    num_simulated_aircraft = len(aircraft_lst)
    
    if t < random_generation_time and random_generation_interval * num_simulated_aircraft <= t:
        occupied_gates = [ac.start for ac in aircraft_lst if ac.type == "D"]
        occupied_rwy_arrs = [ac.start for ac in aircraft_lst if ac.type == "A"]
        available_gates = [gate for gate in gates if gate not in occupied_gates]
        available_rwy_arrs = [rwy for rwy in rwy_arrs if rwy not in occupied_rwy_arrs]
        choice = random.choice(aircraft_type_choices)
        spawn_time_l =  t
        if choice == "A" and available_rwy_arrs:
            start_node = random.choice(available_rwy_arrs)
            goal_node = random.choice(gates)
        elif choice == "D" and available_gates:
            start_node = random.choice(available_gates)
            goal_node = random.choice(rwy_deps)
        else:
            return
        
        ac = Aircraft(f'A{num_simulated_aircraft}', choice, start_node, goal_node, spawn_time_l, nodes_dict)
        aircraft_lst.append(ac)
        spawn_times.append(spawn_time_l)
        print(f"Aircraft {ac.id} generated at time {t} with start node {start_node} and goal node {goal_node}")
    return
    
    
    
#%% RUN SIMULATION
# =============================================================================
# 0. Initialization
# =============================================================================
nodes_dict, edges_dict, start_and_goal_locations = import_layout(nodes_file, edges_file)
gates = [node for node in nodes_dict if nodes_dict[node]["type"] == "gate"]
rwy_deps = [node for node in nodes_dict if nodes_dict[node]["type"] == "rwy_d"]
rwy_arrs = [node for node in nodes_dict if nodes_dict[node]["type"] == "rwy_a"]
charging_nodes = [node for node in nodes_dict if nodes_dict[node]["type"] == "charging"]
aircraft_type_choices = ["A", "D"]

graph = create_graph(nodes_dict, edges_dict, plot_graph)
heuristics = calc_heuristics(graph, nodes_dict)
if random_schedule:
    aircraft_lst, spawn_times = [], [] #List which can contain aircraft agents
else:
    aircraft_lst, spawn_times = parse_schedule("schedule.csv", nodes_dict)
    
tugs_lst = parse_tugs("tugs.csv", nodes_dict) #List which can contain tug agents

if visualization:
    map_properties = map_initialization(nodes_dict, edges_dict) #visualization properties

# =============================================================================
# 1. While loop and visualization
# =============================================================================

#Start of while loop    
running=True
escape_pressed = False
time_end = simulation_time if simulation_time else 999999
dt = 0.1 #should be factor of 0.5 (0.5/dt should be integer)
t= 0
collisions=[]
aircraft_queue = [] #queue of aircraft that are waiting for a tug
available_tugs = [] #list of available tugs
print("Simulation Started")
while running:
    t= round(t,2)
    continuous_random_generation(t) if random_schedule else None #generate random aircraft if random_schedule is true
    # aircraft_lst, spawn_times = parse_schedule("schedule.csv", nodes_dict)
    active_aircrafts = [ac for ac in aircraft_lst if (ac.spawntime <= t and ac.status != "done")]
    #quit
    if t >= time_end or escape_pressed or pg.event.get(pg.QUIT): 
        running = False
        pg.quit()
        print(f'\nTotal collisions:\n{"\n".join(collisions)}')
        print("Simulation Stopped")
        break 
    
    #Visualization: Update map if visualization is true
    if visualization:
        current_aircrafts = {} #Collect current states of all aircraft
        for ac in active_aircrafts:
            current_aircrafts[ac.id] = {"ac_id": ac.id,
                                        "xy_pos": ac.position,
                                        "heading": ac.heading,
                                        "status": ac.status}
        
        current_tugs = {} #Collect current states of all tugs
        for tug in tugs_lst:
            current_tugs[tug.id] = {"tug_id": tug.id,
                                         "xy_pos": tug.position,
                                         "heading": tug.heading,
                                         "status": tug.status,
                                         "energy": tug.energy,
                                         "assigned_ac": tug.assigned_ac.id if tug.assigned_ac else None,
                                         "secondary_ac": tug.secondary_assigned_ac.id if tug.secondary_assigned_ac else None}
        escape_pressed = map_running(map_properties, current_aircrafts, current_tugs, t, dt, collisions,tugs=1)
        timer.sleep(visualization_speed) 
      
        
    #Do planning 
    if planner == "Independent":     
        run_independent_planner(aircraft_lst, nodes_dict, edges_dict, heuristics, t)
    elif planner == "Prioritized":
        run_prioritized_planner()
    elif planner == "CBS":

        # get available tugs
        available_tugs = [tug for tug in tugs_lst]

        # get aircraft that are waiting for a tug
        aircraft_queue = [ac for ac in active_aircrafts if ac.status == "waiting"]
        while aircraft_queue:
            ac_to_assign = aircraft_queue.pop(0)  # Get the first aircraft in the queue
            assign_tug_to_aircraft(ac_to_assign, available_tugs, t)

        new_tugging_started=False
        
        for tug in tugs_lst: ## Check if tug is available
            if tug.status == "assigned":
                tug.path_to_goal = [] #reset path to goal
                tug.plan_free_path(heuristics, t)  # Plan free path for tug
                print(tug.path_to_goal)
        
        for tug in tugs_lst:
            if tug.status == "moving_free":
                if tug.path_to_goal == []:
                    tug.goal = tug.assigned_ac.start  # Set goal to the aircraft's start node
                    tug.plan_free_path(heuristics, t)  # Plan free path for tug

            if tug.status == "moving_tugging":
                if not tug.path_to_goal:
                    # This tug just started tugging this timestep
                    new_tugging_started = True
            if 'moving' in tug.status:
                tug.move(dt, t)
            
            if tug.status == "low_energy":
                if charging_nodes:
                    tug.assigned_ac = None #detach aircraft
                    tug.path_to_goal = [] #reset path to goal
                    tug.goal = tug.find_closest_charging_node(heuristics)
                    
                    tug.plan_free_path(heuristics, t)  # Plan free path for tug
                    
            if tug.status == "charging":
                tug.charge(dt)
        if new_tugging_started or t in spawn_times:
            for tug in tugs_lst:
                if tug.status == "moving_tugging":
                    print("I am running")
                    tug.plan_tugging_path(
                        tug_list=tugs_lst,
                        aircraft_list=active_aircrafts,
                        nodes_dict=nodes_dict,
                        edges_dict=edges_dict,
                        heuristics=heuristics,
                        current_time=t,
                        max_static_block=10
                    )
                    break  # Only one call is needed since the method handles all tugging tugs

        for ac in active_aircrafts:
            if ac.status == "attached":
                ac.move(tugs_mode=1,dt=dt,t=t)
        
    t = t + dt
          
# =============================================================================
# 2. Implement analysis of output data here
# =============================================================================
#what data do you want to show?
