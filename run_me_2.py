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

#%% SET SIMULATION PARAMETERS
#Input file names (used in import_layout) -> Do not change those unless you want to specify a new layout.
nodes_file = "nodes.xlsx" #xlsx file with for each node: id, x_pos, y_pos, type
edges_file = "edges.xlsx" #xlsx file with for each edge: from  (node), to (node), length

#Parameters that can be changed:
simulation_time = 30
planner = "CBS" #choose which planner to use (currently only Independent is implemented)
#Visualization (can also be changed)
plot_graph = False    #show graph representation in NetworkX
visualization = True        #pygame visualization
visualization_speed = 0.05 #set at 0.1 as default

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

def assign_tug_to_aircraft(aircraft_queue_l,available_tugs_l):
    """
    Needs to be implemented.
    Input:
        - aircraft_queue_l: list of aircraft waiting for a tug
        - available_tugs_l: list of available tugs
    """
    ac = aircraft_queue_l.pop(0)
    tug = available_tugs_l.pop(0)
    ac.assign_tug(tug)
    tug.assign_ac(ac)
    return 

#%% RUN SIMULATION
# =============================================================================
# 0. Initialization
# =============================================================================
nodes_dict, edges_dict, start_and_goal_locations = import_layout(nodes_file, edges_file)
graph = create_graph(nodes_dict, edges_dict, plot_graph)
heuristics = calc_heuristics(graph, nodes_dict)
aircraft_lst, spawn_times = parse_schedule("schedule.csv", nodes_dict)   #List which can contain aircraft agents
tugs_lst = parse_tugs("tugs.csv", nodes_dict) #List which can contain tug agents
print(f'aircraft_lst: {aircraft_lst}')
print(f'tugs_lst: {[tug for tug in tugs_lst]}')
print('spawn_times', spawn_times)

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
                                         "status": tug.status}
        escape_pressed = map_running(map_properties, current_aircrafts, current_tugs, t, dt, collisions)
        timer.sleep(visualization_speed) 
      
        
    #Do planning 
    if planner == "Independent":     
        run_independent_planner(aircraft_lst, nodes_dict, edges_dict, heuristics, t)
    elif planner == "Prioritized":
        run_prioritized_planner()
    elif planner == "CBS":
        # get available tugs
        available_tugs = [tug for tug in tugs_lst if tug.status == "ready"]
        
        # get aircraft that are waiting for a tug
        aircraft_queue = [ac for ac in active_aircrafts if ac.status == "waiting"]
        while aircraft_queue and available_tugs:
            assign_tug_to_aircraft(aircraft_queue, available_tugs)
            print(f"Assigned tug {tug} to ac \n{ac}")
        
        
        for tug in tugs_lst:
            if tug.status == "assigned":
                if tug.goal is None:  # Ensure goal is set
                    tug.goal = tug.assigned_ac.start  # Set goal to the aircraft's start node
                tug.plan_free_path(heuristics, t)  # Plan free path for tug
        
        for tug in tugs_lst:
            if tug.status == "moving_free":
                if tug.path_to_goal == []:
                    tug.goal = tug.assigned_ac.start  # Set goal to the aircraft's start node
                    tug.plan_free_path(heuristics, t)  # Plan free path for tug
            if tug.status == "moving_tugging":
                if tug.path_to_goal == []:
                    tug.goal = tug.assigned_ac.goal #set goal to the aircraft's goal
                    tug.plan_tugging_path(heuristics, t) #plan path to aircraft goal
            if 'moving' in tug.status:
                tug.move(dt, t)
                
                
                
        for ac in active_aircrafts:
            if ac.status == "attached":
                ac.move()
                
    t = t + dt
          
# =============================================================================
# 2. Implement analysis of output data here
# =============================================================================
#what data do you want to show?
