"""
Run-me.py is the main file of the simulation. Run this file to run the simulation.
"""

import os
import pandas as pd
import networkx as nx
import matplotlib.pyplot as plt
import time as timer
import pygame as pg
from single_agent_planner import calc_heuristics
from visualization import map_initialization, map_running
from Aircraft import Aircraft,find_closest_node
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
num_aircraft = 8 #number of aircraft to be generated
if os.path.exists("run_config.py"): ###### sensitivity analysis
    exec(open("run_config.py").read()) ###### sensitivity analysis

planner = "CBS" #choose which planner to use (currently only Independent is implemented)
#Visualization (can also be changed)
plot_graph = False    #show graph representation in NetworkX
visualization = True        #pygame visualization
visualization_speed = 0.005 #set at 0.1 as default

# Don't change
last_aircraft_spawn = 0 #time of last aircraft spawn used in random generation
num_spawned_aircraft = 0 #number of aircraft spawned in random generation

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

def parse_tugs(file_path, nodes_dict):
    
    df = pd.read_csv(file_path)
    lst = []
    
    for i, row in df.iterrows():
        if row.starting_node not in nodes_dict.keys():
            raise Exception("Error: Start node of tug does not exist in nodes_dict")
        lst.append(Tug(row.tug_id, row.starting_node, row.starting_energy, nodes_dict))
    
    return lst


def continuous_random_generation(t):
    '''
    Generates aircraft at constant intervals until the maximum number of aircraft is reached.
    Aircraft are generated at random gates and runways.
    The logic takes into account the availability of gates and runways.
    '''
    global last_aircraft_spawn, num_spawned_aircraft
    # Test if it's time to generate a new aircraft
    random_generation_interval = random_generation_time // num_aircraft
    num_simulated_aircraft = len(aircraft_lst)
    active_aircrafts = [ac for ac in aircraft_lst if ac.status != "done"]
    

    if  num_spawned_aircraft < num_aircraft and t - last_aircraft_spawn >= random_generation_interval:
        occupied_gates = [ac.start for ac in active_aircrafts if ac.type == "D"]
        occupied_rwy_arrs = [ac.start for ac in active_aircrafts if ac.type == "A"]
        available_gates = [gate for gate in gates if gate not in occupied_gates]
        available_rwy_arrs = [rwy for rwy in rwy_arrs if rwy not in occupied_rwy_arrs]
        choice = random.choice(aircraft_type_choices)
        spawn_time_l =  t
        
        goal_gates = set()
        for ac in active_aircrafts:
            if ac.type == "A":
                goal_gates.add(ac.goal)
        
        if choice == "A" and available_rwy_arrs:
            start_node = random.choice(available_rwy_arrs)
            goal_node = random.choice(gates)
        elif choice == "D" and available_gates:
            departure_gates = [gate for gate in available_gates if gate not in goal_gates]
            if not departure_gates:
                return
            start_node = random.choice(departure_gates)
            goal_node = random.choice(rwy_deps)
        else:
            return
        
        ac = Aircraft(f'A{num_simulated_aircraft}', choice, start_node, goal_node, spawn_time_l, nodes_dict)
        last_aircraft_spawn = spawn_time_l
        num_spawned_aircraft += 1
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
    aircraft_lst, spawn_times = parse_schedule("schedule.csv", nodes_dict)  #List which can contain aircraft agents

tugs_lst = parse_tugs("tugs.csv", nodes_dict) #List which can contain tug agents
print(f'aircraft_lst: {aircraft_lst}')
print(f'tugs_lst: {[tug.id for tug in tugs_lst]}')
print('spawn_times', spawn_times)

if visualization:
    map_properties = map_initialization(nodes_dict, edges_dict) #visualization properties

# =============================================================================
# 1. While loop and visualization
# =============================================================================

#First create dictionaries for all data required for simulation
time_per_node_all = {}
idle_time_per_node_all = {}
location_tracking = {}


#Start of while loop    
running=True
escape_pressed = False
time_end = simulation_time if simulation_time else 999999
dt = 0.1 #should be factor of 0.5 (0.5/dt should be integer)
t= 0
tugs_mode=0

print("Simulation Started")
while running:    
    t= round(t,2)
    continuous_random_generation(t) if random_schedule else None #generate random aircraft if random_schedule is true
    # aircraft_lst, spawn_times = parse_schedule("schedule.csv", nodes_dict)
    active_aircrafts = [ac for ac in aircraft_lst if (ac.spawntime <= t and ac.status != "done")]   
       
    #Check conditions for termination
    if t >= time_end or escape_pressed or pg.event.get(pg.QUIT): 
        running = False
        pg.quit()
        print("Simulation Stopped")
        break 
    
    #Visualization: Update map if visualization is true
    if visualization:
        current_aircrafts = {} #Collect current states of all aircraft
        for ac in aircraft_lst:
            if ac.status == "taxiing":
                current_aircrafts[ac.id] = {"ac_id": ac.id,
                                         "xy_pos": ac.position,
                                         "heading": ac.heading,
                                            "status": ac.status}
        
        current_tugs = {} #Collect current states of all tugs
        for tug in tugs_lst:
            current_tugs[tug.id] = {"tug_id": tug.id,
                                         "xy_pos": tug.position,
                                         "heading": tug.heading}
        escape_pressed = map_running(map_properties, current_aircrafts, current_tugs, t, dt,collisions=[],tugs=tugs_mode)
        timer.sleep(visualization_speed) 
      
        
    #Do planning 
    if planner == "Independent":     
        run_independent_planner(aircraft_lst, nodes_dict, edges_dict, heuristics, t)
    elif planner == "Prioritized":
        run_prioritized_planner()
    elif planner == "CBS":
        if t in spawn_times:
            starts = []
            goals = []
            active_aircrafts = []
            for ac in aircraft_lst:
                if ac.status != "done":
                    if ac.status == "taxiing":
                        current_node = find_closest_node(ac.position, nodes_dict)
                        starts.append(current_node)
                        goals.append(ac.goal)
                        active_aircrafts.append(ac)
                    else:
                        if t == ac.spawntime:
                            starts.append(ac.start)
                            goals.append(ac.goal)
                            active_aircrafts.append(ac)
                
            run_CBS(graph, active_aircrafts, nodes_dict, edges_dict, heuristics, t, starts, goals)
    #elif planner == -> you may introduce other planners here
    else:
        raise Exception("Planner:", planner, "is not defined.")
                       
    #Move the aircraft that are taxiing
    for ac in aircraft_lst: 
        if ac.status == "taxiing":
            # Aircraft moves with assigned tug
            ac.move(tugs_mode,dt,t)
                           
    t = t + dt
          
# =============================================================================
# 2. Implement analysis of output data here
# =============================================================================
#what data do you want to show?

total_node_times_all_aircraft = {}
total_dwell_times_all_aircraft = {}
#Location of an agent at a given timestep can be determined with ac.closest_node.
results = []
for ac in aircraft_lst:
    #get rid of rounding errors
    rounded_total_times = {node: round(time, 2) for node, time in ac.node_total_times.items()}
    rounded_dwell_times = {node: round(time, 2) for node, time in ac.node_dwell_times.items()}

    # Calculate total time for each node
    for node, time in rounded_total_times.items():
        total_node_times_all_aircraft[node] = total_node_times_all_aircraft.get(node, 0) + time

    #Calculate total waiting time for each node
    for node, time in rounded_dwell_times.items():
        total_dwell_times_all_aircraft[node] = total_dwell_times_all_aircraft.get(node, 0) + time

    if ac.start_time is not None:  # Ensure aircraft was spawned
        entry = {
            "aircraft_id": ac.id,
            "start_time": ac.start_time,
            "end_time": round(ac.end_time, 2),
            "time_to_destination": ac.get_time_to_destination() if ac.get_time_to_destination() is not None else "FAILED",
            "moving_time": round(ac.moving_time, 2) if hasattr(ac, "moving_time") else "N/A",
            "idle_time": round(ac.idle_time, 2) if hasattr(ac, "idle_time") else "N/A",
            "status": "planned" if ac.end_time is not None else "failed",
            "path": ac.visited_nodes,
            "total_time_on_node": rounded_total_times,
            "time_waiting_node": rounded_dwell_times,
            "travelled distance": round(ac.total_distance)
        }
        results.append(entry)

print(total_node_times_all_aircraft) #Time spent on each node can be taken from here
print(total_dwell_times_all_aircraft) #Time waited on each node can be taken from here

# Save to CSV
results_df = pd.DataFrame(results)
results_df.to_csv("output_time_to_destination_scenario_A.csv", index=False)
print("Time-to-destination data saved to: output_time_to_destination_scenario_A.csv")
