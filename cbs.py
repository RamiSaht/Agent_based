import heapq
from single_agent_planner import simple_single_agent_astar
import math

def euclidean_distance(pos1, pos2):
    x1, y1 = pos1
    x2, y2 = pos2
    return math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)

def detect_collision(path1, path2):
    t_1_start=path1[0][1]
    t_2_start=path2[0][1]
    t_1=path1[0][1]
    t_2=path2[0][1]
    loc1=path1[0][0]
    loc2=path2[0][0]
    n_1=0
    n_2=0
    while ((n_1<len(path1))and(n_2<len(path2))):
        if (t_1<t_2):
            while t_1<t_2:
                n_1+=1
                t_1+=0.5
                loc1=path1[n_1][0]
        else:
            while t_1>t_2:
                n_2+=1
                t_2+=0.5
                loc2=path2[n_2][0]
        if loc1 == loc2:
            return {'a1': path1, 'a2': path2, 'loc': [loc1], 'timestep': t_1}
        if (((t_1-t_1_start) > 0) and ((t_2-t_2_start)>0)):
            prev_loc1 = path1[n_1-1][0]
            prev_loc2 = path2[n_2-1][0]
            if prev_loc1 == loc2 and prev_loc2 == loc1:
                return {'a1': path1, 'a2': path2, 'loc': [prev_loc1, loc1], 'timestep': t_1}
        t_1+=0.5 #could be dt
        t_2+=0.5
        n_1+=1
        n_2+=1
        if ((n_1<len(path1))and(n_2<len(path2))):
            loc1 = path1[n_1][0]
            loc2 = path2[n_2][0]
    return False


def detect_collisions(paths):
    collisions = []
    num_agents = len(paths)
    for i in range(num_agents):
        for j in range(i + 1, num_agents):
            collision = detect_collision(paths[i], paths[j])
            if collision:
                collisions.append(
                    {'agent1': i, 'agent2': j, 'loc': collision['loc'], 'timestep': collision['timestep']})

    return collisions


def standard_splitting(collision):
    constraints = []
    if len(collision['loc']) == 1:
        constraints.append({'positive': False,'agent': collision['agent1'], 'loc': collision['loc'], 'timestep': collision['timestep']})
        constraints.append({'positive': False,'agent': collision['agent2'], 'loc': collision['loc'], 'timestep': collision['timestep']})
    else:
        constraints.append({'positive': False,'agent': collision['agent1'], 'loc': collision['loc'], 'timestep': collision['timestep']})
        constraints.append(
            {'positive': False,'agent': collision['agent2'], 'loc': collision['loc'][::-1], 'timestep': collision['timestep']})
    return constraints

class CBSSolver:
    def __init__(self, graph, nodes_dict, starts, goals,t,heuristics, static_obstacles=None,personal_obstacles=None):
        self.graph = graph  # This should be a DiGraph
        self.nodes_dict = nodes_dict
        self.starts = starts
        self.goals = goals
        self.num_of_agents = len(goals)
        self.num_of_generated = 0
        self.num_of_expanded = 0
        self.open_list = []
        self.heuristics = heuristics
        self.time = t
        self.static_obstacles = static_obstacles or []
        self.personal_obstacles = personal_obstacles or [] #prevent return
    def push_node(self, node):
        heapq.heappush(self.open_list, (node['cost'], len(node['collisions']), self.num_of_generated, node))
        self.num_of_generated += 1

    def pop_node(self):
        _, _, _, node = heapq.heappop(self.open_list)
        self.num_of_expanded += 1
        return node

    def find_solution(self):
        start_time = self.time
        root = {'cost': 0, 'constraints': [], 'paths': [], 'collisions': []}
        for node_id, t_start, t_end in self.static_obstacles:
            timestep = round(t_start, 2)
            while timestep <= round(t_end, 2):
                for agent in range(self.num_of_agents):
                    root['constraints'].append({
                        'positive': False,
                        'agent': agent,
                        'loc': [node_id],
                        'timestep': timestep
                    })
                timestep = round(timestep + 0.5, 2)
        for node_id, t_start, t_end, agent in self.personal_obstacles: #avoid return to previous node
            timestep = round(t_start, 2)
            while timestep <= round(t_end, 2):
                root['constraints'].append({
                    'positive': False,
                    'agent': agent,
                    'loc': [node_id],
                    'timestep': timestep
                })
                timestep = round(timestep + 0.5, 2)


        for i in range(self.num_of_agents):
            path = simple_single_agent_astar(self.nodes_dict, self.starts[i], self.goals[i], self.heuristics, start_time,
                                             root['constraints'], i)
            if not path[0]:
                raise Exception("No solution found")
            root['paths'].append(path[1])
        root['cost'] = sum(len(p) for p in root['paths'])
        root['collisions'] = detect_collisions(root['paths'])
        self.push_node(root)

        while self.open_list:
            node = self.pop_node()

            if not node['collisions']:
                return node['paths']

            collision = node['collisions'][0]
            constraints = standard_splitting(collision)
            for constraint in constraints:
                new_node = {
                    'cost': node['cost'],
                    'constraints': node['constraints'] + [constraint],
                    'paths': [],
                    'collisions': []
                }
                
                for i in range(self.num_of_agents):
                    path = simple_single_agent_astar(self.nodes_dict, self.starts[i], self.goals[i], self.heuristics, start_time,
                                                    new_node['constraints'], i)
                    
                    if not path[0]:
                        continue
                    new_node['paths'].append(path[1])
                new_node['cost'] = sum(len(p) for p in new_node['paths'])
                new_node['collisions'] = detect_collisions(new_node['paths'])

                self.push_node(new_node)
        return root['paths']

def run_CBS(graph, aircraft_lst, nodes_dict, edges_dict, heuristics, t, starts, goals,personal_obstacles=[]):
    """
    Run the CBS algorithm to compute conflict-free paths for aircraft.
    """
    # Create a directed graph (DiGraph)
    graph = graph
    # Add edges to the graph
    for edge, edge_data in edges_dict.items():
        if isinstance(edge_data, dict):
            weight = edge_data.get("weight", 1)  # Extract weight, default to 1 if missing
        else:
            weight = edge_data  # If already a number, use it directly

        # Ensure edge has valid nodes
        if edge[0] in nodes_dict and edge[1] in nodes_dict:
            graph.add_edge(edge[0], edge[1], weight=weight)
        else:
            print(f"Warning: Skipping invalid edge {edge}, missing nodes.")

    # Debug: Check if graph is empty
    if len(graph.nodes) == 0:
        print("Error: Graph is empty! Cannot run CBS.")
        return

    # Initialize CBS solver
    cbs_solver = CBSSolver(graph, nodes_dict, starts, goals, t, heuristics,personal_obstacles=personal_obstacles)
    # Compute paths using CBS
    try:
        paths = cbs_solver.find_solution()
    except Exception as e:
        print(f"CBS failed: {e}")
        return

    # Assign paths to all active aircraft
    active_aircraft = [ac for ac in aircraft_lst if ac.status != "done"]
    for ac, path in zip(active_aircraft, paths):
        ac.path_to_goal = path[1:]  # Remove the first node (current position)
        
        if path is None or len(path) < 2: #######-
            print(f"[CBS FAIL] Path for aircraft {ac.id} is too short or None at time {t}. Path: {path}")
            print(f"           Start: {starts[0]} Goal: {goals[0]}")
            print(f"           Raw path: {path}") 
            ac_status = "done" # Mark AC inactive to prevent crashing
            return None
        else:
            ac.from_to = [path[0][0], path[1][0]]  # Set from_node and to_node ########-

        ac.status = "taxiing"
        if ac.status == "taxiing":
            # Update position to the closest node in the new path
            ac.position = nodes_dict[path[0][0]]["xy_pos"]

def run_cbs_for_attached_tugs(tug_list, aircraft_list, nodes_dict, edges_dict, heuristics, t, max_static_block=10):
    import networkx as nx
    graph = nx.DiGraph()

    # Build the graph
    for edge, edge_data in edges_dict.items():
        weight = edge_data.get("weight", 1) if isinstance(edge_data, dict) else edge_data
        if edge[0] in nodes_dict and edge[1] in nodes_dict:
            graph.add_edge(edge[0], edge[1], weight=weight)

    # -- 1. Build CBS agent sets --
    starts = []
    goals = []

    # a) Moving agents (tugged aircraft)
    for tug in tug_list:
        if tug.attached_ac:
            ac = tug.attached_ac
            start_node = tug.position  # Or ac.position_node, depending on model
            goal_node = tug.attached_ac.goal
            starts.append(start_node)
            goals.append(goal_node)

    # b) Temporarily static agents (aircraft waiting for tug)
    static_blocks = []
    for ac in aircraft_list:
        if ac.status == "waiting":
            static_node = ac.position
            # Block for max_static_block time steps starting at t
            static_blocks.append((static_node, t, t + max_static_block))

    # -- 2. Initialize CBS solver with static constraints --
    cbs_solver = CBSSolver(graph, nodes_dict, starts, goals, t, heuristics,
                           static_obstacles=int(static_blocks))  # <-- You'd need to add support for this

    # -- 3. Solve
    try:
        paths = cbs_solver.find_solution()
    except Exception as e:
        print(f"CBS failed: {e}")
        return

    # -- 4. Assign paths
    for tug, path in zip([t for t in tug_list if t.attached_aircraft], paths):
        tug.path = path
        tug.status = "tugging"
        tug.attached_aircraft.path = path  # optional, for visualization
