"""
This file contains single agent planner functions  that can be used by other planners.
Consider functions in this file as supporting functions.
"""

import heapq
import networkx as nx


def calc_heuristics(graph, nodes_dict):
    heuristics = {}
    for i in nodes_dict:
        heuristics[nodes_dict[i]["id"]] = {}
        for j in nodes_dict:
            path, path_length = heuristicFinder(graph, nodes_dict[i]["id"], nodes_dict[j]["id"])
            if path:
                heuristics[nodes_dict[i]["id"]][nodes_dict[j]["id"]] = path_length
    return heuristics


def heuristicFinder(graph, start_node, goal_node):
    try:
        path = nx.dijkstra_path(graph, start_node, goal_node, weight="weight")
        path_length = nx.dijkstra_path_length(graph, start_node, goal_node)
    except:
        raise Exception('Heuristic cannot be calculated: No connection between', start_node, "and", goal_node)
    return path, path_length


def build_constraint_table(constraints, agent):
    """
    Builds a constraint table for the given agent.
    INPUT:
        - constraints: List of constraints
        - agent: The agent ID for whom the constraints are being built
    RETURNS:
        - constraint_table: A dictionary where keys are timesteps (can be floats) and values are lists of constraints
    """
    positive = []  # Collect positive constraints
    negative = []  # Collect negative constraints
    constraint_table = {}  # Use a dictionary to handle float timesteps

    # Organize constraints by type and agent
    for constraint in constraints:
        if constraint['agent'] == agent:
            if constraint['positive']==True:  # Positive constraints apply to all agents
                if constraint['agent'] == agent:
                    positive.append(constraint)
                else:
                    negative.append(constraint)
            elif constraint['agent'] == agent:  # Negative constraints apply only to the specified agent
                negative.append(constraint)

            # Add constraints to the constraint table
            timestep = constraint['timestep']
            if timestep not in constraint_table:
                constraint_table[timestep] = []

            if constraint['positive']:  # Positive constraints
                if len(constraint['loc']) == 1:  # Vertex constraint
                    constraint_table[timestep].append({'loc': constraint['loc'], 'positive': True})
                else:  # Edge constraint
                    constraint_table[timestep - 0.5].append({'loc': [constraint['loc'][0]], 'positive': True})
                    constraint_table[timestep].append({'loc': [constraint['loc'][1]], 'positive': True})
            else:  # Negative constraints
                if len(constraint['loc']) == 1:  # Vertex constraint
                    constraint_table[timestep].append({'loc': constraint['loc'], 'positive': False})
                else:  # Edge constraint
                    constraint_table[timestep].append({'loc': constraint['loc'], 'positive': False})
    return constraint_table

def is_constrained(curr_loc, next_loc, next_time, constraint_table):
    """
    Checks if a move from curr_loc to next_loc at next_time is constrained.
    INPUT:
        - curr_loc: Current location of the agent
        - next_loc: Next location the agent wants to move to
        - next_time: Timestep at which the move is planned (can be a float)
        - constraint_table: Constraint table for the agent (dictionary with float keys)
    RETURNS:
        - True if the move is constrained, False otherwise
    """
    if next_time not in constraint_table:
        return False  # No constraints at this timestep

    for constraint in constraint_table[next_time]:
        if constraint['positive']:  # Positive constraint
            if len(constraint['loc']) == 1:  # Vertex constraint
                if constraint['loc'][0] != next_loc:
                    return True  # Violates positive vertex constraint
            else:  # Edge constraint
                if constraint['loc'] != [curr_loc, next_loc]:
                    return True  # Violates positive edge constraint
        else:  # Negative constraint
            if len(constraint['loc']) == 1:  # Vertex constraint
                if constraint['loc'][0] == next_loc:
                    return True  # Violates negative vertex constraint
            else:  # Edge constraint
                if constraint['loc'] == [curr_loc, next_loc]:
                    return True  # Violates negative edge constraint

    return False  # No constraints violated

def simple_single_agent_astar(nodes_dict, from_node, goal_node, heuristics, time_start, constraints, agent):
    """
    Performs A* search for a single agent while respecting constraints.
    - Prevents backtracking to the previous node while moving.
    - Only updates 'prev_loc' when the agent changes nodes (not stationary).
    """
    from_node_id = from_node
    goal_node_id = goal_node
    constraint_table = build_constraint_table(constraints, agent)
    open_list = []
    closed_list = {}
    h_value = heuristics[from_node_id][goal_node_id]
    root = {
        'loc': from_node_id,
        'g_val': 0,
        'h_val': h_value,
        'parent': None,
        'timestep': time_start,
        'prev_loc': None  # No previous location at start
    }
    push_node(open_list, root)
    closed_list[(root['loc'], root['timestep'])] = root

    while open_list:
        curr = pop_node(open_list)
        if curr['loc'] == goal_node_id:
            return True, get_path(curr)

        # Get original neighbors (use a copy to avoid modifying the original)
        neighbors = set(nodes_dict[curr['loc']]["neighbors"])

        # Remove prev_loc from neighbors (if it exists and we're not stationary)
        if curr['prev_loc'] is not None and curr['prev_loc'] != curr['loc']:
            neighbors.discard(curr['prev_loc'])  # Prevents backtracking

        # Add current node to neighbors (optional, if needed for waiting)
        neighbors.add(curr['loc'])

        for neighbor in neighbors:
            next_timestep = curr['timestep'] + 0.5  # Assuming 0.5 timestep per move
            if is_constrained(curr['loc'], neighbor, next_timestep, constraint_table):
                continue  # Skip constrained moves

            # Update prev_loc ONLY if moving to a new node
            if neighbor != curr['loc']:
                new_prev_loc = curr['loc']  # Update prev_loc (we moved)
            else:
                new_prev_loc = curr['prev_loc']  # Stayed stationary, keep prev_loc

            child = {
                'loc': neighbor,
                'g_val': curr['g_val'] + 1,  # Cost = 1 per move
                'h_val': heuristics[neighbor][goal_node_id],
                'parent': curr,
                'timestep': next_timestep,
                'prev_loc': new_prev_loc  # Updated conditionally
            }

            # Check if this node+timestep is already in closed_list
            if (child['loc'], child['timestep']) in closed_list:
                existing_node = closed_list[(child['loc'], child['timestep'])]
                if compare_nodes(child, existing_node):
                    closed_list[(child['loc'], child['timestep'])] = child
                    push_node(open_list, child)
            else:
                closed_list[(child['loc'], child['timestep'])] = child
                push_node(open_list, child)

    print("No path found, " + str(len(closed_list)) + " nodes visited")
    return False, []

def push_node(open_list, node):
    heapq.heappush(open_list, (node['g_val'] + node['h_val'], node['h_val'], node['loc'], node))


def pop_node(open_list):
    _, _, _, curr = heapq.heappop(open_list)
    return curr


def compare_nodes(n1, n2):
    return n1['g_val'] + n1['h_val'] < n2['g_val'] + n2['h_val']


def get_path(goal_node):
    path = []
    curr = goal_node
    while curr is not None:
        path.append((curr['loc'], curr['timestep']))
        curr = curr['parent']
    path.reverse()
    return path