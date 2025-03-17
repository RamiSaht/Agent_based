import heapq
import networkx as nx
from time import time


def detect_collision(path1, path2):
    max_t = max(len(path1), len(path2))
    for t in range(max_t):
        loc1 = get_location(path1, t)
        loc2 = get_location(path2, t)

        if loc1 == loc2:
            return {'a1': path1, 'a2': path2, 'loc': [loc1], 'timestep': t}

        if t > 0:
            prev_loc1 = get_location(path1, t - 1)
            prev_loc2 = get_location(path2, t - 1)
            if prev_loc1 == loc2 and prev_loc2 == loc1:
                return {'a1': path1, 'a2': path2, 'loc': [prev_loc1, loc1], 'timestep': t}

    return None


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
        constraints.append({'agent': collision['agent1'], 'loc': collision['loc'], 'timestep': collision['timestep']})
        constraints.append({'agent': collision['agent2'], 'loc': collision['loc'], 'timestep': collision['timestep']})
    else:
        constraints.append({'agent': collision['agent1'], 'loc': collision['loc'], 'timestep': collision['timestep']})
        constraints.append(
            {'agent': collision['agent2'], 'loc': collision['loc'][::-1], 'timestep': collision['timestep']})
    return constraints


class CBSSolver:
    def __init__(self, graph, starts, goals):
        self.graph = graph
        self.starts = starts
        self.goals = goals
        self.num_of_agents = len(goals)
        self.num_of_generated = 0
        self.num_of_expanded = 0
        self.open_list = []
        self.heuristics = {goal: nx.single_source_dijkstra_path_length(graph, goal, weight='weight') for goal in goals}

    def push_node(self, node):
        heapq.heappush(self.open_list, (node['cost'], len(node['collisions']), self.num_of_generated, node))
        self.num_of_generated += 1

    def pop_node(self):
        _, _, _, node = heapq.heappop(self.open_list)
        self.num_of_expanded += 1
        return node

    def find_solution(self):
        start_time = time()
        root = {'cost': 0, 'constraints': [], 'paths': [], 'collisions': []}

        for i in range(self.num_of_agents):
            path = simple_single_agent_astar(self.graph, self.starts[i], self.goals[i], self.heuristics, 0,
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
                    path = simple_single_agent_astar(self.graph, self.starts[i], self.goals[i], self.heuristics, 0,
                                                     new_node['constraints'], i)
                    if not path[0]:
                        continue
                    new_node['paths'].append(path[1])

                new_node['cost'] = sum(len(p) for p in new_node['paths'])
                new_node['collisions'] = detect_collisions(new_node['paths'])
                self.push_node(new_node)

        return root['paths']