import unittest
import math
import networkx as nx
from Aircraft import Aircraft
from Tug import Tug
from single_agent_planner import simple_single_agent_astar, calc_heuristics, build_constraint_table, is_constrained


class TestAircraft(unittest.TestCase):
    def setUp(self):
        """Setup test nodes and aircraft"""
        self.nodes_dict = {
            1: {"xy_pos": (0, 0), "type": "gate", "neighbors": {2}, "id": 1},
            2: {"xy_pos": (10, 0), "type": "taxiway", "neighbors": {1, 3, 5}, "id": 2},  # Add connection to node 5
            3: {"xy_pos": (20, 0), "type": "rwy_a", "neighbors": {2}, "id": 3},
            4: {"xy_pos": (0, 10), "type": "charging", "neighbors": {5}, "id": 4},
            5: {"xy_pos": (10, 10), "type": "taxiway", "neighbors": {2, 4, 6}, "id": 5},  # Add connection to node 2
            6: {"xy_pos": (20, 10), "type": "rwy_d", "neighbors": {5}, "id": 6},
        }

        # Create a graph with bidirectional edges for pathfinding
        self.graph = nx.DiGraph()
        for node_id, data in self.nodes_dict.items():
            self.graph.add_node(node_id, **data)

        # Add edges in both directions to ensure connectivity
        for node_id, data in self.nodes_dict.items():
            for neighbor in data["neighbors"]:
                self.graph.add_edge(node_id, neighbor, weight=1)
                self.graph.add_edge(neighbor, node_id, weight=1)  # Add reverse edge

        # Calculate heuristics after ensuring the graph is fully connected
        self.heuristics = calc_heuristics(self.graph, self.nodes_dict)

        # Create test aircraft
        self.arrival = Aircraft(1, "a", 3, 1, 0, self.nodes_dict)  # Arrival at gate 1
        self.departure = Aircraft(2, "d", 1, 6, 0, self.nodes_dict)  # Departure from gate 1

    def test_aircraft_initialization(self):
        """Test aircraft initialization with correct properties"""
        self.assertEqual(self.arrival.id, 1)
        self.assertEqual(self.arrival.type, "a")
        self.assertEqual(self.arrival.position, (20, 0))
        self.assertEqual(self.arrival.status, "waiting")

        self.assertEqual(self.departure.id, 2)
        self.assertEqual(self.departure.type, "d")
        self.assertEqual(self.departure.position, (0, 0))

    def test_heading_calculation(self):
        """Test aircraft heading calculations"""
        # Test horizontal movement (right)
        self.arrival.get_heading((0, 0), (10, 0))
        self.assertEqual(self.arrival.heading, 90)

        # Test horizontal movement (left)
        self.arrival.get_heading((10, 0), (0, 0))
        self.assertEqual(self.arrival.heading, 270)

        # Test vertical movement (up)
        self.arrival.get_heading((0, 0), (0, 10))
        self.assertEqual(self.arrival.heading, 0)

        # Test vertical movement (down)
        self.arrival.get_heading((0, 10), (0, 0))
        self.assertEqual(self.arrival.heading, 180)

    def test_aircraft_movement(self):
        """Test aircraft movement without tugs"""
        self.arrival.path_to_goal = [(2, 10), (1, 20.0)]
        self.arrival.from_to=[3,2]
        self.arrival.status = "taxiing"

        # Move halfway to next node
        self.arrival.move(tugs_mode=0, dt=5, t=0)
        self.assertEqual(self.arrival.position, (15, 0))
        self.assertEqual(self.arrival.heading, 270)
        # Complete move to next node
        self.arrival.move(tugs_mode=0, dt=5, t=5)
        self.assertEqual(self.arrival.position, (10, 0))

        # Verify path update
        self.assertEqual(self.arrival.from_to, [2, 1])

    def test_aircraft_movement_with_waiting(self):
        """Test aircraft waiting behavior at nodes"""
        # Test 1: Aircraft should wait at current node until scheduled time
        self.arrival.path_to_goal = [(2, 10),(2,20), (1, 30.0)]  # Wait at node 3 until t=5
        self.arrival.from_to = [3, 2]  # Initial waiting state
        self.arrival.position = self.nodes_dict[3]["xy_pos"]  # Start at node 3

        # Initial movement towards node 2
        self.arrival.move(tugs_mode=0, dt=5, t=0)
        self.assertEqual(self.arrival.position, (15.0,0.0)) #Should be in between nodes
        self.assertEqual(self.arrival.from_to, [3, 2])  # Should be moving

        #Continue movement as scheduled
        self.arrival.move(tugs_mode=0, dt=5, t=5)
        print(self.arrival)
        print(self.arrival.from_to)
        self.assertEqual(self.arrival.position, self.nodes_dict[2]["xy_pos"]) #Should be on node 2
        self.assertEqual(self.arrival.from_to, [2, 2])  # Should now have updated from_to to account to waiting

        # Should now wait at node
        self.arrival.move(tugs_mode=0,dt=5,t=10)
        print(self.arrival)
        print(self.arrival.from_to)
        self.assertEqual(self.arrival.position, self.nodes_dict[2]["xy_pos"])  # Should be waiting on node 2
        self.assertEqual(self.arrival.from_to, [2, 2])  # Should be waiting but have next element


class TestTug(unittest.TestCase):
    def setUp(self):
        """Setup test nodes and tugs"""
        self.nodes_dict = {
            1: {"xy_pos": (0, 0), "type": "gate", "neighbors": {2}, "id": 1},
            2: {"xy_pos": (10, 0), "type": "taxiway", "neighbors": {1, 3}, "id": 2},
            3: {"xy_pos": (20, 0), "type": "rwy_a", "neighbors": {2}, "id": 3},
            4: {"xy_pos": (0, 10), "type": "charging", "neighbors": {5}, "id": 4},
            5: {"xy_pos": (10, 10), "type": "taxiway", "neighbors": {4, 6}, "id": 5},
            6: {"xy_pos": (20, 10), "type": "rwy_d", "neighbors": {5}, "id": 6},
        }

        # Create a graph with bidirectional edges
        self.graph = nx.DiGraph()
        for node_id, data in self.nodes_dict.items():
            self.graph.add_node(node_id, **data)

        # Add edges in both directions
        for node_id, data in self.nodes_dict.items():
            for neighbor in data["neighbors"]:
                self.graph.add_edge(node_id, neighbor, weight=1)
                self.graph.add_edge(neighbor, node_id, weight=1)


        # Create test tugs
        self.tug1 = Tug(1, 4, 100, self.nodes_dict)  # Tug at charging station
        self.tug2 = Tug(2, 2, 50, self.nodes_dict)  # Tug on taxiway with medium energy
        self.tug3 = Tug(3, 1, 10, self.nodes_dict)  # Low energy tug

    def test_tug_initialization(self):
        """Test tug initialization with correct properties"""
        self.assertEqual(self.tug1.id, 1)
        self.assertEqual(self.tug1.position, (0, 10))
        self.assertEqual(self.tug1.status, "ready")
        self.assertEqual(self.tug1.energy, 100)

        self.assertEqual(self.tug3.energy, 10)
        self.assertEqual(self.tug3.status, "low_energy")

    def test_energy_management(self):
        """Test tug energy consumption and charging"""
        # Test energy consumption
        initial_energy = self.tug2.energy
        self.tug2.consume_energy(5)  # 5 distance units
        self.assertEqual(self.tug2.energy, initial_energy - 5 * self.tug2.charge_per_distance)

        # Test charging
        self.tug3.status = "charging"
        self.tug3.charge(1)  # Charge for 1 time unit
        self.assertEqual(self.tug3.energy, 10 + self.tug3.charge_speed * 1)

    def test_aircraft_assignment(self):
        """Test tug assignment to aircraft"""
        test_aircraft = Aircraft(1, "a", 3, 1, 0, self.nodes_dict)

        # Verify initial state
        self.assertIsNone(test_aircraft.assigned_tug)
        self.assertEqual(test_aircraft.status, "waiting")

        # Assign aircraft to tug
        self.tug1.assign_ac(test_aircraft)
        test_aircraft.assign_tug(self.tug1)  # Add this line to properly assign tug

        # Verify assignment
        self.assertEqual(self.tug1.assigned_ac, test_aircraft)
        self.assertEqual(test_aircraft.assigned_tug, self.tug1)
        self.assertEqual(self.tug1.status, "assigned")
        self.assertEqual(test_aircraft.status, "requested")


class TestPathfinding(unittest.TestCase):
    def setUp(self):
        """Setup test nodes with proper connectivity"""
        self.nodes_dict = {
            1: {"xy_pos": (0, 0), "type": "gate", "neighbors": {2, 5}, "id": 1},
            2: {"xy_pos": (10, 0), "type": "taxiway", "neighbors": {1, 3}, "id": 2},
            3: {"xy_pos": (20, 0), "type": "rwy_a", "neighbors": {2}, "id": 3},
            4: {"xy_pos": (0, 10), "type": "charging", "neighbors": {1, 5}, "id": 4},
            5: {"xy_pos": (10, 10), "type": "taxiway", "neighbors": {1, 4, 6}, "id": 5},
            6: {"xy_pos": (20, 10), "type": "rwy_d", "neighbors": {5}, "id": 6},
        }

        # Create a fully connected graph
        self.graph = nx.DiGraph()
        for node_id, data in self.nodes_dict.items():
            self.graph.add_node(node_id, **data)

        # Add edges in both directions for bidirectional movement
        for node_id, data in self.nodes_dict.items():
            for neighbor in data["neighbors"]:
                self.graph.add_edge(node_id, neighbor, weight=1)
                self.graph.add_edge(neighbor, node_id, weight=1)

        # Calculate heuristics
        self.heuristics = calc_heuristics(self.graph, self.nodes_dict)

    def test_single_agent_pathfinding(self):
        """Test basic A* pathfinding without constraints"""
        success, path = simple_single_agent_astar(
            self.nodes_dict,
            from_node=1,
            goal_node=3,
            heuristics=self.heuristics,
            time_start=0,
            constraints=[],
            agent="test"
        )

        self.assertTrue(success)
        self.assertEqual(len(path), 3)  # Should be 1 -> 2 -> 3
        self.assertEqual(path[0][0], 1)  # Start node
        self.assertEqual(path[-1][0], 3)  # Goal node

    def test_constraint_handling(self):
        """Test pathfinding with constraints"""
        # Create constraint that blocks node 2 at timestep 0.5
        constraints = [{
            'positive': False,
            'agent': "test",
            'loc': [2],
            'timestep': 0.5
        }]

        success, path = simple_single_agent_astar(
            self.nodes_dict,
            from_node=1,
            goal_node=3,
            heuristics=self.heuristics,
            time_start=0,
            constraints=constraints,
            agent="test"
        )

        self.assertTrue(success)
        # Should find alternative path or wait to avoid constraint
        for node, time in path:
            if abs(time - 0.5) < 0.01:  # Check if this is the constrained timestep
                self.assertNotEqual(node, 2)  # Shouldn't be at node 2 at timestep 0.5

    def test_build_constraint_table(self):
        """Test constraint table construction"""
        constraints = [
            {'positive': False, 'agent': "test", 'loc': [2], 'timestep': 1},
            {'positive': False, 'agent': "test", 'loc': [2, 3], 'timestep': 2},
            {'positive': True, 'agent': "test", 'loc': [4], 'timestep': 1.5}
        ]

        table = build_constraint_table(constraints, "test")

        self.assertIn(1, table)
        self.assertIn(2, table)
        self.assertIn(1.5, table)
        self.assertEqual(len(table[2]), 1)  # Edge constraint at timestep 2

    def test_is_constrained(self):
        """Test constraint checking"""
        constraints = [
            {'positive': False, 'agent': "test", 'loc': [2], 'timestep': 1},
            {'positive': False, 'agent': "test", 'loc': [2, 3], 'timestep': 2}
        ]

        table = build_constraint_table(constraints, "test")

        # Test vertex constraint
        self.assertTrue(is_constrained(1, 2, 1, table))

        # Test edge constraint
        self.assertTrue(is_constrained(2, 3, 2, table))

        # Test unconstrained move
        self.assertFalse(is_constrained(1, 2, 0.5, table))


if __name__ == "__main__":
    unittest.main()