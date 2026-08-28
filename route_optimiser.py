#import math
import copy
import random

class LocalSearchOptimizer:
    def __init__(self, instance, verbose=False):
        self.instance = instance
        self.verbose = verbose
        # Cache tool weights: {tool_id: weight}
        self.tool_weights = {t.ID: t.weight for t in instance.Tools}
        # Ensure distances are calculated
        if not hasattr(instance, 'calcDistance') or instance.calcDistance is None:
            instance.calculateDistances()
        self.CAPACITY = instance.Capacity
        self.MAX_DIST = instance.MaxDistance

        self.cost_distance = instance.DistanceCost
        self.cost_vehicle_day = instance.VehicleDayCost
        self.cost_vehicle_global = instance.VehicleCost
        
        self.vehicle_penalty = self.cost_vehicle_day + self.cost_vehicle_global





    def get_dist(self, n1, n2):
        """Get distance between two nodes (0=Depot, >0=Delivery, <0=Pickup)."""
        loc1 = 0 if n1 == 0 else self.instance.Requests[abs(n1)-1].node
        loc2 = 0 if n2 == 0 else self.instance.Requests[abs(n2)-1].node
        return self.instance.calcDistance[loc1][loc2]

    def calculate_route_cost(self, route):
        """Calculates total distance of a single route."""
        dist = 0
        for i in range(len(route) - 1):
            dist += self.get_dist(route[i], route[i+1])
        return dist

    def calculate_day_total_cost(self, routes):
        """
        Calculates weighted cost for the day:
        Cost = (Total Distance * DistanceCost) + (Num Routes * VehiclePenalty)
        """
        total_dist = sum(self.calculate_route_cost(r) for r in routes)
        # Routes with length <= 2 are just [0, 0] (depot-depot) and are effectively empty.
        active_vehicles = sum(1 for r in routes if len(r) > 2)
        
        cost = (total_dist * self.cost_distance) + (active_vehicles * self.vehicle_penalty)
        return cost


    def is_route_valid(self, route):
        """
        Validates Capacity and Max Distance.
        Constraint: Total weight on board must never exceed Capacity.
        Assumption: All deliveries are loaded at start.
        """
        current_dist = 0
        current_load = 0
        
        for node_id in route:
            if node_id > 0: # Delivery
                req = self.instance.Requests[node_id - 1]
                current_load += req.toolCount * self.tool_weights[req.tool]
        
        if current_load > self.CAPACITY:
            return False

        # Simulate Traversal and constraints
        for i in range(len(route) - 1):
            curr = route[i]
            next_node = route[i+1]
            
            # Distance Constraint
            d = self.get_dist(curr, next_node)
            current_dist += d
            if current_dist > self.MAX_DIST:
                return False
            
            # Capacity Constraint Update
            if next_node != 0:
                req = self.instance.Requests[abs(next_node) - 1]
                weight = req.toolCount * self.tool_weights[req.tool]
                # print(req,weight)
                if next_node > 0: # Delivery -> Drop off
                    current_load -= weight
                else: # Pickup -> Load
                    current_load += weight
                
                if current_load > self.CAPACITY:
                    return False
        
        return True
    
    def _clean_route(self, route):
        """Removes consecutive duplicate nodes (specifically 0, 0) from a route."""
        if not route: return route
        cleaned = [route[0]]
        for node in route[1:]:
            if node != cleaned[-1]:
                cleaned.append(node)
        return cleaned












    # Heuristic Functions

    def move_inside_route(self, routes):
        """Moves a visit into a new location inside a route."""
        if not routes: return False
        r_idx = random.randrange(len(routes))
        route = routes[r_idx][:]
        
        if len(route) <= 3: return False # Depot-Node-Depot (length 3) cannot move inside
        
        # Pick a node to move (indices 1 to len-2)
        i = random.randint(1, len(route) - 2)
        node = route.pop(i)
        route = self._clean_route(route)

        # Pick a new position
        j = random.randint(1, len(route) - 1) # route is now shorter
        route.insert(j, node)
        if self.is_route_valid(route):
            original_cost = self.calculate_route_cost(routes[r_idx])
            new_cost = self.calculate_route_cost(route)
            if new_cost < original_cost:
                routes[r_idx] = route
                return True
        return False

    def move_block_inside_route(self, routes):
        """Moves a block of some visits into a new location inside a route."""
        if not routes: return False
        r_idx = random.randrange(len(routes))
        route = routes[r_idx][:]
        
        n = len(route)
        if n < 5: return False # Need enough nodes to form a block and move it
        
        # Block [i, j]
        i = random.randint(1, n - 3)
        max_block_size = min(4, n - 2 - i) # Limit block size for stability
        block_len = random.randint(1, max_block_size)
        j = i + block_len
        block = route[i:j]
        del route[i:j]
        route = self._clean_route(route)

        # Insert Block
        k = random.randint(1, len(route) - 1)
        new_route = route[:k] + block + route[k:]
        if self.is_route_valid(new_route):
            if self.calculate_route_cost(new_route) < self.calculate_route_cost(routes[r_idx]):
                routes[r_idx] = new_route
                return True
        return False


    def reverse_block_in_route(self, routes):
        """Reverses a block of visits inside a route."""
        if not routes: return False
        r_idx = random.randrange(len(routes))
        route = routes[r_idx][:]
        if len(route) <= 3: return False
        
        # Indices for reversing
        i = random.randint(1, len(route) - 3)
        j = random.randint(i + 1, len(route) - 2)
        # Reverse segment [i, j]
        new_route = route[:i] + route[i:j+1][::-1] + route[j+1:]
        
        if self.is_route_valid(new_route):
            if self.calculate_route_cost(new_route) < self.calculate_route_cost(routes[r_idx]):
                routes[r_idx] = new_route
                return True
        return False
    
    def swap_inside_route(self, routes):
        """Swaps two visits inside the same route."""
        if not routes: return False
        r_idx = random.randrange(len(routes))
        route = routes[r_idx][:]
        
        if len(route) <= 3: return False
        i, j = random.sample(range(1, len(route)-1), 2)
        route[i], route[j] = route[j], route[i]
        if self.is_route_valid(route):
            if self.calculate_route_cost(route) < self.calculate_route_cost(routes[r_idx]):
                routes[r_idx] = route
                return True
        return False

    def move_between_routes(self, routes):
        """Moves a visit from one route to another (Relocate elsevhere)."""
        if len(routes) < 2: return False
        
        idx1, idx2 = random.sample(range(len(routes)), 2)
        r1, r2 = routes[idx1][:], routes[idx2][:]
        if len(r1) <= 2: return False # Empty route check
        
        # Pick node from r1
        src_idx = random.randint(1, len(r1) - 2)
        node = r1.pop(src_idx)
        r1 = self._clean_route(r1)
        # Insert into r2
        dst_idx = random.randint(1, len(r2) - 1)
        r2.insert(dst_idx, node)
        
        # Check Validity & Cost
        if self.is_route_valid(r1) and self.is_route_valid(r2):
            old_dist = self.calculate_route_cost(routes[idx1]) + self.calculate_route_cost(routes[idx2])
            old_cost = (old_dist * self.cost_distance) + (2 * self.vehicle_penalty)            # If r1 becomes empty ([0,0]), we save VehicleDayCost
            new_dist = self.calculate_route_cost(r1) + self.calculate_route_cost(r2)
            active_vehicles_new = 2
            if len(r1) <= 2: # r1 is now empty (Depot-Depot)
                active_vehicles_new = 1
            
            new_cost = (new_dist * self.cost_distance) + (active_vehicles_new * self.vehicle_penalty)
            
            if new_cost < old_cost:
                routes[idx1] = r1
                routes[idx2] = r2
                return True
        return False


    def swap_between_routes(self, routes):
        """Swaps visits between two different routes."""
        if len(routes) < 2: return False
        idx1, idx2 = random.sample(range(len(routes)), 2)
        r1, r2 = routes[idx1][:], routes[idx2][:]
        
        if len(r1) <= 2 or len(r2) <= 2: return False
        
        i = random.randint(1, len(r1) - 2)
        j = random.randint(1, len(r2) - 2)
        
        # Swap
        r1[i], r2[j] = r2[j], r1[i]
        
        if self.is_route_valid(r1) and self.is_route_valid(r2):
            old_cost = self.calculate_route_cost(routes[idx1]) + self.calculate_route_cost(routes[idx2])
            new_cost = self.calculate_route_cost(r1) + self.calculate_route_cost(r2)
            
            if new_cost < old_cost:
                routes[idx1] = r1
                routes[idx2] = r2
                return True
        return False

##########################################################################
##########################################################################
##########################################################################

    def optimize_day(self, routes, iterations=1000):
        """Applies random heuristics to improve a specific day's routes."""
        if not routes: return routes
        
        heuristics = [
            self.move_inside_route,
            self.move_block_inside_route,
            self.reverse_block_in_route,
            self.move_between_routes,
            self.swap_inside_route,
            self.swap_between_routes,
        ]
        
        # best_cost = self.calculate_day_cost(routes)
        
        # try "random" (fix seed) heuristics
        random.seed = 20011009
        for _ in range(iterations):
            heuristic = random.choice(heuristics)
            random.seed = 20011009 + _  # no random thing here... :)
            success = heuristic(routes)     # modify inplace
            
            if success:
                # If valid improvement found, the heuristic has already updated 'routes'
                print(f"JEEEEEEEE")
                routes = [r for r in routes if len(r) > 2]
                route = self._clean_route(route)
                pass

        return routes





    def optimize(self, routes_by_day, iterations_per_day=1000):
        """Main entry point. Iterates over all days and optimizes them."""
        optimized_routes = {}
        total_cost_before = 0
        total_cost_after = 0
        
        for day in sorted(routes_by_day.keys()):
            routes = copy.deepcopy(routes_by_day[day])
            
            initial_cost = self.calculate_day_total_cost(routes)
            total_cost_before += initial_cost
            
            if self.verbose:
                n_trucks = len([r for r in routes if len(r)>2])
                print(f"Optimizing Day {day}: {n_trucks} trucks, Cost {int(initial_cost)}... ", end="")
            
            # Optimization
            routes = self.optimize_day(routes, iterations=iterations_per_day)
            
            final_cost = self.calculate_day_total_cost(routes)
            total_cost_after += final_cost
            optimized_routes[day] = routes
            
            if self.verbose:
                n_trucks_after = len([r for r in routes if len(r)>2])
                print(f"-> {n_trucks_after} trucks, Cost {int(final_cost)}")
                
        if self.verbose:
            print(f"Total Weighted Cost Improvement: {int(total_cost_before)} -> {int(total_cost_after)}")
            
        return optimized_routes

def improve_routes(instance, routes_by_day, verbose=True):
    optimizer = LocalSearchOptimizer(instance, verbose)
    return optimizer.optimize(routes_by_day)