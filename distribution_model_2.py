from ortools.sat.python import cp_model
from InstanceCVRPTWUI import InstanceCVRPTWUI
import math
from collections import defaultdict, deque

def get_dist(instance, n1, n2):
    instance.calculateDistances()
    return int(instance.calcDistance[n1][n2])

def build_distribution_model(instance, max_time=60.0, num_workers=8, verbose=False):
    """
    Solves distribution using CP-SAT and routes with a 'Critical Pair' aware heuristic.
    UPDATED: Includes Look-Ahead Feasibility and Desperation Mode to prevent locked tools.
    """
    # 1. Solve the High-Level Assignment
    result = _solve_model(instance, max_time, num_workers, verbose, strict=True)
    
    if result is None:
        if verbose:
            print("\n--- STRICT MODEL INFEASIBLE: RUNNING DIAGNOSTIC RELAXED MODEL ---")
        _solve_model(instance, max_time, num_workers, verbose, strict=False)
        return None

    # 2. Setup Routing Context
    instance.calculateDistances()
    if instance.calcDistance is None:
        raise ValueError("Distance calculation failed.")

    VEH_CAP = instance.Capacity
    MAX_TRIP_DIST = instance.MaxDistance
    Tools = {t.ID: t for t in instance.Tools}
    
    tasks_by_day = defaultdict(lambda: {'deliveries': [], 'pickups': []})
    
    # Track when tools are strictly absent from the depot
    tools_at_customer_start = defaultdict(lambda: defaultdict(int))

    for rid, (d_day, p_day, r) in result.items():
        tasks_by_day[d_day]['deliveries'].append(r)
        tasks_by_day[p_day]['pickups'].append(r)
        for d in range(d_day + 1, p_day + 1):
             tools_at_customer_start[d][r.tool] += r.toolCount

    routes_by_day = {}
    total_distance_all = 0
    final_assignment = {} 

    sorted_days = sorted(tasks_by_day.keys())
    
    for day in sorted_days:
        day_deliveries = tasks_by_day[day]['deliveries'][:]
        day_pickups = tasks_by_day[day]['pickups'][:]
        
        # deficit to correct
        depot_start_inventory = {}
        tool_demand = defaultdict(int)
        
        for t_id, tool_obj in Tools.items():
            used_at_start = tools_at_customer_start[day][t_id]
            depot_start_inventory[t_id] = tool_obj.amount - used_at_start
        
        for r in day_deliveries:
            tool_demand[r.tool] += r.toolCount
            
        critical_pickup_needed = defaultdict(int)
        for t_id, demand in tool_demand.items():
            avail = depot_start_inventory.get(t_id, 0)
            if demand > avail:
                critical_pickup_needed[t_id] = demand - avail

        day_routes = []

        # greedy        
        while day_deliveries or day_pickups:
            route = [0]
            curr_node = 0
            curr_dist = 0
            onboard_tools = defaultdict(int)
            #print(onboard_tools)
            critical_onboard = defaultdict(int) 
            
            route_active = True
            
            while route_active:
                curr_load = sum(count * Tools[t_id].weight for t_id, count in onboard_tools.items())
                best_candidate = None
                best_cost = float('inf')
                best_type = None 
                best_source_mode = None
                
                # ---------------- EVALUATE DELIVERIES ----------------
                for i, req in enumerate(day_deliveries):
                    req_weight = req.toolCount * Tools[req.tool].weight
                    needed = req.toolCount
                    kind = req.tool
                    
                    dist_direct = get_dist(instance, curr_node, req.node)
                    dist_via_depot = get_dist(instance, curr_node, 0) + get_dist(instance, 0, req.node)
                    dist_return = get_dist(instance, req.node, 0)
                    
                    # if i==1: print(kind, onboard_tools[kind], needed, onboard_tools[kind] >= needed)
                    if onboard_tools[kind] >= needed:
                        total_trip = curr_dist + dist_direct + dist_return
                        if total_trip <= MAX_TRIP_DIST:
                            priority_bonus = 0
                            if critical_onboard[kind] > 0:
                                priority_bonus = 100000 
                            cost = dist_direct - priority_bonus
                            if cost < best_cost:
                                best_candidate = (i, req)
                                best_cost = cost
                                best_type = 'DELIVERY'
                                best_source_mode = 'ONBOARD'
                    
                    # From Depot (Needs tools -> Fetch from Depot)
                    # Only if we have stock at depot start of day
                    missing = max(0, needed - onboard_tools[kind])
                    if missing > 0 and depot_start_inventory[kind] >= missing:
                        # If we are at depot, cost is direct. If not, cost is via depot.
                        travel_cost = dist_direct if curr_node == 0 else dist_via_depot
                        mode = 'VIA_DEPOT_INSTANT' if curr_node == 0 else 'VIA_DEPOT_TRAVEL'

                        if curr_load + (missing * Tools[kind].weight) <= VEH_CAP:
                            total_trip = curr_dist + travel_cost + dist_return
                            if total_trip <= MAX_TRIP_DIST and travel_cost < best_cost:
                                best_candidate = (i, req)
                                best_cost = travel_cost
                                best_type = 'DELIVERY'
                                best_source_mode = mode

                # ---------------- EVALUATE PICKUPS ----------------
                for i, req in enumerate(day_pickups):
                    req_weight = req.toolCount * Tools[req.tool].weight
                    dist_direct = get_dist(instance, curr_node, req.node)
                    dist_return = get_dist(instance, req.node, 0)
                    kind = req.tool
                    
                    if curr_load + req_weight <= VEH_CAP:
                        total_trip = curr_dist + dist_direct + dist_return
                        if total_trip <= MAX_TRIP_DIST:
                            cost = dist_direct
                            
                            is_critical_need = critical_pickup_needed[kind] > 0
                            
                            if is_critical_need:
                                # If we pick this up, can we actually deliver it?
                                # We scan available deliveries for this tool.
                                can_deliver_this_tool = False
                                for d_req in day_deliveries:
                                    if d_req.tool == kind:
                                        # Hypothetical path: Curr -> Pickup -> Delivery -> Depot
                                        d_dist = get_dist(instance, req.node, d_req.node)
                                        d_ret = get_dist(instance, d_req.node, 0)
                                        # !!! We don't add curr_dist here because total_trip calculation above 
                                        # ensures we can get to pickup and back. We need to check the triangle.
                                        full_cycle = curr_dist + dist_direct + d_dist + d_ret
                                        
                                        if full_cycle <= MAX_TRIP_DIST:
                                            can_deliver_this_tool = True
                                            break
                                
                                if not can_deliver_this_tool:
                                    # We need this tool, but this truck CANNOT deliver it.
                                    # Do not pick it up. Leave it for a fresh truck.
                                    continue
                                    
                                cost -= 50000 
                            
                            if sum(critical_onboard.values()) > 0:
                                cost += 5000 
                            
                            if cost < best_cost:
                                best_candidate = (i, req)
                                best_cost = cost
                                best_type = 'PICKUP'
                                best_source_mode = 'DIRECT'
                
                # If no "good" move found, and we only have pickups left, force a fetch.
                if best_candidate is None and not day_deliveries and day_pickups:
                     for i, req in enumerate(day_pickups):
                        req_weight = req.toolCount * Tools[req.tool].weight
                        dist_p = get_dist(instance, curr_node, req.node)
                        dist_r = get_dist(instance, req.node, 0)
                        
                        if (curr_load + req_weight <= VEH_CAP) and (curr_dist + dist_p + dist_r <= MAX_TRIP_DIST):
                            best_candidate = (i, req)
                            best_type = 'PICKUP'
                            best_source_mode = 'DIRECT'
                            break # Just take the first one that fits








                # ---------------- EXECUTE BEST ----------------
                if best_candidate:
                    idx, req = best_candidate
                    
                    if best_type == 'PICKUP':
                        curr_dist += get_dist(instance, curr_node, req.node)
                        curr_node = req.node
                        # curr_load += (req.toolCount * Tools[req.tool].weight)
                        onboard_tools[req.tool] += req.toolCount
                        route.append(-req.ID)
                        day_pickups.pop(idx)
                        
                        needed_cunt = critical_pickup_needed[req.tool]
                        if needed_cunt > 0:
                            amount_critical = min(req.toolCount, needed_cunt)
                            critical_onboard[req.tool] += amount_critical
                            critical_pickup_needed[req.tool] -= amount_critical
                        
                    elif best_type == 'DELIVERY':
                        needed = req.toolCount
                        kind = req.tool
                        # weight = needed * Tools[kind].weight
                        
                        if best_source_mode in ['VIA_DEPOT_TRAVEL', 'VIA_DEPOT_INSTANT']:
                            if best_source_mode == 'VIA_DEPOT_TRAVEL':
                                if route[-1] != 0: route.append(0)
                                curr_dist += get_dist(instance, curr_node, 0)
                                curr_dist += get_dist(instance, 0, req.node)
                            else:
                                curr_dist += get_dist(instance, curr_node, req.node)

                            missing_from_onboard = max(0, needed - onboard_tools[kind])
                            depot_start_inventory[kind] -= missing_from_onboard
                            onboard_tools[kind] += missing_from_onboard
                            #curr_load += (missing_from_onboard * Tools[kind].weight)
                            
                        elif best_source_mode == 'ONBOARD':
                            curr_dist += get_dist(instance, curr_node, req.node)

                        curr_node = req.node
                        route.append(req.ID)
                        onboard_tools[kind] -= needed
                        # curr_load -= weight
                        
                        if critical_onboard[kind] > 0:
                            critical_onboard[kind] = max(0, critical_onboard[kind] - needed)
                        # print(onboard_tools)

                        day_deliveries.pop(idx)
                
                else:
                    # no "good" move. 
                    # If we have critical tools onboard, try to force ANY feasible delivery to clear them.
                    rescued = False
                    if sum(critical_onboard.values()) > 0:
                        for i, req in enumerate(day_deliveries):
                            if critical_onboard[req.tool] > 0:
                                # Check simple feasibility (Distance only, unloading reduces load)
                                dist_direct = get_dist(instance, curr_node, req.node)
                                dist_return = get_dist(instance, req.node, 0)
                                
                                if curr_dist + dist_direct + dist_return <= MAX_TRIP_DIST:
                                    # Force execution
                                    curr_dist += dist_direct
                                    curr_node = req.node
                                    route.append(req.ID)
                                    
                                    needed = req.toolCount
                                    onboard_tools[kind] -= needed
                                    critical_onboard[kind] = max(0, critical_onboard[kind] - needed)
                                    kind = req.tool
                                    # weight = needed * Tools[kind].weight
                                    # curr_load -= weight
                                    day_deliveries.pop(i)
                                    rescued = True
                                    if verbose: print(f"  > Emergency Delivery of {kind} to node {req.node}")
                                    break # Loop back to main active loop
                    
                    if rescued:
                        continue

                    # If still stuck:
                    if sum(critical_onboard.values()) > 0:
                        if verbose:
                            print(f"Warning: Truck returning to depot with {dict(critical_onboard)} critical tools. Rerunning strictness recommended.")
                    
                    route_active = False
            
            # Close Route
            if curr_node != 0:
                curr_dist += get_dist(instance, curr_node, 0)
                route.append(0)
            
            if len(route) > 2:
                day_routes.append(route)
                total_distance_all += curr_dist
            else:
                if verbose and (day_deliveries or day_pickups):
                    print(f"STUCK on Day {day}: {len(day_deliveries)} deliveries, {len(day_pickups)} pickups left.")
                    return None
                break

        routes_by_day[day] = day_routes

    for rid, (d_day, p_day, r) in result.items():
        final_assignment[rid] = (d_day, p_day, 0)

    return final_assignment, routes_by_day, int(total_distance_all)


def _solve_model(instance: InstanceCVRPTWUI, max_time, num_workers, verbose, strict=True):
    """
    Solve the CP-SAT distribution model using InstanceCVRPTWUI structure.
    Returns mapping req_id -> delivery_day when strict feasible; returns None otherwise.
    If strict==False produce a diagnostic printout of slack needs and also return None.
    """
    model = cp_model.CpModel()

    Requests = instance.Requests              # list of Request objects
    Tools = {t.ID: t for t in instance.Tools} # dict {tool_id: Tool}

    # If explicit distance matrix exists, use it. Else Euclidean.
    use_explicit_dist = hasattr(instance, 'calcDistance') and instance.calcDistance

    coords = {}# {node_id: (x,y)}
    for loc_coord in instance.Coordinates:
        loc_id, x, y = loc_coord.ID, loc_coord.X, loc_coord.Y
        coords[loc_id] = (x, y)

    max_pickup_day = 0
    for r in Requests:
        # Check for bad data (Impossible time windows)
        if r.fromDay > r.toDay:
            if verbose: print(f"CRITICAL DATA ERROR: Request {r.ID} has fromDay {r.fromDay} > toDay {r.toDay}")
            return None
        max_possible_pickup = r.toDay + r.numDays
        if max_possible_pickup > max_pickup_day:
            max_pickup_day = max_possible_pickup
    # The scheduling horizon (days 1..max_pickup_day)
    horizon_days = list(range(1, max_pickup_day + 1))

    VEH_CAP = instance.Capacity if hasattr(instance, "Capacity") else 99999999
    # MAX_TRIP_DIST = instance.MaxDistance if hasattr(instance, "MaxDistance") else 99999999

    # ---------- Decision variables ----------
    deliver = {}            # (req_id, day) -> bool
    vars_by_request = {r.ID: [] for r in Requests}
    for r in Requests:
        for d in range(r.fromDay, r.toDay + 1):
            var = model.NewBoolVar(f"deliver_r{r.ID}_d{d}")
            deliver[(r.ID, d)] = var
            vars_by_request[r.ID].append(var)

    # Exactly one delivery per request
    for r in Requests:
        model.Add(sum(var for var in vars_by_request[r.ID]) == 1)

    # Daily Tool Availability
    tool_ids = list(Tools.keys())
    slacks = {}

    for t in horizon_days:
        for k in tool_ids:
            tool_limit = Tools[k].amount
            # collect linear terms coeff * var
            terms = []
            for r in Requests:
                if r.tool != k:
                    continue
                # r active on day t if delivered on a day d such that:
                #    d <= t <= d + r.numDays - 1  <=>  d in [t - r.numDays + 1, t]
                dmin = max(r.fromDay, t - r.numDays + 1)
                dmax = min(r.toDay, t)
                if dmin <= dmax:
                    for d in range(dmin, dmax + 1):
                        terms.append((r.toolCount, deliver[(r.ID, d)]))
            if terms:
                # create linear expression sum(coeff * var)
                expr = sum(coeff * var for coeff, var in terms)
                if strict:
                    model.Add(expr <= tool_limit)
                else:
                    # add slack variable
                    slack = model.NewIntVar(0, sum(coeff for coeff,_ in terms), f"slack_day{t}_tool{k}")
                    model.Add(expr <= tool_limit + slack)
                    slacks[(t,k)] = slack
            else:
                pass

    # single-request fits vehicle capacity check (quick infeasibility test)
    for r in Requests:
        t_weight = Tools[r.tool].weight * r.toolCount
        if t_weight > VEH_CAP:
             if verbose: print(f"IMPOSSIBLE REQUEST: Request {r.ID} weight ({t_weight}) > Vehicle Cap ({VEH_CAP})")
             return None

    solver = cp_model.CpSolver()
    solver.parameters.max_time_in_seconds = max_time
    solver.parameters.num_search_workers = num_workers
    solver.parameters.log_search_progress = verbose

    if not strict:
        # minimize total slack
        model.Minimize(sum(slacks.values()) if slacks else 0)

    status = solver.Solve(model)

    if strict:
        if status not in (cp_model.OPTIMAL, cp_model.FEASIBLE):
            return None
    else:
        # diagnostic output
        if status in (cp_model.OPTIMAL, cp_model.FEASIBLE):
            total_violations = solver.ObjectiveValue() if slacks else 0
            if verbose:
                print("=== DIAGNOSTIC REPORT ===")
                print(f"Total extra tools required (sum of slacks) = {int(total_violations)}")
                for (day,k), var in sorted(slacks.items(), key=lambda x: x[0]):
                    v = solver.Value(var)
                    if v > 0:
                        print(f" Day {day} Tool {k} needs +{v} (limit {Tools[k].amount})")
                print("=========================")
        else:
            if verbose:
                print("Diagnostic model also not solved to a feasible solution.")
        return None

    # Collect assignment
    assignment = {}
    for r in Requests:
        assigned = False
        for d in range(r.fromDay, r.toDay + 1):
            if solver.Value(deliver[(r.ID, d)]) == 1:
                assignment[r.ID] = (d, d + r.numDays, r)
                assigned = True
                break
        if not assigned:
            if verbose:
                print(f"Solver returned solution but request {r.ID} has no assigned day (unexpected).")
            return None

    return assignment
