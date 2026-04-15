from simulator import VehicleType, haversine_distance_meters
from collections import defaultdict
import math

_PROXIMITY_M = 50.0

# Vehicle cost per km (for decision making)
COST_PER_KM = {
    VehicleType.CargoShip: 0.01,
    VehicleType.Train: 0.02,
    VehicleType.SemiTruck: 0.05,
    VehicleType.Drone: 0.30,
    VehicleType.Airplane: 0.50,
}

CAPACITY = {
    VehicleType.SemiTruck: 50,
    VehicleType.Train: 500,
    VehicleType.Airplane: 100,
    VehicleType.CargoShip: 1000,
    VehicleType.Drone: 5,
}

# Cache for discovered facilities
_facilities_cache = {}
_vehicle_destinations = {}  # Track next destination for vehicles

def distance_m(loc1, loc2):
    """Cached distance calculation"""
    key = (loc1, loc2) if loc1 < loc2 else (loc2, loc1)
    if key not in _facilities_cache:
        _facilities_cache[key] = haversine_distance_meters(loc1, loc2)
    return _facilities_cache[key]

def is_overseas(origin, dest):
    """Quick check if route needs water crossing"""
    dist = distance_m(origin, dest)
    if dist > 500000:
        return True
    
    o_lat, o_lon = origin
    d_lat, d_lon = dest
    
    # Atlantic crossing
    if (o_lon < -60 and d_lon > -10) or (o_lon > -10 and d_lon < -60):
        return abs(o_lat - d_lat) < 50
    return False

def best_vehicle(num_boxes, distance_m, is_overseas_route):
    """Choose cheapest vehicle for the job"""
    if is_overseas_route:
        # Overseas: ship cheapest, plane faster but expensive
        if num_boxes > 100:
            return VehicleType.CargoShip
        else:  
            return VehicleType.Airplane
    
    # Land route
    if num_boxes > 50 and distance_m > 50000:
        return VehicleType.Train
    else: 
        return VehicleType.SemiTruck

def step(sim_state):
    tick = sim_state.tick
    vehicles = sim_state.get_vehicles()
    boxes = sim_state.get_boxes()
    
    # Discover hubs from all box locations
    hubs = set()
    for box in boxes.values():
        hubs.add(box["location"])
        hubs.add(box["destination"])
    hubs = list(hubs)
    
    # ===== 1. UNLOAD =====
    for vid, v in vehicles.items():
        if v["destination"] is None and v["cargo"]:
            to_unload = [
                bid for bid in v["cargo"]
                if distance_m(v["location"], boxes[bid]["destination"]) <= _PROXIMITY_M
            ]
            if to_unload:
                sim_state.unload_vehicle(vid, to_unload)
                boxes = sim_state.get_boxes()  # Refresh
    
    # ===== 2. MANAGE EXISTING VEHICLES =====
    for vid, v in vehicles.items():
        if v["destination"] is not None:
            continue  # Still moving
        
        loc = v["location"]
        vtype = VehicleType[v["vehicle_type"]]
        capacity_left = CAPACITY[vtype] - len(v["cargo"])
        
        # Load boxes at current location
        if capacity_left > 0:
            loadable = [
                bid for bid, box in boxes.items()
                if not box["delivered"] and box["vehicle_id"] is None
                and distance_m(loc, box["location"]) <= _PROXIMITY_M
            ]
            if loadable:
                to_load = loadable[:capacity_left]
                sim_state.load_vehicle(vid, to_load)
                boxes = sim_state.get_boxes()
        
        if v["cargo"]:
            # Get target destination
            target = boxes[v["cargo"][0]]["destination"]
            
            # Check if we need to switch vehicle types
            if vtype in [VehicleType.SemiTruck, VehicleType.Train]:
                if is_overseas(loc, target):
                    # Need to go to port/airport first
                    # Find nearest hub to transfer
                    nearest = min(hubs, key=lambda h: distance_m(loc, h))
                    if distance_m(loc, nearest) > 1000:
                        sim_state.move_vehicle(vid, nearest)
                    else:
                        # At hub, unload and create ship/plane
                        sim_state.unload_vehicle(vid, v["cargo"])
                        boxes = sim_state.get_boxes()
                        for new_type in [VehicleType.CargoShip, VehicleType.Airplane]:
                            try:
                                new_vid = sim_state.create_vehicle(new_type, loc)
                                sim_state.load_vehicle(new_vid, v["cargo"])
                                sim_state.move_vehicle(new_vid, target)
                                break
                            except ValueError:
                                continue
                else:
                    sim_state.move_vehicle(vid, target)
            
            elif vtype in [VehicleType.CargoShip, VehicleType.Airplane]:
                if not is_overseas(loc, target):
                    # Reached land, switch to land vehicle
                    sim_state.unload_vehicle(vid, v["cargo"])
                    boxes = sim_state.get_boxes()
                    for new_type in [VehicleType.Train, VehicleType.SemiTruck]:
                        try:
                            new_vid = sim_state.create_vehicle(new_type, loc)
                            sim_state.load_vehicle(new_vid, v["cargo"])
                            sim_state.move_vehicle(new_vid, target)
                            break
                        except ValueError:
                            continue
                else:
                    sim_state.move_vehicle(vid, target)
        
        elif not v["cargo"]:
            # Empty vehicle - go to nearest box or hub
            nearest_box = None
            min_dist = float('inf')
            for bid, box in boxes.items():
                if not box["delivered"] and box["vehicle_id"] is None:
                    dist = distance_m(loc, box["location"])
                    if dist < min_dist:
                        min_dist = dist
                        nearest_box = box["location"]
            
            if nearest_box:
                sim_state.move_vehicle(vid, nearest_box)
            elif hubs:
                sim_state.move_vehicle(vid, min(hubs, key=lambda h: distance_m(loc, h)))
    
    # ===== 3. SPAWN NEW VEHICLES =====
    if tick == 0 or (len(vehicles) < 20 and tick % 50 == 0):
        undelivered = [bid for bid, box in boxes.items() 
                      if not box["delivered"] and box["vehicle_id"] is None]
        
        if undelivered:
            # Group by origin
            origin_boxes = defaultdict(list)
            for bid in undelivered:
                origin_boxes[boxes[bid]["location"]].append(bid)
            
            # Process largest group first
            for origin, box_ids in sorted(origin_boxes.items(), key=lambda x: -len(x[1])):
                num_boxes = len(box_ids)
                overseas = is_overseas(origin, boxes[box_ids[0]]["destination"])
                
                # Choose best vehicle
                vtype = best_vehicle(num_boxes, 0, overseas)
                
                # For overseas routes, consider going to port first
                if overseas and vtype == VehicleType.CargoShip:
                    # Check if we can spawn ship directly
                    try:
                        vid = sim_state.create_vehicle(vtype, origin)
                        to_load = box_ids[:CAPACITY[vtype]]
                        sim_state.load_vehicle(vid, to_load)
                        sim_state.move_vehicle(vid, boxes[to_load[0]]["destination"])
                        break
                    except ValueError:
                        # Can't spawn ship here, use truck to port
                        vtype = VehicleType.SemiTruck
                
                # Spawn vehicle
                try:
                    vid = sim_state.create_vehicle(vtype, origin)
                    to_load = box_ids[:CAPACITY[vtype]]
                    sim_state.load_vehicle(vid, to_load)
                    if to_load:
                        dest = boxes[to_load[0]]["destination"]
                        sim_state.move_vehicle(vid, dest)
                    break
                except ValueError:
                    # Fallback to truck
                    try:
                        vid = sim_state.create_vehicle(VehicleType.SemiTruck, origin)
                        to_load = box_ids[:50]
                        sim_state.load_vehicle(vid, to_load)
                        if to_load:
                            sim_state.move_vehicle(vid, boxes[to_load[0]]["destination"])
                        break
                    except ValueError:
                        continue
                        continue
