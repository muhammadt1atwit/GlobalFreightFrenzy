from simulator import VehicleType, haversine_distance_meters
from collections import defaultdict
import math

_PROXIMITY_M = 50.0

# Vehicle capabilities and restrictions
VEHICLE_RESTRICTIONS = {
    VehicleType.SemiTruck: {
        "terrain": "land",
        "spawn": "hub",
        "load_unload": "hub",
        "max_distance": None
    },
    VehicleType.Train: {
        "terrain": "land", 
        "spawn": "hub",
        "load_unload": "hub",
        "max_distance": None
    },
    VehicleType.Airplane: {
        "terrain": "any",
        "spawn": "airport",
        "load_unload": "airport",
        "max_distance": None
    },
    VehicleType.CargoShip: {
        "terrain": "water",
        "spawn": "ocean_port",
        "load_unload": "ocean_port",
        "max_distance": None
    },
    VehicleType.Drone: {
        "terrain": "any",
        "spawn": "airport",
        "load_unload": "airport",
        "max_distance": 20000  # 20km range
    }
}

def is_water_crossing(from_loc, to_loc):
    """Detect if path requires water crossing"""
    dist = haversine_distance_meters(from_loc, to_loc)
    if dist > 500000:
        return True
    
    from_lat, from_lon = from_loc
    to_lat, to_lon = to_loc
    
    # Atlantic crossing
    if (from_lon < -60 and to_lon > -10) or (from_lon > -10 and to_lon < -60):
        if abs(from_lat - to_lat) < 50:
            return True
    
    # Pacific crossing  
    if (from_lon < -120 and to_lon > 120) or (from_lon > 120 and to_lon < -120):
        return True
    
    return False

def nearest_facility(loc, facilities):
    """Return the closest facility coordinate to loc."""
    best = None
    best_dist = float('inf')
    for fac in facilities:
        d = haversine_distance_meters(loc, fac)
        if d < best_dist:
            best_dist = d
            best = fac
    return best

def can_vehicle_handle_route(vtype, from_loc, to_loc, num_boxes, sim_state):
    """Check if vehicle can handle this route given restrictions"""
    restrictions = VEHICLE_RESTRICTIONS[vtype]
    
    if restrictions["max_distance"]:
        dist = haversine_distance_meters(from_loc, to_loc)
        if dist > restrictions["max_distance"]:
            return False
    
    if num_boxes > vtype.value.capacity:
        return False
    
    return True

def step(sim_state):
    tick = sim_state.tick
    vehicles = sim_state.get_vehicles()
    boxes = sim_state.get_boxes()

    # FIX 3: Use actual hub/airport/port locations for spawning, not box locations
    hub_coords = list(sim_state.get_shipping_hubs())
    airport_coords = list(sim_state.get_airports())
    ocean_port_coords = list(sim_state.get_ocean_ports())

    # UNLOAD: Deliver boxes at destination
    for vid, v in vehicles.items():
        if v["destination"] is None and v["cargo"]:
            # FIX 1: Build to_unload only once — removed the duplicate for-loop
            to_unload = [
                bid for bid in v["cargo"]
                if haversine_distance_meters(v["location"], boxes[bid]["destination"]) <= _PROXIMITY_M
            ]
            if to_unload:
                sim_state.unload_vehicle(vid, to_unload)
                boxes = sim_state.get_boxes()
    
    # MANAGE VEHICLES: Load, move, transload
    for vid, v in vehicles.items():
        if v["destination"] is None:
            loc = v["location"]
            vtype = VehicleType[v["vehicle_type"]]
            capacity_left = vtype.value.capacity - len(v["cargo"])
            
            # Load available boxes at current location
            if capacity_left > 0:
                loadable = [
                    bid for bid, box in boxes.items()
                    if not box["delivered"] and box["vehicle_id"] is None
                    and haversine_distance_meters(loc, box["location"]) <= _PROXIMITY_M
                ]
                if loadable:
                    to_load = loadable[:capacity_left]
                    sim_state.load_vehicle(vid, to_load)
                    boxes = sim_state.get_boxes()
            
            if v["cargo"]:
                first_bid = v["cargo"][0]
                dest = boxes[first_bid]["destination"]
                num_boxes = len(v["cargo"])
                
                if can_vehicle_handle_route(vtype, loc, dest, num_boxes, sim_state):
                    sim_state.move_vehicle(vid, dest)
                else:
                    nearest_hub = nearest_facility(loc, hub_coords) if hub_coords else None
                    
                    if nearest_hub:
                        cargo_copy = list(v["cargo"])
                        target_dest = dest

                        # FIX 2: Fixed typo sims_state -> sim_state
                        sim_state.unload_vehicle(vid, cargo_copy)
                        boxes = sim_state.get_boxes()
                        
                        if is_water_crossing(loc, dest):
                            try:
                                spawn_port = nearest_facility(nearest_hub, ocean_port_coords) if ocean_port_coords else nearest_hub
                                new_vid = sim_state.create_vehicle(VehicleType.CargoShip, spawn_port)
                                sim_state.load_vehicle(new_vid, cargo_copy)
                                sim_state.move_vehicle(new_vid, target_dest)
                            except ValueError:
                                try:
                                    spawn_airport = nearest_facility(nearest_hub, airport_coords) if airport_coords else nearest_hub
                                    new_vid = sim_state.create_vehicle(VehicleType.Airplane, spawn_airport)
                                    sim_state.load_vehicle(new_vid, cargo_copy)
                                    sim_state.move_vehicle(new_vid, target_dest)
                                except ValueError:
                                    sim_state.move_vehicle(vid, target_dest)
                        else:
                            if num_boxes >= 20:
                                try:
                                    new_vid = sim_state.create_vehicle(VehicleType.Train, nearest_hub)
                                    sim_state.load_vehicle(new_vid, cargo_copy)
                                    sim_state.move_vehicle(new_vid, target_dest)
                                except ValueError:
                                    sim_state.move_vehicle(vid, target_dest)
                            else:
                                sim_state.move_vehicle(vid, target_dest)
                    else:
                        sim_state.move_vehicle(vid, dest)
            
            elif not v["cargo"]:
                nearest = None
                min_dist = float('inf')
                for bid, box in boxes.items():
                    if not box["delivered"] and box["vehicle_id"] is None:
                        dist = haversine_distance_meters(loc, box["location"])
                        if dist < min_dist:
                            min_dist = dist
                            nearest = box["location"]
                if nearest:
                    sim_state.move_vehicle(vid, nearest)
    
    # SPAWN: Create vehicles at valid facility locations
    if tick == 0 or (len(vehicles) < 15 and tick % 50 == 0):
        undelivered = [bid for bid, box in boxes.items() if not box["delivered"] and box["vehicle_id"] is None]
        
        if undelivered:
            origin_boxes = defaultdict(list)
            for bid in undelivered:
                origin_boxes[boxes[bid]["location"]].append(bid)
            
            for origin, box_ids in origin_boxes.items():
                num_boxes = len(box_ids)
                
                needs_water = False
                for bid in box_ids[:5]:
                    if is_water_crossing(origin, boxes[bid]["destination"]):
                        needs_water = True
                        break
                
                if needs_water:
                    if num_boxes >= 10 and num_boxes <= 1000:
                        try:
                            # FIX 3: Spawn at nearest ocean port, not raw box origin
                            spawn_loc = nearest_facility(origin, ocean_port_coords) if ocean_port_coords else nearest_facility(origin, hub_coords)
                            vid = sim_state.create_vehicle(VehicleType.CargoShip, spawn_loc)
                            to_load = box_ids[:1000]
                            sim_state.load_vehicle(vid, to_load)
                            if to_load:
                                dest = boxes[to_load[0]]["destination"]
                                sim_state.move_vehicle(vid, dest)
                            break
                        except ValueError:
                            pass
                    
                    if num_boxes <= 100:
                        try:
                            # FIX 3: Spawn at nearest airport, not raw box origin
                            spawn_loc = nearest_facility(origin, airport_coords) if airport_coords else nearest_facility(origin, hub_coords)
                            vid = sim_state.create_vehicle(VehicleType.Airplane, spawn_loc)
                            to_load = box_ids[:100]
                            sim_state.load_vehicle(vid, to_load)
                            if to_load:
                                dest = boxes[to_load[0]]["destination"]
                                sim_state.move_vehicle(vid, dest)
                            break
                        except ValueError:
                            pass
                
                if num_boxes >= 20 and num_boxes <= 500:
                    try:
                        # FIX 3: Spawn at nearest hub, not raw box origin
                        spawn_loc = nearest_facility(origin, hub_coords)
                        vid = sim_state.create_vehicle(VehicleType.Train, spawn_loc)
                        to_load = box_ids[:500]
                        sim_state.load_vehicle(vid, to_load)
                        if to_load:
                            dest = boxes[to_load[0]]["destination"]
                            sim_state.move_vehicle(vid, dest)
                        break
                    except ValueError:
                        pass
                
                if num_boxes <= 50:
                    try:
                        # FIX 3: Spawn at nearest hub, not raw box origin
                        spawn_loc = nearest_facility(origin, hub_coords)
                        vid = sim_state.create_vehicle(VehicleType.SemiTruck, spawn_loc)
                        to_load = box_ids[:50]
                        sim_state.load_vehicle(vid, to_load)
                        if to_load:
                            dest = boxes[to_load[0]]["destination"]
                            sim_state.move_vehicle(vid, dest)
                        break
                    except ValueError:
                        continue
