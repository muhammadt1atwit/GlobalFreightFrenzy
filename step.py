from simulator import VehicleType
import math

def distance(coord1, coord2):
    lat1 = math.radians(coord1[0])
    long1 = math.radians(coord2[0])
    lat2 = math.radians(coord1[1])
    long2 = math.radians(coord2[1])
    
    #return math.sqrt((coord1[0]-coord2[0])**2 + (coord1[1]-coord2[1])**2)

def step(sim_state):
    # Unload first - always check for deliveries
    vehicles = sim_state.get_vehicles()
    boxes = sim_state.get_boxes()
    
    for vid, v in vehicles.items():
        if v["destination"] is None and v["cargo"]:
            to_unload = [bid for bid in v["cargo"] 
                        if distance(boxes[bid]["destination"], v["location"]) < 0.0005]
            if to_unload:
                sim_state.unload_vehicle(vid, to_unload)
    
    # Only spawn vehicles at tick 0 or when needed
    if sim_state.tick == 0:
        for bid, box in boxes.items():
            if not box["delivered"] and box["vehicle_id"] is None:
                # Direct route - no hub hopping
                dist = distance(box["location"], box["destination"])
                
                # Choose cheapest vehicle that can do the job
                if dist > 0.5:  # Long distance
                    vtype = VehicleType.Train  # 0.02/km
                else:  # Short distance
                    vtype = VehicleType.SemiTruck  # 0.05/km
                
                try:
                    vid = sim_state.create_vehicle(vtype, box["location"])
                    sim_state.load_vehicle(vid, [bid])
                    sim_state.move_vehicle(vid, box["destination"])
                except ValueError:
                    # Try truck if train fails
                    try:
                        vid = sim_state.create_vehicle(VehicleType.SemiTruck, box["location"])
                        sim_state.load_vehicle(vid, [bid])
                        sim_state.move_vehicle(vid, box["destination"])
                    except ValueError:
                        pass
    
    # For vehicles that arrived but not at destination (wrong facility)
    for vid, v in vehicles.items():
        if v["destination"] is None and v["cargo"]:
            # Move to actual destination
            for bid in v["cargo"]:
                dest = boxes[bid]["destination"]
                if distance(dest, v["location"]) > 0.0005:
                    sim_state.move_vehicle(vid, dest)
                    break
