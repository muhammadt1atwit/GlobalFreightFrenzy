from simulator import VehicleType, haversine_distance_meters
from collections import defaultdict

_PROXIMITY_M = 50.0

def step(sim_state):
    tick = sim_state.tick
    vehicles = sim_state.get_vehicles()
    boxes = sim_state.get_boxes()

    # Get facility locations from the API
    hubs = list(sim_state.get_shipping_hubs())
    airports = list(sim_state.get_airports())
    ports = list(sim_state.get_ocean_ports())
    # If no airports are configured, fall back to hubs (API spec)
    if not airports:
        airports = hubs

    # Helper: check if a vehicle can unload at the current location
    def can_unload_here(vtype, loc):
        if vtype in [VehicleType.SemiTruck, VehicleType.Train]:
            return any(haversine_distance_meters(loc, h) <= 5000 for h in hubs)
        elif vtype == VehicleType.Airplane:
            return any(haversine_distance_meters(loc, a) <= 5000 for a in airports)
        elif vtype == VehicleType.CargoShip:
            return any(haversine_distance_meters(loc, p) <= 5000 for p in ports)
        else:  # Drone
            return any(haversine_distance_meters(loc, a) <= 5000 for a in airports)

    # Helper: check if a route is overseas (would require water crossing)
    def is_overseas(origin, dest):
        dist = haversine_distance_meters(origin, dest)
        # If distance > 500 km, assume it's overseas (conservative)
        return dist > 500000

    # ---------- 1. UNLOAD (with airport/port/hub check) ----------
    for vid, v in vehicles.items():
        if v["destination"] is None and v["cargo"]:
            vtype = VehicleType[v["vehicle_type"]]
            loc = v["location"]
            if can_unload_here(vtype, loc):
                to_unload = [bid for bid in v["cargo"]
                             if haversine_distance_meters(loc, boxes[bid]["destination"]) <= _PROXIMITY_M]
                if to_unload:
                    sim_state.unload_vehicle(vid, to_unload)
                    boxes = sim_state.get_boxes()
            else:
                # Not at a valid facility – move to the nearest appropriate one
                if vtype == VehicleType.Airplane:
                    target = min(airports, key=lambda a: haversine_distance_meters(loc, a)) if airports else None
                elif vtype == VehicleType.CargoShip:
                    target = min(ports, key=lambda p: haversine_distance_meters(loc, p)) if ports else None
                else:
                    target = min(hubs, key=lambda h: haversine_distance_meters(loc, h)) if hubs else None
                if target:
                    sim_state.move_vehicle(vid, target)

    # ---------- 2. MANAGE VEHICLES ----------
    for vid, v in vehicles.items():
        if v["destination"] is not None:
            continue
        loc = v["location"]
        vtype = VehicleType[v["vehicle_type"]]
        capacity_left = vtype.value.capacity - len(v["cargo"])

        # Load boxes at current location (only if at a valid facility – optional but safe)
        if capacity_left > 0 and can_unload_here(vtype, loc):
            loadable = [bid for bid, box in boxes.items()
                        if not box["delivered"] and box["vehicle_id"] is None
                        and haversine_distance_meters(loc, box["location"]) <= _PROXIMITY_M]
            if loadable:
                to_load = loadable[:capacity_left]
                sim_state.load_vehicle(vid, to_load)
                boxes = sim_state.get_boxes()

        if v["cargo"]:
            first_bid = v["cargo"][0]
            dest = boxes[first_bid]["destination"]
            # If this is a land vehicle and the route is overseas, do NOT go – instead go to nearest hub
            if vtype in [VehicleType.SemiTruck, VehicleType.Train] and is_overseas(loc, dest):
                hub = min(hubs, key=lambda h: haversine_distance_meters(loc, h)) if hubs else None
                if hub:
                    sim_state.move_vehicle(vid, hub)
                # else: stay still (better than water crossing)
            else:
                sim_state.move_vehicle(vid, dest)
        else:
            # Empty vehicle: go to nearest undelivered box's location
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
            elif hubs:
                sim_state.move_vehicle(vid, min(hubs, key=lambda h: haversine_distance_meters(loc, h)))

    # ---------- 3. SPAWN (only at tick 0, and only for land‑only boxes) ----------
    if tick == 0:
        # Group boxes by origin
        origin_boxes = defaultdict(list)
        for bid, box in boxes.items():
            if not box["delivered"]:
                origin_boxes[box["location"]].append(bid)

        for origin, bids in origin_boxes.items():
            # Separate overseas boxes (ignore them to avoid penalties)
            land_bids = [bid for bid in bids if not is_overseas(origin, boxes[bid]["destination"])]
            if not land_bids:
                continue

            # Try to spawn a train (cheap per km, large capacity)
            try:
                vid = sim_state.create_vehicle(VehicleType.Train, origin)
                to_load = land_bids[:500]
                sim_state.load_vehicle(vid, to_load)
                # Determine most common destination among loaded boxes
                dest_counts = defaultdict(int)
                for bid in to_load:
                    dest_counts[boxes[bid]["destination"]] += 1
                primary_dest = max(dest_counts, key=dest_counts.get)
                sim_state.move_vehicle(vid, primary_dest)
            except ValueError:
                # Fallback to truck
                try:
                    vid = sim_state.create_vehicle(VehicleType.SemiTruck, origin)
                    to_load = land_bids[:50]
                    sim_state.load_vehicle(vid, to_load)
                    dest_counts = defaultdict(int)
                    for bid in to_load:
                        dest_counts[boxes[bid]["destination"]] += 1
                    primary_dest = max(dest_counts, key=dest_counts.get)
                    sim_state.move_vehicle(vid, primary_dest)
                except ValueError:
                    pass
