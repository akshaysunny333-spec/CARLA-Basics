# Script to spawn a vehicle in the CARLA simulator with optional autopilot.
import carla
import random

def spawn_vehicle(world, autopilot=False):
    import random

    bp_lib = world.get_blueprint_library()
    vehicle_bp = random.choice(bp_lib.filter('vehicle.*'))
    spawn_point = random.choice(world.get_map().get_spawn_points())
    vehicle = world.spawn_actor(vehicle_bp, spawn_point)

    if autopilot:
        vehicle.set_autopilot(True)

    # Position spectator camera behind the vehicle
    spectator = world.get_spectator()
    transform = vehicle.get_transform()

    # Compute offset behind vehicle based on forward vector
    forward_vector = transform.get_forward_vector()
    spectator_location = transform.location - forward_vector * 5  # 5 meters behind
    spectator_location.z += 3  # slightly above

    spectator_rotation = carla.Rotation(
        pitch=-10,  # slight look down
        yaw=transform.rotation.yaw,
        roll=0
    )

    spectator_transform = carla.Transform(spectator_location, spectator_rotation)
    spectator.set_transform(spectator_transform)

    return vehicle
