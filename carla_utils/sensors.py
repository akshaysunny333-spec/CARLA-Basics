# Attach various sensors to a vehicle in CARLA simulator
import carla


# RGB Camera attachment
import carla

def attach_rgb_camera(world, vehicle, callback):
    """
    Attach RGB camera above the vehicle based on bounding box height
    """
    camera_bp = world.get_blueprint_library().find('sensor.camera.rgb')
    camera_bp.set_attribute('image_size_x', '800')
    camera_bp.set_attribute('image_size_y', '600')
    camera_bp.set_attribute('fov', '90')

    # Get vehicle bounding box to compute height
    bbox = vehicle.bounding_box  # carla.BoundingBox
    vehicle_height = bbox.extent.z * 2  # extent.z is half height

    # Place camera above vehicle + small offset
    camera_location = carla.Location(x=0.0, y=0.0, z=vehicle_height + 0.5)
    camera_transform = carla.Transform(camera_location)

    camera = world.spawn_actor(camera_bp, camera_transform, attach_to=vehicle)
    camera.listen(callback)

    return camera



# LiDAR attachment
def attach_lidar(world, vehicle, callback):
    bp_lib = world.get_blueprint_library()
    lidar_bp = bp_lib.find('sensor.lidar.ray_cast')

    lidar_bp.set_attribute('range', '100')
    lidar_bp.set_attribute('rotation_frequency', '10')
    lidar_bp.set_attribute('channels', '32')
    lidar_bp.set_attribute('points_per_second', '56000')

    transform = carla.Transform(
        carla.Location(x=0, z=2.5)
    )

    lidar = world.spawn_actor(
        lidar_bp,
        transform,
        attach_to=vehicle
    )

    lidar.listen(callback)
    return lidar


# GNSS attachment
def attach_gnss(world, vehicle, callback):
    bp_lib = world.get_blueprint_library()
    gnss_bp = bp_lib.find('sensor.other.gnss')

    transform = carla.Transform(
        carla.Location(x=0, z=2.0)
    )

    gnss = world.spawn_actor(
        gnss_bp,
        transform,
        attach_to=vehicle
    )

    gnss.listen(callback)
    return gnss


# IMU attachment
def attach_imu(world, vehicle, callback):
    bp_lib = world.get_blueprint_library()
    imu_bp = bp_lib.find('sensor.other.imu')

    transform = carla.Transform(
        carla.Location(x=0, z=2.0)
    )

    imu = world.spawn_actor(
        imu_bp,
        transform,
        attach_to=vehicle
    )

    imu.listen(callback)
    return imu
