# CARLA RGB + LiDAR visualization

import carla
import random
import cv2
import numpy as np
import open3d as o3d
import time


def main():
    #Setting up CARLA client and world
    client = carla.Client('localhost', 2000)
    client.set_timeout(10.0)

    world = client.get_world()
    bp_lib = world.get_blueprint_library()
    spawn_points = world.get_map().get_spawn_points()

    # Spawn a vehicle and set it to autopilot
    vehicle_bp = random.choice(bp_lib.filter('vehicle.*'))
    vehicle = world.try_spawn_actor(vehicle_bp, random.choice(spawn_points))
    if vehicle is None:
        print("Vehicle spawn failed")
        return

    vehicle.set_autopilot(True)

    # Adjust spectator to follow the vehicle
    spectator = world.get_spectator()
    transform = vehicle.get_transform()
    spectator.set_transform(
        carla.Transform(
            transform.location + carla.Location(x=-6, z=3),
            transform.rotation
        )
    )

    #Setup RGB camera
    camera_bp = bp_lib.find('sensor.camera.rgb')
    camera_bp.set_attribute('image_size_x', '800')
    camera_bp.set_attribute('image_size_y', '600')
    camera_bp.set_attribute('fov', '90')

    camera = world.spawn_actor(
        camera_bp,
        carla.Transform(carla.Location(x=1.5, z=2.4)),
        attach_to=vehicle
    )

    image_data = {"frame": None}

    def camera_callback(image):
        img = np.frombuffer(image.raw_data, dtype=np.uint8)
        img = img.reshape((image.height, image.width, 4))
        image_data["frame"] = img[:, :, :3]

    camera.listen(camera_callback)

    #Setup LiDAR sensor
    lidar_bp = bp_lib.find('sensor.lidar.ray_cast')
    lidar_bp.set_attribute('range', '50')
    lidar_bp.set_attribute('rotation_frequency', '10')
    lidar_bp.set_attribute('points_per_second', '100000')

    lidar = world.spawn_actor(
        lidar_bp,
        carla.Transform(carla.Location(x=0, z=2.5)),
        attach_to=vehicle
    )

    lidar_data = {"points": None}

    def lidar_callback(point_cloud):
        points = np.frombuffer(
            point_cloud.raw_data, dtype=np.float32
        ).reshape(-1, 4)
        lidar_data["points"] = points[:, :3]

    lidar.listen(lidar_callback)

    # Setup Open3D visualizer for LiDAR point cloud
    vis = o3d.visualization.Visualizer()
    vis.create_window("LiDAR Point Cloud", 960, 720)

    render_opt = vis.get_render_option()
    render_opt.background_color = np.array([0, 0, 0])
    render_opt.point_size = 2.0

    pcd = o3d.geometry.PointCloud()

    cv2.namedWindow("Camera Feed", cv2.WINDOW_AUTOSIZE)

    print("Running... Press 'q' to quit")

    lidar_initialized = False

    try:
        while True:
            #Camera Feed
            if image_data["frame"] is not None:
                cv2.imshow("Camera Feed", image_data["frame"])

            #LiDAR Point Cloud
            if lidar_data["points"] is not None:
                pts = lidar_data["points"]
                # Colorize points based on height (z-axis)
                if pts.shape[0] > 0:
                    pcd.points = o3d.utility.Vector3dVector(pts)        # Set point positions
                    z = pts[:, 2]                                       # Extract z-coordinates
                    z_range = np.ptp(z)                                 # Max - Min of z
                    if z_range < 1e-6:                                  # Avoid division by zero
                        z_norm = np.zeros_like(z)                       # All zeros if no variation
                    else:
                        z_norm = (z - z.min()) / z_range                # Normalize to [0, 1]

                    colors = np.zeros((pts.shape[0], 3))                # Initialize colors
                    colors[:, 0] = z_norm                               # Red channel       
                    colors[:, 2] = 1.0 - z_norm                         # Blue channel
                    pcd.colors = o3d.utility.Vector3dVector(colors)     # Set point colors

                    if not lidar_initialized:                           # First time adding geometry
                        vis.add_geometry(pcd)   
                        vis.poll_events()
                        vis.update_renderer()
                        vis.reset_view_point(True)
                        lidar_initialized = True
                    else:
                        vis.update_geometry(pcd)                        # Update existing geometry

            vis.poll_events()  
            vis.update_renderer() 

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

            time.sleep(0.01)

    except KeyboardInterrupt:
        print("\n Interrupted by user")

    finally:
        camera.stop()
        lidar.stop()

        camera.destroy()
        lidar.destroy()
        vehicle.destroy()

        cv2.destroyAllWindows()
        vis.destroy_window()

        print("All cleaned up.")


if __name__ == "__main__":
    main()
