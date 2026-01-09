# Visualizer for Camera, LiDAR, GNSS, and IMU data
import numpy as np
import cv2
import open3d as o3d
import time


class SensorVisualizer:
    def __init__(self):

        self.image_frame = None
        self.lidar_points = None
        self.gnss_data = None
        self.imu_data = None

        # OpenCV Camera Window
        cv2.namedWindow("Camera Feed", cv2.WINDOW_AUTOSIZE)

        # Open3D LiDAR Window
        self.vis = o3d.visualization.Visualizer()
        self.vis.create_window("LiDAR Point Cloud", 960, 720)

        render_opt = self.vis.get_render_option()
        render_opt.background_color = np.array([0, 0, 0])
        render_opt.point_size = 2.0

        self.pcd = o3d.geometry.PointCloud()
        self.lidar_initialized = False

        # Running flag for main loop
        self.running = True

    # Sensor Callbacks

    def camera_callback(self, image):
        img = np.frombuffer(image.raw_data, dtype=np.uint8)
        img = img.reshape((image.height, image.width, 4))
        self.image_frame = img[:, :, :3]

    def lidar_callback(self, point_cloud):
        points = np.frombuffer(point_cloud.raw_data, dtype=np.float32).reshape(-1, 4)
        self.lidar_points = points[:, :3]

    def gnss_callback(self, data):
        self.gnss_data = data

    def imu_callback(self, data):
        self.imu_data = data

    # Non-blocking tick update

    def run_tick(self):
        """
        Run one iteration of visualization. Call this each frame from scenario.py
        Allows external updates like dynamic spectator movement.
        """
        # Camera feed
        if self.image_frame is not None:
            frame = self.image_frame.copy()

            # Overlay GNSS
            if self.gnss_data is not None:
                cv2.putText(frame, f"GNSS Lat: {self.gnss_data.latitude:.6f}", (10, 25),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                cv2.putText(frame, f"GNSS Lon: {self.gnss_data.longitude:.6f}", (10, 50),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                cv2.putText(frame, f"GNSS Alt: {self.gnss_data.altitude:.2f} m", (10, 75),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

            # Overlay IMU
            if self.imu_data is not None:
                acc = self.imu_data.accelerometer
                gyro = self.imu_data.gyroscope

                cv2.putText(frame, f"Accel: x={acc.x:.2f} y={acc.y:.2f} z={acc.z:.2f}", (10, 110),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
                cv2.putText(frame, f"Gyro:  x={gyro.x:.2f} y={gyro.y:.2f} z={gyro.z:.2f}", (10, 135),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)

            cv2.imshow("Camera Feed", frame)

        # LiDAR point cloud
        if self.lidar_points is not None:
            pts = self.lidar_points
            if pts.shape[0] > 0:
                self.pcd.points = o3d.utility.Vector3dVector(pts)

                # Height-based coloring
                z = pts[:, 2]
                z_range = np.ptp(z)
                z_norm = (z - z.min()) / z_range if z_range > 1e-6 else np.zeros_like(z)

                colors = np.zeros((pts.shape[0], 3))
                colors[:, 0] = z_norm
                colors[:, 2] = 1.0 - z_norm
                self.pcd.colors = o3d.utility.Vector3dVector(colors)

                if not self.lidar_initialized:
                    self.vis.add_geometry(self.pcd)
                    self.vis.reset_view_point(True)
                    self.lidar_initialized = True
                else:
                    self.vis.update_geometry(self.pcd)

        self.vis.poll_events()
        self.vis.update_renderer()

        # Quit condition
        if cv2.waitKey(1) & 0xFF == ord('q'):
            self.running = False

    # Blocking run loop (optional)

    def run(self):
        """
        Original blocking run loop (optional).
        """
        print("Visualization running... Press 'q' to quit")
        try:
            while self.running:
                self.run_tick()
                time.sleep(0.01)
        finally:
            self.close()

    # Cleanup

    def close(self):
        cv2.destroyAllWindows()
        self.vis.destroy_window()
        print("[INFO] Visualization closed cleanly")
