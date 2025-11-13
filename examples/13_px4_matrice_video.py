#!/usr/bin/env python
"""
| File: 13_px4_matrice_video.py
| Description: Simulate a Matrice 300RTK using Pegasus API and record video on demand with terminal controls.
"""

# Imports to start Isaac Sim from this script
import carb
from isaacsim import SimulationApp

# Start simulation
simulation_app = SimulationApp({"headless": False})

# Imports after simulation_app is created
import omni.timeline
from omni.isaac.core.world import World

from pegasus.simulator.params import ROBOTS, SIMULATION_ENVIRONMENTS
from pegasus.simulator.logic.state import State
from pegasus.simulator.logic.backends.px4_mavlink_backend import PX4MavlinkBackend, PX4MavlinkBackendConfig
from pegasus.simulator.logic.vehicles.multirotor import Multirotor, MultirotorConfig
from pegasus.simulator.logic.interface.pegasus_interface import PegasusInterface
from pegasus.simulator.logic.graphical_sensors.monocular_camera import MonocularCamera

import os
import cv2
import numpy as np
from scipy.spatial.transform import Rotation
import datetime
import threading


class PegasusApp:
    def __init__(self):
        """Initialize the simulation app and environment."""
        self.timeline = omni.timeline.get_timeline_interface()
        self.pg = PegasusInterface()

        self.pg._world = World(**self.pg._world_settings)
        self.world = self.pg.world

        self.pg.load_environment(SIMULATION_ENVIRONMENTS["Crasto"])

        # Create and store the camera sensor before drone creation
        self.monocular_camera = MonocularCamera(
            "pegasus_camera",
            config={
                "update_rate": 30.0,
                "orientation": [0.0, -60.0, 180.0],
                "resolution": (1920, 1080),
            }
        )

        # Configure multirotor with PX4 backend and attach the camera
        config_multirotor = MultirotorConfig()
        mavlink_config = PX4MavlinkBackendConfig({
            "vehicle_id": 0,
            "px4_autolaunch": True,
            "px4_dir": self.pg.px4_path,
            "px4_vehicle_model": self.pg.px4_default_airframe
        })
        config_multirotor.backends = [PX4MavlinkBackend(mavlink_config)]
        config_multirotor.graphical_sensors = [self.monocular_camera]

        # Spawn the drone
        self.drone = Multirotor(
            "/World/quadrotor",
            ROBOTS['Matrice300'],
            0,
            [-12.5, -75, 12.8],
            Rotation.from_euler("XYZ", [0.0, 0.0, 90.0], degrees=True).as_quat(),
            config=config_multirotor,
        )

        self.pg.set_viewport_camera([5.0, 9.0, 6.5], [0.0, 0.0, 0.0])
        self.world.reset()
        self.stop_sim = False

        # Video settings
        timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        self.video_output_dir = os.path.expanduser(f"~/pegasus_camera_videos/{timestamp}")
        os.makedirs(self.video_output_dir, exist_ok=True)
        self.video_output_path = os.path.join(self.video_output_dir, "output_video.avi")

        self.frame_size = (1920, 1080)
        self.fps = 30.0
        self.capture_interval = 1.0 / self.fps

        self.video_writer = cv2.VideoWriter(
            self.video_output_path,
            cv2.VideoWriter_fourcc(*"XVID"),
            self.fps,
            self.frame_size
        )

        if not self.video_writer.isOpened():
            raise RuntimeError(f"Failed to open video writer at {self.video_output_path}")

        self.recording = False
        print(f"[INFO] Ready to record.")
        print("Enter 'r' to start recording, 's' to stop, 'q' to quit.")
        print(f"[INFO] Video will be saved to: {self.video_output_path}")

    def run(self):
        self.timeline.play()
        last_capture_time = 0.0

        while simulation_app.is_running() and not self.stop_sim:
            self.world.step(render=True)
            sim_time = self.world.current_time

            # Save video frame if recording 
            if self.recording and (sim_time - last_capture_time) >= self.capture_interval:
                if self.monocular_camera and self.monocular_camera.state is not None:
                    image = self.monocular_camera.state.get("image")
                    if isinstance(image, np.ndarray) and image.size > 0:
                        bgr_image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
                        bgr_image = cv2.resize(bgr_image, self.frame_size)
                        self.video_writer.write(bgr_image)
                        print(f"[{sim_time:.2f}s] Frame recorded.")
                        last_capture_time = sim_time

        # Cleanup
        self.video_writer.release()
        carb.log_warn("PegasusApp Simulation App is closing.")
        self.timeline.stop()
        simulation_app.close()


def keyboard_listener(app: PegasusApp):
    """Terminal input listener in a background thread."""
    while not app.stop_sim:
        try:
            cmd = input().strip().lower()
            if cmd == 'r':
                app.recording = True
                print("[INFO] Recording started.")
            elif cmd == 's':
                app.recording = False
                print("[INFO] Recording stopped.")
            elif cmd == 'q':
                app.stop_sim = True
                print("[INFO] Quitting simulation...")
        except EOFError:
            break


def main():
    try:
        app = PegasusApp()
        listener_thread = threading.Thread(target=keyboard_listener, args=(app,), daemon=True)
        listener_thread.start()
        app.run()
    except KeyboardInterrupt:
        print("[INFO] Simulation interrupted by user.")
    finally:
        simulation_app.close()


if __name__ == "__main__":
    main()
