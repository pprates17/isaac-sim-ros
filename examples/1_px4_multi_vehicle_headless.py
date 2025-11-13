#!/usr/bin/env python
"""
| File: 1_px4_single_vehicle_headless.py
| Author: Marcelo Jacinto (marcelo.jacinto@tecnico.ulisboa.pt)
| License: BSD-3-Clause. Copyright (c) 2023, Marcelo Jacinto. All rights reserved.
| Description: Pegasus Simulator with WebRTC Livestream enabled
"""

import platform
import sys

# Exit early if running on ARM64 (aarch64) architecture
if platform.machine().lower() in ["aarch64", "arm64"]:
    print("Livestream is not supported on ARM64 architecture. Exiting.")
    sys.exit(0)

import carb
from isaacsim import SimulationApp

# Livestream config
CONFIG = {
    "width": 1920,
    "height": 1080,
    "window_width": 1920,
    "window_height": 1080,
    "headless": True,
    "hide_ui": True,
    "renderer": "RaytracedLighting",
    "display_options": 3286,
}

# Start Isaac Sim's simulation environment
simulation_app = SimulationApp(launch_config=CONFIG)

# Import the extension enabler
from isaacsim.core.utils.extensions import enable_extension

# Enable Livestream extension
enable_extension("omni.services.livestream.nvcf")


print("\n" + "="*60)
print("WebRTC Livestream enabled")
print("Access stream at: http://100.120.72.105:49100")
print("="*60 + "\n")

# -----------------------------------
# The actual Pegasus script starts here
# -----------------------------------
import omni.timeline
from omni.isaac.core.world import World

# Import the Pegasus API for simulating drones
from pegasus.simulator.params import ROBOTS, SIMULATION_ENVIRONMENTS
from pegasus.simulator.logic.state import State
from pegasus.simulator.logic.backends.px4_mavlink_backend import PX4MavlinkBackend, PX4MavlinkBackendConfig
from pegasus.simulator.logic.vehicles.multirotor import Multirotor, MultirotorConfig
from pegasus.simulator.logic.interface.pegasus_interface import PegasusInterface
from scipy.spatial.transform import Rotation


class PegasusApp:
    """
    A Template class that serves as an example on how to build a simple Isaac Sim standalone App with Livestream.
    """

    def __init__(self):
        """
        Method that initializes the PegasusApp and is used to setup the simulation environment.
        """

        # Acquire the timeline that will be used to start/stop the simulation
        self.timeline = omni.timeline.get_timeline_interface()

        # Start the Pegasus Interface
        self.pg = PegasusInterface()

        # Acquire the World
        self.pg._world = World(**self.pg._world_settings)
        self.world = self.pg.world

        # Launch one of the worlds provided by NVIDIA
        self.pg.load_environment(SIMULATION_ENVIRONMENTS["Warehouse"])
        
      # Create 10 multirotor vehicles
        for i in range(2):
            vehicle_id = i
            
            # Create positions in a 3x4 grid (leaving one empty for 10 total)
            x = (i % 4) * 2.0  # 0, 2, 4, 6
            y = (i // 4) * 2.0  # 0, 0, 0, 0, 2, 2, 2, 2, 4, 4
            
            config_multirotor = MultirotorConfig()
            mavlink_config = PX4MavlinkBackendConfig({
                "vehicle_id": vehicle_id,
                "px4_autolaunch": True,
                "px4_dir": self.pg.px4_path,
                "px4_vehicle_model": self.pg.px4_default_airframe
            })
            config_multirotor.backends = [PX4MavlinkBackend(mavlink_config)]

            Multirotor(
                f"/World/quadrotor_{i}",
                ROBOTS['Iris'],
                vehicle_id,
                [x, y, 0.07],  # Grid layout with 2m spacing
                Rotation.from_euler("XYZ", [0.0, 0.0, 0.0], degrees=True).as_quat(),
                config=config_multirotor,
            )

        
        # Reset the simulation environment so that all articulations (aka robots) are initialized
        self.world.reset()

        # Auxiliar variable for the timeline callback example
        self.stop_sim = False

    def run(self):
        """
        Method that implements the application main loop, where the physics steps are executed.
        """

        # Start the simulation
        self.timeline.play()

        # The "infinite" loop
        while simulation_app.is_running() and not self.stop_sim:
            # Update the UI of the app and perform the physics step
            self.world.step(render=True)
        
        # Cleanup and stop
        carb.log_warn("PegasusApp Simulation App is closing.")
        self.timeline.stop()
        simulation_app.close()


def main():
    # Instantiate the template app
    pg_app = PegasusApp()

    # Run the application loop
    pg_app.run()


if __name__ == "__main__":
    main()
