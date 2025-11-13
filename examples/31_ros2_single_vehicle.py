#!/usr/bin/env python
"""
| File: 3_ros2_single_vehicle.py
| Author: Marcelo Jacinto (marcelo.jacinto@tecnico.ulisboa.pt)
| License: BSD-3-Clause. Copyright (c) 2024, Marcelo Jacinto. All rights reserved.
| Description: This files serves as an example on how to build an app that makes use of the Pegasus API to run a 
simulation with a single vehicle, controlled using the ROS2 backend system. NOTE: this ROS2 interface only works on Ubuntu 22.04LTS and ROS2 Humble
"""

# Imports to start Isaac Sim from this script
import os
import carb
from isaacsim import SimulationApp

public_ip = os.getenv("APP_LIVESTREAM_PUBLIC_IP", "127.0.0.1")
print("IP: " + public_ip)
# Start Isaac Sim's simulation environment
# Note: this simulation app must be instantiated right after the SimulationApp import, otherwise the simulator will crash
# as this is the object that will load all the extensions and load the actual simulator.
simulation_app = SimulationApp({
    "headless": True,                               
    "hide_gui": False,
    "/app/window/dpiScale": 1.0,
    "/app/window/enabled": True,
    "/app/renderer/enabled": True,
    "/app/livestream/enabled": True,
    "/app/livestream/proto": "webrtc",
    "/app/livestream/webrtc/enabled": True,
    "/app/livestream/webrtc/public_ip": os.getenv("PUBLIC_IP", "127.0.0.1"),
    "/app/livestream/webrtc/port": 49100,
})

# -----------------------------------
# The actual script should start here
# -----------------------------------
import omni.timeline
from omni.isaac.core.world import World

from isaacsim.core.utils.extensions import enable_extension
enable_extension("omni.kit.uiapp")
enable_extension("omni.kit.livestream.webrtc")
enable_extension("isaacsim.ros2.bridge")

# Import the Pegasus API for simulating drones
from pegasus.simulator.params import ROBOTS, SIMULATION_ENVIRONMENTS
from pegasus.simulator.logic.backends.ros2_backend import ROS2Backend
from pegasus.simulator.logic.graphical_sensors.monocular_camera import MonocularCamera
from pegasus.simulator.logic.vehicles.multirotor import Multirotor, MultirotorConfig
from pegasus.simulator.logic.interface.pegasus_interface import PegasusInterface

# Auxiliary scipy and numpy modules
from scipy.spatial.transform import Rotation
from pxr import Usd, UsdPhysics, PhysxSchema, UsdGeom
from isaacsim.core.utils.prims import create_prim

class PegasusApp:
    """
    A Template class that serves as an example on how to build a simple Isaac Sim standalone App.
    """

    def __init__(self):
        """
        Method that initializes the PegasusApp and is used to setup the simulation environment.
        """

        # Acquire the timeline that will be used to start/stop the simulation
        self.timeline = omni.timeline.get_timeline_interface()

        # Start the Pegasus Interface
        self.pg = PegasusInterface()

        # Acquire the World, .i.e, the singleton that controls that is a one stop shop for setting up physics, 
        # spawning asset primitives, etc.
        self.pg._world = World(**self.pg._world_settings)
        self.world = self.pg.world

        # Launch one of the worlds provided by NVIDIA
        self.pg.load_environment(SIMULATION_ENVIRONMENTS["QuintaDoCrasto"])

        # Create a flat plane using a scaled cube
       #     create_prim(
       #         prim_path="/World/plane",
       #         prim_type="Cube",
       #         position=(163, 73, -25.25),
       #         scale=(0.5, 0.5, 0.01)
       #     )
        plane_prim = create_prim(
             prim_path="/World/plane",
             prim_type="Plane",
             translation=(163, 73, -25.25),
             scale=(0.5, 0.5, 1.0)
         )

        # Access the stage and the prim
        stage = self.world.stage
        plane_prim = stage.GetPrimAtPath("/World/plane")

        # Apply a rigid body (static)
        UsdPhysics.RigidBodyAPI.Apply(plane_prim)
        UsdPhysics.CollisionAPI.Apply(plane_prim)

        # Mark it as kinematic (non-moving/static)
        rigid_api = UsdPhysics.RigidBodyAPI.Get(stage, "/World/plane")
        rigid_api.CreateRigidBodyEnabledAttr(True)
        rigid_api.CreateKinematicEnabledAttr(True)

        # Get the Cube's geometry and set it to invisible
        cube_geom = UsdGeom.Cube(self.world.stage.GetPrimAtPath("/World/plane"))
        cube_geom.CreateVisibilityAttr("invisible")

        # Create and store the camera sensor before drone creation
        self.monocular_camera = MonocularCamera(
            "pegasus_camera",
            config={"update_rate": 10.0, "orientation": [0.0, -60.0, 180.0], "resolution": (1280, 720)}
        )

        # Create the vehicle
        # Try to spawn the selected robot in the world to the specified namespace
        config_multirotor = MultirotorConfig()
        config_multirotor.backends = [ROS2Backend(vehicle_id=1, config={
            "namespace": 'drone',
            "pub_sensors": True,
            "pub_graphical_sensors": True,
            "pub_state": True,
            "pub_tf": False,
            "sub_control": True}
            )
        ]
        config_multirotor.graphical_sensors = [MonocularCamera("camera", config={"update_rate": 60.0})]
        
        # Spawn the drone
        self.drone = Multirotor(
            "/World/quadrotor",
            ROBOTS['Matrice300'],
            0,
            [163, 73, -24.8], # [-12.5, -75, 12.8],
            Rotation.from_euler("XYZ", [0.0, 0.0, 180.0], degrees=True).as_quat(),
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
