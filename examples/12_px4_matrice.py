#!/usr/bin/env python
"""
| File: 12_px4_matrice.py
| Description: Build an app that uses the Pegasus API to simulate a Matrice 300RTK and record camera images.
"""

# Imports to start Isaac Sim from this script
import carb
import os
from isaacsim import SimulationApp

# Start simulation
public_ip = os.getenv("APP_LIVESTREAM_PUBLIC_IP", "127.0.0.1")
print("IP: " + public_ip)

# Start Isaac Sim's simulation environment
# Note: this simulation app must be instantiated right after the SimulationApp import, otherwise the simulator will crash
# as this is the object that will load all the extensions and load the actual simulator.
simulation_app = SimulationApp({
    "headless": True,
    "hide_ui": False,  # Show the GUI
    "/app/window/dpiScale": 1.0,
    "/app/window/enabled": True,
    "/app/renderer/enabled": True,
    "/app/livestream/enabled": True,
    "/app/livestream/proto": "webrtc",
    "/app/livestream/webrtc/enabled": True,
    #"--/app/livestream/publicEndpointAddress": public_ip,
    #"--/app/livestream/port": 49100,
    "width": 1280,
    "height": 720,
    "window_width": 1920,
    "window_height": 1080,
    "renderer": "RaytracedLighting",
    "display_options": 3286,  # Set display options to show default grid
})

from isaacsim.core.utils.extensions import enable_extension
enable_extension("omni.kit.uiapp")
enable_extension("omni.kit.livestream.webrtc")
enable_extension("isaacsim.ros2.bridge")
enable_extension("omni.services.livestream.nvcf")

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

from omni.isaac.core.utils.prims import create_prim
from pxr import Usd, UsdPhysics, PhysxSchema, UsdGeom
from isaacsim.sensors.camera import Camera
import isaacsim.core.utils.numpy.rotations as rot_utils
import omni.graph.core as og
import omni.replicator.core as rep
import omni.syntheticdata._syntheticdata as sd

class PegasusApp:

    def publish_rgb(self, camera: Camera, freq):
        # The following code will link the camera's render product and publish the data to the specified topic name.
        render_product = camera._render_product_path
        step_size = int(60/freq)
        topic_name = camera.name+"_rgb"
        queue_size = 1
        node_namespace = ""
        frame_id = camera.prim_path.split("/")[-1] # This matches what the TF tree is publishing.
        rv = omni.syntheticdata.SyntheticData.convert_sensor_type_to_rendervar(sd.SensorType.Rgb.name)
        writer = rep.writers.get(rv + "ROS2PublishImage")
        writer.initialize(
            frameId=frame_id,
            nodeNamespace=node_namespace,
            queueSize=queue_size,
            topicName=topic_name
        )
        writer.attach([render_product])
        # Set step input of the Isaac Simulation Gate nodes upstream of ROS publishers to control their execution rate
        gate_path = omni.syntheticdata.SyntheticData._get_node_path(
            rv + "IsaacSimulationGate", render_product
        )
        og.Controller.attribute(gate_path + ".inputs:step").set(step_size)
        return


    def __init__(self):
        """Initialize the simulation app and environment."""
        self.timeline = omni.timeline.get_timeline_interface()
        self.pg = PegasusInterface()

        self.pg._world = World(**self.pg._world_settings)
        self.world = self.pg.world

        self.pg.load_environment(SIMULATION_ENVIRONMENTS["QuintaDoCrasto"])
        #self.pg.set_global_coordinates(41.162037, -7.626251, 40)
        # Create a flat plane using a scaled cube
        create_prim(
            prim_path="/World/plane",
            prim_type="Cube",
            #position=(163, 73, -25.25),
            position=(0, 0, 0),
            scale=(0.5, 0.5, 0.01)
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
        self.monocular_camera_1 = MonocularCamera(
            "pegasus_camera",
            config={"update_rate": 10.0, "orientation": [0.0, -60.0, 180.0], "resolution": (1280, 720)}
        )

        self.monocular_camera_2 = MonocularCamera(
            "follow_camera",
            config={"update_rate": 10.0, "position":  [-5.0, -0.7, 4.2], "orientation": [0.0, -45.0, 180.0], "resolution": (1280, 720)}
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
        #config_multirotor.graphical_sensors = [self.monocular_camera_1]
        config_multirotor.graphical_sensors = [self.monocular_camera_1, self.monocular_camera_2]

        # Spawn the drone
        self.drone = Multirotor(
            "/World/quadrotor",
            ROBOTS['Matrice300'],
            0,
            #[163, 73, -24.25], # [-12.5, -75, 12.8],
            [72.61498, -144.73527, -27.65874], # [-12.5, -75, 12.8],
            #[0, 0, 0], # [-12.5, -75, 12.8],
            Rotation.from_euler("XYZ", [0.0, 0.0, 0.0], degrees=True).as_quat(),
            config=config_multirotor,
        )


        pos, quat = self.drone.get_world_pose()
        pos = np.array(pos)

        camera = Camera(
            prim_path="/World/quadrotor/body/camera",
            position=(pos + np.array([0.0, 0.0, 25.0])),
            frequency=20,
            resolution=(256, 256),
            orientation=rot_utils.euler_angles_to_quats(np.array([0, 90, 0]), degrees=True),
        )

        #self.pg.set_viewport_camera([5.0, 9.0, 6.5], [0.0, 0.0, 0.0])
        self.world.reset()
        self.stop_sim = False
        camera.initialize()
        self.publish_rgb(camera, 15)

        #camera.add_motion_vectors_to_frame()
        # Image saving config
        #self.image_output_dir = os.path.expanduser("~/pegasus_camera_images")
        #os.makedirs(self.image_output_dir, exist_ok=True)
        #self.image_count = 0

    def run(self):
        self.timeline.play()

        # Time when the last frame was saved
        #last_capture_time = 0.0  

        while simulation_app.is_running() and not self.stop_sim:
            self.world.step(render=True)

            # Check simulation time
            sim_time = self.world.current_time

            # Save frame
            #if (sim_time - last_capture_time) >= 0.1:
            #    if self.monocular_camera and self.monocular_camera.state is not None:
            #        image = self.monocular_camera.state.get("image")
            #        if image is not None:
            #            filename = os.path.join(self.image_output_dir, f"frame_{self.image_count:04d}.png")
            #            cv2.imwrite(filename, cv2.cvtColor(image, cv2.COLOR_RGB2BGR))
            #            print(f"Saved image at t={sim_time:.2f}s: {filename}")
            #            self.image_count += 1
            #            last_capture_time = sim_time  # Update capture time

        carb.log_warn("PegasusApp Simulation App is closing.")
        self.timeline.stop()
        simulation_app.close()
        
   

def main():
    app = PegasusApp()
    app.run()


if __name__ == "__main__":
    main()
