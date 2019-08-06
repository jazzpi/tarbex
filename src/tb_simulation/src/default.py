#!/usr/bin/env python3

import math
import os
from morse import builder

src_dir = os.path.dirname(__file__)
sim_dir = os.path.dirname(src_dir)
env_path = os.path.join(sim_dir, 'data', 'environment', 'environment.blend')

copter = builder.Quadrotor()
copter.name = "indoor"
copter.add_default_interface('ros')

waypoint_controller = builder.RotorcraftWaypoint()
waypoint_controller.properties(
    HorizontalPgain=0.03, HorizontalDgain=0.1, VerticalPgain=0.75,
    VerticalDgain=2.0, YawPgain=0.2, YawDgain=0.8, RollPitchPgain=9.7,
    RollPitchDgain=2.0)
# TODO: Add overlay?
copter.append(waypoint_controller)

cam_depth = builder.DepthCamera()
cam_depth.properties(classpath='morse.sensors.depth_camera.DepthVideoCamera')
cam_depth.translate(x=0.14, z=-0.05)
cam_depth.rotate(x=0.0, y=-math.pi/4, z=0.0)
#wideangle
cam_depth.properties(cam_width=640, cam_height=480, cam_focal=10.0, Vertical_Flip=True)
cam_depth.frequency(30)
cam_depth.add_stream('ros')
copter.append(cam_depth)

depth_cloud = builder.DepthCamera()
depth_cloud.translate(x=0.14, z=-0.05)
depth_cloud.rotate(x=0.0, y=-math.pi/4, z=0.0)
#wideangle
depth_cloud.properties(cam_width=640, cam_height=480, cam_focal=10.0)
depth_cloud.frequency(30)
depth_cloud.add_stream('ros')
copter.append(depth_cloud)

cam_video = builder.VideoCamera()
cam_video.translate(x=0.14, z=-0.05)
cam_video.rotate(x=0.0, y=-math.pi/4, z=0.0)
#wideangle
cam_video.properties(cam_width=640, cam_height=480, cam_focal=10.0)
cam_video.frequency(30)
cam_video.add_stream('ros')
copter.append(cam_video)

pose = builder.Pose()
pose.add_stream('ros')
copter.append(pose)

copter.translate(x=14.0, y=7.5, z=1.3)
copter.rotate(x=0.0, y=0.0, z=math.pi)

env = builder.Environment(env_path, fastmode=False)
env.set_camera_location([16, 7.5, 4])
env.set_camera_rotation([1.0470, 0, 0.7854])
env.properties(longitude=13.657874, latitude=52.307934, altitude=50.0)
env.set_material_mode(material_mode='GLSL')
env.show_framerate(True)
