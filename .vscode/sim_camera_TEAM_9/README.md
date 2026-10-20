# Synthetic camera input for Team 9

This package publishes synthetic JPEG images for testing the tracker report
pipeline. Every image is labelled SYNTHETIC TEST.

It draws an opponent marker using relative TF and an 80-degree horizontal
field of view. It does not render Gazebo, model obstacle occlusion, or provide
photographic evidence of a capture.

Use only in simulation. Do not connect it to the competition master server
or run it alongside a real camera on the same topic.

The default output matches the six-terminal tracker command:
`/tb3_0/image_raw/compressed`.

## Build

```bash
source /opt/ros/humble/setup.bash
cd ~/ros2_ws
colcon build --packages-select sim_camera_TEAM_9
source install/setup.bash
```

## Run

```bash
ros2 launch sim_camera_TEAM_9 sim_camera.launch.py
```

Map, localisation and simulation TF must already be running.
The node requires a running simulation clock.

In RViz, use Add → Image and select the published image topic.
This package does not publish CameraInfo for RViz's Camera display.

Successful report publication with these images demonstrates message
plumbing only. Actual camera capture requires separate validation.