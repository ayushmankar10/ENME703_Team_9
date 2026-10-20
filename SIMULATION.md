# Team 9 simulation

## Environment

Use ROS 2 Humble in The Construct's ROS 2 Navigation course.
The course simulation must already be running with:
- tb3_0: chaser
- tb3_1: opponent

Both robots use the shared /tf and /tf_static topics.

## Build

In Terminal 1:

```bash
source /opt/ros/humble/setup.bash
cd ~/ros2_ws

colcon build --base-paths src \
  --build-base build_team9 \
  --install-base install_team9 \
  --packages-up-to path_planner_server_TEAM_9 tracker_TEAM_9
```

Continue only if the build succeeds.

## Start navigation and RViz

Stop previous manually launched map servers, AMCL nodes, Nav2
servers and RViz instances. Leave the course simulation running.

In Terminal 1:

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install_team9/setup.bash

team9_project=$(dirname "$(find ~/ros2_ws/src \
  -name team9_rviz.rviz -print -quit)")

ros2 launch path_planner_server_TEAM_9 \
  team9_sim_navigation.launch.py \
  rviz_config:="$team9_project/team9_rviz.rviz"
```

This starts the shared map server, localization and navigation
for both robots, plus RViz. It does not start Gazebo or the tracker.

## Set initial poses

Keep RViz's Fixed Frame set to map.

In Panels → Tool Properties:
1. Set 2D Pose Estimate's topic to /tb3_0/initialpose.
2. Set tb3_0's actual position and heading on the map.
3. Change the topic to /tb3_1/initialpose.
4. Set tb3_1's actual position and heading.

Initial pose estimates update localization; they do not move robots.
Confirm that each robot's laser scan aligns with the map.

The saved RViz configuration initially displays /tb3_0/scan.
Add another LaserScan display for /tb3_1/scan and select Best Effort
reliability to inspect the second robot.

## Test navigation

Wait for both navigation lifecycle managers to report:
Managed nodes are active.

Set 2D Goal Pose's topic to /tb3_0/goal_pose.
Choose a nearby clear destination away from obstacles.

Confirm:
- The robot moves in Gazebo.
- The laser scan remains aligned with the map.
- Nav2 reports Goal succeeded.

Repeat using /tb3_1/goal_pose for the opponent.

Do not run the tracker while testing manual navigation goals.

## Test pursuit

After both robots are localized and navigation works,
open Terminal 2:

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install_team9/setup.bash

ros2 launch tracker_TEAM_9 tracker_sim.launch.py
```

The tracker sends navigation goals to tb3_0 using tb3_1's TF pose.
Do not send manual navigation goals to tb3_0 while tracking.

Begin with a stationary opponent. Then send navigation goals
to tb3_1 and check whether tb3_0 updates its pursuit.

## Capture reporting: unfinished integration

The intended simulation camera topic is:
/tb3_0/camera/image_raw/compressed

A real simulated camera must publish this topic before image-based
capture reporting can be tested. An empty or fabricated image is
not valid capture evidence.

The tracker currently checks capture geometry and image freshness.
It does not verify opponent visibility or obstacle occlusion.

The current five-second interval limits local report publication.
Cooldown following master-server hit confirmation remains to be
implemented once the server interface is supplied.

Simulation IDs 1 and 2 are test values. Confirm competition IDs,
report topic and message package with the teaching team.

## Evidence to record

Record navigation successes, moving-opponent pursuit and capture
report validation separately. Save screenshots or recordings and
note failures and fixes. Do not claim untested features as working.