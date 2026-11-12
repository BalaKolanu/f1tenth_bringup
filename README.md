# f1tenth_bringup

## Purpose

Launch and orchestration package for the current F1TENTH workspace. It owns
platform-specific launch files, stack-level configuration, mapping, raceline
processing, map storage, and joystick-driven map save/reset behavior.

Hardware drivers remain in f1tenth_system; this package selects and configures
them for a particular platform.

## Platform branch workflow

Keep the shared main branch usable as the common baseline. Create one branch
per platform:

~~~bash
cd /home/nvidia/f1tenth_ws/src/f1tenth_bringup
git fetch origin
git switch main
git pull --ff-only origin main
git switch -c platform1
~~~

Use a different branch name, such as platform2, for another vehicle or
hardware configuration.

All platform-specific launch files and configuration files belong here,
including:

- LiDAR and sensor selection
- SLAM and mapping parameters
- controller launch files
- joystick mappings
- topic remappings
- map and raceline paths
- vehicle-specific orchestration parameters

Keep reusable algorithm code in its owning package. Keep generic hardware
defaults in f1tenth_system; put platform overrides in this repository.

## Making a platform change

Edit only files under this repository, for example:

~~~text
launch/map.launch.py
launch/process_raceline.launch.py
config/slam_toolbox.yaml
maps/
manual_paths/
lanes/
paths/
~~~

Review the change:

~~~bash
git status
git diff
python3 -m py_compile launch/*.launch.py
~~~

Commit it to the platform branch:

~~~bash
git add launch/ config/ maps/ manual_paths/ lanes/ paths/
git commit -m "Configure mapping and raceline pipeline for Platform 1"
~~~

Push the branch:

~~~bash
git push -u origin platform1
~~~

Do not commit generated build, install, or log directories.

## Reproducing a platform from the root manifest

The root workspace manifest normally points to main. To reproduce a
platform-specific configuration, change only the f1tenth_bringup entry in
the root f1tenth_ws.repos file:

~~~yaml
src/f1tenth_bringup:
  type: git
  url: git@github.com:BalaKolanu/f1tenth_bringup.git
  version: platform1
~~~

Then import or update the workspace:

~~~bash
cd /home/nvidia/f1tenth_ws
vcs import < f1tenth_ws.repos
vcs pull src
~~~

Commit the manifest change in the root repository so the complete platform
configuration is reproducible.

## Mapping

Launch mapping with:

~~~bash
ros2 launch f1tenth_bringup map.launch.py
~~~

The launch starts the current vehicle bringup and the mapping session manager.
Button 3 saves the map and button 0 clears saved map files and restarts SLAM.

SLAM parameters are in:

~~~text
config/slam_toolbox.yaml
~~~

Maps are saved under:

~~~text
maps/
~~~

## Raceline processing

Launch the lane-bound raceline pipeline with:

~~~bash
ros2 launch f1tenth_bringup process_raceline.launch.py
~~~

The pipeline runs:

~~~text
generate_lanes -> process_raceline -> paths/raceline.csv
~~~

The launch accepts these overrides:

- map_directory
- manual_path_directory
- fallback_path_directory
- lane_directory
- output_directory
- archive_directory
- preview_png

## Theory and further reading

This package coordinates existing ROS 2 nodes and algorithms; it does not
implement the underlying SLAM, localization, or controller theory. See the
[ROS 2 launch documentation](https://docs.ros.org/en/humble/Tutorials/Intermediate/Launch/Launch-system.html),
[SLAM Toolbox](https://github.com/SteveMacenski/slam_toolbox), and
[F1TENTH documentation](https://f1tenth.github.io/website/learn/).
