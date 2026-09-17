# Collection validation

This record concerns offline collection checks. The course demonstration is recorded separately in [EXPERIMENTS.md](EXPERIMENTS.md).

## Checks

| Check | Result |
| --- | --- |
| Imported file integrity | 281 file sizes and SHA-256 hashes match the archive manifest |
| Python syntax | 22 files parsed successfully |
| ROS package XML | Four manifests parsed successfully |
| Waypoint input | Nine numeric rows, each containing at least x/y/z |
| Local documentation links | 35 file and directory targets verified |

Checks used Python 3.12 on Windows. Imported source bytes were preserved, including existing formatting. Original executable permission bits are recorded in the manifest and carried into Git. No source was modified to make a validation check pass.

## Runtime checks still required for reproduction

- ROS 2 `colcon build` and ROS-dependent tests in the matching Linux environment.
- C++ dependency declarations and Gazebo library compatibility in a clean workspace.
- Coordinate origin and axis agreement, TF frame names and landing-handler connection.
- A complete PX4 SITL/Gazebo replay using the course world and model assets.

The existing mission tests are ROS ament copyright/formatting/docstring checks. The PX4 ROS communication tests depend on ROS interfaces. They were preserved and were not represented as passing runtime tests on Windows.
