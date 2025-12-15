# Wall Tracking & Localization (X100 UGV + Velodyne 3D LiDAR, ROS)

Bachelor thesis project (University of Sarajevo, Sept 2023): implementation of a wall-tracking and localization algorithm for the X100 mobile robot using a Velodyne 3D LiDAR sensor and ROS. The robot autonomously follows a wall while keeping a defined distance, then returns to its initial position and stops.

## What it does
- Uses LiDAR data to detect and track walls for autonomous navigation. 
- Core behavior:
  1) Find the closest wall/obstacle
  2) Move toward it
  3) Rotate so the wall is on the robot’s right side
  4) Follow the wall until returning to the start position, then stop
- Localization combines LiDAR-based estimation and odometry.

## Implementation notes
- Closest-obstacle detection is done by scanning through `sensor_msgs/LaserScan` ranges and selecting the minimum distance/angle.
- Localization support includes:
  - `laser_scan_matcher` (scan matching between consecutive LaserScan messages; can publish pose / tf transforms).
  - Odometry updates via `nav_msgs/Odometry`.
  - Motion commands published as `geometry_msgs/Twist`.
- The final “returned to start” detection was implemented using updated odometry position (robotX/robotY) with a time-gate to avoid stopping immediately at the beginning.

## Testing
- Verified in simulation (Gazebo + RViz) and additionally tested on a real X100 robot (wall following confirmed; full stop-on-return could not be fully verified due to time/space constraints).

## Tech stack
- ROS (ROS-based communication and packages)
- C++ (ROS node implementation)
- Robot: XMachines X100 UGV + Velodyne 3D LiDAR

## Repository structure
- `src/` – ROS node(s) (C++)
- `media/` – screenshots/images (Gazebo/RViz results)
- `docs/` – thesis PDF
