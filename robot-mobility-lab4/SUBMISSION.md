# Lab 4: Pure Pursuit and Motion Planning

Fill in your YouTube video links below. Each part has a simulation video and a
hardware video. See the Deliverables section of the README for what each video
must show.

## Part A: Pure Pursuit

**Deliverable A1 (simulation):** Pure Pursuit completing a full lap in sim, with
both the full waypoint set and the currently tracked waypoint visualized.

[FILL ME IN](https://youtu.be/your-link-here)

**Deliverable A3 (team, hardware):** The real car following waypoints in the AI
Makerspace using the provided particle filter, shown alongside RViz with the map
and waypoint markers.

[FILL ME IN](https://youtu.be/your-link-here)

## Part B: Motion Planning (RRT)

**Deliverable B2 (simulation):** RRT running in sim with at least one obstacle,
showing obstacle avoidance and a visualization of the planned paths and goal
point.

[FILL ME IN](https://youtu.be/your-link-here)

**Deliverable B3 (group, hardware):** The car running RRT in the AIMS hallway
with at least one obstacle, demonstrating obstacle avoidance.

[FILL ME IN](https://youtu.be/your-link-here)

---

## Implementation notes

Both parts are written in **Python**, one file per node, for the simulator
(`/ego_racecar/odom`, `/scan`, `/drive`). All tuning values are constants at the
top of each file. The C++ skeletons are left as provided (they still compile).

### How to run

```bash
colcon build --packages-select pure_pursuit motion_planning && source install/setup.bash
ros2 launch pure_pursuit pure_pursuit_launch.py      # Part A (+ RViz)
ros2 launch motion_planning rrt_launch.py            # Part B (+ RViz), do not run Part A too
ros2 run pure_pursuit waypoint_logger.py             # record a lap -> ./waypoints.csv
```

Part B was tested on `motion_planning/maps/levine_rrt.yaml`: Levine with four
boxes, each attached to a wall and blocking the route, leaving about 0.9 m on
the other side. Set `map_path` in `f1tenth_gym_ros/config/sim.yaml` to this map
and rebuild `f1tenth_gym_ros` to use it.

### Waypoints (`pure_pursuit/waypoints/`)

All routes are closed loops. `levine.csv` follows the corridor centre lines of
Levine with 0.8 m rounded corners, one point every 0.1 m. `aims.csv` is the same
kind of loop around the three locker islands of the AIMS map.

### Part A: Pure Pursuit (`pure_pursuit/scripts/pure_pursuit_node.py`)

1. Closest waypoint, searched only in the next 50 waypoints after the previous
   closest one (so the car never jumps to another part of the track), then walk
   forward along the loop to the first waypoint at least `LOOKAHEAD` away.
2. Transform it into the car frame (rotation by -yaw).
3. Signed curvature `2y / d^2`, steering `atan(wheelbase * curvature)`,
   clipped to the steering limit.
4. Constant speed, lower when steering hard.

RViz: `/pure_pursuit/waypoints` (all points) and `/pure_pursuit/target`.
In sim it completes repeated laps of Levine at up to 3 m/s.

### Part B: RRT (`motion_planning/scripts/rrt_node.py`)

Every 0.1 s:
1. **Occupancy grid** in the car frame (5.5 m x 5 m, 5 cm cells): every LiDAR
   hit is an obstacle, grown by 0.22 m (half the car width + margin).
2. **Goal**: the waypoint 2.5 m ahead on the route (moved further along the route
   if it is inside an obstacle).
3. If the route up to the goal is free, the car follows the global waypoints.
   If not, RRT plans from the car to the goal. The path is straightened
   (shortcut through free straight lines), resampled every 5 cm and followed
   with Pure Pursuit, using a shorter lookahead and lower speed. If RRT finds no
   path, the car stops.

RViz: `/rrt/grid`, `/rrt/tree`, `/rrt/path`, `/rrt/goal`, `/rrt/target`,
`/rrt/waypoints`. In sim it laps `levine_rrt` and goes around all four boxes
without collision.

### Part C: RRT* (extra credit)

`USE_RRT_STAR = True` (switch it off for plain RRT). On top of RRT:

* **near**: all tree nodes within `NEAR_RADIUS` (0.6 m) of the new node.
* **choose parent**: the neighbour giving the lowest `cost(neighbour) + distance`
  with a collision-free edge.
* **rewire**: a neighbour is re-parented to the new node when that makes its path
  shorter and the edge is free.
* `cost(node)` walks up the parents and adds up the edge lengths, so it is always
  up to date after rewiring. Rewiring cannot create a cycle: an ancestor of the
  new node already has a lower cost, so going through the new node is never
  cheaper for it.
* RRT stops at the first path. RRT* keeps growing the tree for all `MAX_ITER`
  iterations, then returns the cheapest path to the goal.
