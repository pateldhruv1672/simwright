---
name: ros2-graph-debugging
description: Find why a ROS 2 robot does not move, a topic is silent, or data never arrives - inspect the live graph, QoS, remaps, TF, sim time and controllers in order.
---

# ROS 2 detective work

Start from the symptom, then walk the chain publisher -> topic -> subscriber -> controller -> joint. Use `ros2_graph` once, then `ros2_trace <topic>`.

1. **Does the node exist?** Missing node: launch file did not start it, crashed (check its log), or namespace differs.
2. **Is the topic name what you think?** Look for near-duplicates (`/cmd_vel` vs `/diff_drive_controller/cmd_vel_unstamped` vs
   `/robot1/cmd_vel`). Fix remappings in the launch file, not by renaming in code.
3. **Types match?** `geometry_msgs/msg/Twist` vs `TwistStamped` is the classic ros2_control Jazzy break (use `use_stamped_vel`).
4. **QoS compatible?** BEST_EFFORT publisher + RELIABLE subscriber = silence. Sensor topics: subscribe with SensorDataQoS.
   TRANSIENT_LOCAL subscribers (maps, robot_description) need TRANSIENT_LOCAL publishers.
5. **Time**: in simulation every node needs `use_sim_time: true` and something must publish `/clock` (Isaac Sim ROS 2 clock graph / gz bridge).
6. **TF**: one publisher per transform; `map -> odom -> base_link -> sensors`. Two odom publishers (EKF + diff drive with `enable_odom_tf`) fight.
7. **Controllers**: `ros2 control list_controllers` must show them `active`; hardware interfaces claimed; joint names match URDF `<ros2_control>`.
8. **Networking across Windows/WSL/Docker**: same `ROS_DOMAIN_ID` and RMW everywhere; prefer WSL mirrored networking
   (`networkingMode=mirrored` in `%USERPROFILE%\.wslconfig`) so DDS discovery reaches Isaac Sim on Windows; Docker containers need `--network host`.

Report the root cause with evidence (the exact endpoint/QoS lines), fix it in the owning file, rebuild if needed, re-run `ros2_trace`.

## Isaac Sim as the data source
- Isaac Sim publishes ROS 2 through the `isaacsim.ros2.bridge` extension and OmniGraph nodes (clock, joint states, TF, cameras, lidar). Look up the node types with `isaac_docs` (kind=extensions / code) before building a graph.
- On Windows + WSL, ROS 2 in WSL reaches Isaac Sim on the host only with mirrored networking (or a matching ROS_DOMAIN_ID and a DDS profile); check `/clock` first, then `ros2_graph`.
