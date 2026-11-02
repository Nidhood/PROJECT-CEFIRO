# CEFIRO — local, event-driven Gazebo startup

Start the full scene with:

```bash
ros2 launch cefiro_gz start_gz.launch.py
```

## Order and readiness

1. Start Gazebo Harmonic (`main_world`) and the existing `robot_description` publisher and ROS-Gazebo bridge.
2. The helper `scripts/wait_for_gazebo_ready.py` waits until the **local Gazebo transport** service `/world/main_world/create` is advertised.
3. Run `ros_gz_sim create -file` on the **local** file `models/desk/sdf/desk.sdf`. Wait for its process to exit successfully.
4. Then run `ros_gz_sim create -file` on the **local** file `models/lamp/sdf/lamp.sdf`. Wait for its process to exit successfully.
5. Then run `spawn_robot.launch.py` using the existing ROS `robot_description` topic.

Each transition is handled by ROS 2 `OnProcessExit`, with a zero return code required for model creation. No `TimerAction` or scene/entity-name parsing is used. The service checker uses a short retry interval solely for querying availability, and has a maximum-failure timeout. Neither value determines successful model-spawn ordering.

The previous version tried to parse `/world/main_world/scene/info` *after* the local create executables had already returned successfully. Depending on Gazebo CLI response formatting, that parser could miss the entities and block the robot spawn. The redundant scene-check step has been removed.

The word “service” refers to **Gazebo Transport's local process interface**, not an Internet server. Both scenery models load directly from package files; Gazebo Fuel and remote resources are not needed for them.

`spawn_models.launch.py` can still be used independently in an already-running `main_world`. By default it creates just desk and lamp:

```bash
ros2 launch cefiro_gz spawn_models.launch.py
```

To have it start the robot too, use `spawn_robot_after_models:=true` and ensure the robot-description publisher is running.

## Expected log sequence

```text
[CEFIRO] Local Gazebo create service is ready
[CEFIRO] Gazebo ready; spawning scenery...
[CEFIRO] Creating local desk model...
[CEFIRO] Desk acknowledged; creating local lamp...
[CEFIRO] Desk + lamp acknowledged; spawning robot...
```

The robot appears after Gazebo has acknowledged the creation commands, **not after its GUI has rendered all textures**; the latter is asynchronous and cannot be guaranteed by the process-exit event alone.

If the desk or lamp create process fails, the launch is stopped with a relevant error and CEFIRO does not spawn.
