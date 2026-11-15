# CEFIRO Docker — local starter (no commits, no push)

This is a **starter configuration** prepared against the public `Nidhood/PROJECT-CEFIRO` repository as observed on 2026-11-15. It has been statically checked, **not tested by running Docker/Gazebo on your hardware**. You may need to adjust packages, OpenGL, and CycloneDDS configuration on your machine.

## Layout and guarantees

Place this entire `docker/` folder at the root of your existing local CEFIRO checkout, next to `cefiro_gz/`, `cefiro_gui/`, and the other ROS packages. Do not put it in your home directory separately. `compose.yaml` uses paths relative to this folder.

- `sim`: ROS 2 Jazzy + Gazebo Harmonic + RViz2 + Qt6 GUI. Browser access at `http://localhost:6080/vnc.html?autoconnect=1`. Software-rendered by default to be portable.
- `observer`: separate networked ROS 2 CLI/rosbag service, with read-only source and install mounts. It uses the same CycloneDDS discovery configuration and Docker bridge network.
- `../` (source) is a **bind mount**. Edit files on the host and see changes in the container immediately. Rebuild code after C++ changes. `colcon --symlink-install` helps reflect installed non-compiled assets.
- `../.docker-data/{build,install,log,records}` are **bind mounts**, preserved on the host after containers are stopped/removed.
- No real MCU, USB serial ports, host network, elevated privileges, or motor authority are passed to the containers.
- Browser VNC has **no password**: port 6080 is intentionally bound to `127.0.0.1` only; never publish this service to a LAN or the Internet.

## Windows 11

1. Install/enable WSL 2 + an Ubuntu distribution, Docker Desktop with WSL 2 backend and WSL integration.
2. Open an **Ubuntu WSL terminal**, not PowerShell, and put the repository in the Linux filesystem (e.g. `~/cefiro_ws/src/PROJECT-CEFIRO`) for significantly better file I/O than `/mnt/c/...`.
3. Follow the commands below from `.../PROJECT-CEFIRO/docker`. Use the browser desktop for the portable first pass.
4. Windows NVIDIA GPUs can be exposed through WSL 2 for compute, but real hardware-accelerated OpenGL Gazebo/RViz windows need additional WSLg configuration; browser/Xvfb mode normally uses CPU rendering. Verify before assuming GPU acceleration.

## Ubuntu 24.04

Install Docker Engine + Compose plugin. In a terminal at your local repository root:

```bash
cd docker
cp .env.example .env
sed -i "s/^LOCAL_UID=.*/LOCAL_UID=$(id -u)/; s/^LOCAL_GID=.*/LOCAL_GID=$(id -g)/" .env
mkdir -p ../.docker-data/{build,install,log,records}
docker compose build
docker compose up -d
```

The same commands work in WSL Ubuntu. Browse to:

```text
http://localhost:6080/vnc.html?autoconnect=1
```

### Build the actual CEFIRO packages (first run)

The CEFIRO source is mounted as `/cefiro_ws/src/PROJECT-CEFIRO` and contains all ROS packages. First check for missing dependency keys:

```bash
docker compose exec sim cefiro-run ros2 pkg list
docker compose exec sim cefiro-run colcon list
```

All main CEFIRO dependencies are preinstalled in the Dockerfile. If an additional package dependency is reported by CMake/colcon, add it to the Dockerfile and rebuild the image. Avoid installing packages manually in a running container, since they vanish on recreation.

```bash
docker compose exec sim cefiro-run colcon build --symlink-install --parallel-workers 2
```

`--parallel-workers 2` is a conservative first build; increase it if RAM permits. The build and install directories persist under `.docker-data` in the host repository.

### Run the existing project launch files

Run Gazebo in one terminal:

```bash
docker compose exec sim cefiro-run ros2 launch cefiro_gz start_gz.launch.py use_sim_time:=true enable_sensors:=true
```

Open a second terminal and run the project's own Qt interface:

```bash
docker compose exec sim cefiro-run ros2 launch cefiro_gui cefiro_gui.launch.py
```

Optional RViz (third terminal):

```bash
docker compose exec sim cefiro-run ros2 launch cefiro_description display.launch.py
```

Note that `display.launch.py` is a standalone visualization launch: it may duplicate robot_state_publisher and TF nodes if run concurrently with `start_gz.launch.py`. To observe the existing simulation instead, use `docker compose exec sim cefiro-run rviz2` and configure the desired topics and frames.

### ROS networking and recording in a different container

```bash
docker compose exec observer cefiro-run ros2 topic list
docker compose exec observer cefiro-run ros2 topic hz /imu/data
docker compose exec observer cefiro-run ros2 bag record -o /cefiro_data/records/test_01 /imu/data /cefiro/control/status
```

Data appears at `.docker-data/records/test_01/` in the host. CLI/discovery between services uses `cefiro_ros` (bridge network) and explicit CycloneDDS peer names, plus multicast. ROS 2 nodes running **outside** these containers require additional DDS networking/discovery configuration; `network_mode: host` is not a portable substitute on Windows.

### Resources and performance

```bash
docker stats
docker compose exec sim cefiro-run glxinfo -B
docker compose exec sim cefiro-run bash -lc 'free -h; nproc'
```

Important: `glxinfo -B` in browser mode will usually report `llvmpipe` (CPU rasterization). This is a design tradeoff for ease of access, and Gazebo may run slowly. A passthrough GPU can be made available via Compose, but graphics acceleration still requires a working GLX/EGL display path.

## Optional: Ubuntu native X11/XWayland GUI (faster OpenGL route)

On the Ubuntu host, install `xauth` if needed, and prepare a per-display X11 cookie:

```bash
sudo apt install -y xauth
cd docker
bash scripts/prepare-xauth.sh
export XAUTHORITY_FILE=/tmp/cefiro-docker.xauth
docker compose -f compose.yaml -f compose.native-linux.yaml up -d --force-recreate
```

Native GUI launches Gazebo/RViz/Qt windows on the Ubuntu desktop rather than in a web browser. To check native graphics:

```bash
docker compose -f compose.yaml -f compose.native-linux.yaml exec sim cefiro-run glxinfo -B
```

For Intel/AMD direct rendering on Ubuntu, additionally expose `/dev/dri` with a `devices:` mapping if necessary, and ensure the container user can access the `render`/`video` group or pass matching supplemental groups. For NVIDIA on native Ubuntu, install/configure the NVIDIA Container Toolkit and driver, then start with the additional GPU overlay:

```bash
docker compose -f compose.yaml -f compose.native-linux.yaml -f compose.nvidia.yaml up -d --force-recreate
```

`gpus: all` exposes the GPU; it does **not guarantee** that OGRE/Gazebo or RViz will use hardware OpenGL. Confirm using `glxinfo -B` and `nvidia-smi`. The host's X11 authentication and the NVIDIA stack must be compatible.

To restore portable browser mode:

```bash
docker compose -f compose.yaml -f compose.native-linux.yaml down
docker compose up -d
```

## Safe cleanup

```bash
docker compose down              # Stop/remove containers and network. Host files remain.
docker compose up -d             # Start again.
```

Do **not** delete `.docker-data/` unless you intentionally want to discard build outputs, logs, and recorded experiments. Do not run `git add`/`git push` to use Docker: it only reads your local checkout. To keep your Git status clean, add `.docker-data/` (and optionally `docker/` for this local experiment) to your local `.git/info/exclude`; no commit required.

## Troubleshooting

- `Unable to locate package libgz-sim8-dev` / other `libgz-*`: the Dockerfile now adds the official OSRF Gazebo repository **before** installing dependencies. Rebuild using `docker compose build --progress=plain`. If errors persist, check access from your network to `https://packages.osrfoundation.org/gazebo/ubuntu-stable` and whether the repository key was downloaded successfully.
- `unable to locate package` (other dependencies): verify repository configuration and package names inside the container build.
- `could not connect to display`: use browser mode first, or inspect XAUTHORITY/DISPLAY for native Linux.
- `Failed to create swapchain`, `libGL`, `EGL`: run `glxinfo -B`, test CPU/software mode, and validate host/GPU capabilities.
- No ROS topics in `observer`: check `ROS_DOMAIN_ID`, CycloneDDS config, running ROS nodes, `docker compose logs` and network membership.
- Empty Gazebo scene or custom plugin not found: make sure `colcon build` completed and the workspace overlay is sourced by `cefiro-run`.
- Root-owned local files: verify `LOCAL_UID` and `LOCAL_GID` before the first `up` and create `.docker-data` as your Linux user.

## Optional: Windows 11 direct WSLg windows (alternative to browser)

This mode opens separate Gazebo / Qt / RViz Linux windows on the Windows desktop. The starter leaves software rendering on to avoid assuming that GPU-passthrough OpenGL works automatically. In an Ubuntu WSL 2 terminal:

```bash
ls -l /mnt/wslg/.X11-unix/X0
docker compose -f compose.yaml -f compose.wslg.yaml up -d --force-recreate
docker compose -f compose.yaml -f compose.wslg.yaml exec sim cefiro-run glxinfo -B
```

Then launch `cefiro_gz start_gz.launch.py` as usual, but prepend the two `-f` flags to your `docker compose` commands. Check WSLg direct GPU acceleration separately before changing `LIBGL_ALWAYS_SOFTWARE` to `0`. Microsoft's WSLg container guidance documents `/mnt/wslg`, `/dev/dxg`, and `/usr/lib/wsl` passthrough, but Docker Desktop may impose additional constraints, so this is not enabled in the portable default.

Restore browser GUI:

```bash
docker compose -f compose.yaml -f compose.wslg.yaml down
docker compose up -d
```


### Docker build: UID/GID 1000 already exists

The base Ubuntu 24.04 image may already define a user and group with numeric ID
1000. The Dockerfile handles this by reusing and renaming the existing account
to `cefiro`, instead of failing with `groupadd: GID '1000' already exists`.
The bind-mounted source files still retain host ownership by numeric UID/GID.
