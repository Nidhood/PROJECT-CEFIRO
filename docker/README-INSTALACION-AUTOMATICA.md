# CEFIRO — Windows + Docker Desktop + ROS 2 Jazzy + Gazebo Harmonic + Qt6 + Xpra

These are **additive automation files** for the existing [PROJECT-CEFIRO](https://github.com/Nidhood/PROJECT-CEFIRO). Tested **statically** against the current GitHub source and its package manifests; Docker execution on your Windows PC must still be verified. Nothing is pushed/committed to GitHub by these files.

## Requirements

- Windows 10/11 x64, virtualization, Docker Desktop with Linux containers (WSL2 backend is fine; **no separately installed Ubuntu distro required**)
- Windows PowerShell **5.1** or later
- Xpra Windows client to see individual Qt6 windows; noVNC is also available
- For the Raspberry live telemetry: a working ROS 2 Jazzy hardware node and `zenoh-bridge-ros2dds` running on the Pi, reachable at `192.168.68.59:7447` (change the IP if necessary)

## Installing the files without losing a working setup

This ZIP contains `docker/...` paths. Extract it at the **repository root** (`PROJECT-CEFIRO`), not inside its `docker` directory.

Do not delete existing `docker/` or `.docker-data/`. Back up `docker/Dockerfile` before replacing it if you have customized it. The overlay **does not contain `docker/.env`**, so it preserves local passwords. `docker/Dockerfile` is extended with the explicitly listed package dependencies in `requirements.apt.txt`; `docker/scripts/start-xpra.sh` contains the already validated no-IBus/no-D-Bus startup fix. No other existing Compose or Zenoh files are replaced.

### First-time setup (PowerShell from `PROJECT-CEFIRO`)

```powershell
powershell -ExecutionPolicy Bypass -File .\docker\windows\install-prerequisites.ps1
# If Docker Desktop or the Windows Xpra client is missing, run:
powershell -ExecutionPolicy Bypass -File .\docker\windows\install-prerequisites.ps1 -InstallMissing

powershell -ExecutionPolicy Bypass -File .\docker\windows\configure.ps1 -RaspberryIp 192.168.68.59
powershell -ExecutionPolicy Bypass -File .\docker\windows\install-project.ps1
powershell -ExecutionPolicy Bypass -File .\docker\windows\start.ps1
```

`install-project.ps1` builds the base Ubuntu image before the Xpra image (which depends on it), then compiles all eight local ROS packages using `colcon`. It installs the ROS 2, Gazebo, controller, Eigen, Qt6, NumPy/SciPy and other Linux dependencies through the **Dockerfile**, not on Windows.

If PowerShell scripts are blocked by policy, the `powershell -ExecutionPolicy Bypass -File ...` form changes policy only for the invoked process; it does not change system-wide policy.

### Every subsequent use

```powershell
powershell -ExecutionPolicy Bypass -File .\docker\windows\start.ps1
powershell -ExecutionPolicy Bypass -File .\docker\windows\status.ps1 -Topics
powershell -ExecutionPolicy Bypass -File .\docker\windows\stop.ps1
```

- `start.ps1` starts `sim`, `observer`, `zenoh` and `xpra` then launches `cefiro_gui` on display `:100` (no automatic hardware node or motor commands). The Windows Xpra client starts automatically if its executable is discoverable; otherwise connect manually using **Seamless / TCP / 127.0.0.1 / port 14500 / Start & Attach** with your local `.env` password.
- `start.ps1 -NoPi` skips starting the Zenoh Windows bridge. `-SkipGui` skips Qt6 launch. `-SkipClient` doesn't open the client.
- `status.ps1 -Topics` checks ROS discovery as seen from the Xpra container.
- `stop.ps1` stops/removes containers and the Docker network but keeps images, compiled builds, experiment recordings, and the source repository.

### Full removal of the project's Docker resources

```powershell
# Containers and network only:
powershell -ExecutionPolicy Bypass -File .\docker\windows\remove.ps1

# Containers, network and the two CEFIRO application images:
powershell -ExecutionPolicy Bypass -File .\docker\windows\remove.ps1 -RemoveImages

# Also PERMANENTLY erase .docker-data/{build,install,log,records}
# (requires typing DELETE CEFIRO DATA):
powershell -ExecutionPolicy Bypass -File .\docker\windows\remove.ps1 -RemoveImages -PurgeData
```

`remove.ps1` **never** removes the repository, the user `.env`, Docker Desktop itself, Windows Xpra, unrelated Docker images, or the Raspberry's native installation. A data purge deletes experiment records; create your own backup first.

## Specific CEFIRO package dependencies

| ROS package | Key dependencies |
|---|---|
| `cefiro_msgs` | `rosidl_default_generators`, `std_msgs` |
| `cefiro_hardware_interface` | `rclcpp`, `sensor_msgs`, `geometry_msgs`, `std_msgs`, `std_srvs`, C++20, `cefiro_msgs` |
| `cefiro_control` | `controller_interface`, `hardware_interface`, `pluginlib`, `realtime_tools`, Eigen, NumPy/SciPy |
| `cefiro_gui` | Qt 6 Core / Gui / Widgets / Charts; `rclcpp`, `cefiro_msgs` |
| `cefiro_gz` | Gazebo Harmonic `gz-sim8`, `gz-plugin2`, `gz-transport13`, `gz-msgs10`, `ros_gz_*` |
| `cefiro_description` | Xacro, robot state publisher |
| `cefiro_actuators` | `rclcpp`, standard ROS messages |
| `cefiro_sensors` | ament CMake |

For a new Windows machine, `Dockerfile` and `Dockerfile.xpra` provide **all Linux dependencies inside the images**. The Windows host needs Docker Desktop and the Xpra client, not native Qt or Ubuntu.

## Raspberry Pi — optional native hardware install

If hardware should run natively on the Raspberry (recommended), copy `docker/raspberry/install-cefiro-pi.sh` to the Pi and execute `bash install-cefiro-pi.sh ~/cefiro_ws`. This installs/builds the **hardware interface dependencies only**, does not start it, and does not install Zenoh or ROS 2 themselves. On the Pi start your Zenoh bridge separately using its working configuration. The bridge Windows/Docker may only expose seven whitelisted telemetry topics; it is not an unrestricted ROS network.

**Physical safety:** `start.ps1` never opens a real ESC port. Avoid enabling command transmission automatically. The motor GUI and simulation can still publish commands *inside the DDS domain*, so keep your Pi bridge read-only unless you deliberately implement and verify a safety interlock.

## Useful diagnostics

```powershell
cd docker
docker compose --profile pi -f compose.yaml -f compose.xpra.yaml ps -a
docker compose --profile pi -f compose.yaml -f compose.xpra.yaml logs --tail=100 xpra
docker compose --profile pi -f compose.yaml -f compose.xpra.yaml logs --tail=100 zenoh
# Qt6 GUI stdout/stderr is saved under .docker-data/log/cefiro-gui-xpra.log
```

## Confidentiality / tracked `.env`

**Attention:** on the public repository examined, `docker/.env` is already tracked by Git. A new `.gitignore` rule does not remove the existing tracked file. Rotate any exposed password, and (after reviewing) run:

```powershell
git rm --cached docker/.env
```

Commit that deletion in your own workflow and audit Git history if the file ever contained a usable secret. The provided ZIP does not contain or modify your `.env` and these scripts never push changes.

## Notes

- Xpra improves the desktop experience; it does **not** magically GPU-accelerate Gazebo (the Docker images currently use Mesa software rendering).
- Do not run `docker compose down -v` or `docker system prune --volumes` if you need experimental data.
- If building takes place inside a OneDrive-synced source folder, file operations can be slower and symlinks may need additional permissions.
