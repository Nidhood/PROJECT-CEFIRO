# Reproducible CEFIRO development environment (Ubuntu 24.04 / ROS 2 Jazzy).
# The source repository is bind-mounted at runtime, never copied into the image.
FROM osrf/ros:jazzy-desktop

ENV DEBIAN_FRONTEND=noninteractive \
    ROS_DISTRO=jazzy \
    LANG=C.UTF-8 \
    LC_ALL=C.UTF-8

# Install the official Gazebo Harmonic APT repository before requesting gz-sim8
# development libraries. The ROS image contains ROS apt sources, but not OSRF's.
# See https://gazebosim.org/docs/harmonic/install_ubuntu/
RUN apt-get update \
 && apt-get install -y --no-install-recommends ca-certificates curl gnupg \
 && mkdir -p /usr/share/keyrings \
 && curl -fsSL https://packages.osrfoundation.org/gazebo.gpg \
      -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
 && . /etc/os-release \
 && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] https://packages.osrfoundation.org/gazebo/ubuntu-stable ${VERSION_CODENAME} main" \
      > /etc/apt/sources.list.d/gazebo-stable.list

RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential cmake git curl nano htop procps \
    python3-colcon-common-extensions python3-rosdep python3-numpy \
    python3-scipy python3-yaml python3-pip \
    ros-jazzy-ros-gz ros-jazzy-ros2-control gz-harmonic \
    ros-jazzy-rmw-cyclonedds-cpp \
    ros-jazzy-xacro ros-jazzy-rviz2 ros-jazzy-joint-state-publisher-gui \
    ros-jazzy-rosidl-default-generators \
    ros-jazzy-realtime-tools \
    libgz-sim8-dev libgz-plugin2-dev \
    libgz-transport13-dev libgz-msgs10-dev \
    qt6-base-dev libqt6charts6-dev \
    mesa-utils libgl1-mesa-dri libegl1 \
    xvfb x11vnc novnc websockify openbox xterm x11-xserver-utils \
 && rm -rf /var/lib/apt/lists/*

# Separate layer so adding ping does not invalidate the expensive ROS/Gazebo install.
RUN apt-get update \
 && apt-get install -y --no-install-recommends iputils-ping \
 && rm -rf /var/lib/apt/lists/*

# Ubuntu 24.04 base images can already contain UID/GID 1000 (e.g. ubuntu).
# Reuse those numeric IDs by renaming the existing account/group instead of
# attempting to allocate duplicate IDs. This keeps bind-mount ownership aligned.
ARG DEV_UID=1000
ARG DEV_GID=1000
RUN set -eux; \
    existing_group="$(getent group "${DEV_GID}" | cut -d: -f1 || true)"; \
    if [ -n "$existing_group" ]; then \
        if [ "$existing_group" != cefiro ]; then groupmod -n cefiro "$existing_group"; fi; \
    else \
        groupadd --gid "${DEV_GID}" cefiro; \
    fi; \
    existing_user="$(getent passwd "${DEV_UID}" | cut -d: -f1 || true)"; \
    if [ -n "$existing_user" ]; then \
        if [ "$existing_user" != cefiro ]; then usermod -l cefiro "$existing_user"; fi; \
        usermod -g cefiro -d /home/cefiro -m -s /bin/bash cefiro; \
    else \
        useradd --uid "${DEV_UID}" --gid cefiro --shell /bin/bash --create-home cefiro; \
    fi; \
    mkdir -p /cefiro_ws/src /cefiro_ws/build /cefiro_ws/install /cefiro_ws/log; \
    mkdir -p /cefiro_data/records /cefiro_data/log; \
    chown -R cefiro:cefiro /cefiro_ws /cefiro_data

COPY scripts/entrypoint.sh /usr/local/bin/cefiro-entrypoint
COPY scripts/start-desktop.sh /usr/local/bin/cefiro-desktop
COPY scripts/cefiro-run.sh /usr/local/bin/cefiro-run
COPY config/cyclonedds.xml /etc/cefiro/cyclonedds.xml
RUN sed -i 's/\r$//' /usr/local/bin/cefiro-* \
 && chmod +x /usr/local/bin/cefiro-* \
 && if [ ! -e /etc/ros/rosdep/sources.list.d/20-default.list ]; then rosdep init; fi

USER cefiro
WORKDIR /cefiro_ws
ENV HOME=/home/cefiro \
    RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
    CYCLONEDDS_URI=file:///etc/cefiro/cyclonedds.xml

ENTRYPOINT ["/usr/local/bin/cefiro-entrypoint"]
CMD ["/usr/local/bin/cefiro-desktop"]
