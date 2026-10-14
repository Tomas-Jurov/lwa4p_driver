# syntax=docker/dockerfile:1
# Build context: the repository root containing docker/ and src/, see README.md.
# The existing simulation uses Gazebo Classic plugins unavailable in Jazzy.
FROM ros:humble-ros-base-jammy

SHELL ["/bin/bash", "-o", "pipefail", "-c"]

ARG DEBIAN_FRONTEND=noninteractive
ARG USER_UID=1000
ARG USER_GID=1000

ENV RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
    ROS_DOMAIN_ID=42 \
    ROS_LOCALHOST_ONLY=1 \
    QT_X11_NO_MITSHM=1 \
    QT_QPA_PLATFORM=xcb \
    PYTHONUNBUFFERED=1

# Explicit runtime dependencies include controllers omitted by the upstream
# MoveIt manifest, plus an X server for the headless RGB-D camera.
RUN apt-get update && apt-get install -y --no-install-recommends \
        build-essential \
        can-utils \
        iproute2 \
        libgl1-mesa-dri \
        libglx-mesa0 \
        mesa-utils \
        python3-colcon-common-extensions \
        python3-numpy \
        python3-pip \
        python3-rosdep \
        ros-humble-gazebo-ros-pkgs \
        ros-humble-gazebo-ros2-control \
        ros-humble-gripper-controllers \
        ros-humble-joint-state-broadcaster \
        ros-humble-joint-trajectory-controller \
        ros-humble-forward-command-controller \
        ros-humble-moveit \
        ros-humble-rmw-fastrtps-cpp \
        ros-humble-ros2-control \
        ros-humble-rviz2 \
        ros-humble-sensor-msgs-py \
        ros-humble-xacro \
        xauth \
        xvfb \
    && rm -rf /var/lib/apt/lists/*

# Neither dependency is declared in the driver's package.xml/setup.py.
RUN python3 -m pip install --no-cache-dir canopen==2.3.0 python-can==4.6.1

WORKDIR /opt/lwa4p_ws
COPY src/ src/

# The generated MoveIt manifest also lists an optional MongoDB warehouse.
# It has no Humble binary here and is not used by real.launch.py/gazebo.launch.py.
RUN if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then rosdep init; fi \
    && rosdep update --rosdistro humble \
    && apt-get update \
    && rosdep install --from-paths src --ignore-src --rosdistro humble \
        --skip-keys warehouse_ros_mongo -y \
    && rm -rf /var/lib/apt/lists/* \
    && source /opt/ros/humble/setup.bash \
    && colcon build --symlink-install --executor sequential \
        --event-handlers console_direct+

COPY <<'EOF' /usr/local/bin/lwa4p-env.bash
source /opt/ros/humble/setup.bash
if [ -f /usr/share/gazebo-11/setup.sh ]; then
    source /usr/share/gazebo-11/setup.sh
fi
source /opt/lwa4p_ws/install/local_setup.bash
EOF

COPY --chmod=755 <<'EOF' /usr/local/bin/lwa4p-entrypoint
#!/bin/bash
set -e
source /usr/local/bin/lwa4p-env.bash
# Docker forwards stop signals to the launch process. Make launch explicitly
# signal its children instead of relying on terminal process-group delivery.
if [ "${1:-}" = "ros2" ] && [ "${2:-}" = "launch" ]; then
    shift 2
    set -- ros2 launch --noninteractive "$@"
fi
exec "$@"
EOF

# Match the host user's IDs for X11 access and writable bind-mounted logs.
RUN groupadd --gid "${USER_GID}" ros \
    && useradd --uid "${USER_UID}" --gid "${USER_GID}" --create-home --shell /bin/bash ros \
    && mkdir -p /home/ros/.ros /home/ros/.gazebo \
    && printf '\nsource /usr/local/bin/lwa4p-env.bash\n' >> /home/ros/.bashrc \
    && chown -R ros:ros /home/ros /opt/lwa4p_ws

USER ros

# Fail the build if imports, installed resources, or launch arguments break.
# These checks do not connect to CAN and do not start a simulator or robot.
RUN source /usr/local/bin/lwa4p-env.bash \
    && python3 -c 'import can, canopen; import lwa4p_driver.driver_node; import lwa4p_driver.gripper_node' \
    && ros2 launch lwa4p_gazebo gazebo.launch.py --show-args \
    && ros2 launch lwa4p_moveit_config real.launch.py --show-args

STOPSIGNAL SIGINT
ENTRYPOINT ["/usr/local/bin/lwa4p-entrypoint"]
CMD ["ros2", "launch", "lwa4p_gazebo", "gazebo.launch.py"]
