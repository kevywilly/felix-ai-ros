FROM ultralytics/ultralytics:latest-jetson-jetpack6 AS stage1

# ---- Tier 1: Environment (free, never invalidates anything below) ----
ENV BLINKA_FORCECHIP=T234 \
    BLINKA_FORCEBOARD=JETSON_ORIN_NANO \
    JETSON_MODEL_NAME=JETSON_ORIN_NANO \
    CLAUDE_CONFIG_DIR="/.claude" \
    PIP_INDEX_URL=https://pypi.org/simple \
    LANG=en_US.UTF-8 \
    LC_ALL=en_US.UTF-8 \
    RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
    ROS_DOMAIN_ID=42 \
    ROS_DISTRO=humble

# ---- Tier 2: Locale (rarely changes) ----
RUN apt-get update && apt-get install -y --no-install-recommends locales \
 && locale-gen en_US en_US.UTF-8 \
 && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 \
 && rm -rf /var/lib/apt/lists/*

 # Prevent interactive prompts during apt installs (tzdata, keyboard-configuration, etc.)
ENV DEBIAN_FRONTEND=noninteractive
ENV TZ=America/New_York
RUN apt-get update && apt-get install -y --no-install-recommends tzdata \
 && ln -fs /usr/share/zoneinfo/$TZ /etc/localtime \
 && dpkg-reconfigure -f noninteractive tzdata \
 && rm -rf /var/lib/apt/lists/*

# ---- Tier 3: System apt packages (changes when you add a system dep) ----
RUN apt-get update && apt-get install -y --no-install-recommends \
      vim i2c-tools \
      libzmq3-dev libzmq5 pkg-config \
      gstreamer1.0-nice gstreamer1.0-plugins-bad gstreamer1.0-plugins-ugly \
      libsrtp2-dev libavdevice-dev libavfilter-dev \
      libopus-dev libvpx-dev \
      avahi-utils libnss-mdns \
      curl ca-certificates gnupg lsb-release software-properties-common \
    && rm -rf /var/lib/apt/lists/*

# ---- Tier 4: ROS apt repo setup (changes ~never) ----
RUN add-apt-repository universe \
 && curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
      -o /usr/share/keyrings/ros-archive-keyring.gpg \
 && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu jammy main" \
      > /etc/apt/sources.list.d/ros2.list

# ---- Tier 5: ROS core (changes when you add a ROS package) ----
RUN apt-get update && apt-get install -y --no-install-recommends \
      ros-humble-ros-base \
      ros-humble-cv-bridge \
      ros-humble-image-transport \
      ros-humble-vision-msgs \
      ros-humble-tf2-ros ros-humble-tf2-tools \
      ros-humble-rmw-cyclonedds-cpp \
      python3-colcon-common-extensions \
      python3-rosdep python3-vcstool \
 && rosdep init \
 && rm -rf /var/lib/apt/lists/*

# ---- Tier 6: ROS robotics packages (more likely to churn than core) ----
RUN apt-get update && apt-get install -y --no-install-recommends \
      ros-humble-slam-toolbox \
      ros-humble-rplidar-ros \
      ros-humble-nav2-bringup \
      ros-humble-robot-localization \
      ros-humble-teleop-twist-keyboard \
      ros-humble-xacro \
      ros-humble-robot-state-publisher \
      ros-humble-foxglove-bridge \
      ros-humble-message-filters \
      ros-humble-tf2-geometry-msgs \
      ros-humble-camera-calibration \
      ros-humble-compressed-image-transport \
 && rm -rf /var/lib/apt/lists/*

# ---- Tier 7: Hardware Python deps (rare changes) ----
RUN pip3 install --no-cache-dir \
      pyserial smbus2 Jetson.GPIO \
      adafruit-blinka adafruit-circuitpython-vl53l0x \
      pyrplidar

# ---- Tier 8: Web/teleop Python deps (occasional changes) ----
RUN pip3 install --no-cache-dir --ignore-installed blinker \
 && pip3 install --no-cache-dir \
      flask flask-cors flask-sock \
      nicegui click pandas asyncio-atexit \
      aiortc aiohttp av

# ---- Tier 9: Dev/test Python deps (changes during dev) ----
# setuptools<80: ROS 2 Humble's `colcon build --symlink-install` breaks on
# setuptools>=80 ("option --editable not recognized"). Keep this pin. See BUILD_NOTES.md.
RUN pip3 install --no-cache-dir "setuptools<80" \
 && pip3 install --no-cache-dir \
      pytest pytest-asyncio \
      transforms3d pyyaml

# ---- Tier 10: cv2 GStreamer fix + assertion ----
# Must run AFTER the ROS install so cv-bridge's apt opencv is in place.
RUN pip3 uninstall -y opencv-python opencv-python-headless 2>/dev/null || true \
 && python3 -c "import cv2, sys; \
print('cv2:', cv2.__file__); \
sys.exit(0 if any(l.strip().startswith('GStreamer') and 'YES' in l for l in cv2.getBuildInformation().splitlines()) else 'cv2 lacks GStreamer support')"

# ---- Tier 11: App scripts (changes often during dev — keep last) ----
COPY ./scripts/entrypoint-ros.sh /entrypoint.sh
COPY ./scripts/felix.sh /
COPY ./scripts/train.sh /

# ---- stage2: Node + Claude Code ----
FROM stage1 AS stage2
RUN curl -fsSL https://deb.nodesource.com/gpgkey/nodesource-repo.gpg.key \
      | gpg --dearmor -o /usr/share/keyrings/nodesource.gpg \
 && echo "deb [signed-by=/usr/share/keyrings/nodesource.gpg] https://deb.nodesource.com/node_20.x nodistro main" \
      > /etc/apt/sources.list.d/nodesource.list \
 && apt-get update && apt-get install -y --no-install-recommends nodejs \
 && apt-get clean && rm -rf /var/lib/apt/lists/*
RUN npm install -g @anthropic-ai/claude-code
RUN echo "source /entrypoint.sh" >> /root/.bashrc