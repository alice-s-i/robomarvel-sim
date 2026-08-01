# Образ для симуляции Robomarvel: ROS 2 Jazzy, терминал + Foxglove/Lichtblick.
# GUI внутри контейнера не используется.

FROM osrf/ros:jazzy-desktop-full

ENV DEBIAN_FRONTEND=noninteractive \
    ROS_DISTRO=jazzy \
    LANG=C.UTF-8 \
    LC_ALL=C.UTF-8

# apt-get upgrade обязателен и должен быть в одном слое с install: базовый образ -
# снимок на дату сборки, пакеты ниже приезжают свежими из packages.ros.org.
# Зависимости в ROS объявлены нестрого, поэтому ABI расходится молча и ноды падают
# с undefined symbol при первом обращении к сломанной библиотеке.
RUN apt-get update && apt-get upgrade -y && apt-get install -y --no-install-recommends \
    build-essential \
    git \
    vim \
    nano \
    tmux \
    sudo \
    wget \
    graphviz \
    liburdfdom-tools \
    python3-pip \
    python3-venv \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-numpy \
    python3-matplotlib \
    python3-opencv \
    portaudio19-dev \
    python3-pyaudio \
    alsa-utils \
    ros-${ROS_DISTRO}-foxglove-bridge \
    ros-${ROS_DISTRO}-teleop-twist-keyboard \
    ros-${ROS_DISTRO}-tf2-tools \
    ros-${ROS_DISTRO}-vision-msgs \
    # перечислением, а не маской ros-*-rqt*: маска затягивает rqt_graph, убранный из курса
    ros-${ROS_DISTRO}-rqt \
    ros-${ROS_DISTRO}-rqt-console \
    ros-${ROS_DISTRO}-rqt-plot \
    ros-${ROS_DISTRO}-rqt-topic \
    ros-${ROS_DISTRO}-rqt-publisher \
    ros-${ROS_DISTRO}-rqt-reconfigure \
    ros-${ROS_DISTRO}-rqt-tf-tree \
    ros-${ROS_DISTRO}-navigation2 \
    ros-${ROS_DISTRO}-nav2-bringup \
    # одометрия и SLAM: rtabmap вместо slam_toolbox + AMCL
    ros-${ROS_DISTRO}-rtabmap-odom \
    ros-${ROS_DISTRO}-rtabmap-slam \
    ros-${ROS_DISTRO}-rtabmap-util \
    ros-${ROS_DISTRO}-rtabmap-msgs \
    ros-${ROS_DISTRO}-rmw-cyclonedds-cpp \
    && rm -rf /var/lib/apt/lists/*

# На роботе используется CycloneDDS; включается раскомментированием.
# ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# --system-site-packages: иначе venv не видит apt-овые colcon, catkin_pkg, empy.
# colcon внутри venv: setuptools прописывает в console_scripts шебанг того интерпретатора,
# которым шла сборка. Системный python3 -> `ros2 run` не видит ultralytics, хотя
# `python3 node.py` видит.
RUN python3 -m venv --system-site-packages /opt/course_venv \
 && /opt/course_venv/bin/pip install --no-cache-dir --upgrade pip \
 && /opt/course_venv/bin/pip install --no-cache-dir \
      torch torchvision --index-url https://download.pytorch.org/whl/cpu \
 && /opt/course_venv/bin/pip install --no-cache-dir \
      colcon-common-extensions \
      ultralytics \
      filterpy \
      scipy \
      grpcio \
      grpcio-tools \
 && find /opt/course_venv -type d -name __pycache__ -prune -exec rm -rf {} + || true

ENV VIRTUAL_ENV=/opt/course_venv
ENV PATH="/opt/course_venv/bin:$PATH"

ARG USERNAME=student
ARG USER_UID=1000
ARG USER_GID=$USER_UID

RUN (groupadd --gid $USER_GID $USERNAME 2>/dev/null || groupmod -n $USERNAME $(getent group $USER_GID | cut -d: -f1)) \
    && (useradd --uid $USER_UID --gid $USER_GID -m $USERNAME 2>/dev/null || usermod -l $USERNAME -d /home/$USERNAME -m $(getent passwd $USER_UID | cut -d: -f1)) \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME

# Алиасы в /etc/profile.d, а не в ~/.bashrc - доступны любому пользователю.
# cb с --symlink-install: иначе правки launch-файлов, конфигов и миров не подхватываются,
# так как data_files копируются, а не линкуются.
RUN printf '%s\n' \
    "source /opt/ros/${ROS_DISTRO}/setup.bash" \
    "alias cb='cd ~/ros2_ws && colcon build --symlink-install'" \
    "alias cs='source ~/ros2_ws/install/setup.bash'" \
    "alias ws='cd ~/ros2_ws'" \
    > /etc/profile.d/ros_env.sh \
 && chmod 644 /etc/profile.d/ros_env.sh \
 && echo '. /etc/profile.d/ros_env.sh' >> /etc/bash.bashrc

RUN rosdep update --rosdistro=${ROS_DISTRO} || true

USER $USERNAME
WORKDIR /home/$USERNAME/ros2_ws

EXPOSE 8765

CMD ["/bin/bash"]
