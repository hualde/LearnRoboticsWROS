FROM osrf/ros:humble-desktop-full

ENV DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-c"]

# --- Base utilities & ROS deps ---
RUN apt-get update && apt-get install -y --no-install-recommends \
    git \
    build-essential \
    cmake \
    python3-pip \
    nano \
    curl \
    wget \
    ros-dev-tools \
    python3-colcon-common-extensions \
    python3-vcstool \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-gazebo-ros \
    gazebo \
    libgazebo-dev \
    ros-humble-gazebo-ros2-control \
    ros-humble-moveit \
    ros-humble-moveit-ros-visualization \
    ros-humble-xacro \
    ros-humble-joint-state-publisher-gui \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-realtime-tools \
    ros-humble-control-toolbox \
    ros-humble-control-msgs \
    ros-humble-rmw-cyclonedds-cpp \
    mesa-utils \
    libgl1-mesa-dri \
    libxkbcommon0 \
    x11-apps \
    && rm -rf /var/lib/apt/lists/*

# --- Workspace layout ---
WORKDIR /ros2_ws

# --- Copiar código fuente ---
COPY . /ros2_ws/src/

# --- Instalar dependencias con rosdep ---
RUN rosdep update || true && \
    rosdep install --from-paths src --ignore-src -r -y --rosdistro humble || true

# --- Dependencias manuales (no en package.xml) ---
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-pymoveit2 \
    ros-humble-robotiq-description \
    ros-humble-tf-transformations \
    ros-humble-moveit-ros-perception \
    && rm -rf /var/lib/apt/lists/*

# --- Compilar workspace ---
RUN source /opt/ros/humble/setup.bash && \
    colcon build --symlink-install

# --- Entrypoint ---
COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

# --- Setup automático ---
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc && \
    echo "source /ros2_ws/install/setup.bash" >> ~/.bashrc

# --- Variables de entorno ---
ENV ROS_DOMAIN_ID=0
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ENV QT_X11_NO_MITSHM=1

ENTRYPOINT ["/entrypoint.sh"]
CMD ["/bin/bash"]