FROM osrf/ros:kinetic-desktop-full-xenial AS burger

# gazebo7のアップデート & 依存パッケージインストール
#
# dbusについて
#   ときどきコマンド実行時に/etc/machine-idがないと怒られることがあるので
#   dbus-uuidgenコマンドでそれを生成するのに使用する
#     免責事項：この怒られの原因は理解していないので、この対処法が正しいという保証はないです。
RUN echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" \
      > /etc/apt/sources.list.d/gazebo-stable.list \
    && apt-get update && apt-get install -y wget \
    && wget http://packages.osrfoundation.org/gazebo.key -O - | apt-key add - \
    && apt-get update \
    && apt-get upgrade -y \
    && apt-get install -y \
         python-pip \
         ros-kinetic-turtlebot3 \
         ros-kinetic-turtlebot3-msgs \
         ros-kinetic-turtlebot3-simulations \
         ros-kinetic-aruco-ros \
         gazebo7 \
         dbus \
    && rm -rf /var/lib/apt/lists/*

RUN pip install requests flask

COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]

ARG UID
ARG GID
RUN groupadd -g ${GID:?} burger && useradd -m -s /bin/bash -u ${UID:?} -g burger burger

RUN echo "burger ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

USER burger

ENV GAZEBO_MODEL_PATH=/home/burger/catkin_ws/src/burger_war/burger_war/models/
ENV TURTLEBOT3_MODEL=burger
ENV QT_X11_NO_MITSHM=1
ENV CATKIN_WS=/home/burger/catkin_ws
ENV BURGER_WAR_REPO=/home/burger/catkin_ws/src/burger_war

RUN echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc \
    && rosdep update \
    && mkdir /home/burger/.gazebo \
    && mkdir -p /home/burger/catkin_ws/src \
    && cd /home/burger/catkin_ws/src \
    && bash -i -c "catkin_init_workspace" \
    && cd /home/burger/catkin_ws \
    && bash -i -c "catkin_make" \
    && echo "source /home/burger/catkin_ws/devel/setup.bash" >> ~/.bashrc \
    && echo 'export PATH=$BURGER_WAR_REPO/docker-compose/bin:$PATH' >> ~/.bashrc

USER root
RUN apt-get update \
    && apt-get install -y \
      ros-kinetic-vision-opencv \
      python-opencv \
      libopencv-dev \
      ros-kinetic-cv-camera \
      ros-kinetic-dwa-local-planner \
    && rm -rf /var/lib/apt/lists/*
RUN pip install --upgrade pip && pip install opencv-python==4.2.0.32

USER burger
WORKDIR /home/burger
