FROM ubuntu:24.04

ENV TZ=Asia/Tokyo
RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone
ENV DEBIAN_FRONTEND=noninteractive

USER root

RUN apt update && apt install locales && \
    locale-gen en_US en_US.UTF-8 && \
    update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 && \
    apt -y clean && \
    rm -rf /var/lib/apt/lists/*
ENV LANG=en_US.UTF-8

RUN apt update && \
    apt install -y git wget curl gnupg2 lsb-release build-essential cmake python3-pip

RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null && \
    sed -i -e 's/ubuntu .* main/ubuntu noble main/g' /etc/apt/sources.list.d/ros2.list

RUN apt update && \
    apt install -y ros-jazzy-desktop ros-jazzy-demo-nodes-cpp \
    python3-colcon-common-extensions python3-rosdep \
    libcpp-httplib-dev nlohmann-json3-dev qrencode python3-vcstool

RUN rosdep init && rosdep update

WORKDIR /root/ros2_ws
COPY . /root/ros2_ws/src/ros2_console_browser

RUN cd /root/ros2_ws && \
    vcs import src < src/ros2_console_browser/build_depends.repos

RUN cd /root/ros2_ws && \
    apt update && \
    rosdep install -y --from-paths src --ignore-src --rosdistro jazzy

RUN cd /root/ros2_ws && \
    . /opt/ros/jazzy/setup.sh && \
    colcon build

RUN echo '#!/bin/bash\n\
set -e\n\
source /opt/ros/jazzy/setup.bash\n\
source /root/ros2_ws/install/setup.bash\n\
exec "$@"' > /ros_entrypoint.sh && \
    chmod +x /ros_entrypoint.sh

RUN echo '#!/bin/bash\n\
source /ros_entrypoint.sh\n\
ros2 launch demo_nodes_cpp talker_listener_launch.py &\n\
sleep 5\n\
ros2 run ros2_console_rest_api_node rest_api_server_node' > /start_test.sh && \
    chmod +x /start_test.sh

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["/start_test.sh"]