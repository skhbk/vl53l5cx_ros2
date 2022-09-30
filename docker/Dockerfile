FROM ros:humble

RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >>~/.bashrc

RUN apt-get update && export DEBIAN_FRONTEND=noninteractive \
    && apt-get -y install --no-install-recommends \
    git \
    libgpiod-dev
