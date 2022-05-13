# Development Docker Image

ARG ROS_DISTRO=noetic

FROM ros:$ROS_DISTRO AS base
LABEL maintainer="Daniel M. Sahu"
ARG ROS_DISTRO

# install basic utilities
RUN apt-get update -qq \
    && apt-get upgrade -y \
    && apt-get install -q -y \
      vim \
      byobu \
      gdb \
    && rm -rf /var/lib/apt/lists/*

# install clang and libclang stuff
RUN apt-get update -qq \
    && apt-get install -q -y \
      clang \
      libclang-dev \
      cmake \
    && rm -rf /var/lib/apt/lists/*

# drop into expected workspace
WORKDIR /workspace
CMD byobu

