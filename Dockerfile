# Development Docker Image

ARG ROS_DISTRO=noetic

FROM ros:$ROS_DISTRO AS base_dependencies
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

# install core utilities used in clang plugin
RUN apt-get update -qq \
    && apt-get install -q -y \
      clang \
      libclang-dev \
      cmake \
      libyaml-cpp-dev \
    && rm -rf /var/lib/apt/lists/*

# install general ROS packages for building and analysis
RUN apt-get update -qq \
    && apt-get install -q -y \
      python3-catkin-tools \
    && rm -rf /var/lib/apt/lists/*

# install test-specific dependencies
#  this is inefficient, but unless our test directory grows
#  too big it shouldn't be a problem
COPY test/ /tmp/test/
RUN apt-get update -qq \
    && rosdep update \
    && rosdep install -y --from-paths /tmp/test/ \
    && rm -rf /var/lib/apt/lists/*

# drop into expected workspace
WORKDIR /workspace
CMD byobu

