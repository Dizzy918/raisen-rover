# Minimal cuVSLAM image: NVIDIA Isaac ROS base + the visual_slam package only.
# Deliberately NOT poc-isaac-ros/docker/Dockerfile — that pulls in mavros,
# orbbec, PeopleNet, ESS, RAFT and Depth-Anything for the drone project. A
# ground rover doing stereo pose needs none of it.
FROM isaac-ros-base:3.2
RUN apt-get update \
 && apt-get install -y --no-install-recommends ros-humble-isaac-ros-visual-slam \
 && rm -rf /var/lib/apt/lists/*
