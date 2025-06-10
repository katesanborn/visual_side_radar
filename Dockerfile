FROM ros:noetic

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_WORKSPACE=/catkin_ws

# Install dependencies
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-rosdep \
    python3-catkin-tools \
    ros-noetic-geometry-msgs \
    && rm -rf /var/lib/apt/lists/*

# Initialize rosdep
RUN rosdep init || true
RUN rosdep update

# Copy source files
WORKDIR /
COPY catkin_ws /catkin_ws

# Make scripts executable
RUN chmod +x /catkin_ws/src/radar_display/scripts/*.py

# Build workspace
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && \
    cd /catkin_ws && \
    rosdep install --from-paths src --ignore-src -r -y && \
    catkin_make"

# Source environment on shell start
RUN echo 'source /opt/ros/noetic/setup.bash' >> ~/.bashrc && \
    echo 'source /catkin_ws/devel/setup.bash' >> ~/.bashrc

# Default command: launch the visualization
# CMD ["/bin/bash", "-c", "source /opt/ros/noetic/setup.bash && \
#                          source /catkin_ws/devel/setup.bash && \
#                          roslaunch radar_display radar_display.launch"]


COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["display:=number"]

