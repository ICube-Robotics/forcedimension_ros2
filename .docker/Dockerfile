ARG ROS_DISTRO="jazzy"
FROM ros:${ROS_DISTRO}

COPY . /ros_ws/src/forcedimension_ros2
RUN apt update && apt upgrade -y
RUN apt install -y usbutils libusb-1.0-0
RUN apt install -y python3-pip git tree clang-format-14 cppcheck
RUN cd /ros_ws/src && \
    vcs import . < forcedimension_ros2/forcedimension_ros2.repos && \
    cd .. && \
    . /opt/ros/${ROS_DISTRO}/setup.sh && \
    rosdep install --ignore-src --from-paths . -y -r && \
    colcon build --symlink-install
COPY .docker/ros_entrypoint.sh /ros_entrypoint.sh
RUN chmod 755 /ros_entrypoint.sh
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]

