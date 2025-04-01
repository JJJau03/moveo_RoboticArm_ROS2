FROM osrf/ros:jazzy-desktop
ARG USERNAME=rosuser
ARG USER_UID=1000
ARG USER_GID=$USER_UID

# Delete existing user if present
RUN if id -u $USER_UID >/dev/null 2>&1; then userdel $(id -un $USER_UID); fi

# Create the user with sudo privileges
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && apt-get update \
    && apt-get install -y sudo \
    && echo "$USERNAME ALL=(root) NOPASSWD:ALL" > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME

# Install system dependencies (consolidated RUN commands)
RUN apt-get update && apt-get upgrade -y \
    && apt-get install -y --no-install-recommends \
    python3-pip \
    ros-jazzy-ros2-control \
    ros-jazzy-ros2-controllers \
    ros-jazzy-xacro \
    ros-jazzy-ros-gz-* \
    ros-jazzy-*-ros2-control \
    ros-jazzy-joint-state-publisher-gui \
    ros-jazzy-tf-transformations \
    ros-jazzy-moveit* \
    && rm -rf /var/lib/apt/lists/*

RUN apt-get update -y 

# Set up environment (generic paths)
RUN echo "source /opt/ros/jazzy/setup.bash" >> /home/$USERNAME/.bashrc \
    && echo "if [ -f /home/$USERNAME/workspace/install/setup.bash ]; then source /home/$USERNAME/workspace/install/setup.bash; fi" >> /home/$USERNAME/.bashrc \
    && echo "cd /home/$USERNAME/workspace" >> /home/$USERNAME/.bashrc

# Create workspace directory
RUN mkdir -p /home/$USERNAME/workspace \
    && chown -R $USERNAME:$USERNAME /home/$USERNAME

ENV SHELL /bin/bash
USER $USERNAME
WORKDIR /home/$USERNAME/workspace
CMD ["/bin/bash"]