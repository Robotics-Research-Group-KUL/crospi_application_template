FROM ghcr.io/robotics-research-group-kul/crospi-betfsm-podman:ros2-lyrical

# Create a non-root user to avoid file permission issues
ARG USERNAME=developer
ARG USER_UID=1000
ARG USER_GID=$USER_UID

RUN groupadd --gid $USER_GID $USERNAME \
    && useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME

# RUN pip3 install setuptools==58.2.0 \
#     && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME\
#     && chmod 0440 /etc/sudoers.d/$USERNAME

# RUN pip3 install jsonref jsonschema colorama flask flask-cors requests

# Create a workspace
RUN mkdir -p /home/$USERNAME/devel_ws/src
WORKDIR /home/$USERNAME/devel_ws

USER $USERNAME