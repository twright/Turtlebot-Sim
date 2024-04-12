# ROS distribution to use
FROM thomasdwright/turtlebot4-dev:latest AS dev

ARG USERNAME=devuser
ARG UID=1000
ARG GID=${UID}

USER root
# RUN usermod -d /home/${USERNAME} -m -l $USERNAME twright
# RUN passwd -d $USERNAME
# Create new user and home directory
# RUN    echo ${USERNAME} ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/${USERNAME} \
# && chmod 0440 /etc/sudoers.d/${USERNAME}

# Set the ownership of the overlay workspace to the new user
RUN chown -R ${UID}:${GID} /ws

# RUN usermod -aG fuse,video $USERNAME

RUN apt update && apt install -y jupyter-notebook  python3-jupyter-server
RUN cp -r /usr/share/jupyter/kernels/python3 /usr/share/jupyter/kernels/python3ros

COPY kernel.json /usr/share/jupyter/kernels/python3ros/kernel.json

RUN echo "source /opt/entrypoint.sh" >> /root/.profile
USER ${USERNAME}
RUN echo "source /opt/entrypoint.sh" >> /home/${USERNAME}/.bashrc

