FROM ros:humble

ENV DEBIAN_FRONTEND=noninteractive
ENV SHELL /bin/bash
SHELL ["/bin/bash", "-o", "pipefail", "-c"]

# Support https apt sources
RUN apt-get update \
  && apt-get install -y apt-transport-https ca-certificates \
  && rm -rf /var/lib/apt/lists/* \
  && apt-get clean

# Change apt sources.list (you can change to your sources.list if needed)
COPY sources.list /etc/apt/sources.list

# Change ros2 apt sources to ustc (you can change to your mirror if needed)
RUN sh -c '. /etc/lsb-release && echo "deb https://mirrors.ustc.edu.cn/ros2/ubuntu/ `lsb_release -cs` main" > /etc/apt/sources.list.d/ros2-latest.list'

# Install humble for dev
# https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html
ARG RTI_NC_LICENSE_ACCEPTED=yes
RUN apt-get update \
  && apt-get install -y ros-humble-desktop ros-dev-tools ros-humble-rmw-cyclonedds-cpp \
  && rm -rf /var/lib/apt/lists/* \
  && apt-get clean

# Add dev user
ARG USERNAME=dev
ARG USER_UID=1000
ARG USER_GID=$USER_UID
RUN groupadd --gid $USER_GID $USERNAME \
  && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
  && apt-get update \
  && apt-get install -y sudo \
  && rm -rf /var/lib/apt/lists/* \
  && apt-get clean \
  && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
  && chmod 0440 /etc/sudoers.d/$USERNAME

# Add dev user to video group
RUN usermod -a -G video $USERNAME

# Add ssh server
# RUN apt-get update \
#   && apt-get install -y openssh-server \
#   && rm -rf /var/lib/apt/lists/* \
#   && apt-get clean \
#   && mkdir /var/run/sshd

# Create sshd_config
# RUN ( \
#   echo 'LogLevel DEBUG2'; \
#   echo 'PermitRootLogin yes'; \
#   echo 'PasswordAuthentication yes'; \
#   echo 'X11Forwarding yes'; \
#   echo 'Subsystem sftp /usr/lib/openssh/sftp-server'; \
#   ) > /etc/ssh/sshd_config_dev

# Add novnc support
# https://github.com/theasp/docker-novnc

# Install git, supervisor, VNC, & X11 packages
RUN apt-get update \
  && apt-get install -y \
      bash \
      fluxbox \
      git \
      net-tools \
      novnc \
      supervisor \
      x11vnc \
      xterm \
      xvfb \
  && rm -rf /var/lib/apt/lists/* \
  && apt-get clean

# Setup environment variables
ENV HOME=/home/dev \
    DEBIAN_FRONTEND=noninteractive \
    LANG=en_US.UTF-8 \
    LANGUAGE=en_US.UTF-8 \
    LC_ALL=C.UTF-8 \
    DISPLAY=:0.0 \
    DISPLAY_WIDTH=1920 \
    DISPLAY_HEIGHT=1080 \
    RUN_XTERM=yes \
    RUN_FLUXBOX=yes

# Install clangd for c++
RUN apt-get update \
  && apt-get install -y clangd \
  && rm -rf /var/lib/apt/lists/* \
  && apt-get clean
COPY clangd.yaml /home/dev/.config/clangd/config.yaml

# Install autopep8 for python
RUN apt-get update \
  && apt-get install -y python3-autopep8 \
  && rm -rf /var/lib/apt/lists/* \
  && apt-get clean

USER dev

# source humble to bashrc
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc \
  && echo 'export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp' >> ~/.bashrc

USER root

COPY supervisord.conf /app/supervisord.conf
COPY conf.d /app/conf.d

# entrypoint
COPY entrypoint.sh /app/entrypoint.sh
RUN chmod +x /app/entrypoint.sh
ENTRYPOINT ["/app/entrypoint.sh"]

# run ssh server
# EXPOSE 22
EXPOSE 8080
# CMD ["/usr/sbin/sshd", "-D", "-e", "-f", "/etc/ssh/sshd_config_dev"]
# CMD ["websockify"," --web=/usr/share/novnc/", " 8080", " localhost:5900"]
# CMD ["supervisord", "-c", "/app/supervisord.conf"]