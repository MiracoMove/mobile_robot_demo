FROM superkoh/mm-ros-devcontainer:humble-20230811

ENV DEBIAN_FRONTEND=noninteractive
ENV SHELL /bin/bash
SHELL ["/bin/bash", "-o", "pipefail", "-c"]

# Change apt sources.list (you can change to your sources.list if needed)
COPY sources.list /etc/apt/sources.list

# Change ros2 apt sources to ustc (you can change to your mirror if needed)
RUN sh -c '. /etc/lsb-release && echo "deb https://mirrors.ustc.edu.cn/ros2/ubuntu/ `lsb_release -cs` main" > /etc/apt/sources.list.d/ros2-latest.list'