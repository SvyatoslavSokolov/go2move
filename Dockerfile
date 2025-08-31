FROM nvcr.io/nvidia/isaac-lab:2.0.0
WORKDIR /workspace/isaaclab/_isaac_sim
COPY additional_requirements.txt /workspace/isaaclab/_isaac_sim
RUN ./python.sh -m pip install -r additional_requirements.txt

RUN apt update
RUN apt install software-properties-common -y
RUN add-apt-repository universe -y

RUN apt update && apt install curl wget -y
RUN curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/1.1.0/ros2-apt-source_1.1.0.jammy_all.deb" # If using Ubuntu derivates use $UBUNTU_CODENAME
RUN dpkg -i /tmp/ros2-apt-source.deb
RUN apt upgrade -y
RUN apt-get update && apt-get install -y --allow-downgrades libbrotli1=1.0.9-2build6
RUN apt install libfreetype6-dev -y
RUN apt install ros-humble-desktop -y

RUN source /opt/ros/humble/setup.bash
RUN apt-get install ros-humble-teleop-twist-keyboard -y

ENTRYPOINT ["/bin/bash"]