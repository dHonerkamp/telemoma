FROM ros:noetic-robot

RUN apt-get update && apt-get install -q -y --no-install-recommends \
    dirmngr \
    gnupg2 \
    psmisc \
    vim \
    wget \
    curl \
    git \
    openssh-client \
    libglib2.0-0 \
    htop \
    less \
    && rm -rf /var/lib/apt/lists/*

# fzf
RUN git clone --depth 1 https://github.com/junegunn/fzf.git ~/.fzf
RUN ~/.fzf/install --all

# WORKDIR $HOME/catkin_ws_hsr

ENV DEBIAN_FRONTEND=noninteractive

# HSR: install couchdb for ros-noetic-tmc-desktop-full to work: https://docs.couchdb.org/en/latest/setup/single-node.html
RUN apt update && apt install -y curl apt-transport-https gnupg \
    && curl https://couchdb.apache.org/repo/keys.asc | gpg --dearmor | tee /usr/share/keyrings/couchdb-archive-keyring.gpg >/dev/null 2>&1 \
    && . /etc/os-release \
    && echo "deb [signed-by=/usr/share/keyrings/couchdb-archive-keyring.gpg] https://apache.jfrog.io/artifactory/couchdb-deb/ ${VERSION_CODENAME} main" \
    | tee /etc/apt/sources.list.d/couchdb.list >/dev/null \
    && apt-get update \
    && echo "couchdb couchdb/mode select standalone" | debconf-set-selections \
    && echo "couchdb couchdb/bindaddress select 0.0.0.0" | debconf-set-selections \
    && echo "couchdb couchdb/adminpass select ''" | debconf-set-selections \
    && echo "couchdb couchdb/adminpass_again select ''" | debconf-set-selections \
    && echo "couchdb couchdb/cookie select randomvalue" | debconf-set-selections \
    && apt-get install -y couchdb \
    && rm -rf /var/lib/apt/lists/*

# HSR KEYS: https://docs.hsr.io/manual_en/howto/pc_install.html?highlight=install
RUN sh -c 'echo "deb https://hsr-user:jD3k4G2e@packages.hsr.io/ros/ubuntu `lsb_release -cs` main" > /etc/apt/sources.list.d/tmc.list' \
    && sh -c 'echo "deb https://hsr-user:jD3k4G2e@packages.hsr.io/tmc/ubuntu `lsb_release -cs` multiverse main" >> /etc/apt/sources.list.d/tmc.list' \
    && sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list' \
    && wget https://hsr-user:jD3k4G2e@packages.hsr.io/tmc.key -O - | apt-key add - \
    && wget http://packages.osrfoundation.org/gazebo.key -O - | apt-key add - \
    && apt-get update \
    && apt-get install -y ros-noetic-tmc-desktop-full \
    && rm -rf /var/lib/apt/lists/*

RUN apt update && apt install -y rviz ros-noetic-rqt ros-noetic-rqt-common-plugins \
    && rm -rf /var/lib/apt/lists/*


#####################
# INSTALL CONDA
#####################
RUN wget --quiet https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh -O ~/miniconda.sh && \
    /bin/bash ~/miniconda.sh -b -p /opt/conda && \
    rm ~/miniconda.sh && \
    /opt/conda/bin/conda clean -tipy && \
    ln -s /opt/conda/etc/profile.d/conda.sh /etc/profile.d/conda.sh && \
    echo ". /opt/conda/etc/profile.d/conda.sh" >> ~/.bashrc

ENV PATH=/opt/conda/bin:$PATH \
    ENV_NAME=telemoma

RUN conda create --name ${ENV_NAME} python==3.9 -y \
    && conda clean -afy

COPY requirements.txt .
RUN conda run -n ${ENV_NAME} pip install -r requirements.txt
# RUN conda run -n ${ENV_NAME} pip install igibson
RUN conda run -n ${ENV_NAME} pip install pyrealsense2
# RUN conda run -n ${ENV_NAME} pip install -e .

RUN apt update && apt install -y ros-noetic-realsense2-camera apt-transport-https \
    && rm -rf /var/lib/apt/lists/*


# realsense viewer (https://github.com/IntelRealSense/librealsense/blob/master/doc/distribution_linux.md)
RUN mkdir -p /etc/apt/keyrings && curl -sSf https://librealsense.intel.com/Debian/librealsense.pgp | tee /etc/apt/keyrings/librealsense.pgp > /dev/null
RUN echo "deb [signed-by=/etc/apt/keyrings/librealsense.pgp] https://librealsense.intel.com/Debian/apt-repo `lsb_release -cs` main" | \
    tee /etc/apt/sources.list.d/librealsense.list
RUN apt update && apt install -y  librealsense2-dkms librealsense2-utils \
    && rm -rf /var/lib/apt/lists/*

# for real world
RUN git clone https://github.com/mjd3/tracikpy.git
RUN apt update && apt install -y libeigen3-dev liborocos-kdl-dev libkdl-parser-dev liburdfdom-dev libnlopt-dev libnlopt-cxx-dev swig \
    && rm -rf /var/lib/apt/lists/*
RUN cd tracikpy && conda run -n ${ENV_NAME} pip install -e .

# Set real hsr as master
ENV ROS_MASTER_URI=http://192.168.167.12:11311 \
    ROS_IP=10.8.105.174

RUN conda run -n ${ENV_NAME} pip install roslibpy rospkg catkin_pkg empy

CMD bash