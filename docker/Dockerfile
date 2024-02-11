FROM ros:noetic-perception as limo-base

# System settings
# max watches on files (commented out because it's not necessary right now)
# RUN echo "fs.inotify.max_user_watches = 524288" >> /etc/sysctl.conf && sysctl -p --system

# Install some basic utilities
RUN apt-get update && apt-get install --no-install-recommends -y \
    build-essential \
    cmake \
    libopencv-dev \
    libgoogle-glog-dev \
    libatlas-base-dev \
    libceres-dev \
    libeigen3-dev \
    libsuitesparse-dev \
    libpng++-dev \
    libpcl-dev\
    git \
    libx11-6 \
    libgtk2.0-0 \
    vim \
    nano \
    wget \
    ros-noetic-opencv-apps \
    ros-noetic-diagnostic-updater \
    ros-noetic-tf-conversions \
    ros-noetic-rviz \
    lcov \
    tmux \
    python3-pip \
    zsh \
    && apt-get clean

# RUN echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list && \
#     apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654 && \
#     apt-get update && \
#     apt-get install --no-install-recommends -y \
#     ros-noetic-ros-base \
#     ros-noetic-pcl-conversions \
#     ros-noetic-rosbag \
#     ros-noetic-sensor-msgs \
#     ros-noetic-std-msgs \
#     ros-noetic-velodyne-driver \
#     ros-noetic-imu-complementary-filter \
#     && apt-get clean


RUN pip install --upgrade pip

RUN pip install --default-timeout=100 catkin-tools \
    pykitti \
    jupyter \
    jupytext --upgrade
    
# WORKDIR /workspace/limo
# RUN catkin config --profile limo_release -x _limo_release --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DCMAKE_CXX_FLAGS=-Wall -Wextra -Wno-unused-parameter -Werror=address -Werror=array-bounds=1 -Werror=bool-compare -Werror=comment -Werror=enum-compare -Werror=format -Werror=init-self -Werror=logical-not-parentheses -Werror=maybe-uninitialized -Werror=memset-transposed-args -Werror=nonnull -Werror=nonnull-compare -Werror=openmp-simd -Werror=parentheses -Werror=return-type -Werror=sequence-point -Werror=sizeof-pointer-memaccess -Werror=switch -Werror=tautological-compare -Werror=trigraphs -Werror=uninitialized -Werror=volatile-register-var

# solve issue https://github.com/johannes-graeter/limo/issues/50
RUN cp -rf /usr/include/eigen3/Eigen /usr/include/Eigen -R
RUN ln -sf /usr/include/eigen3/Eigen /usr/include/Eigen
RUN ln -sf /usr/bin/python3 /usr/bin/python
RUN ln -sf /usr/local/lib/python3.8/dist-packages/numpy/core/include/numpy /usr/include/numpy

WORKDIR /
# Ensure the debian frontent is unset again to allow for interactive
# installs inside the container
ENV DEBIAN_FRONTEND=

RUN useradd -u 1000 -U -m -s /bin/bash zeb


FROM limo-base as limo-dev

SHELL ["/bin/bash", "-c"]

# Make catkin workspace
WORKDIR /workspace/limo/src
RUN /bin/bash -c 'source /opt/ros/noetic/setup.bash && catkin_init_workspace /workspace/limo/src'

USER root

WORKDIR /workspace/limo/
COPY entry.sh /opt/entry.sh
RUN chmod a+x /opt/entry.sh && chown zeb:zeb /opt/entry.sh

USER zeb

ENTRYPOINT ["/opt/entry.sh"]
# CMD ["catkin", "build"]
#
#WORKDIR /workspace/limo/src/feature_tracking/feature_tracking_core/notebook
#RUN jupytext feature_tracking_notebook.py --to notebook
#
#COPY entry.sh /entry.sh
#RUN chmod +x /entry.sh
#
#ENTRYPOINT ["/entry.sh"]
## CMD ["bash"]
#CMD /bin/bash -c 'source /opt/ros/noetic/setup.bash && source /workspace/limo/devel/setup.bash && jupyter notebook --port=8888 --no-browser --ip=0.0.0.0 --allow-root /workspace/limo/src/feature_tracking/feature_tracking_core/notebook/feature_tracking_notebook.ipynb'
