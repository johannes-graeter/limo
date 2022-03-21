FROM ros:melodic-perception

# System settings
# max watches on files (commented out because it's not necessary right now)
# RUN echo "fs.inotify.max_user_watches = 524288" >> /etc/sysctl.conf && sysctl -p --system

# Install some basic utilities
RUN apt update && apt install --no-install-recommends -y \
    build-essential \
    cmake \
    libopencv-dev \
    libgoogle-glog-dev \
    libatlas-base-dev \
    libeigen3-dev \
    libsuitesparse-dev \
    libpng++-dev \
    git \
    libx11-6 \
    libgtk2.0-0 \
    vim \
    nano \
    wget \
    python-catkin-tools \
    ros-melodic-opencv-apps \
    ros-melodic-diagnostic-updater \
    ros-melodic-tf-conversions \
    lcov \
    tmux \
    python-pip \
    && apt clean \
    && rm -rf /var/lib/apt/lists/* \
    && rm -rf /tmp/*

RUN pip install pykitti \
    jupyter \
    jupytext --upgrade

# Install ceres solver
# Why is rm necessary? remove it when solved.
# WORKDIR /opt
# RUN rm -rf /opt/ceres-solver \
#     && git clone https://ceres-solver.googlesource.com/ceres-solver \
#     && mkdir -p /opt/ceres-solver/build
RUN apt update && apt install --no-install-recommends -y libceres-dev

# WORKDIR /opt/ceres-solver/build
# RUN cmake .. -DBUILD_TESTING=OFF -DBUILD_EXAMPLES=OFF \
#     && make \
#     && make install

# Make catkin workspace
RUN rm -rf /workspace \
    && mkdir -p /workspace/limo_ws/src \
    && /bin/bash -c 'source /opt/ros/melodic/setup.bash && catkin_init_workspace /workspace/limo_ws/src'

# WORKDIR /workspace/limo_ws
# RUN catkin config --profile limo_release -x _limo_release --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DCMAKE_CXX_FLAGS=-Wall -Wextra -Wno-unused-parameter -Werror=address -Werror=array-bounds=1 -Werror=bool-compare -Werror=comment -Werror=enum-compare -Werror=format -Werror=init-self -Werror=logical-not-parentheses -Werror=maybe-uninitialized -Werror=memset-transposed-args -Werror=nonnull -Werror=nonnull-compare -Werror=openmp-simd -Werror=parentheses -Werror=return-type -Werror=sequence-point -Werror=sizeof-pointer-memaccess -Werror=switch -Werror=tautological-compare -Werror=trigraphs -Werror=uninitialized -Werror=volatile-register-var

# Install limo
WORKDIR /workspace/limo_ws/src
RUN rm -rf /workspace/limo_ws/src/limo \
    && git clone https://github.com/johannes-graeter/limo.git \
    && rm -rf /workspace/limo_ws/src/feature_tracking \
    && git clone https://github.com/johannes-graeter/feature_tracking.git \
    && cd feature_tracking \
    && git checkout python_binding \
    && cd .. \
    && rm -rf /workspace/limo_ws/src/mono_lidar_depth \
    && git clone https://github.com/johannes-graeter/mono_lidar_depth.git \
    && rm -rf /workspace/limo_ws/src/viso2.git \
    && git clone https://github.com/johannes-graeter/viso2.git \
    && rm -rf /workspace/limo_ws/src/mrt_cmake_modules \
    && git clone https://github.com/johannes-graeter/mrt_cmake_modules.git \
    && rm -rf /workspace/limo_ws/src/rosinterface_handler \
    && git clone https://github.com/johannes-graeter/rosinterface_handler.git

# solve issue https://github.com/johannes-graeter/limo/issues/50
RUN cp -rf /usr/include/eigen3/Eigen /usr/include/Eigen -R
RUN ln -sf /usr/include/eigen3/Eigen /usr/include/Eigen

WORKDIR /workspace/limo_ws/
RUN /bin/bash -c 'source /opt/ros/melodic/setup.bash && catkin build'

WORKDIR /workspace/limo_ws/src/feature_tracking/feature_tracking_core/notebook
RUN jupytext feature_tracking_notebook.py --to notebook

COPY entry.sh /entry.sh
RUN chmod +x /entry.sh

ENTRYPOINT ["/entry.sh"]
# CMD ["bash"]
CMD /bin/bash -c 'source /opt/ros/melodic/setup.bash && source /workspace/limo_ws/devel/setup.bash && jupyter notebook --port=8888 --no-browser --ip=0.0.0.0 --allow-root /workspace/limo_ws/src/feature_tracking/feature_tracking_core/notebook/feature_tracking_notebook.ipynb'
