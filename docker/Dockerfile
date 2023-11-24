FROM ros:noetic-perception as limo-base

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
    ros-noetic-opencv-apps \
    ros-noetic-diagnostic-updater \
    ros-noetic-tf-conversions \
    ros-noetic-rviz \
    lcov \
    tmux \
    python3-pip \
    && apt clean \
    && rm -rf /var/lib/apt/lists/* \
    && rm -rf /tmp/*


RUN pip install --upgrade pip

RUN pip install catkin-tools \
    pykitti \
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

# WORKDIR /workspace/limo
# RUN catkin config --profile limo_release -x _limo_release --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DCMAKE_CXX_FLAGS=-Wall -Wextra -Wno-unused-parameter -Werror=address -Werror=array-bounds=1 -Werror=bool-compare -Werror=comment -Werror=enum-compare -Werror=format -Werror=init-self -Werror=logical-not-parentheses -Werror=maybe-uninitialized -Werror=memset-transposed-args -Werror=nonnull -Werror=nonnull-compare -Werror=openmp-simd -Werror=parentheses -Werror=return-type -Werror=sequence-point -Werror=sizeof-pointer-memaccess -Werror=switch -Werror=tautological-compare -Werror=trigraphs -Werror=uninitialized -Werror=volatile-register-var

# solve issue https://github.com/johannes-graeter/limo/issues/50
RUN cp -rf /usr/include/eigen3/Eigen /usr/include/Eigen -R
RUN ln -sf /usr/include/eigen3/Eigen /usr/include/Eigen
RUN ln -sf /usr/bin/python3 /usr/bin/python

WORKDIR /
# Ensure the debian frontent is unset again to allow for interactive
# installs inside the container
ENV DEBIAN_FRONTEND=

RUN useradd -u 1000 -U -m -s /bin/bash zeb


FROM limo-base as limo-dev

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
