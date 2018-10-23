# vision_module
FROM osrf/ros:indigo-desktop-full

# Installing dependencies for linux installation of opencv
RUN apt-get update && \
        apt-get install -y \
        build-essential \
        cmake \
        git \
        wget \
        unzip \
        yasm \
        pkg-config \
        libswscale-dev \
        libtbb2 \
        libtbb-dev \
        libjpeg-dev \
        libpng-dev \
        libtiff-dev \
        libjasper-dev \
        libavformat-dev \
        libpq-dev

# GUI (if you want to use GTK instead of Qt, replace 'qt5-default' with 'libgtkglext1-dev' and remove '-DWITH_QT=ON' option in CMake):

RUN wget https://github.com/opencv/opencv/archive/2.4.13.3.zip \
&& unzip 2.4.13.3.zip \
&& mkdir /opencv-2.4.13.3/cmake_binary \
&& cd /opencv-2.4.13.3/cmake_binary \
&& cmake -DWITH_QT=OFF \
        -DWITH_OPENGL=ON \
        -DFORCE_VTK=OFF \
        -DWITH_TBB=ON \
        -DWITH_GDAL=ON \
        -DWITH_XINE=ON \
        -DBUILD_EXAMPLES=OFF \
        -DENABLE_PRECOMPILED_HEADERS=OFF .. \
&& make install -j 16\
&& rm /2.4.13.3.zip \
&& rm -r /opencv-2.4.13.3

RUN ln /dev/null /dev/raw1394

RUN apt-get install -y terminator
RUN apt-get install -y python-pip python-dev
# RUN pip install --upgrade pip
RUN pip install https://storage.googleapis.com/tensorflow/linux/cpu/tensorflow-0.11.0-cp27-none-linux_x86_64.whl
# RUN pip install https://storage.googleapis.com/tensorflow/linux/gpu/tensorflow_gpu-0.11.0-cp27-none-linux_x86_64.whl

WORKDIR /catkin_ws/src
RUN mkdir -p /catkin_ws/src
RUN git clone https://github.com/MojaX2/vision_module.git
WORKDIR /catkin_ws/src/vision_module
RUN mkdir /b 
RUN mv object_* visualizer/ face_* darknet/ plane_detection/ image_capture /b

WORKDIR /catkin_ws
RUN /bin/bash -c "source /opt/ros/indigo/setup.bash; catkin_make"
RUN mv /b/* src/vision_module/
RUN touch /catkin_ws/src/CMakeLists.txt
RUN /bin/bash -c "source /opt/ros/indigo/setup.bash; catkin_make"

WORKDIR /catkin_ws/src
RUN git clone https://github.com/MojaX2/vision_module_wrapper.git
WORKDIR /catkin_ws
RUN /bin/bash -c "source /opt/ros/indigo/setup.bash; catkin_make"

RUN apt install libcanberra-gtk-module libcanberra-gtk3-module
CMD bash