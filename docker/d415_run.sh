docker run --rm --net=host \
            --privileged \
            --volume=/dev:/dev \
            --net="host" \
            -it iory/docker-ros-d415:docker.d415 /bin/bash \
            -i -c 'roslaunch realsense2_camera rs_rgbd.launch enable_pointcloud:=true align_depth:=false depth_registered_processing:=true align_depth:=true'