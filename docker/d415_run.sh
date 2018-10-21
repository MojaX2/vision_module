docker run --rm --net=host \
        --privileged \
        --volume=/dev:/dev \
        --net="host" \
        -e ROS_MASTER_URI=http://192.168.1.88:11311 \
        -e ROS_HOSTNAME="192.168.1.33" \
        --add-host="octopus:192.168.1.33" \
        --add-host="baxter.local:192.168.1.88" \
        -it --rm iory/docker-ros-d415:docker.d415 /bin/bash \
        -i -c 'roslaunch realsense2_camera rs_rgbd.launch enable_pointcloud:=true align_depth:=false depth_registered_processing:=true align_depth:=true'