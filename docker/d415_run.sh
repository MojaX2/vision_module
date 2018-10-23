HOST_IP=`hostname -I | cut -f1 -d' '`
MASTER_IP="192.168.1.88"

docker run --rm --net=host \
        --privileged \
        --volume=/dev:/dev \
        --net="host" \
        -e ROS_MASTER_URI=http://$MASTER_IP:11311 \
        -e ROS_HOSTNAME="$HOST_IP" \
        --add-host="octopus:$HOST_IP" \
        --add-host="baxter.local:$MASTER_IP" \
        -it --rm iory/docker-ros-d415:docker.d415 /bin/bash \
        -i -c 'roslaunch realsense2_camera rs_rgbd.launch enable_pointcloud:=true align_depth:=false depth_registered_processing:=true align_depth:=true'