IMAGE_NAME="moja2/vision_module"

docker build -t $IMAGE_NAME .

docker run ${RUN_ARGS} \
       -it --rm --name vision \
       --net="host" \
       -e ROS_MASTER_URI=http://192.168.1.33:11311 \
       -e ROS_IP=127.0.0.1 \
       -e DISPLAY=$DISPLAY \
       -v $HOME/.Xauthority:/root/.Xauthority \
       -v /tmp/.X11-unix:/tmp/.X11-unix \
       -e QT_X11_NO_MITSHM=1 \
       -e LIBGL_ALWAYS_SOFTWARE=1 \
       -e HOME=/workspace \
       -v `pwd`/..:/catkin_ws/src/vision_module \
       moja2/vision_module bash
    #    moja2/vision_module bash -c "ln /dev/null /dev/raw1394; source /catkin_ws/devel/setup.bash; roslaunch vision_module  vision_module.launch"