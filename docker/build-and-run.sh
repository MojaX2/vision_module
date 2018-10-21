IMAGE_NAME="moja2/vision_module"

xhost +

docker build -t $IMAGE_NAME .

docker run ${RUN_ARGS} \
       -it --rm --name vision \
       --net="host" \
       -e ROS_MASTER_URI=http://192.168.1.88:11311 \
       -e ROS_IP=192.168.1.33 \
       -e ROS_HOSTNAME="192.168.1.33" \
       --add-host="octopus:192.168.1.33" \
       --add-host="baxter.local:192.168.1.88" \
       -e DISPLAY=$DISPLAY \
       -v /tmp/.X11-unix:/tmp/.X11-unix \
       -e LIBGL_ALWAYS_SOFTWARE=1 \
       -e HOME=/workspace \
       -v `pwd`/..:/catkin_ws/src/vision_module \
       -v `pwd`/../../vision_module_wrapper:/catkin_ws/src/vision_module_wrapper \
       moja2/vision_module bash -c "ln /dev/null /dev/raw1394; source /catkin_ws/devel/setup.bash; roslaunch vision_module  vision.launch"