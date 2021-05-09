rm -rf /home/mbsrobot/catkin_ws/src/mbsrobot_project/mbsrobot/maps/house.pgm 
rm -rf /home/mbsrobot/catkin_ws/src/mbsrobot_project/mbsrobot/maps/house.yaml

rosservice call /write_state /home/mbsrobot/catkin_ws/src/mbsrobot_project/mbsrobot/maps/map.bag.pbstream

rosrun cartographer_ros cartographer_pbstream_to_ros_map -pbstream_filename=/home/mbsrobot/catkin_ws/src/mbsrobot_project/mbsrobot/maps/map.bag.pbstream


cp  /home/mbsrobot/catkin_ws/src/mbsrobot_project/mbsrobot/maps/map.pgm /home/mbsrobot/catkin_ws/src/mbsrobot_project/mbsrobot/maps/house.pgm

cp  /home/mbsrobot/catkin_ws/src/mbsrobot_project/mbsrobot/maps/map.yaml /home/mbsrobot/catkin_ws/src/mbsrobot_project/mbsrobot/maps/house.yaml

