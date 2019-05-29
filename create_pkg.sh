catkin_create_pkg $1 std_msgs roscpp geometry_msgs tf dynamic_reconfigure sensor_msgs nav_msgs visualization_msgs  interactive_markers pluginlib rospy tf_conversions std_srvs actionlib_msgs move_base_msgs forwardx_nav_msgs forwardx_motor_msgs actionlib forwardx_nav_core forwardx_nav_utils forwardx_nav_adapter  forwardx_nav_grid xmlrpcpp forwardx_motor 

touch ./$1/src/$1.cpp
touch ./$1/src/$1_node.cpp

touch ./$1/include/$1/$1.h

typeset -u name
name="__$1_h__"
echo "#ifndef $name" >> ./$1/include/$1/$1.h
echo "#define $name" >> ./$1/include/$1/$1.h
echo "#endif" >> ./$1/include/$1/$1.h