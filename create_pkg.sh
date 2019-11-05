catkin_create_pkg $1 forwardx_global_planner_ros forwardx_costmap_2d forwardx_local_planner_ros forwardx_nav_msgs  forwardx_motor_msgs forwardx_nav_core forwardx_nav_utils forwardx_nav_adapter  forwardx_nav_grid forwardx_motor  std_msgs roscpp geometry_msgs tf dynamic_reconfigure sensor_msgs nav_msgs visualization_msgs  interactive_markers pluginlib rospy tf_conversions std_srvs actionlib_msgs move_base_msgs actionlib message_runtime message_generation  pcl_conversions xmlrpcpp  angles
# nav_core costmap_2d forwardx_global_planner_ros forwardx_costmap_2d forwardx_local_planner_ros forwardx_nav_msgs  forwardx_motor_msgs forwardx_nav_core forwardx_nav_utils forwardx_nav_adapter  forwardx_nav_grid forwardx_motor eigen

mkdir ./$1/launch
mkdir ./$1/src
mkdir ./$1/include/$1

touch ./$1/src/$1.cpp
touch ./$1/src/$1_node.cpp
touch ./$1/include/$1/$1.h
touch ./$1/launch/$1.launch

typeset -u name
name="__$1_h__"

echo "
#ifndef $name
#define $name

// http://docs.ros.org/diamondback/api/roscpp/html/classros_1_1NodeHandle.html
#include <ros/ros.h>

// http://docs.ros.org/api/rosconsole/html/dir_16d3284317caf78c0e25eb64bbba3d38.html
#include <ros/assert.h>

// http://docs.ros.org/jade/api/tf/html/c++/namespacetf.html
#include <tf/tf.h>

// geometry_msgs
// http://docs.ros.org/api/geometry_msgs/html/index-msg.html
#include <geometry_msgs/Accel.h>
#include <geometry_msgs/AccelStamped.h>
#include <geometry_msgs/AccelWithCovariance.h>
#include <geometry_msgs/AccelWithCovarianceStamped.h>
#include <geometry_msgs/Inertia.h>
#include <geometry_msgs/InertiaStamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Polygon.h>
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <geometry_msgs/Transform.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/TwistWithCovariance.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/Wrench.h>
#include <geometry_msgs/WrenchStamped.h>

// nav_msgs
// http://docs.ros.org/kinetic/api/nav_msgs/html/index-msg.html
#include <nav_msgs/Path.h>
#include <nav_msgs/GridCells.h>
#include <nav_msgs/MapMetaData.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
// srv
#include <nav_msgs/GetMap.h>
#include <nav_msgs/GetPlan.h>
#include <nav_msgs/SetMap.h>

// http://docs.ros.org/jade/api/angles/html/namespaceangles.html
#include <angles/angles.h>

// sensor_msgs
// http://docs.ros.org/jade/api/sensor_msgs/html/index-msg.html
#include <sensor_msgs/BatteryState.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/ChannelFloat32.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/FluidPressure.h>
#include <sensor_msgs/Illuminance.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/JoyFeedback.h>
#include <sensor_msgs/JoyFeedbackArray.h>
#include <sensor_msgs/LaserEcho.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/MagneticField.h>
#include <sensor_msgs/MultiDOFJointState.h>
#include <sensor_msgs/MultiEchoLaserScan.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/NavSatStatus.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointField.h>
#include <sensor_msgs/Range.h>
#include <sensor_msgs/RegionOfInterest.h>
#include <sensor_msgs/RelativeHumidity.h>
#include <sensor_msgs/Temperature.h>
#include <sensor_msgs/TimeReference.h>
//srv
#include <sensor_msgs/SetCameraInfo.h>

// visualization_msgs
// http://docs.ros.org/api/visualization_msgs/html/index-msg.html
#include <visualization_msgs/ImageMarker.h>
#include <visualization_msgs/InteractiveMarker.h>
#include <visualization_msgs/InteractiveMarkerControl.h>
#include <visualization_msgs/InteractiveMarkerFeedback.h>
#include <visualization_msgs/InteractiveMarkerInit.h>
#include <visualization_msgs/InteractiveMarkerPose.h>
#include <visualization_msgs/InteractiveMarkerUpdate.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/MenuEntry.h>

#include <dynamic_reconfigure/server.h>

// c++
// http://www.cplusplus.com/reference/
#include <algorithm>
#include <chrono>
#include <codecvt>
#include <complex>
#include <exception>
#include <functional>
#include <initializer_list>
#include <iterator>
#include <limits>
#include <locale>
#include <memory>
#include <new>
#include <numeric>
#include <random>
#include <ratio>
#include <regex>
#include <stdexcept>
#include <string>
#include <system_error>
#include <tuple>
#include <typeindex>
#include <typeinfo>
#include <type_traits>
#include <utility>
#include <valarray>
#include <atomic>
#include <condition_variable>
#include <future>
#include <mutex>
#include <thread>


// costmap2d
#include <forwardx_costmap_2d/forwardx_costmap_2d.h>
#include <forwardx_costmap_2d/costmap_2d_publisher.h>
#include <forwardx_costmap_2d/costmap_2d_ros.h>
#include <forwardx_costmap_2d/cost_values.h>
#include <forwardx_costmap_2d/costmap_math.h>

// forwardx_local_planner_ros
#include <forwardx_local_planner_ros/world_model.h>
#include <forwardx_local_planner_ros/point_grid.h>
#include <forwardx_local_planner_ros/costmap_model.h>
#include <forwardx_local_planner_ros/voxel_grid_model.h>
#include <forwardx_local_planner_ros/trajectory_planner.h>
#include <forwardx_local_planner_ros/map_grid_visualizer.h>
#include <forwardx_local_planner_ros/planar_laser_scan.h>
#include <forwardx_local_planner_ros/BaseLocalPlannerConfig.h>
#include <forwardx_local_planner_ros/odometry_helper_ros.h>
#include <forwardx_local_planner_ros/map_cell.h>
#include <forwardx_local_planner_ros/map_grid.h>
#include <forwardx_local_planner_ros/footprint_helper.h>
#include <forwardx_local_planner_ros/trajectory.h>
#include <forwardx_local_planner_ros/Position2DInt.h>
#include <forwardx_local_planner_ros/goal_functions.h>

// eigen
// #include \"Eigen/Dense\"

// forwardx_nav_core
#include <forwardx_nav_core/global_planner.h>
#include <forwardx_nav_core/common.h>
#include <forwardx_nav_core/costmap.h>
#include <forwardx_nav_core/exceptions.h>

// forwardx_msgs
#include <forwardx_nav_msgs/Path2D.h>
#include <forwardx_nav_msgs/Point2D.h>
#include <forwardx_nav_msgs/Pose2DStamped.h>

// pluginlib
#include <pluginlib/class_list_macros.h>

// adapter
#include <forwardx_nav_adapter/costmap_adapter.h>

// forwardx_global_planner_ros
#include <forwardx_global_planner_ros/orientation_filter.h>


class $1_cls
{
public:
    $1_cls();
    virtual ~$1_cls();

    double double_var;
    bool bool_var;
    std::string string_var;

private:
    geometry_msgs::Twist twist_topic_;

    ros::Subscriber topic_sub_;
    ros::Publisher topic_pub_;

    std::thread thread_name_;

    ros::Timer timer_;
    void thread_function_();
    void topic_callback_function_(const geometry_msgs::Twist::ConstPtr &msg);
    void timer_callback_function_(const ros::TimerEvent& event);

protected:
};

#endif
" >> ./$1/include/$1/$1.h

echo "
#include <$1/$1.h>

$1_cls::$1_cls()
{
    ros::NodeHandle nh_(\"~\");

    nh_.param(\"param_description\", double_var, 1.0);
    nh_.param(\"param_description\", bool_var, true);
    nh_.param(\"param_description\", string_var, std::string(\"xxx\"));

    topic_sub_ = nh_.subscribe<geometry_msgs::Twist>(\"/cmd_vel\", 1, boost::bind(&$1_cls::topic_callback_function_, this, _1) );
    topic_pub_ = nh_.advertise<geometry_msgs::Twist>(\"/cmd_vel_\", 1);

    thread_name_ = std::thread(boost::bind(&$1_cls::thread_function_, this));

    timer_ =  nh_.createTimer(1.0, &$1_cls::timer_callback_function_, this, false, false);  // one_shot=false(default), auto_start=false
    timer_.start();
}

$1_cls::~$1_cls()
{
    thread_name_.join();
}

void $1_cls::thread_function_()
{
    ros::Rate rate(10);

    while (ros::ok())
    {
        ROS_INFO(\"Thread function...\");
        topic_pub_.publish(twist_topic_);
        rate.sleep();
    }
}

void $1_cls::topic_callback_function_(const geometry_msgs::Twist::ConstPtr &msg)
{
    ROS_WARN(\"Received topic...\");
}

void $1_cls::timer_callback_function_(const ros::TimerEvent& event)
{
    ROS_ERROR(\"Timer callback...\");
}
"  >> ./$1/src/$1.cpp


echo "
#include <$1/$1.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, \"$1\");

    ros::NodeHandle nh(\"~\");

    $1_cls $1;

    ros::spin();

    return 0;
}
" >> ./$1/src/$1_node.cpp

echo "
<?xml version=\"1.0\"?>

<launch>
    <arg name=\"debug\" default=\"false\" />
    <arg if=\"$(arg debug)\" name=\"launch_prefix\" value=\"xterm -e gdb --args \" />
    <arg unless=\"$(arg debug)\" name=\"launch_prefix\" value=\"\" />

    <node pkg=\"$1\" type=\"$1_node\" respawn=\"false\" name=\"$1\" output=\"screen\" launch-prefix=\"$(arg launch_prefix)\">
        <rosparam>
        </rosparam>
    </node>
</launch>
" >> ./$1/launch/$1.launch