catkin_create_pkg $1 std_msgs roscpp geometry_msgs tf dynamic_reconfigure sensor_msgs nav_msgs visualization_msgs  interactive_markers pluginlib rospy tf_conversions std_srvs actionlib_msgs move_base_msgs forwardx_nav_msgs forwardx_motor_msgs actionlib forwardx_nav_core forwardx_nav_utils forwardx_nav_adapter  forwardx_nav_grid xmlrpcpp forwardx_motor angles

touch ./$1/src/$1.cpp
touch ./$1/src/$1_node.cpp

touch ./$1/include/$1/$1.h

typeset -u name
name="__$1_h__"

echo "
#ifndef $name
#define $name

// http://docs.ros.org/api/rosconsole/html/dir_16d3284317caf78c0e25eb64bbba3d38.html
#include <ros/ros.h>
#include <ros/assert.h>

// http://docs.ros.org/jade/api/tf/html/c++/namespacetf.html
#include <tf/tf.h>

// msg
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

// msg
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

//msg
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

// msg
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


class $1_cls
{
public:
    $1_cls();
    ~$1_cls();

private:

protected:
};

#endif
" >> ./$1/include/$1/$1.h

echo "
#include <$1/$1.h>

$1_cls::$1_cls()
{

}

$1_cls::~$1_cls()
{

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
