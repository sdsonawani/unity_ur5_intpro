#define NUM_CONTROLLERS 3
#define NUM_DRIVE_CONTROLLERS 2

#define PI 3.14159
#define CUTOFF_FREQ 25

#define DELTAT(_nowtime,_thentime) ((_thentime>_nowtime)?((0xffffffff-_thentime)+_nowtime):(_nowtime-_thentime))
#define NORMALIZE(_z) atan2(sin(_z), cos(_z))


// Define following to enable cmdvel debug output
#define _CMDVEL_DEBUG

// Define following to enable odom debug output
//#define _ODOM_DEBUG

// Define following to publish additional sensor information
#define _ODOM_SENSORS
#define _AUX_DATA
#define _TUNING_DATA


#include <ros/ros.h>
#include <ros/console.h>
#include <serial/serial.h>
#include <array>
#include <signal.h>
#include <string>
#include <sstream>
#include <cmath>

#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>

#include <tf/tf.h>
#include <geometry_msgs/Quaternion.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float32.h>
#include <roboteq_diff_msgs/ControlData.h>
#include <roboteq_diff_msgs/AuxData.h>
#include <roboteq_diff_msgs/TuningData.h>
#ifdef _ODOM_COVAR_SERVER
#include "roboteq_diff_msgs/OdometryCovariances.h"
#include "roboteq_diff_msgs/RequestOdometryCovariances.h"
#endif

#include <roboteq_diff_msgs/RequestBrushSpeed.h>
#include <roboteq_diff_msgs/RequestBrushActivate.h>
#include <roboteq_diff_msgs/EMO.h>
#include <roboteq_diff_msgs/RequestChargeActivate.h>
#include <roboteq_diff_msgs/ToggleClosedLoop.h>

