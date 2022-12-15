//http://www.cs.columbia.edu/~allen/F17/NOTES/icckinematics.pdf



// Define following to enable motor test mode
//  Runs both motors in forward direction at 10% of configured maximum (rpms in close_loop mode, power in open_loop mode)
//  If configured correctly robot should translate forward in a straight line
//#define _CMDVEL_FORCE_RUN

//
// odom publisher
//

#include <roboteq_diff_driver/driver.h>

void mySigintHandler(int sig)
{
  ROS_INFO("Received SIGINT signal, shutting down...");
  ros::shutdown();
}


uint32_t millis()
{
	ros::WallTime walltime = ros::WallTime::now();
  //	return (uint32_t)((walltime._sec*1000 + walltime.nsec/1000000.0) + 0.5);
  //	return (uint32_t)(walltime.toNSec()/1000000.0+0.5);
	return (uint32_t)(walltime.toNSec()/1000000);
}

float lowPassFilterInductive(int32_t sensor_reading, int32_t last_filtered_reading, float dt)
{
  float alpha = (2*PI*dt*CUTOFF_FREQ) / (2*PI*dt*CUTOFF_FREQ+1);
  float res = alpha*sensor_reading + (1-alpha)*last_filtered_reading; 
  return(res);
}

bool digitalDebounce(bool new_reading, bool last_reading)
{
  bool flag = false;
  if (new_reading == last_reading && new_reading == true) flag = true;
  return(flag);
}

class MainNode
{
public:
  MainNode();

public:

  //
  // cmd_vel subscriber
  //
  void cmdvel_callback(const geometry_msgs::Twist& twist_msg);
  void cmdvel_setup();
  
  //
  // odom publisher
  //
  void odom_setup();
  void odom_stream();    
  void odom_loop();
  void odom_aux_run(); 
  void odom_publish();

  //
  // helper
  //
  enum class ControllerList {Brush, Front, Rear, END};

  //
  //main
  //
  int run();

protected:
  ros::NodeHandle nh;

  std::array<serial::Serial, NUM_CONTROLLERS> controllers;

  uint32_t start_time;
  uint32_t aux_timer;

  uint32_t last_receive_time;
  uint32_t receive_time;

  //
  // cmd_vel subscriber
  //
  ros::Subscriber cmdvel_sub;

  // odom publisher
  //
  // geometry_msgs::TransformStamped tf_msg;
  // tf::TransformBroadcaster odom_broadcaster;
  // nav_msgs::Odometry odom_msg;
  // ros::Publisher odom_pub;

  roboteq_diff_msgs::ControlData control_msg;
  ros::Publisher control_pub;
  #ifdef _AUX_DATA
  roboteq_diff_msgs::AuxData aux_msg;
  ros::Publisher aux_pub;
  #endif
  #ifdef _TUNING_DATA
  roboteq_diff_msgs::TuningData tuning_msg;
  ros::Publisher tuning_pub;
  #endif

  // buffer for reading encoder counts
  std::array<int, NUM_CONTROLLERS> odom_idx;
  std::array<std::array<char, 100>, NUM_CONTROLLERS> odom_buf;

  // toss out initial encoder readings
  int32_t odom_encoder_toss;

  int32_t odom_hall_lf;
  int32_t odom_hall_rf;
  int32_t odom_hall_lr;
  int32_t odom_hall_rr;
  int32_t odom_hall_bf;
  int32_t odom_hall_br;
  int32_t measured_speed_lf;
  int32_t measured_speed_lr;
  int32_t measured_speed_rf;
  int32_t measured_speed_rr;
  int32_t measured_speed_bf;
  int32_t measured_speed_br;
  int32_t track_error_lf;
  int32_t track_error_lr;
  int32_t track_error_rf;
  int32_t track_error_rr;
  int32_t track_error_bf;
  int32_t track_error_br;

  float odom_x;
  float odom_y;
  float odom_yaw;
  float odom_last_x;
  float odom_last_y;
  float odom_last_yaw;

  uint32_t odom_last_time;

  int32_t inductive_prox_left;
  int32_t inductive_prox_right;
  float inductive_prox_left_filtered;
  float inductive_prox_right_filtered;
  bool ir_aluminum_lf;
  bool ir_aluminum_rf;
  bool ir_aluminum_lr;
  bool ir_aluminum_rr;
  bool ir_retro_left;
  bool ir_retro_right;
  bool chrg_hall;
  float v_bat;

  int32_t last_inductive_prox_left;
  int32_t last_inductive_prox_right;
  bool last_ir_aluminum_lf;
  bool last_ir_aluminum_rf;
  bool last_ir_aluminum_lr;
  bool last_ir_aluminum_rr;
  bool last_ir_retro_left;
  bool last_ir_retro_right;
  bool last_chrg_hall;
  

  #ifdef _AUX_DATA
  std::array<float, NUM_CONTROLLERS> voltage;
  std::array<float, NUM_CONTROLLERS> mot_current_ch1;
  std::array<float, NUM_CONTROLLERS> mot_current_ch2;
  std::array<float, NUM_CONTROLLERS> bat_current_ch1;
  std::array<float, NUM_CONTROLLERS> bat_current_ch2;
  std::array<float, NUM_CONTROLLERS> temperature_mcu;
  std::array<uint32_t, NUM_CONTROLLERS> current_last_time;
  #endif

  // settings
  bool pub_odom_tf;
  std::string odom_frame;
  std::string base_frame;
  std::string cmd_vel_topic;
  std::string port0;
  std::string port1;
  std::string port2;
  int baud;
  bool open_loop;
  bool open_loop_brush;
  double wheel_circumference;
  double gear_ratio_drive;
  double gear_ratio_brush;
  double track_width;
  double wheelbase;
  int encoder_ppr;
  double encoder_cpr;
  double max_amps;
  int max_rpm;
  int max_rpm_open_loop;
  int accel_drive;
  int accel_brush;

  //brush
  bool emo_cb(roboteq_diff_msgs::EMO::Request& req, roboteq_diff_msgs::EMO::Response& res);
  bool brush_speed_cb(roboteq_diff_msgs::RequestBrushSpeed::Request& req, roboteq_diff_msgs::RequestBrushSpeed::Response& res);
  bool brush_activate_cb(roboteq_diff_msgs::RequestBrushActivate::Request& req, roboteq_diff_msgs::RequestBrushActivate::Response& res);
  bool charge_activate_cb(roboteq_diff_msgs::RequestChargeActivate::Request& req, roboteq_diff_msgs::RequestChargeActivate::Response& res);
  bool toggle_closed_loop_cb(roboteq_diff_msgs::ToggleClosedLoop::Request & req, roboteq_diff_msgs::ToggleClosedLoop::Response& res);
  void brush_loop();
  ros::ServiceServer emo_server;
  ros::ServiceServer brush_speed_server;
  ros::ServiceServer brush_activate_server;
  ros::ServiceServer charge_activate_server;
  ros::ServiceServer toggle_closed_loop_server;
  double brush_rpm;
  bool brush_active;

  //motor PID values
  int kp_lf;
  int ki_lf;
  int kd_lf;
  int kp_lr;
  int ki_lr;
  int kd_lr;
  int kp_rf;
  int ki_rf;
  int kd_rf;
  int kp_rr;
  int ki_rr;
  int kd_rr;
  int kp_bf;
  int ki_bf;
  int kd_bf;
  int kp_br;
  int ki_br;
  int kd_br;    

};

MainNode::MainNode() : 
  start_time(0),
  odom_idx{},
  odom_encoder_toss(5),
  odom_hall_lf(0),
  odom_hall_rf(0),
  odom_hall_lr(0),
  odom_hall_rr(0),
  odom_hall_bf(0),
  odom_hall_br(0),
  measured_speed_lf(0),
  measured_speed_lr(0),
  measured_speed_rf(0),
  measured_speed_rr(0),
  measured_speed_bf(0),
  measured_speed_br(0),
  track_error_lf(0),
  track_error_lr(0),
  track_error_rf(0),
  track_error_rr(0),
  track_error_bf(0),
  track_error_br(0),
  odom_x(0.0),
  odom_y(0.0),
  odom_yaw(0.0),
  odom_last_x(0.0),
  odom_last_y(0.0),
  odom_last_yaw(0.0),
  odom_last_time(0.0),
  inductive_prox_left(0),
  inductive_prox_right(0),
  inductive_prox_left_filtered(0.0),
  inductive_prox_right_filtered(0.0),
  ir_aluminum_lf(false),
  ir_aluminum_rf(false),
  ir_aluminum_lr(false),
  ir_aluminum_rr(false),
  ir_retro_left(false),
  ir_retro_right(false),
  chrg_hall(false),
  last_inductive_prox_left(0),
  last_inductive_prox_right(0),
  last_ir_aluminum_lf(false),
  last_ir_aluminum_rf(false),
  last_ir_aluminum_lr(false),
  last_ir_aluminum_rr(false),
  last_ir_retro_left(false),
  last_ir_retro_right(false),
  last_chrg_hall(false),
  #ifdef _AUX_DATA
  voltage{},
  mot_current_ch1{},
  mot_current_ch2{},
  bat_current_ch1{},
  bat_current_ch2{},
  temperature_mcu{},
  current_last_time{},
  #endif
  brush_active(false),
  max_rpm_open_loop(100),
  open_loop(false),
  v_bat(0.0)
{  
  // CBA Read local params (from launch file)
  ros::NodeHandle nhLocal("~");
  nhLocal.param("pub_odom_tf", pub_odom_tf, true);
  ROS_INFO_STREAM("pub_odom_tf: " << pub_odom_tf);
  nhLocal.param<std::string>("odom_frame", odom_frame, "odom");
  ROS_INFO_STREAM("odom_frame: " << odom_frame);
  nhLocal.param<std::string>("base_frame", base_frame, "base_link");
  ROS_INFO_STREAM("base_frame: " << base_frame);
  nhLocal.param<std::string>("cmd_vel_topic", cmd_vel_topic, "cmd_vel");
  ROS_INFO_STREAM("cmd_vel_topic: " << cmd_vel_topic);
  nhLocal.param<std::string>("port0", port0, "/dev/ttyACM1");
  ROS_INFO_STREAM("port0: " << port0);
  nhLocal.param<std::string>("port1", port1, "/dev/ttyACM2");
  ROS_INFO_STREAM("port1: " << port1);
  nhLocal.param<std::string>("port2", port2, "/dev/ttyACM0");
  ROS_INFO_STREAM("port2: " << port2);
  nhLocal.param("baud", baud, 115200);
  ROS_INFO_STREAM("baud: " << baud);
  nhLocal.param("open_loop_brush", open_loop_brush, false);
  ROS_INFO_STREAM("open_loop_brush: " << open_loop_brush);
  nhLocal.param("wheel_circumference", wheel_circumference, 0.3192);
  ROS_INFO_STREAM("wheel_circumference: " << wheel_circumference);
  nhLocal.param("gear_ratio_drive", gear_ratio_drive, 30.0);
  ROS_INFO_STREAM("gear_ratio_drive: " << gear_ratio_drive);
  nhLocal.param("gear_ratio_brush", gear_ratio_brush, 8.125);
  ROS_INFO_STREAM("gear_ratio_brush: " << gear_ratio_brush);
  nhLocal.param("track_width", track_width, 0.4318);
  ROS_INFO_STREAM("track_width: " << track_width);
  nhLocal.param("wheelbase", wheelbase, 0.4318);
  ROS_INFO_STREAM("wheelbase: " << wheelbase);
  nhLocal.param("encoder_cpr", encoder_cpr, 3600.0);
  ROS_INFO_STREAM("encoder_cpr: " << encoder_cpr);
  nhLocal.param("max_amps", max_amps, 8.0);
  ROS_INFO_STREAM("max_amps: " << max_amps);
  nhLocal.param("max_rpm", max_rpm, 2900);
  ROS_INFO_STREAM("max_rpm: " << max_rpm);
  nhLocal.param("brush_rpm", brush_rpm, 25.0);
  ROS_INFO_STREAM("brush_rpm: " << brush_rpm);
  nhLocal.param("accel_drive", accel_drive, 500);
  ROS_INFO_STREAM("accel_drive: " << accel_drive);
  nhLocal.param("accel_brush", accel_brush, 350);
  ROS_INFO_STREAM("accel_brush: " << accel_brush);
  nhLocal.param("kp_lf", kp_lf, 0);
  ROS_INFO_STREAM("kp_lf: " << kp_lf);
  nhLocal.param("ki_lf", ki_lf, 0);
  ROS_INFO_STREAM("ki_lf: " << ki_lf);
  nhLocal.param("kd_lf", kd_lf, 0);
  ROS_INFO_STREAM("kd_lf: " << kd_lf);
  nhLocal.param("kp_lr", kp_lr, 0);
  ROS_INFO_STREAM("kp_lr: " << kp_lr);
  nhLocal.param("ki_lr", ki_lr, 0);
  ROS_INFO_STREAM("ki_lr: " << ki_lr);
  nhLocal.param("kd_lr", kd_lr, 0);
  ROS_INFO_STREAM("kd_lr: " << kd_lr);
  nhLocal.param("kp_rf", kp_rf, 0);
  ROS_INFO_STREAM("kp_rf: " << kp_rf);
  nhLocal.param("ki_rf", ki_rf, 0);
  ROS_INFO_STREAM("ki_rf: " << ki_rf);
  nhLocal.param("kd_rf", kd_rf, 0);
  ROS_INFO_STREAM("kd_rf: " << kd_rf);
  nhLocal.param("kp_rr", kp_rr, 0);
  ROS_INFO_STREAM("kp_rr: " << kp_rr);
  nhLocal.param("ki_rr", ki_rr, 0);
  ROS_INFO_STREAM("ki_rr: " << ki_rr);
  nhLocal.param("kd_rr", kd_rr, 0);
  ROS_INFO_STREAM("kd_rr: " << kd_rr);
  nhLocal.param("kp_bf", kp_bf, 0);
  ROS_INFO_STREAM("kp_bf: " << kp_bf);
  nhLocal.param("ki_bf", ki_bf, 0);
  ROS_INFO_STREAM("ki_bf: " << ki_bf);
  nhLocal.param("kd_bf", kd_bf, 0);
  ROS_INFO_STREAM("kd_bf: " << kd_bf);
  nhLocal.param("kp_br", kp_br,0);
  ROS_INFO_STREAM("kp_br: " << kp_br);
  nhLocal.param("ki_br", ki_br, 0);
  ROS_INFO_STREAM("ki_br: " << ki_br);
  nhLocal.param("kd_br", kd_br, 0);
  ROS_INFO_STREAM("kd_br: " << kd_br);
}

//
// cmd_vel subscriber
//
void MainNode::cmdvel_callback( const geometry_msgs::Twist& twist_msg) 
{
   float left_speed = twist_msg.linear.x + track_width * twist_msg.angular.z / 2.0;
   float right_speed = twist_msg.linear.x - track_width * twist_msg.angular.z / 2.0;
  //ROS_INFO_STREAM("cmdvel speed right: " << right_speed << " left: " << left_speed);

  std::stringstream right_cmd;
  std::stringstream left_cmd;

  if (open_loop)
  {
    // motor power (scale 0-1000)
    int32_t right_power = right_speed / wheel_circumference * 60.0 / max_rpm_open_loop * 1000.0;
    int32_t left_power = left_speed / wheel_circumference * 60.0 / max_rpm_open_loop * 1000.0;
  #ifdef _CMDVEL_DEBUG
  ROS_DEBUG_STREAM("cmdvel power right: " << right_power << " left: " << left_power);
  #endif
    right_cmd << "!G 1 " << right_power << "\r";
    left_cmd << "!G 2 " << left_power << "\r";
  }
  else
  {
    // motor speed (rpm)
    int32_t right_rpm = right_speed / wheel_circumference * 60.0 * gear_ratio_drive;
    int32_t left_rpm = left_speed / wheel_circumference * 60.0 * gear_ratio_drive;
  #ifdef _CMDVEL_DEBUG
  ROS_DEBUG_STREAM("cmdvel rpm right: " << right_rpm << " left: " << left_rpm);
  #endif
    right_cmd << "!S 1 " << right_rpm << "\r";
    left_cmd << "!S 2 " << left_rpm << "\r";
  }
  for(int i=1; i<=NUM_DRIVE_CONTROLLERS; i++)
  {
    controllers.at(i).write(right_cmd.str());
    controllers.at(i).write(left_cmd.str());
    //controllers.at(i).flush();
  }
}

void MainNode::cmdvel_setup()
{
  ControllerList init_idx = ControllerList::Brush;
  for(int i=0; i < NUM_CONTROLLERS; i++)
  {
    // stop motors
    controllers.at(i).write("!G 1 0\r");
    controllers.at(i).write("!G 2 0\r");
    controllers.at(i).write("!S 1 0\r");
    controllers.at(i).write("!S 2 0\r");
    controllers.at(i).flush();

    // disable echo
    controllers.at(i).write("^ECHOF 1\r");
    controllers.at(i).flush();

    // enable watchdog timer (100 ms)
    //controllers.at(i).write("^RWD 1000\r");

    // set motor amps limit (A * 10)
    std::stringstream right_ampcmd;
    std::stringstream left_ampcmd;
    right_ampcmd << "^ALIM 1 " << (int)(max_amps * 10) << "\r";
    left_ampcmd << "^ALIM 2 " << (int)(max_amps * 10) << "\r";
    controllers.at(i).write(right_ampcmd.str());
    controllers.at(i).write(left_ampcmd.str());

    // set max speed (rpm) for relative speed commands
    std::stringstream right_rpmcmd;
    std::stringstream left_rpmcmd;
    right_rpmcmd << "^MXRPM 1 " << max_rpm << "\r";
    left_rpmcmd << "^MXRPM 2 " << max_rpm << "\r";
    controllers.at(i).write(right_rpmcmd.str());
    controllers.at(i).write(left_rpmcmd.str());

    // set encoder mode (18 for feedback on motor1, 34 for feedback on motor2)
    // controllers.at(i).write("^EMOD 1 18\r");
    // controllers.at(i).write("^EMOD 2 34\r");
    
    // set PID parameters (gain * 10)
    std::stringstream temp;
    switch (init_idx) 
    {      
      case ControllerList::Brush: 
      {
        // set max acceleration rate 
        // set motor operating mode (1 for closed-loop speed)
        if (open_loop_brush)
        {
          // open-loop speed mode
          controllers.at(i).write("^MMOD 1 0\r");
          controllers.at(i).write("^MMOD 2 0\r");
        }
        else
        {
          // closed-loop speed mode
        controllers.at(i).write("^MMOD 1 1\r");
        controllers.at(i).write("^MMOD 2 1\r");
        }
        temp.str("");
        temp << "^MAC 1 " << accel_brush << "\r";
        controllers.at(i).write(temp.str());
        temp.str("");
        temp << "^MAC 2 " << accel_brush << "\r";
        controllers.at(i).write(temp.str());
        temp.str("");
        temp << "^MDEC 1 " << accel_brush << "\r";
        controllers.at(i).write(temp.str());
        temp.str("");
        temp << "^MDEC 2 " << accel_brush << "\r";
        controllers.at(i).write(temp.str());
        // set PID parameters 
        temp.str("");
	      temp << "^KPG 1 " << kp_bf << "\r";
        controllers.at(i).write(temp.str());
        temp.str(""); 
        temp << "^KPG 2 " << kp_br << "\r";
        controllers.at(i).write(temp.str());
        temp.str(""); 
        temp << "^KIG 1 " << ki_bf << "\r";
        controllers.at(i).write(temp.str());
        temp.str(""); 
        temp << "^KIG 2 " << ki_br << "\r";
        controllers.at(i).write(temp.str());
        temp.str(""); 
        temp << "^KDG 1 " << kd_bf << "\r";
        controllers.at(i).write(temp.str());
        temp.str(""); 
        temp << "^KDG 2 " << kd_br << "\r";
        controllers.at(i).write(temp.str());
        controllers.at(i).flush();
        init_idx = ControllerList::Front;
        break;
      }
      case ControllerList::Front: 
      {
        controllers.at(i).write("^MMOD 1 0\r");
        controllers.at(i).write("^MMOD 2 0\r");
        // set max acceleration rate 
        temp.str("");
	      temp << "^MAC 1 " << accel_drive << "\r";
        controllers.at(i).write(temp.str());
        temp.str("");
        temp << "^MAC 2 " << accel_drive << "\r";
        controllers.at(i).write(temp.str());
        temp.str("");
        temp << "^MDEC 1 " << accel_drive << "\r";
        controllers.at(i).write(temp.str());
        temp.str("");
        temp << "^MDEC 2 " << accel_drive << "\r";
        controllers.at(i).write(temp.str());
        // set PID parameters
        temp.str(""); 
	      temp << "^KPG 1 " << kp_lf << "\r";
        controllers.at(i).write(temp.str());
        temp.str(""); 
        temp << "^KPG 2 " << kp_rf << "\r";
        controllers.at(i).write(temp.str());
	      temp.str(""); 
        temp << "^KIG 1 " << ki_lf << "\r";
        controllers.at(i).write(temp.str());
        temp.str(""); 
        temp << "^KIG 2 " << ki_rf << "\r";
        controllers.at(i).write(temp.str());
        temp.str(""); 
        temp << "^KDG 1 " << kd_lf << "\r";
        controllers.at(i).write(temp.str());
        temp.str(""); 
        temp << "^KDG 2 " << kd_rf << "\r";
        controllers.at(i).write(temp.str());
        controllers.at(i).flush();
        init_idx = ControllerList::Rear;
        break;
      }

      case ControllerList::Rear: 
      {
        controllers.at(i).write("^MMOD 1 0\r");
        controllers.at(i).write("^MMOD 2 0\r");
        // set max acceleration rate 
        temp.str("");
        temp << "^MAC 1 " << accel_drive << "\r";
        controllers.at(i).write(temp.str());
        temp.str("");
        temp << "^MAC 2 " << accel_drive << "\r";
        controllers.at(i).write(temp.str());
        temp.str("");
        temp << "^MDEC 1 " << accel_drive << "\r";
        controllers.at(i).write(temp.str());
        temp.str("");
        temp << "^MDEC 2 " << accel_drive << "\r";
        controllers.at(i).write(temp.str());
        // set PID parameters
        temp.str(""); 
        temp << "^KPG 1 " << kp_lr << "\r";
        controllers.at(i).write(temp.str());
        temp.str(""); 
        temp << "^KPG 2 " << kp_rr << "\r";
        controllers.at(i).write(temp.str());
        temp.str(""); 
        temp << "^KIG 1 " << ki_lr << "\r";
        controllers.at(i).write(temp.str());
        temp.str(""); 
        temp << "^KIG 2 " << ki_rr << "\r";
        controllers.at(i).write(temp.str());
        temp.str(""); 
        temp << "^KDG 1 " << kd_lr << "\r";
        controllers.at(i).write(temp.str());
        temp.str(""); 
        temp << "^KDG 2 " << kd_rr << "\r";
        controllers.at(i).write(temp.str());
        controllers.at(i).flush();
        init_idx = ControllerList::END;
        break;
      }

      default: {}
        // undefined behavior - bad
       
    }
  }


  ROS_INFO_STREAM("Subscribing to topic " << cmd_vel_topic);
  cmdvel_sub = nh.subscribe(cmd_vel_topic, 1000, &MainNode::cmdvel_callback, this);

  ROS_INFO("Advertising service on roboteq/emo_srv");
  emo_server = nh.advertiseService("roboteq/emo_srv", &MainNode::emo_cb, this);
  ROS_INFO("Advertising service on roboteq/emo_srv");
  brush_activate_server = nh.advertiseService("roboteq/brush_activate_srv", &MainNode::brush_activate_cb, this);
  ROS_INFO("Advertising service on roboteq/brush_speed_srv");
  brush_speed_server = nh.advertiseService("roboteq/brush_speed_srv", &MainNode::brush_speed_cb, this);
  ROS_INFO("Advertising service on roboteq/charge_activate_srv");
  charge_activate_server = nh.advertiseService("roboteq/charge_activate_srv", &MainNode::charge_activate_cb, this);
  ROS_INFO("Advertising service on roboteq/toggle_closed_loop_srv");
  toggle_closed_loop_server = nh.advertiseService("roboteq/toggle_closed_loop_srv", &MainNode::toggle_closed_loop_cb, this);
}


void MainNode::odom_setup()
{
  ROS_INFO("Publishing to topic roboteq/control_data");
  control_pub = nh.advertise<roboteq_diff_msgs::ControlData>("roboteq/control_data", 1000);

  #ifdef _AUX_DATA
  ROS_INFO("Publishing to topic roboteq/aux_data");
  aux_pub = nh.advertise<roboteq_diff_msgs::AuxData>("roboteq/aux_data", 1000);
  #endif

  #ifdef _TUNING_DATA
  ROS_INFO("Publishing to topic roboteq/tuning_data");
  tuning_pub = nh.advertise<roboteq_diff_msgs::TuningData>("roboteq/tuning_data", 1000);
  #endif

  // start encoder streaming
  odom_stream();
  odom_last_time = millis();
  #ifdef _AUX_DATA
  for(int i=0; i<current_last_time.size(); i++)
  {
    current_last_time.at(i) = millis();
  }
  #endif
  
}

void MainNode::odom_stream()
{
  ControllerList init_idx = ControllerList::Brush;
  for(int i=0; i < NUM_CONTROLLERS; i++) 
  { 
    
    switch (init_idx)
    {
      case ControllerList::Brush:
      {
        controllers.at(i).write("# C\r");
        controllers.at(i).write("/\"S=\",\" \"?AI 2_?AI 1_?DI 7_?DI 6_?DI 5_?DI 4_?DI 10_?DI 9_?DI 8_# 10\r");
        #ifdef _TUNING_DATA
        controllers.at(i).write("/\"C=\",\" \"?BS 1_?BS 2_?E 1_?E 2_# 10\r");
        #endif
        controllers.at(i).write("/\"A=\",\" \"?A 1_?A 2_?BA 1_?BA 2_?V 2_?T 1_# 10000\r");
        init_idx = ControllerList::Front;
        break;
      }
      case ControllerList::Front:
      case ControllerList::Rear:
      {
        controllers.at(i).write("# C\r");
        #ifdef _TUNING_DATA
        controllers.at(i).write("/\"C=\",\" \"?BCR 1_?BCR 2_?BS 1_?BS 2_?E 1_?E 2_# 10\r");
        #else
        controllers.at(i).write("/\"C=\",\" \"?BCR 1_?BCR 2_# 10\r");
        #endif
        controllers.at(i).write("/\"A=\",\" \"?A 1_?A 2_?BA 1_?BA 2_?V 2_?T 1_# 10000\r");
        init_idx = ControllerList::Rear;
        break;
      }
    } 
    controllers.at(i).flush();
  }
}

void MainNode::odom_loop()
{
  last_receive_time = receive_time;
  uint32_t nowtime =  receive_time = millis();


  // if we haven't received encoder counts in some time then restart streaming
  if( DELTAT(nowtime,odom_last_time) >= 1000 )
  {
    odom_stream();
    odom_last_time = nowtime;
  }

  for(int i=0; i<NUM_CONTROLLERS; i++)
  {
    // read sensor data stream from motor controller
    if (controllers.at(i).available())
    {
      char ch = 0;
      if ( controllers.at(i).read((uint8_t*)&ch, 1) == 0 ) return;
      if (ch == '\r')
      {
        odom_buf.at(i).at(odom_idx.at(i)) = 0;

        #ifdef _ODOM_DEBUG
        std::string temp(std::begin(odom_buf.at(i)), std::end(odom_buf.at(i)));
        ROS_DEBUG_STREAM( "line " << i << ": " << temp);
        #endif

        //C is control data
        if (odom_buf.at(i).at(0) == 'C' && odom_buf.at(i).at(1) == '=' )
        {
          if ( odom_encoder_toss > 0 )
          {
            --odom_encoder_toss;
            break;
          }
         
          char* indx;

          if (i == 0) //brush controller
          {
            #ifdef _TUNING_DATA
            measured_speed_bf = (int32_t)strtol(odom_buf.at(i).begin()+2, &indx, 10);
            measured_speed_br = (int32_t)strtol(indx, &indx, 10);
            track_error_bf = (int32_t)strtol(indx, &indx, 10);
            track_error_br = (int32_t)strtol(indx, &indx, 10);
            #endif
          }
          else if (i == 1) //front contoller
          {
            odom_hall_lf = (int32_t)strtol(odom_buf.at(i).begin()+2, &indx, 10);
            odom_hall_rf = (int32_t)strtol(indx, &indx, 10);
            #ifdef _TUNING_DATA
            measured_speed_lf = (int32_t)strtol(indx, &indx, 10);
            measured_speed_rf = (int32_t)strtol(indx, &indx, 10);
            track_error_lf = (int32_t)strtol(indx, &indx, 10);
            track_error_rf = (int32_t)strtol(indx, &indx, 10);
            #endif
          }
          else //back controller
          {
            odom_hall_lr = (int32_t)strtol(odom_buf.at(i).begin()+2, &indx, 10);
            odom_hall_rr = (int32_t)strtol(indx, &indx, 10);
            #ifdef _TUNING_DATA
            measured_speed_lr = (int32_t)strtol(indx, &indx, 10);
            measured_speed_rr = (int32_t)strtol(indx, &indx, 10);
            track_error_lr = (int32_t)strtol(indx, &indx, 10);
            track_error_rr = (int32_t)strtol(indx, &indx, 10);
            #endif
          }   
          odom_idx.at(i) = 0;
        }
        #ifdef _AUX_DATA
        // A is auxillary data
        else if ( odom_buf.at(i).at(0) == 'A' && odom_buf.at(i).at(1) == '=' )
        {
          char* indx;
          mot_current_ch1.at(i) = (int32_t)strtol(odom_buf.at(i).begin()+2, &indx, 10);
          mot_current_ch2.at(i) = (int32_t)strtol(indx, &indx, 10);
          bat_current_ch1.at(i) = (int32_t)strtol(indx, &indx, 10);
          bat_current_ch2.at(i) = (int32_t)strtol(indx, &indx, 10);
          voltage.at(i) = (int32_t)strtol(indx, &indx, 10);
          temperature_mcu.at(i) = (int32_t)strtol(indx, &indx, 10);
        }
        #endif
        // S is sensor data
        else if(odom_buf.at(i).at(0) == 'S' && odom_buf.at(i).at(1) == '=')
        {
          char* indx;
          inductive_prox_left = (int32_t)strtol(odom_buf.at(i).begin()+2, &indx, 10);
          inductive_prox_right = (int32_t)strtol(indx, &indx, 10);
          ir_aluminum_lf = (bool)strtol(indx, &indx, 10);
          ir_aluminum_rf = (bool)strtol(indx, &indx, 10);
          ir_aluminum_lr = (bool)strtol(indx, &indx, 10);
          ir_aluminum_rr = (bool)strtol(indx, &indx, 10);
          ir_retro_left = !((bool)strtol(indx, &indx, 10));
          ir_retro_right = !((bool)strtol(indx, &indx, 10));
          chrg_hall = (bool)strtol(indx, &indx, 10);
          
          odom_idx.at(i) = 0;
          odom_publish();    
        }
        odom_idx.at(i) = 0;
      }
      else if ( odom_idx.at(i) < (sizeof(odom_buf.at(i))-1) && ( ch != '+'))
      {
        odom_buf.at(i).at(odom_idx.at(i)++) = ch;
      }
    }
  }
}


void MainNode::odom_aux_run()
{
  #ifdef _AUX_DATA
  v_bat = 0;
  float num_reports = 0;
  for(int i=0; i<NUM_CONTROLLERS; i++)
  {
    if (voltage.at(i) >= 200)
    {
      v_bat+=voltage.at(i);
      num_reports += 10;
    }
  }
  v_bat = v_bat/num_reports;
  aux_msg.v_bat = v_bat;

  aux_msg.bc_i_bat = bat_current_ch1.at(0) + bat_current_ch2.at(0);
  aux_msg.bc_i_mot1 = mot_current_ch1.at(0);
  aux_msg.bc_i_mot2 = mot_current_ch2.at(0);
  aux_msg.bc_temp_mcu = temperature_mcu.at(0);

  aux_msg.fc_i_bat = bat_current_ch1.at(1) + bat_current_ch2.at(1);
  aux_msg.fc_i_mot1 = mot_current_ch1.at(1);
  aux_msg.fc_i_mot2 = mot_current_ch2.at(1);
  aux_msg.fc_temp_mcu = temperature_mcu.at(1);
  
  aux_msg.rc_i_bat = bat_current_ch1.at(2) + bat_current_ch2.at(2);
  aux_msg.rc_i_mot1 = mot_current_ch1.at(2);
  aux_msg.rc_i_mot2 = mot_current_ch2.at(2);
  aux_msg.rc_temp_mcu = temperature_mcu.at(2);

  aux_pub.publish(aux_msg);

  #endif
}

void MainNode::odom_publish()
{
  // determine delta time in seconds
  uint32_t nowtime = millis();
  float dt = (float)DELTAT(nowtime,odom_last_time) / 1000.0;
  float linear_distance = 0;
  float angular_distance = 0;
  if((odom_hall_lf != 0 || odom_hall_rf != 0) && (odom_hall_lr != 0 || odom_hall_rr != 0))
  {
    float delta_left = (float)(odom_hall_lf+odom_hall_lr) * wheel_circumference / (2 * encoder_cpr);
    float delta_right = (float)(odom_hall_rf+odom_hall_rr) * wheel_circumference / (2 * encoder_cpr);
    linear_distance = delta_left + delta_right / 2.0;
    angular_distance = atan2(delta_right - delta_left, track_width);
    odom_hall_lf = 0;
    odom_hall_rf = 0;
    odom_hall_lr = 0;
    odom_hall_rr = 0;
  }

  control_msg.linear_distance = linear_distance;
  control_msg.angular_distance = angular_distance;
  control_msg.dt = dt;
  control_msg.inductive_prox_left_filtered = lowPassFilterInductive(inductive_prox_left, last_inductive_prox_left, dt);
  control_msg.inductive_prox_right_filtered = lowPassFilterInductive(inductive_prox_right, last_inductive_prox_right, dt);
  control_msg.ir_aluminum_lf = digitalDebounce(ir_aluminum_lf, last_ir_aluminum_lf);
  control_msg.ir_aluminum_rf = digitalDebounce(ir_aluminum_rf, last_ir_aluminum_rf);
  control_msg.ir_aluminum_lr = digitalDebounce(ir_aluminum_lr, last_ir_aluminum_lr);
  control_msg.ir_aluminum_rr = digitalDebounce(ir_aluminum_rr, last_ir_aluminum_rr);
  control_msg.ir_retro_left = digitalDebounce(ir_retro_left, last_ir_retro_left);
  control_msg.ir_retro_right = digitalDebounce(ir_retro_right, last_ir_retro_right);
  control_msg.chrg_hall = digitalDebounce(chrg_hall, last_chrg_hall);
  control_msg.v_bat = v_bat;
  control_pub.publish(control_msg);

  #ifdef _TUNING_DATA
  tuning_msg.measured_speed_lf = measured_speed_lf / gear_ratio_drive;
  tuning_msg.measured_speed_rf = measured_speed_rf / gear_ratio_drive;
  tuning_msg.measured_speed_lr = measured_speed_lr / gear_ratio_drive;
  tuning_msg.measured_speed_rr = measured_speed_rr / gear_ratio_drive;
  tuning_msg.measured_speed_bf = measured_speed_bf / gear_ratio_brush;
  tuning_msg.measured_speed_br = measured_speed_br / gear_ratio_brush;
  tuning_msg.track_error_lf = track_error_lf;
  tuning_msg.track_error_rf = track_error_rf;
  tuning_msg.track_error_lr = track_error_lr;
  tuning_msg.track_error_rr = track_error_rr;
  tuning_msg.track_error_bf = track_error_bf;
  tuning_msg.track_error_br = track_error_br;
  tuning_pub.publish(tuning_msg);
  #endif

  odom_last_time = nowtime;
  last_inductive_prox_left = inductive_prox_left;
  last_inductive_prox_right = inductive_prox_right;
  last_ir_aluminum_lf = ir_aluminum_lf;
  last_ir_aluminum_rf = ir_aluminum_rf;
  last_ir_aluminum_lr = ir_aluminum_lr;
  last_ir_aluminum_rr = ir_aluminum_rr;
  last_ir_retro_left = ir_retro_left;
  last_ir_retro_right = ir_retro_right;
  last_chrg_hall = chrg_hall;



  
  
}

bool MainNode::brush_speed_cb( roboteq_diff_msgs::RequestBrushSpeed::Request& req, roboteq_diff_msgs::RequestBrushSpeed::Response& res)
{
  brush_rpm = req.brush_rpm_req;
  brush_loop();
  res.brush_rpm_res = brush_rpm;
  return(true);
}

bool MainNode::brush_activate_cb(roboteq_diff_msgs::RequestBrushActivate::Request& req, roboteq_diff_msgs::RequestBrushActivate::Response& res)
{
  brush_active = req.activate;
  brush_loop();
  res.complete = true;
  return(true);
}

void MainNode::brush_loop()
{
  int32_t temp_rpm_front;
  int32_t temp_rpm_rear;
  if (brush_active)
  { 
    temp_rpm_front = brush_rpm*gear_ratio_brush;
    temp_rpm_rear = brush_rpm*gear_ratio_brush;
  }
  else
  {
    temp_rpm_front = 0;
    temp_rpm_rear = 0;
  } 
  if(temp_rpm_front > 2900) temp_rpm_front = 2900;
  if(temp_rpm_front < 0) temp_rpm_front = 0;
  if(temp_rpm_rear > 2900) temp_rpm_rear = 2900;
  if(temp_rpm_rear < 0) temp_rpm_rear = 0;

  std::stringstream front_cmd1;
  std::stringstream rear_cmd1;
  if (open_loop_brush)
  {
    front_cmd1 << "!G 1 " << temp_rpm_front << "\r";
    rear_cmd1 << "!G 2 " << temp_rpm_rear << "\r";
  }
  else
  {
    front_cmd1 << "!S 1 " << temp_rpm_front << "\r";
    rear_cmd1 << "!S 2 " << temp_rpm_rear << "\r";
  }
  
  controllers.at(0).write(front_cmd1.str());
  controllers.at(0).write(rear_cmd1.str());
  controllers.at(0).flush();
}

bool MainNode::emo_cb(roboteq_diff_msgs::EMO::Request& req, roboteq_diff_msgs::EMO::Response& res)
{
  std::stringstream temp;
  if (req.activate_emo) temp << "!D1 4\r";
  else temp << "!D0 4\r";
  controllers.at(0).write(temp.str());
  controllers.at(0).flush();
  res.complete = true;
  return(true);
}

bool MainNode::charge_activate_cb(roboteq_diff_msgs::RequestChargeActivate::Request& req, roboteq_diff_msgs::RequestChargeActivate::Response& res)
{
  std::stringstream temp;
  if (req.activate_charging) temp << "!D1 1\r";
  else temp << "!D0 1\r";
  controllers.at(0).write(temp.str());
  controllers.at(0).flush();
  res.complete = true;
  return(true);
}

bool MainNode::toggle_closed_loop_cb(roboteq_diff_msgs::ToggleClosedLoop::Request & req, roboteq_diff_msgs::ToggleClosedLoop::Response& res)
{
  if ((control_msg.linear_distance != 0) && (control_msg.angular_distance != 0))
  {
    res.success = false;
    return(false);
  }
  std::stringstream temp;
  for(int i = 1; i < NUM_CONTROLLERS; i++)
  {
    if (req.closed_loop_req)
    {
      controllers.at(i).write("^MMOD 1 1\r");
      controllers.at(i).write("^MMOD 2 1\r");
      open_loop = false;
    }
    else 
    {
      controllers.at(i).write("^MMOD 1 0\r");
      controllers.at(i).write("^MMOD 2 0\r");
      open_loop = true;
    }
    controllers.at(i).flush();
  }
  res.success = true;
  return(true);
}

int MainNode::run()
{

	ROS_INFO("Beginning setup...");

	serial::Timeout timeout = serial::Timeout::simpleTimeout(1000);
  for(int i=0; i<NUM_CONTROLLERS; i++)
  {
    switch(i)
    {
      case 0:
      {
        controllers.at(i).setPort(port0);
        break;
      }
      case 1:
      {
        controllers.at(i).setPort(port1);
        break;
      }
      case 2: 
      {
        controllers.at(i).setPort(port2);
        break;
      }
      default:
      {
        //error state
        break;
      }
    }
    controllers.at(i).setBaudrate(baud);
    controllers.at(i).setTimeout(timeout);
 
    // TODO: support automatic re-opening of port after disconnection
    while ( ros::ok() )
    {
      switch(i)
      {
        case 0:
        {
          ROS_INFO_STREAM("Opening serial port: " << port0 << "..." );
          break;
        }
        case 1:
        {
          ROS_INFO_STREAM("Opening serial port: " << port1 << "..." );
          break;
        }
        case 2:
        {
          ROS_INFO_STREAM("Opening serial port: " << port2 << "..." );
          break;
        }
        default:
        {
          //error state
          break;
        }
      }

      try
      {
        controllers.at(i).open();
        if ( controllers.at(i).isOpen() )
        {
          switch(i)
          {
            case 0:
            {
              ROS_INFO_STREAM("Successfully opened serial port: " << port0 << "!" );
              break;
            }
            case 1:
            {
              ROS_INFO_STREAM("Successfully opened serial port: " << port1 << "!" );
              break;
            }
            case 2:
            {
              ROS_INFO_STREAM("Successfully opened serial port: " << port2 << "!" );
              break;
            }
            default:
            {
              //error state
              break;
            }
          }
          break;
        }
      }
      catch (serial::IOException e)
      {
        ROS_WARN_STREAM("serial::IOException: " << e.what());
      }
      ROS_WARN("Failed to open serial port !?!?!");
      sleep( 5 );
    }
  }

	cmdvel_setup();
	odom_setup();

  start_time = millis();
  aux_timer = start_time;

  //  ros::Rate loop_rate(10);

  ROS_INFO("Beginning looping...");
	
  while (ros::ok())
  {
    odom_loop();
    

    uint32_t nowtime = millis();

    // Handle 1 Hz publishing
    if (DELTAT(nowtime,aux_timer) >= 1000)
    {
      aux_timer = nowtime;
      odom_aux_run();
      // brush_loop();
    }   

    ros::spinOnce();

  //    loop_rate.sleep();
  }

  brush_active = false;
  brush_loop();
  
	for(int i=0; i<NUM_CONTROLLERS; i++)
  {
    if ( controllers.at(i).isOpen() ) controllers.at(i).close();
  }

  ROS_INFO("Exiting");
	
  return 0;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "main_node");

  MainNode node;

  // Override the default ros sigint handler.
  // This must be set after the first NodeHandle is created.
  signal(SIGINT, mySigintHandler);

  return node.run();
}
