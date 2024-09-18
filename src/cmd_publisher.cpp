#include "cmd_publisher.h"

using namespace std::chrono_literals;
using std::placeholders::_1;

CmdPublisher::CmdPublisher() : Node("cmd_publisher") {
  // Publisher
  pub_cmd = this->create_publisher<geometry_msgs::msg::Twist>("robot/cmd", 10);

  // TF listener
  tf_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

  // Timer
  timer_tf = this->create_wall_timer(
      10ms, std::bind(&CmdPublisher::timer_tf_callback, this));
  timer_cmd = this->create_wall_timer(
      10ms, std::bind(&CmdPublisher::timer_cmd_callback, this));
}

void CmdPublisher::timer_tf_callback() {
  //TODO: implement this!
  geometry_msgs::msg::TransformStamped t;

  try {
    t = tf_buffer->lookupTransform(
      "world", "turtlesim1", tf2::TimePointZero);
  } catch (const tf2::TransformException & ex) {
    RCLCPP_INFO(
      this->get_logger(), "Could not transform %s",
      ex.what());
    return;
  }
  double z = t.transform.rotation.z;
  double w = t.transform.rotation.w;

  real_x = t.transform.translation.x;
  real_y = t.transform.translation.y;
  real_th = 2 * atan(z/w);
}


void CmdPublisher::timer_cmd_callback() {
  //TODO: implement this!
  geometry_msgs::msg::Twist cmd_vel;

  switch (a) {
    case 0:
      goal_x = 2;
      goal_y = 0;
      goal_th = 0;
      break;
    case 1:
      goal_x = 2;
      goal_y = 0;
      goal_th = 0.5 * PI;
      break;
    case 2:
      goal_x = 2;
      goal_y = 2;
      goal_th = 0.5 * PI;
      break;
    case 3:
      goal_x = 2;
      goal_y = 2;
      goal_th = 1.0 * PI;
      break;
    case 4:
      goal_x = 0;
      goal_y = 2;
      goal_th = 1.0 * PI;
      break;
    case 5:
      goal_x = 0;
      goal_y = 2;
      goal_th = 1.5 * PI;
      //if (real_th < 0) {
        //goal_th = -0.5 * PI;
      //}
      break;
    case 6:
      goal_x = 0;
      goal_y = 0;
      goal_th = 1.5 * PI; //-0.5 * PI
      break;
    case 7:
      goal_x = 0;
      goal_y = 0;
      goal_th = 0;
      break;
  }

  double d_x = goal_x - real_x;
  double d_y = goal_y - real_y;

  double error_d = sqrt(d_x * d_x + d_y * d_y);
  if ((a==0 && real_x>2)or(a==4 && real_x<0)or(a==2 && real_y>2)or(a==6 && real_y<0))
    error_d = - sqrt(d_x * d_x + d_y * d_y);
  double error_th = goal_th - real_th;
    if(error_th > 2 * PI){
      error_th -= 2 * PI;
    }
    else if(error_th < -2 * PI){
      error_th += 2 * PI;
    }

  double d_error_d = (error_d - prev_error_d) / dt;
  double d_error_th = (error_th - prev_error_th) / dt;

  i_error_d = i_error_d + error_d * dt; 
  i_error_th = i_error_th + error_th * dt;

  //if (a%2 == 0)
  cmd_vel.linear.x = k[0] * error_d + k[1] * d_error_d + k[2] * i_error_d;
  //if (a%2 == 1) 
  cmd_vel.angular.z = k[0] * error_th + k[1] * d_error_th + k[2] * i_error_th;

  RCLCPP_INFO(this->get_logger(), "a, error_d, error_th :[%d, %f, %f]", a, error_d, error_th);
    
  if ((a%2 == 0 && error_d < 0.01))
  {RCLCPP_WARN(this->get_logger(), "distance arrived");}
  if (a%2 == 1 && error_th < 0.001)
  {RCLCPP_WARN(this->get_logger(), "angular arrived");}
  
  if((a%2 == 0 && abs(error_d) < 0.01) or (a%2 == 1 && abs(error_th) < 0.001)){
    a++;
    a = a % 8;
    i_error_d = 0;
    i_error_th = 0;
  }

  prev_error_d = error_d;
  prev_error_th = error_th;

  pub_cmd->publish(cmd_vel);
}

