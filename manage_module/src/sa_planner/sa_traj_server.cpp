#include <nav_msgs/Odometry.h>
#include <std_msgs/Empty.h>
#include <visualization_msgs/Marker.h>
#include <ros/ros.h>
#include <mavros_msgs/PositionTarget.h>
#include <geometry_msgs/PoseStamped.h>

#include "bspline_module/unclamped_uniform_bspline.h"
#include "manage_module/Bspline.h"
#include "prometheus_msgs/PositionReference.h"
#include "prometheus_msgs/ControlCommand.h"

using std::vector;

struct state
{
  Eigen::Vector3d position;
  Eigen::Vector3d velocity;

  state( Eigen::Vector3d pos, Eigen::Vector3d vel )
  {
    position=pos;
    velocity=vel;
  }

};


//急停时间
double time_stop = 0.0;
bool stop_flag = false;
//各种定义
ros::Publisher cmd_vis_pub, pos_cmd_pub, traj_pub,local_pos_pub,pt_cmd_pub;

// 发布的话题
prometheus_msgs::ControlCommand CMD;
prometheus_msgs::PositionReference cmd;


bool receive_traj_ = false;
vector<UnclampedUniformBspline> traj_;
double traj_duration_;
ros::Time start_time_;
int traj_id_;

// yaw control
double last_yaw_;
double time_forward_;

vector<Eigen::Vector3d> traj_cmd_, traj_real_;
vector<state> traj_state_;

Eigen::Vector4d getColor(double v, double vmin, double vmax)
{
  Eigen::Vector4d color;// r g b a
  color[0] = 1;
  color[1] = 1;
  color[2] = 1;
  color[3] = 1;
  // white
  double dv;

  if (v < vmin)
    v = vmin;
  if (v > vmax)
    v = vmax;
  dv = vmax - vmin;

  if (v < (vmin + 0.25 * dv))
  {
    color[0] = 0;
    color[1] = 4 * (v - vmin) / dv;
  }
  else if (v < (vmin + 0.5 * dv))
  {
    color[0] = 0;
    color[2] = 1 + 4 * (vmin + 0.25 * dv - v) / dv;
  }
  else if (v < (vmin + 0.75 * dv))
  {
    color[0] = 4 * (v - vmin - 0.5 * dv) / dv;
    color[2] = 0;
  }
  else
  {
    color[1] = 1 + 4 * (vmin + 0.75 * dv - v) / dv;
    color[2] = 0;
  }

  return (color);
}

void displayTrajState(vector<state> traj, double resolution,int id)
{
  //ref:   https://stackoverflow.com/questions/41294705/sending-different-coloured-markers-in-rviz
  visualization_msgs::Marker mk;
  mk.header.frame_id = "world";
  mk.header.stamp = ros::Time::now();
  mk.type = visualization_msgs::Marker::POINTS;
  mk.action = visualization_msgs::Marker::ADD;
  mk.id = id;
  mk.scale.x = resolution;
  mk.scale.y = resolution;
  mk.scale.z = resolution;
  for (int i = 0; i <traj.size();i++)
  {

    Eigen::Vector4d color=getColor(traj[i].velocity.norm(),0,2.0);
    std_msgs::ColorRGBA c;
    c.r = color(0);
    c.g = color(1);
    c.b = color(2);
    c.a = color(3);
    mk.colors.push_back(c);

    geometry_msgs::Point pt;
    pt.x = traj[i].position(0);
    pt.y = traj[i].position(1);
    pt.z = traj[i].position(2);
    mk.points.push_back(pt);
  }

    traj_pub.publish(mk);
}

void displayTrajWithColor(vector<Eigen::Vector3d> path, double resolution, Eigen::Vector4d color,
                          int id) {
  visualization_msgs::Marker mk;
  mk.header.frame_id = "world";
  mk.header.stamp = ros::Time::now();
  mk.type = visualization_msgs::Marker::SPHERE_LIST;
  // mk.action = visualization_msgs::Marker::DELETE;
  mk.id = id;


  mk.action = visualization_msgs::Marker::ADD;
  mk.pose.orientation.x = 0.0;
  mk.pose.orientation.y = 0.0;
  mk.pose.orientation.z = 0.0;
  mk.pose.orientation.w = 1.0;

  mk.color.r = color(0);
  mk.color.g = color(1);
  mk.color.b = color(2);
  mk.color.a = color(3);

  mk.scale.x = resolution;
  mk.scale.y = resolution;
  mk.scale.z = resolution;

  geometry_msgs::Point pt;
  for (int i = 0; i < int(path.size()); i++) {
    pt.x = path[i](0);
    pt.y = path[i](1);
    pt.z = path[i](2);
    mk.points.push_back(pt);
  }
  traj_pub.publish(mk);
  ros::Duration(0.001).sleep();
}

void drawCmd(const Eigen::Vector3d& pos, const Eigen::Vector3d& vec, const int& id,
             const Eigen::Vector4d& color) {
  visualization_msgs::Marker mk_state;
  mk_state.header.frame_id = "world";
  mk_state.header.stamp = ros::Time::now();
  mk_state.id = id;
  mk_state.type = visualization_msgs::Marker::ARROW;
  mk_state.action = visualization_msgs::Marker::ADD;

  mk_state.pose.orientation.w = 1.0;
  mk_state.scale.x = 0.1;
  mk_state.scale.y = 0.2;
  mk_state.scale.z = 0.3;

  geometry_msgs::Point pt;
  pt.x = pos(0);
  pt.y = pos(1);
  pt.z = pos(2)+0.1;
  mk_state.points.push_back(pt);

  pt.x = pos(0) + vec(0);
  pt.y = pos(1) + vec(1);
  pt.z = pos(2) + vec(2);
  mk_state.points.push_back(pt);

  mk_state.color.r = color(0);
  mk_state.color.g = color(1);
  mk_state.color.b = color(2);
  mk_state.color.a = color(3);

  cmd_vis_pub.publish(mk_state);
}

void bsplineCallback(manage_module::BsplineConstPtr msg) 
{
  //清除急停标志位
  stop_flag=false;

  Eigen::MatrixXd ctrl_pts(3,msg->ctrl_pts.size());

  Eigen::VectorXd knots(msg->knots.size());
  for (int i = 0; i < msg->knots.size(); ++i) {
    knots(i) = msg->knots[i];
  }

  for (int i = 0; i < msg->ctrl_pts.size(); ++i) {
    ctrl_pts(0,i) = msg->ctrl_pts[i].x;
    ctrl_pts(1,i) = msg->ctrl_pts[i].y;
    ctrl_pts(2,i) = msg->ctrl_pts[i].z;
  }

  UnclampedUniformBspline pos_traj(ctrl_pts, msg->degree,1.0);
  pos_traj.setKnot(knots);

  // parse yaw traj
  Eigen::MatrixXd yaw_pts(1,msg->yaw_pts.size());
  for (int i = 0; i < msg->yaw_pts.size(); ++i) {
    yaw_pts(0,i) = msg->yaw_pts[i];
  }
  UnclampedUniformBspline yaw_traj(yaw_pts, msg->degree, msg->yaw_dt);

  start_time_ = msg->start_time;
  traj_id_ = msg->traj_id;

  traj_.clear();
  traj_.push_back(pos_traj);
  traj_.push_back(traj_[0].getDerivative());
  traj_.push_back(traj_[1].getDerivative());
  traj_.push_back(yaw_traj);
  traj_.push_back(yaw_traj.getDerivative());

  traj_duration_ = traj_[0].getTimeSum();

  receive_traj_ = true;
}

void replanCallback(std_msgs::Empty msg) {
  /* reset duration */
  const double time_out = 0.5;
  ros::Time time_now = ros::Time::now();
  double t_stop = (time_now - start_time_).toSec() + time_out;
  traj_duration_ = std::min(t_stop, traj_duration_);
}

void newCallback(std_msgs::Empty msg) {
  // receive_traj_ = false;
  // traj_.clear();
  // traj_cmd_.clear();
  // traj_real_.clear();

  const double time_out = 0.5;
  ros::Time time_now = ros::Time::now();
  double t_stop = (time_now - start_time_).toSec() + time_out;
  traj_duration_ = std::min(t_stop, traj_duration_);
}

void visCallback(const ros::TimerEvent& e) {
  // displayTrajWithColor(traj_real_, 0.03, Eigen::Vector4d(0.925, 0.054, 0.964,
  // 1),
  //                      1);

  // displayTrajWithColor(traj_cmd_, 0.05, Eigen::Vector4d(0, 1, 0, 1), 2);
  displayTrajState(traj_state_,0.05,200);
}

void cmdCallback(const ros::TimerEvent& e) 
{
  /* no publishing before receive traj_ */
  if (!receive_traj_) return;

  ros::Time time_now = ros::Time::now();
  double t_cur = (time_now - start_time_).toSec();

  Eigen::Vector3d pos, vel, acc, pos_f;
  double yaw, yawdot;

  if (t_cur < traj_duration_ && t_cur >= 0.0) {
    pos = traj_[0].evaluateDeBoorT(t_cur);
    vel = traj_[1].evaluateDeBoorT(t_cur);
    acc = traj_[2].evaluateDeBoorT(t_cur);
    yaw = traj_[3].evaluateDeBoorT(t_cur)[0];
    yawdot = traj_[4].evaluateDeBoorT(t_cur)[0];

    double tf = std::min(traj_duration_, t_cur + 2.0);
    pos_f = traj_[0].evaluateDeBoorT(tf);

  } else if (t_cur >= traj_duration_) {
    /* hover when finish traj_ */
    pos = traj_[0].evaluateDeBoorT(traj_duration_);
    vel.setZero();
    acc.setZero();
    yaw = traj_[3].evaluateDeBoorT(traj_duration_)[0];
    yawdot = traj_[4].evaluateDeBoorT(traj_duration_)[0];

    pos_f = pos;

  } else {
    std::cout << "[Traj server]: invalid time." << std::endl;
  }

  auto pos_err = pos_f - pos;
  last_yaw_ = cmd.yaw_ref;

  //普罗米修斯：发送控制命令
  cmd.header.stamp = time_now;
  cmd.header.frame_id = "map";

  cmd.Move_mode = prometheus_msgs::PositionReference::TRAJECTORY;  //TRAJECTORY
  cmd.Move_frame = prometheus_msgs::PositionReference::ENU_FRAME; //ENU_FRAME
  cmd.time_from_start = t_cur;

  cmd.position_ref[0] = pos(0);
  cmd.position_ref[1] = pos(1);
  cmd.position_ref[2] = pos(2);

  cmd.velocity_ref[0] = vel(0);
  cmd.velocity_ref[1] = vel(1);
  cmd.velocity_ref[2] = vel(2);

  cmd.acceleration_ref[0] = acc(0);
  cmd.acceleration_ref[1] = acc(1);
  cmd.acceleration_ref[2] = acc(2);

  cmd.yaw_ref = yaw;

  pos_cmd_pub.publish(cmd); 

  CMD.Mode=prometheus_msgs::ControlCommand::Move;
  CMD.Reference_State=cmd;
  // pt_cmd_pub.publish(CMD); //sitl 必须注释此行！


  // draw cmd 绘制各种控制命令

   drawCmd(pos, vel, 0, Eigen::Vector4d(0, 1, 0, 1));
   drawCmd(pos, acc, 1, Eigen::Vector4d(0, 0, 1, 1));
   Eigen::Vector3d dir(cos(yaw), sin(yaw), 0.0);
   drawCmd(pos,  dir, 2, Eigen::Vector4d(1, 1, 0, 0.7));
   // drawCmd(pos, pos_err, 3, Eigen::Vector4d(1, 1, 0, 0.7));

    traj_cmd_.push_back(pos);
    if (traj_cmd_.size() > 10000) traj_cmd_.erase(traj_cmd_.begin(), traj_cmd_.begin() + 1000);

    traj_state_.push_back(state(pos,vel));
    if (traj_state_.size() > 2000) traj_state_.erase(traj_state_.begin(), traj_state_.begin() + 1000);
}

int main(int argc, char** argv) 
{
  ros::init(argc, argv, "sa_traj_server");
  ros::NodeHandle node;
  ros::NodeHandle nh("~");

 //订阅者
  ros::Subscriber bspline_sub = node.subscribe("planning/bspline", 10, bsplineCallback);
  ros::Subscriber replan_sub = node.subscribe("planning/replan", 10, replanCallback);
  ros::Subscriber new_sub = node.subscribe("planning/new", 10, newCallback);


 //发布者
 cmd_vis_pub = node.advertise<visualization_msgs::Marker>("planning/position_cmd_vis", 10);
 traj_pub = node.advertise<visualization_msgs::Marker>("planning/travel_traj", 10);

 //针对普罗米修斯的发布者
  pos_cmd_pub = node.advertise<prometheus_msgs::PositionReference>("/prometheus/fast_planner/position_cmd", 50);
  pt_cmd_pub = node.advertise<prometheus_msgs::ControlCommand>("/prometheus/sa_planner/control_command", 50);

 //定时器
  ros::Timer cmd_timer = node.createTimer(ros::Duration(0.01), cmdCallback);
  ros::Timer vis_timer = node.createTimer(ros::Duration(0.25), visCallback);

  ros::Duration(1.0).sleep();
  ROS_WARN("[Traj server]: ready.");
  ros::spin();

  return 0;
}

