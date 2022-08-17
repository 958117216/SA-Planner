#pragma once


#include <Eigen/Eigen>
#include <vector>
#include <ros/ros.h>

#include "bspline_module/unclamped_uniform_bspline.h"


#include <iostream>
// #include<Eigen/Dense>
#include<fstream>
#include<vector>

using namespace std;
using std::vector;


struct PlanParameters {
  /* planning algorithm parameters */
  double max_vel_, max_acc_, max_jerk_;  
  double local_traj_len_;              


  /* processing time */
  double time_search_ = 0.0;
  double time_optimize_ = 0.0;
};


struct LocalTrajData {
  /* info of generated traj */
  int traj_id_;
  double duration_;
  ros::Time start_time_;
  Eigen::Vector3d start_pos_;
  UnclampedUniformBspline position_traj_, velocity_traj_, acceleration_traj_, yaw_traj_, yawdot_traj_,
      yawdotdot_traj_;
  UnclampedUniformBspline beforeOpt_position_traj_;
  vector<Eigen::Vector3d> VisitedNodes;
  vector<vector<Eigen::Vector3d>> VisitedNodes_TimeKind;

  Eigen::MatrixXd corriders_;
};

