/**
 * @file sa_fsm.h
 * @author Guo Yilin (958117216@qq.com)
 * @brief   状态向量机
 * @version 0.1
 * @date 2022-03-03
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#pragma once

#include <Eigen/Eigen>
#include <algorithm>
#include <iostream>
#include <nav_msgs/Path.h>
#include <ros/ros.h> 
#include <std_msgs/Empty.h> 

#include <vector>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PoseStamped.h>

#include "manage_module/termcolor.hpp"

#include "bspline_module/unclamped_uniform_bspline.h"
#include "swin_mapping/swin_map.h"
#include "path_search/uu_ctrl_astar.h"
// #include "path_search/uu_ctrl_astar_four.h"
#include "manage_module/sa_planner/sa_traj_info.hpp"
#include "manage_module/sa_planner/sa_manager.h"
#include "visualization_module/planning_visualization.h"
#include "manage_module/Bspline.h"  //msg

using std::vector;
using std::cout;
using std::endl;

using namespace termcolor;


class SA_FSM
{
private:
    /* ---------- flag ---------- */
    ///FSM 状态枚举
    enum FSM_EXEC_STATE { INIT, WAIT_TARGET, GEN_NEW_TRAJ, REPLAN_TRAJ, EXEC_TRAJ};
    ///目标点类型枚举
    enum TARGET_TYPE { MANUAL_TARGET = 1, PRESET_TARGET = 2, INCREASE_TARGET = 3};

    //SA_Manager
    SA_Manager::Ptr sa_manager_;
    //可视化
    PlanningVisualization::Ptr visualization_;

    /* parameters */
    ///目标点类型
    int target_type_;  // 1 mannual select, 2 hard code
    ///不再重规划阈值
    double no_replan_thresh_;
    ///再次重规划阈值
    double replan_thresh_;
    ///提前预设的路径点
    double waypoints_[50][3];
    ///提前预设的路径点数量
    int waypoint_num_;
    ///碰撞检测的距离
    double checkTrajCollision_distance_;

    /*FSM标志*/
    /// 目标点触发
    bool trigger_;
    ///是否得到目标点
    bool have_target_;
    ///是否得到里程计
    bool have_odom_;
    ///FSM执行的状态
    FSM_EXEC_STATE exec_state_;

    ///里程计数据
    Eigen::Vector3d odom_pos_, odom_vel_;  // odometry state
    Eigen::Quaterniond odom_orient_;
    ///起点状态包括位置、速度、加速度、偏航+偏航'+偏航''
    Eigen::Vector3d start_pt_, start_vel_, start_acc_, start_yaw_;  // start state   
    ///终点状态
    Eigen::Vector3d end_pt_, end_vel_;                              // target state
    ///当前的路径点
    int current_wp_;

    /* ROS */
    ///FSM定时器
    ros::Timer exec_timer_;
    ///轨迹检查定时器
    ros::Timer safety_timer_;

    ///订阅目标点
    ros::Subscriber waypoint_sub_;
    ///订阅里程计
    ros::Subscriber odom_sub_;

    ///发布重规划标志
    ros::Publisher replan_pub_;
    ///发布新规划标志
    ros::Publisher new_pub_;
    ///发布规划的轨迹
    ros::Publisher bspline_pub_;


  /* helper functions */
  /// 主要函数 调用Manager重规划
  bool callManagerReplan();   
  ///改变FSM状态
  void changeFSMExecState(FSM_EXEC_STATE new_state, string pos_call);
  ///打印FSM状态
  void printFSMExecState();

  /* ROS functions */
  ///FSM定时器 回调函数
  void execFSMCallback(const ros::TimerEvent& e);
  ///轨迹检查定时器 回调函数
  void checkCollisionCallback(const ros::TimerEvent& e);
  ///订阅目标点 回调函数
  void waypointCallback(const nav_msgs::PathConstPtr& msg);
  ///订阅里程计 回调函数
  void odometryCallback(const nav_msgs::OdometryConstPtr& msg);

public:
  SA_FSM() {}
  ~SA_FSM() { }
  
  /**
   * @brief   状态向量机初始化
   * 
   * @param nh 
   */
  
  void initFSM(ros::NodeHandle& nh);

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

};