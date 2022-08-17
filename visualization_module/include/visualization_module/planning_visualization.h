/**
 * @file planning_visualization.h
 * @author 港科大
 * @brief   特点 同一个pub，不同的id 区分不同的消息
 * @version 0.1
 * @date 2022-03-02
 * 
 * @copyright Copyright (c) 2022
 * 
 */


#pragma once


#include <Eigen/Eigen>
#include <algorithm>
#include <iostream>
#include <ros/ros.h>
#include <vector>
#include <visualization_msgs/Marker.h>

#include "bspline_module/unclamped_uniform_bspline.h"


using std::vector;

class PlanningVisualization {
private:
  enum TRAJECTORY_PLANNING_ID {GOAL = 1,PATH = 200,BSPLINE = 300,BSPLINE_CTRL_PT = 400,POLY_TRAJ = 500};

  /* data */
  /* visib_pub is seperated from previous ones for different info */
  ros::NodeHandle node;

  ros::Publisher traj_pub_;      // 轨迹可视化
  ros::Publisher checkTraj_pub; //安全检查可视化
  ros::Publisher corridor_pub_;     //安全走廊可视化
  ros::Publisher path_pub_;  // 路径搜索可视化
  ros::Publisher expanded_nodes_pub_; //前端扩展过的点可视化
  ros::Publisher yaw_pub_;       // yaw trajectory可视化

  vector<ros::Publisher> pubs_;  

public:

  enum Colors_ID{Red=0, Orange=1, Yellow=2, Green=3, Blue=4, Indigo=5, Purple=6};
  std::vector<Eigen::Vector4d>  Colors_Formula_;
  enum Publisher_ID {Traj=0, CheckTraj=1,Corridor=2,Path=3,ExpNodes=4,Yaw=5};

public:
  PlanningVisualization(/* args */) {}
  ~PlanningVisualization() {}
  PlanningVisualization(ros::NodeHandle& nh);


  Eigen::Vector4d getColor(double h, double alpha = 1.0);



  /**
   * @brief  可视化最基本的函数，可绘制  球体列表，立方体列表，线列表
   * 
   * @param list   要绘制的形状列表
   * @param resolution 分辨率
   * @param color 颜色
   * @param id  visualization_msgs::Marker 的id， 一个topic可以有多个id
   * @param pub_id 发布者的ID
   */
  void displaySphereList(const vector<Eigen::Vector3d>& list, double resolution,
                         const Eigen::Vector4d& color, int id, int pub_id);
  void displayCubeList(const vector<Eigen::Vector3d>& list, double resolution,
                       const Eigen::Vector4d& color, int id, int pub_id);
  void displayLineList(const vector<Eigen::Vector3d>& list1, const vector<Eigen::Vector3d>& list2,
                       double line_width, const Eigen::Vector4d& color, int id, int pub_id);

 void displaySingleArrow(const Eigen::Vector3d& start, const Eigen::Vector3d& end, double resolution,
                      const Eigen::Vector4d& color,int id,int pub_id); 

/**
 * @brief  对外的接口,将可视化最基本的函数进行调用封装，分配对应的pub_id(通过函数默认参数值实现)
 */

  //*****************************************pub_id 发布者的ID 为 Traj=0*******************************************************
    
  //*TRAJECTORY_PLANNING_ID :GOAL = 1
  //绘制目标点
  void drawGoal(Eigen::Vector3d goal, double resolution, const Eigen::Vector4d& color, int id = 0);

  //*TRAJECTORY_PLANNING_ID:BSPLINE = 300,BSPLINE_CTRL_PT = 400
  // 绘制B样条曲线
  void drawBspline(UnclampedUniformBspline& bspline, double size, const Eigen::Vector4d& color,
                   bool show_ctrl_pts = false, double size2 = 0.1,
                   const Eigen::Vector4d& color2 = Eigen::Vector4d(1, 1, 0, 1), int id1 = 0,
                   int id2 = 0);



  //*****************************************pub_id 发布者的ID 为 CheckTraj=1*******************************************************
  //*TRAJECTORY_PLANNING_ID: PATH = 200
  //绘制轨迹安全检测点
  void drawCheckTraj(const vector<Eigen::Vector3d>& path, double resolution,
                         const Eigen::Vector4d& color, int id=0 ,int pub_id=CheckTraj);

  //*****************************************pub_id 发布者的ID 为 Corridor=2*******************************************************
  //*TRAJECTORY_PLANNING_ID: PATH = 200
  //绘制障碍物点
void drawCollisionPlane(const vector<Eigen::Vector3d>& path, double resolution,const Eigen::Vector4d& color, int id = 0);
  //*TRAJECTORY_PLANNING_ID: BSPLINE = 300
void drawCollisionPlaneArrows(const vector<Eigen::Vector3d>& starts, const vector<Eigen::Vector3d>& ends, double resolution,const Eigen::Vector4d& color, int id = 0);


  //*****************************************pub_id 发布者的ID 为 Path=3*******************************************************
  //*TRAJECTORY_PLANNING_ID :GOAL = 1
  //绘制Search模块的起点
  void drawSearchStart(Eigen::Vector3d start, double resolution, const Eigen::Vector4d& color, int id = 0);
  //*TRAJECTORY_PLANNING_ID: PATH = 200
  // draw a piece-wise straight line path 绘制分段直线路径
  void drawGeometricPath(const vector<Eigen::Vector3d>& path, double resolution,const Eigen::Vector4d& color, int id = 0);


  //*****************************************pub_id 发布者的ID 为  ExpNodes=4*******************************************************
  //*TRAJECTORY_PLANNING_ID:BSPLINE_CTRL_PT = 400
  //绘制前端遍历过的点
  void drawVisitedNode(const vector<Eigen::Vector3d>& path, double resolution,
                         const Eigen::Vector4d& color, int id=0 ,int pub_id=ExpNodes);
  void drawLayeredVisitedNode(const vector<vector<Eigen::Vector3d>>& path, double resolution,
                          Eigen::Vector4d color=Eigen::Vector4d::Zero(), int id=1 ,int pub_id=ExpNodes);


  typedef std::shared_ptr<PlanningVisualization> Ptr;
};
