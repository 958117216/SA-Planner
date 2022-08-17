/**
 * @file uu_dirc_ctrl_astar.h
 * @author Guo Yilin (958117216@qq.com)
 * @brief 
 * @version 0.1
 * @date 2022-05-20
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#pragma once

#include <Eigen/Eigen>
#include <iostream>
#include <map>
#include <ros/console.h>
#include <ros/ros.h>
#include <string>
#include <unordered_map>
#include <boost/functional/hash.hpp>
#include <queue>

#include "swin_mapping/swin_map.h"
#include "bspline_module/unclamped_uniform_bspline.h"
#include "bspline_module/conversion_BS2MV.h"
#include "visualization_module/planning_visualization.h"

#define IN_CLOSE_SET 'a'
#define IN_OPEN_SET 'b'
#define NOT_EXPAND 'c'

//matrix_hash 对int 和double 通用
template <typename T>
struct matrix_hash : std::unary_function<T, size_t> {
  std::size_t operator()(T const& matrix) const {
    size_t seed = 0;
    for (size_t i = 0; i < matrix.size(); ++i) {
      auto elem = *(matrix.data() + i);
      seed ^= std::hash<typename T::Scalar>()(elem) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
    }
    return seed;
  }
};


//·1.A* algorithm 数据
class AstarNode {
public:
  /* -------------------- */
  Eigen::Vector3i grid;
  Eigen::Vector3d position;
  double g_score, f_score;
  AstarNode* parent;
  char node_state;

  /* -------------------- */
  AstarNode() {
    parent = NULL;
    node_state = NOT_EXPAND;
  }
  ~AstarNode(){};
};
typedef AstarNode* AstarNodePtr;

//使用谓词(函数对象)改变算法策略，排序从大到小
class AstarNodeComparator {
public:
  bool operator()(AstarNodePtr node1, AstarNodePtr node2) {
    return node1->f_score > node2->f_score;
  }
};

class AstarNodeHashTable{
private:
  /* data */
  std::unordered_map<Eigen::Vector3i, AstarNodePtr, matrix_hash<Eigen::Vector3i>> data_3d_;

public:
  AstarNodeHashTable(/* args */) {
  }
  ~AstarNodeHashTable() {
  }
  void insert(Eigen::Vector3i idx, AstarNodePtr node) {
    data_3d_.insert(make_pair(idx, node));
  }

  AstarNodePtr find(Eigen::Vector3i idx) {
    auto iter = data_3d_.find(idx);
    return iter == data_3d_.end() ? NULL : iter->second;
  }
  void clear() {
    data_3d_.clear();
  }
};

//·2.Ctrl A* algorithm 数据

class CtrlNode {
public:
  /* -------------------- */
  Eigen::Vector3i grid;
  Eigen::Vector3d position;
  double g_score, f_score;
  double time_score;
  CtrlNode* parent;
  char node_state;
  double step;  //-1代表该节点和父节点位置关系固定
  int stage; //该属性表示 此控制点是在第几段被搜索得到的。从0开始，0表示计算获得 如q0-q2
  /* -------------------- */
  CtrlNode() {
    parent = NULL;
    node_state = NOT_EXPAND;
  }
  ~CtrlNode(){};
};
typedef CtrlNode* CtrlNodePtr;

//使用谓词(函数对象)改变算法策略，排序从大到小，top()返回的就是最小值
class CtrlNodeComparator
{
public:
  bool operator()(CtrlNodePtr node1, CtrlNodePtr node2) {
    return node1->f_score > node2->f_score;
  }
};

class CtrlNodeHashTable
{
private:
  /* data */
  std::unordered_map<Eigen::Vector3d, CtrlNodePtr, matrix_hash<Eigen::Vector3d>> data_3d_;

public:
  CtrlNodeHashTable(/* args */) {}
  ~CtrlNodeHashTable() {}

  void insert(Eigen::Vector3d idx, CtrlNodePtr node) {
    data_3d_.insert(make_pair(idx, node));
  }

  CtrlNodePtr find(Eigen::Vector3d idx) {
    auto iter = data_3d_.find(idx);
    return iter == data_3d_.end() ? NULL : iter->second;
  }

  void clear() {
    data_3d_.clear();
  }
};



class UUDircCtrlAstar
{
public:
    UUDircCtrlAstar(){};
    ~UUDircCtrlAstar();

    //*初始化
    void initSearch(ros::NodeHandle& nh,const SWINMap::Ptr& map,const PlanningVisualization::Ptr& visualization);

    //*主要接口
    void reset();
    vector<Eigen::Vector3i> First_AstarGraphSearch(vector<Eigen::Vector3d> pos_start,Eigen::Vector3d pos_goal);
    vector<Eigen::Vector3i> GetAstarResults(AstarNodePtr first_node,AstarNodePtr last_node);
    void AstarGetSucc(AstarNodePtr currentPtr, vector<Eigen::Vector3i> & neighborSets, vector<double> & edgeCostSets);


    vector<Eigen::Vector3d> Second_ExtractAstarPath(vector<Eigen::Vector3i>& AstarPath);


    vector<CtrlNodePtr> Third_Directional_Search(const int stage,vector<CtrlNodePtr>& q0q1q2_stage,const Eigen::Vector3d& goal_next,bool final_goal);
    void  ctrlSearchReset();
    vector<Eigen::Vector3d> RetrieveLocalCPs(const CtrlNodePtr& q_cur);
    double GetStep(const double dist);
    double GetStepEASA(const vector<Eigen::Vector3d>& LocalCPs);
    bool CheckDynamic(const vector<Eigen::Vector3d>& LocalCPs,const Eigen::Vector3d& q_cur );
    //结果包含first_node和last_node
    vector<CtrlNodePtr> GetCtrlAstarResults(CtrlNodePtr first_node ,CtrlNodePtr last_node);
    vector<Eigen::Vector3d> DirectionSafeNodeExpansion(const vector<Eigen::Vector3d>& LocalCPs,const CtrlNodePtr& q_cur,  double& step,const Eigen::Vector3d direction);
    vector<Eigen::Vector3d> SafeNodeExpansion(const vector<Eigen::Vector3d>& LocalCPs,const CtrlNodePtr& q_cur, const double step);
    double AdjustStep(double q0,double q1,double q2,double step);
    std::vector<Eigen::Vector3d> CtrlNode2Path();

    std::vector<Eigen::Vector3d>  ThirdStage_getVisitedNodes();
    Eigen::MatrixXd RetrieveCtrlNodeStep();//询问控制点步长
    vector<int> RetrieveCtrlNodeStage();//询问控制点所属阶段
    int GetPathLength(CtrlNodePtr last_node);


   Eigen::MatrixXd Overall_Search(Eigen::Vector3d pos_start,Eigen::Vector3d vel_start,Eigen::Vector3d acc_start,
            Eigen::Vector3d pos_goal,Eigen::Vector3d vel_goal=Eigen::Vector3d::Zero(),Eigen::Vector3d acc_goal=Eigen::Vector3d::Zero());
    //时间间隔调整 模块
    void AdjustDeltaT(Eigen::Vector3d pos_start,Eigen::Vector3d vel_start,Eigen::Vector3d acc_start);
    double cubic(double b,double c,double d);
    int quartic(double b,double c,double d,double e,double* ans);

    std::vector<Eigen::Vector3d> GetCPsFromInitialState(Eigen::Vector3d pos_start,Eigen::Vector3d vel_start,Eigen::Vector3d acc_start);
    std::vector<Eigen::Vector3d> GetCPsFromFinalState(Eigen::Vector3d pos_goal,Eigen::Vector3d vel_goal,Eigen::Vector3d acc_goal);
    
    /*useful function*/
    template <typename T>
    double getHeu(T x1,T x2,int type);
    //得到 [v0,..,vn]
    Eigen::MatrixXd  Vectors2matrix(const vector<Eigen::Vector3d>& vec);

    /*参数读取接口*/
    int GetDegree();
    double GetDelta_t();
    std::vector<Eigen::Vector3d>  getVisitedNodes();
    std::vector<std::vector<Eigen::Vector3d>>  getVisitedNodes_TimeKind();

    //获得凸分解的路径点
    vector<Eigen::Vector3d,Eigen::aligned_allocator<Eigen::Vector3d>> getDecompPath(int num_stage);


    //*额外定义
    typedef std::shared_ptr<UUDircCtrlAstar> Ptr;
    double max_vel_,max_acc_; //

private:
    std::vector<Eigen::Vector3d> q0q1q2_;
    //·A* algorithm 数据
    vector<AstarNodePtr> astar_node_pool_;
    int astar_node_allocate_num_;
    std::priority_queue<AstarNodePtr, std::vector<AstarNodePtr>, AstarNodeComparator> astar_node_open_set_;
    AstarNodeHashTable astar_node_expanded_nodes_;
    int astar_use_node_num_;//指代扩展过的点的数量，初始值为p个
    //·Ctrl A* algorithm 数据
    vector<CtrlNodePtr> ctrl_node_pool_;
    int ctrl_node_allocate_num_;
    std::priority_queue<CtrlNodePtr, std::vector<CtrlNodePtr>, CtrlNodeComparator> ctrl_node_open_set_;
    CtrlNodeHashTable ctrl_node_expanded_nodes_;
    int ctrl_use_node_num_;//指代扩展过的点的数量，初始值为p个

    vector<Eigen::Vector3d> SecondStageRes_;//存储A*的关键节点

     //·通用参数
    std::vector<CtrlNodePtr> searched_ctrl_nodes_; // 最终的搜索结果，q0->qn-2 控制节点的集合
    double resolution_;

    
    int  iter_num_;  //指代大循环发生的次数，最小为1次
    double delta_t_; //Uniform BSpline曲线的节点向量的间隔
    double delta_t_initial_; //Uniform BSpline
    double margin_;  //τ 希望离障碍物的距离
    double tie_breaker_;
    double lambda_heu_;
    int degree_; //BSpline曲线次数
    double reduced_threshold_,reduced_vel_;
    int  TerminationCPNum_;
    bool show_rviz_;
    SWINMap::Ptr swin_map_;
    PlanningVisualization::Ptr visualization_;
    conversion_BS2MV M;



};

