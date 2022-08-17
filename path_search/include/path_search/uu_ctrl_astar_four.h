/**
 * @file uu_ctrl_astar_four.h
 * @author Guo Yilin (958117216@qq.com)
 * @brief   Unclamped Uniform BSpline控制点 A*算法对象为四次B样条曲线,与另一个cu_ctrl_astar.h 不能同时使用，原因 变量重名
 * @version 0.1
 * @date 2022-05-12
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

class Node {
public:
  /* -------------------- */
  Eigen::Vector3i grid;
  Eigen::Vector3d position;
  double g_score, f_score;
  double time_score;
  Node* parent;
  char node_state;
  double step;  //-1代表该节点和父节点位置关系固定

  /* -------------------- */
  Node() {
    parent = NULL;
    node_state = NOT_EXPAND;
  }
  ~Node(){};
};
typedef Node* NodePtr;

//使用谓词(函数对象)改变算法策略，排序从大到小
class NodeComparator0
{
public:
  bool operator()(NodePtr node1, NodePtr node2) {
    return node1->f_score > node2->f_score;
  }
};

//Hasher
template <typename T>
struct matrix_hash0 : std::unary_function<T, size_t> {
  std::size_t operator()(T const& matrix) const {
    size_t seed = 0;
    for (size_t i = 0; i < matrix.size(); ++i) {
      auto elem = *(matrix.data() + i);
      seed ^= std::hash<typename T::Scalar>()(elem) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
    }
    return seed;
  }
};


//close_set+open_set
// class NodeHashTable0 
// {
// private:
//   /* data */
//   std::unordered_map<Eigen::Vector3i, NodePtr, matrix_hash0<Eigen::Vector3i>> data_3d_;

// public:
//   NodeHashTable0(/* args */) {}
//   ~NodeHashTable0() {}

//   void insert(Eigen::Vector3i idx, NodePtr node) {
//     data_3d_.insert(make_pair(idx, node));
//   }

//   NodePtr find(Eigen::Vector3i idx) {
//     auto iter = data_3d_.find(idx);
//     return iter == data_3d_.end() ? NULL : iter->second;
//   }

//   void clear() {
//     data_3d_.clear();
//   }
// };

class NodeHashTable0 
{
private:
  /* data */
  std::unordered_map<Eigen::Vector3d, NodePtr, matrix_hash0<Eigen::Vector3d>> data_3d_;

public:
  NodeHashTable0(/* args */) {}
  ~NodeHashTable0() {}

  void insert(Eigen::Vector3d idx, NodePtr node) {
    data_3d_.insert(make_pair(idx, node));
  }

  NodePtr find(Eigen::Vector3d idx) {
    auto iter = data_3d_.find(idx);
    return iter == data_3d_.end() ? NULL : iter->second;
  }

  void clear() {
    data_3d_.clear();
  }
};


class UUCtrlAstar_Four
{
public:
    UUCtrlAstar_Four(){};
    ~UUCtrlAstar_Four();

    //*初始化
    void initSearch(ros::NodeHandle& nh,const SWINMap::Ptr& map,const PlanningVisualization::Ptr& visualization);

     //*主要接口
    void reset();
    Eigen::MatrixXd search(Eigen::Vector3d pos_start,Eigen::Vector3d vel_start,Eigen::Vector3d acc_start,
            Eigen::Vector3d pos_goal,Eigen::Vector3d vel_goal=Eigen::Vector3d::Zero(),Eigen::Vector3d acc_goal=Eigen::Vector3d::Zero());


   //查询最终的搜索结果searched_ctrl_nodes_，作为search函数的补充，得到两两控制点间的步长
    Eigen::MatrixXd  RetrieveCtrlNodeStep(); 
    /* heuristic function */
    double getDiagHeu(Eigen::Vector3d x1, Eigen::Vector3d x2);
    double getManhHeu(Eigen::Vector3d x1, Eigen::Vector3d x2);
    double getEuclHeu(Eigen::Vector3d x1, Eigen::Vector3d x2);


    /*useful function*/
    //得到 [v0,..,vn]
    Eigen::MatrixXd  Vectors2matrix(const vector<Eigen::Vector3d>& vec);

    /*参数读取接口*/
    int GetDegree();
    double GetDelta_t();
    std::vector<Eigen::Vector3d>  getVisitedNodes();
    std::vector<std::vector<Eigen::Vector3d>>  getVisitedNodes_TimeKind();
    double GetStep(const double dist);

    
  typedef std::shared_ptr<UUCtrlAstar_Four> Ptr;

private:
    vector<NodePtr> ctrl_node_pool_;
    int allocate_num_;
    std::priority_queue<NodePtr, std::vector<NodePtr>, NodeComparator0> open_set_;
    NodeHashTable0 expanded_nodes_;
    std::vector<NodePtr> searched_ctrl_nodes_; // 最终的搜索结果，q0->qn-2 控制节点的集合
    double resolution_;
    int use_node_num_;//指代扩展过的点的数量，初始值为p个
    int  iter_num_;  //指代大循环发生的次数，最小为1次
    double delta_t_; //Uniform BSpline曲线的节点向量的间隔
    double margin_;  //τ 希望离障碍物的距离
    double tie_breaker_;
    double lambda_heu_;
    int degree_; //BSpline曲线次数
    double max_vel_,max_acc_; //
    SWINMap::Ptr swin_map_;
    PlanningVisualization::Ptr visualization_;
    conversion_BS2MV M;
    
private:
    //此函数与 Unclamped Uniform BSpline不同
    std::vector<Eigen::Vector3d> GetCPsFromInitialState(Eigen::Vector3d pos_start,Eigen::Vector3d vel_start,Eigen::Vector3d acc_start);
    std::vector<Eigen::Vector3d> GetCPsFromFinalState(Eigen::Vector3d pos_goal,Eigen::Vector3d vel_goal,Eigen::Vector3d acc_goal);
    std::vector<Eigen::Vector3d> GetCPsFromQn_2FinalState(Eigen::Vector3d qn_2,Eigen::Vector3d vel_goal,Eigen::Vector3d acc_goal);

    //时间间隔调整 模块
    void AdjustDeltaT(Eigen::Vector3d pos_start,Eigen::Vector3d vel_start,Eigen::Vector3d acc_start);
    double cubic(double b,double c,double d);
    int quartic(double b,double c,double d,double e,double* ans);


    vector<Eigen::Vector3d> NodeExpansion(const NodePtr& q_cur, const double step);

    //调整步长，使其满足加速度约束
    double AdjustStep(double q0,double q1,double q2,double step);
    //安全扩张节点，同时满足加速度和速度约束
    vector<Eigen::Vector3d> SafeNodeExpansion(const vector<Eigen::Vector3d>& LocalCPs,const NodePtr& q_cur, const double step);

    //得到的局部控制点次序 qi-p,...,qi-1 共p个
    vector<Eigen::Vector3d> RetrieveLocalCPs(const NodePtr& q_cur); 
    bool CheckDynamic(const vector<Eigen::Vector3d>& LocalCPs,const Eigen::Vector3d& q_cur );
    //1.reach goal:  end_node->qn-2  2.reach horizon: end_node->qn-2
    void retrievePartCPs(NodePtr end_node);
    //查询最终的搜索结果searched_ctrl_nodes_，获取q0->qn-2 控制点的坐标
    std::vector<Eigen::Vector3d> getPartCPs();
};