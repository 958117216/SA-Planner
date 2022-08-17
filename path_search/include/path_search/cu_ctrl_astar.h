/**
 * @file ctrl_astar.h
 * @author Guo Yilin (958117216@qq.com)
 * @brief   Clamped Uniform BSpline控制点 A*算法，其动力学检测没办法换基，弃用
 * @version 0.1
 * @date 2022-02-28
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


#define IN_CLOSE_SET 'a'
#define IN_OPEN_SET 'b'
#define NOT_EXPAND 'c'


class Node {
public:
  /* -------------------- */
  Eigen::Vector3i grid;
  Eigen::Vector3d position;
  double g_score, f_score;
  Node* parent;
  char node_state;

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
class NodeHashTable0 
{
private:
  /* data */
  std::unordered_map<Eigen::Vector3i, NodePtr, matrix_hash0<Eigen::Vector3i>> data_3d_;

public:
  NodeHashTable0(/* args */) {}
  ~NodeHashTable0() {}

  void insert(Eigen::Vector3i idx, NodePtr node) {
    data_3d_.insert(make_pair(idx, node));
  }

  NodePtr find(Eigen::Vector3i idx) {
    auto iter = data_3d_.find(idx);
    return iter == data_3d_.end() ? NULL : iter->second;
  }

  void clear() {
    data_3d_.clear();
  }
};


class CUCtrlAstar
{
public:
    CUCtrlAstar(){};
    ~CUCtrlAstar(){
      for (int i = 0; i < allocate_num_; i++) delete ctrl_node_pool_[i];
    }

    //*初始化
    void initSearch(ros::NodeHandle& nh,const SWINMap::Ptr& map);

     //*主要接口
    std::vector<Eigen::Vector3d> search(Eigen::Vector3d pos_start,Eigen::Vector3d vel_start,Eigen::Vector3d acc_start,
            Eigen::Vector3d pos_goal,Eigen::Vector3d vel_goal,Eigen::Vector3d acc_goal);

    /* heuristic function */
    double getDiagHeu(Eigen::Vector3d x1, Eigen::Vector3d x2);
    double getManhHeu(Eigen::Vector3d x1, Eigen::Vector3d x2);
    double getEuclHeu(Eigen::Vector3d x1, Eigen::Vector3d x2);


    /*useful function*/
    Eigen::MatrixXd  Vectors2matrix(const vector<Eigen::Vector3d>& vec);


  typedef std::shared_ptr<CUCtrlAstar> Ptr;

private:
    vector<NodePtr> ctrl_node_pool_;
    int allocate_num_;
    std::priority_queue<NodePtr, std::vector<NodePtr>, NodeComparator0> open_set_;
    NodeHashTable0 expanded_nodes_;
    std::vector<NodePtr> all_ctrl_nodes_;
    double resolution_;
    int use_node_num_;//指代扩展过的点的数量，初始值为p个
    int  iter_num_;  //指代大循环发生的次数，最小为1次
    double delta_t_;
    double margin_;  //τ 希望离障碍物的距离
    double tie_breaker_;
    double lambda_heu_;
    int degree_; //BSpline曲线次数
    double max_vel_,max_acc_; //
    SWINMap::Ptr swin_map_;

private:
    std::vector<Eigen::Vector3d> GetCPsFromInitialState(Eigen::Vector3d pos_start,Eigen::Vector3d vel_start,Eigen::Vector3d acc_start);
    std::vector<Eigen::Vector3d> GetCPsFromFinalState(Eigen::Vector3d pos_goal,Eigen::Vector3d vel_goal,Eigen::Vector3d acc_goal);
    double GetStep(const double dist);
    vector<Eigen::Vector3d> NodeExpansion(const NodePtr& q_cur, const double step);
    //得到的局部控制点次序 qi-p,...,qi-1 共p个
    vector<Eigen::Vector3d> RetrieveLocalCPs(const NodePtr& q_cur);
    bool CheckDynamic(const vector<Eigen::Vector3d>& LocalCPs,const Eigen::Vector3d& q_cur );
    //1.reach goal:  end_node->qn-3  2.reach horizon: end_node->qn-2
    void retrievePartCPs(NodePtr end_node);
    std::vector<Eigen::Vector3d> getPartCPs();
};