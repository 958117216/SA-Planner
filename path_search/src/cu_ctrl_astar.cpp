#include "path_search/cu_ctrl_astar.h"


void CUCtrlAstar::initSearch(ros::NodeHandle& nh,const SWINMap::Ptr& map)
{
    nh.param("cu_ctrl_astar/resolution_astar", resolution_, -1.0);
    nh.param("cu_ctrl_astar/lambda_heu", lambda_heu_, -1.0);
    nh.param("cu_ctrl_astar/margin", margin_, -1.0);
    nh.param("cu_ctrl_astar/allocate_num", allocate_num_, -1);
    nh.param("cu_ctrl_astar/degree",degree_,3);
    nh.param("cu_ctrl_astar/max_vel",max_vel_,1.0);
    nh.param("cu_ctrl_astar/max_acc",max_acc_,1.0);
    nh.param("cu_ctrl_astar/delta_t",delta_t_,1.0);

    /* ---------- map params ---------- */
    this->swin_map_ = map;
    // resolution_=swin_map_->GetResolution();

    //   edt_environment_->getMapRegion(origin_, end_);

    //   cout << "origin_: " << origin_.transpose() << endl;
    //   cout << "map end_: " << end_.transpose() << endl;

    /* ---------- pre-allocated node ---------- */
    ctrl_node_pool_.resize(allocate_num_);
    for (int i = 0; i < allocate_num_; i++) {
    ctrl_node_pool_[i] = new Node;
    }

    use_node_num_ = 0;
    iter_num_ = 0;
    cout << "CtrlA* 初始化完成 " << endl;
}

double CUCtrlAstar::GetStep(const double dist)
{
        if(dist<=margin_) return 0;
        else if(dist>margin_&&dist<=(margin_+max_vel_*delta_t_)) return (dist-margin_);
                  else return (max_vel_*delta_t_);
}

std::vector<Eigen::Vector3d> CUCtrlAstar::search(Eigen::Vector3d pos_start,Eigen::Vector3d vel_start,Eigen::Vector3d acc_start,
            Eigen::Vector3d pos_goal,Eigen::Vector3d vel_goal,Eigen::Vector3d acc_goal)
{

    /*建立变量*/
    Eigen::Vector3d qn_2_goal; //以qn-2控制点为终点
    NodePtr cur_node;
    std::vector<Eigen::Vector3d> path;


    if((pos_start-pos_goal).norm()<0.4) 
    {
        std::cout << "终点距离起点太近!"<<std::endl;
        return path;
    }

    /*由最终状态确定最后的p个控制点    以qn-2控制点为终点*/
    std::vector<Eigen::Vector3d> qn_2qn_1qn;
    qn_2qn_1qn= GetCPsFromFinalState(pos_goal,vel_goal,acc_goal);
    qn_2_goal=qn_2qn_1qn[0];

    /*由初始状态确定最初的p个控制点    以第p个控制点为起点*/
    std::vector<Eigen::Vector3d> q0q1q2;
    q0q1q2= GetCPsFromInitialState(pos_start,vel_start,acc_start);

    for(int i=0;i<degree_-1;i++)
    {
        cur_node = ctrl_node_pool_[use_node_num_++];
        if(i==0)  cur_node->parent=NULL;
        else  cur_node->parent=ctrl_node_pool_[(use_node_num_-2)];
        cur_node->position=q0q1q2[i];
        cur_node->grid= swin_map_->WorldToGrid(cur_node->position);
        cur_node->node_state=IN_CLOSE_SET;
        expanded_nodes_.insert(cur_node->grid,cur_node);
    }

    cur_node = ctrl_node_pool_[use_node_num_++];
    cur_node->parent=ctrl_node_pool_[(use_node_num_-2)];
    cur_node->position=q0q1q2[2];
    cur_node->grid= swin_map_->WorldToGrid(cur_node->position);
    cur_node->node_state=IN_OPEN_SET;
    cur_node->g_score=0.0;
    cur_node->f_score=lambda_heu_*getDiagHeu(cur_node->position,qn_2_goal);
    open_set_.push(cur_node);
    expanded_nodes_.insert(cur_node->grid,cur_node);



    /*开始大循环*/
    while(!open_set_.empty())
    {
        cur_node = open_set_.top();
        /* ---------- pop node and add to close set ---------- */
        open_set_.pop();
        cur_node->node_state = IN_CLOSE_SET;
        iter_num_ += 1;

        std::vector<Eigen::Vector3d> LocalCPs;
        LocalCPs=RetrieveLocalCPs(cur_node);
        //计算步长
        double step = GetStep(swin_map_->getDistance(cur_node->grid));

        //到达终点
        if((cur_node->position-qn_2_goal).norm()<=step&&CheckDynamic(LocalCPs,qn_2_goal))
        {
            retrievePartCPs(cur_node);
            path=getPartCPs();
            for(int i=0;i<qn_2qn_1qn.size();i++)  path.push_back(qn_2qn_1qn[i]);
            return path;
        }
        //搜索已达到一定程度
        if(cur_node->g_score>=4*delta_t_)
        {
            retrievePartCPs(cur_node);
            path=getPartCPs();
            path.push_back(path.back());
            path.push_back(path.back());
            return path;
        }

        //如果步长为0，直接跳过
        if(step==0) continue;
       
       std::vector<Eigen::Vector3d> q_nbrs;
       q_nbrs=NodeExpansion(cur_node,step);
    
        for(int i=0;i<q_nbrs.size();i++)
        {  
            Eigen::Vector3d q_nbr = q_nbrs[i];
            
            //如果不在地图内，则跳过
            if( swin_map_->isInMap(q_nbr) == false )    continue;
            
            //如果不满足动力学约束，则跳过
            if(CheckDynamic(LocalCPs,q_nbr) == false)   continue;

            //如果在close_set内，则跳过
            Eigen::Vector3i temp_id = swin_map_->WorldToGrid(q_nbr);
            NodePtr temp_node = expanded_nodes_.find(temp_id);

            if (temp_node != NULL && temp_node->node_state == IN_CLOSE_SET)  continue;

            //计算代价
            double temp_g_score = cur_node->g_score+delta_t_;
            double temp_f_score = temp_g_score+lambda_heu_*getDiagHeu(cur_node->position,qn_2_goal);
            //如果该点未在open_set中，直接加入
            if(temp_node==NULL)
            {
                temp_node = ctrl_node_pool_[use_node_num_++];
                temp_node->position = q_nbr;
                temp_node->grid = temp_id;
                temp_node->f_score = temp_f_score;
                temp_node->g_score = temp_g_score;
                temp_node->parent = cur_node;
                temp_node->node_state = IN_OPEN_SET;
                open_set_.push(temp_node);
            } 
            //如果该点在open_set中
            else if (temp_node->node_state == IN_OPEN_SET) 
            {
                if (temp_g_score < temp_node->g_score) 
                {
                    temp_node->position = q_nbr;
                    temp_node->f_score = temp_f_score;
                    temp_node->g_score = temp_g_score;
                    temp_node->parent = cur_node;
                }
            }else  std::cout << "error type in searching: " << temp_node->node_state << std::endl;
        }
    }
  /* ---------- open set empty, no path ---------- */
  std::cout << "open set empty, 搜索失败"<< std::endl;
  std::cout << "use node num: " << use_node_num_ << std::endl;
  std::cout << "iter num: " << iter_num_ << std::endl;
  return path;
}

std::vector<Eigen::Vector3d> CUCtrlAstar::GetCPsFromInitialState(Eigen::Vector3d pos_start,Eigen::Vector3d vel_start,Eigen::Vector3d acc_start)
{
     std::vector<Eigen::Vector3d> q0q1q2;

     Eigen::Vector3d q0,q1,q2;

     q0=pos_start;
     q0q1q2.push_back(q0);

     q1=delta_t_*vel_start+q0;
     q0q1q2.push_back(q1);

     q2=delta_t_*delta_t_*acc_start+2*q1-q0;
     q0q1q2.push_back(q2);

     return q0q1q2;
}

std::vector<Eigen::Vector3d> CUCtrlAstar::GetCPsFromFinalState(Eigen::Vector3d pos_goal,Eigen::Vector3d vel_goal,Eigen::Vector3d acc_goal)
{
    std::vector<Eigen::Vector3d> qn_2qn_1qn;

    Eigen::Vector3d qn_2,qn_1,qn;

    qn=pos_goal;

    qn_1=qn-delta_t_*vel_goal;

    qn_2=delta_t_*delta_t_*acc_goal+2*qn_1-qn;

    qn_2qn_1qn.push_back(qn_2);
    qn_2qn_1qn.push_back(qn_1);
    qn_2qn_1qn.push_back(qn);
    return qn_2qn_1qn;
}

vector<Eigen::Vector3d> CUCtrlAstar::NodeExpansion(const NodePtr& q_cur, const double step)
{
    Eigen::Vector3d current_pos=q_cur->position;

    std::vector<Eigen::Vector3d> nbrs;
    Eigen::Vector3d nbr(0, 0, 0);

    //保守扩展，防止优化时初始解越界
    double  dir_0_0 = step*0.99;
    double  dir_45_0 = (step/sqrt(2))*0.99;
    double  dir_45_45 = (step/sqrt(3))*0.99;

    for (int i = -1; i < 2; ++i) {
        for (int j = -1; j < 2; ++j) {
        for (int k = -1; k < 2; ++k) {
            switch (abs(i) + abs(j) + abs(k)) {
            case 0:continue;
            case 1:nbr(0) = current_pos(0) + i * dir_0_0;
                nbr(1) = current_pos(1) + j * dir_0_0;
                nbr(2) = current_pos(2) + k * dir_0_0;
                break;
            case 2:nbr(0) = current_pos(0) + i * dir_45_0;
                nbr(1) = current_pos(1) + j * dir_45_0;
                nbr(2) = current_pos(2) + k * dir_45_0;
                break;
            case 3:nbr(0) = current_pos(0) + i * dir_45_45;
                nbr(1) = current_pos(1) + j * dir_45_45;
                nbr(2) = current_pos(2) + k * dir_45_45;
                break;
            default:continue;
            }
            nbrs.push_back(nbr);
        }
      }
    }
    return nbrs;
}

vector<Eigen::Vector3d> CUCtrlAstar::RetrieveLocalCPs(const NodePtr& q_cur)
{
    vector<Eigen::Vector3d> localCPs;
    NodePtr q_temp=q_cur;
    
    for(int i=0; i<degree_;i++)
    {
        localCPs.push_back(q_temp->position);
        q_temp=q_temp->parent;
    }
    reverse(localCPs.begin(),localCPs.end());
}

bool CUCtrlAstar::CheckDynamic(const vector<Eigen::Vector3d>& LocalCPs,const Eigen::Vector3d& q_cur )
{
    vector<Eigen::Vector3d> localCPs=LocalCPs;
    localCPs.push_back(q_cur);

    Eigen::MatrixXd temp_mat;
    temp_mat=Vectors2matrix(localCPs);
    
    Eigen::Matrix<double,4,3>  Derivative_1;
    Derivative_1<<
    -1, 0,  0, 
    1, -1, 0,
    0,  1,  -1, 
    0,  0,  1;

    Eigen::Matrix<double,3,2>  Derivative_2;
    Derivative_2<<
    -1, 0,  
    1, -1,
    0,  1;

    Eigen::MatrixXd vel_ctrl,acc_ctrl;
    vel_ctrl=temp_mat*Derivative_1/delta_t_;
    acc_ctrl=vel_ctrl*Derivative_2/delta_t_;

  if (vel_ctrl.maxCoeff() > max_vel_ ||vel_ctrl.minCoeff() < -max_vel_ ||
      acc_ctrl.maxCoeff() > max_acc_ ||acc_ctrl.minCoeff() < -max_acc_) 
    {
        return false;//Not dynamically feasible
    }

  return true;//Dynamically feasible
}

void CUCtrlAstar::retrievePartCPs(NodePtr end_node)
{
  NodePtr cur_node = end_node;
  all_ctrl_nodes_.push_back(cur_node);

  while (cur_node->parent != NULL) {
    cur_node = cur_node->parent;
    all_ctrl_nodes_.push_back(cur_node);
  }

  reverse(all_ctrl_nodes_.begin(), all_ctrl_nodes_.end());
}

std::vector<Eigen::Vector3d> CUCtrlAstar::getPartCPs() {
  vector<Eigen::Vector3d> path;
  for (int i = 0; i < all_ctrl_nodes_.size(); ++i) {
    path.push_back(all_ctrl_nodes_[i]->position);
  }
  return path;
}

Eigen::MatrixXd CUCtrlAstar::Vectors2matrix(const vector<Eigen::Vector3d>& vec)
{

    Eigen::MatrixXd mat= Eigen::MatrixXd::Zero(3,vec.size());

    for (int i = 0; i < vec.size(); i++) 
    {
        for (int j = 0; j < 3; ++j)     mat(j,i)=vec[i][j];
    }

    return mat;

}

double CUCtrlAstar::getDiagHeu(Eigen::Vector3d x1, Eigen::Vector3d x2)
{
  double heu = (x1 - x2).array().abs().sum() + resolution_ * (sqrt(3) - 3) * (x1 - x2).array().abs().minCoeff();
  heu=heu/max_vel_;
  return tie_breaker_ * heu;
}