#include"path_search/uu_ctrl_astar_four.h"

UUCtrlAstar_Four::~UUCtrlAstar_Four()
{
    for (int i = 0; i < allocate_num_; i++) delete ctrl_node_pool_[i];
}

void UUCtrlAstar_Four::initSearch(ros::NodeHandle& nh,const SWINMap::Ptr& map,const PlanningVisualization::Ptr& visualization)
{
    nh.param("uu_ctrl_astar_four/resolution_astar", resolution_, -1.0);
    nh.param("uu_ctrl_astar_four/lambda_heu", lambda_heu_, -1.0);
    nh.param("uu_ctrl_astar_four/margin", margin_, -1.0);
    nh.param("uu_ctrl_astar_four/allocate_num", allocate_num_, -1);
    nh.param("uu_ctrl_astar_four/degree",degree_,4);
    nh.param("uu_ctrl_astar_four/max_vel",max_vel_,1.0);
    nh.param("uu_ctrl_astar_four/max_acc",max_acc_,1.0);
    nh.param("uu_ctrl_astar_four/delta_t",delta_t_,1.0);
    tie_breaker_ = 1.0 + 1.0 / 1000;

    /* ---------- map params ---------- */
    this->visualization_=visualization;
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

double UUCtrlAstar_Four::GetStep(const double dist)
{
    double mini=0.2*delta_t_;
        if(dist<=margin_) return mini;
        else if(dist>margin_&&dist<=(margin_+max_vel_*delta_t_))
        {
            double k=(max_vel_-mini)/max_vel_*delta_t_;
            double b=mini-margin_*k;
            return (k*dist+b);
        }
        else return (max_vel_*delta_t_);
}

void UUCtrlAstar_Four::reset()
{
    expanded_nodes_.clear();
    searched_ctrl_nodes_.clear();

    std::priority_queue<NodePtr, std::vector<NodePtr>, NodeComparator0> empty_queue;
    open_set_.swap(empty_queue);

    for (int i = 0; i < use_node_num_; i++) {
    NodePtr node = ctrl_node_pool_[i];
    node->parent = NULL;
    node->node_state = NOT_EXPAND;
    }

    use_node_num_ = 0;
    iter_num_ = 0;
}

//函数有改变
Eigen::MatrixXd UUCtrlAstar_Four::search(Eigen::Vector3d pos_start,Eigen::Vector3d vel_start,Eigen::Vector3d acc_start,
            Eigen::Vector3d pos_goal,Eigen::Vector3d vel_goal,Eigen::Vector3d acc_goal)
{
    // std::cout << "最初地图版本:" << swin_map_->version_ << std::endl;
    /*建立变量*/
    Eigen::Vector3d qn_3_goal; //以qn-3控制点为终点
    NodePtr cur_node;
    std::vector<Eigen::Vector3d> path;

    if((pos_start-pos_goal).norm()<0.4) 
    {
        std::cout << "终点距离起点太近!"<<std::endl;
        return Vectors2matrix(path);
    }
    /*由初始状态确定时间间隔*/
    // AdjustDeltaT(pos_start,vel_start,acc_start);

    /*由最终状态确定最后的p个控制点    以qn-3控制点为终点*/
    std::vector<Eigen::Vector3d> qn_3qn_2qn_1qn;
    qn_3qn_2qn_1qn= GetCPsFromFinalState(pos_goal,vel_goal,acc_goal);
    qn_3_goal=qn_3qn_2qn_1qn[0];
    //  std::cout << "qn_2_goal:"<<qn_2_goal.transpose()<<std::endl;


    /*由初始状态确定最初的p个控制点    以第p个控制点为起点*/
    std::vector<Eigen::Vector3d> q0q1q2q3;
    q0q1q2q3= GetCPsFromInitialState(pos_start,vel_start,acc_start);

    visualization_->drawSearchStart(q0q1q2q3[3],0.4,Eigen::Vector4d(1,0.5,0,1));

    for(int i=0;i<degree_-1;i++)
    {
        cur_node = ctrl_node_pool_[use_node_num_++];
        if(i==0)  cur_node->parent=NULL;
        else  cur_node->parent=ctrl_node_pool_[(use_node_num_-2)];
        cur_node->step=-1.0;
        cur_node->position=q0q1q2q3[i];
        cur_node->grid= swin_map_->WorldToGrid(cur_node->position);
        cur_node->node_state=IN_CLOSE_SET;
        cur_node->time_score=0.0;
        expanded_nodes_.insert(cur_node->position,cur_node);
        // expanded_nodes_.insert(cur_node->grid,cur_node);
    }

    cur_node = ctrl_node_pool_[use_node_num_++];
    cur_node->parent=ctrl_node_pool_[(use_node_num_-2)];
    cur_node->step=-1.0;
    cur_node->position=q0q1q2q3[3];
    cur_node->grid= swin_map_->WorldToGrid(cur_node->position);
    cur_node->node_state=IN_OPEN_SET;
    cur_node->time_score=0.0;
    cur_node->g_score=0.0;
    cur_node->f_score=lambda_heu_*getDiagHeu(cur_node->position,qn_3_goal);                //getEuclHeu(cur_node->position,qn_2_goal);    
    open_set_.push(cur_node);
     expanded_nodes_.insert(cur_node->position,cur_node);
    // expanded_nodes_.insert(cur_node->grid,cur_node);


    /*开始大循环*/
    while(!open_set_.empty())
    {
        cur_node = open_set_.top();
        /* ---------- pop node and add to close set ---------- */
        open_set_.pop();
        cur_node->node_state = IN_CLOSE_SET;
        iter_num_ += 1;

        //得到局部曲线的前degree_个控制点
        std::vector<Eigen::Vector3d> LocalCPs;
        LocalCPs=RetrieveLocalCPs(cur_node);
        
       //计算步长,主要是考虑起点q2是不是位于地图内
    //    if( swin_map_->isInMap(cur_node->position) == false )    
    //    {
    //      std::cout << "起点q2 不在地图内!其数值为:"<<q0q1q2[2].transpose() <<std::endl;
    //     continue;
    //    }
        double step = GetStep(swin_map_->getDistance(cur_node->grid));

        //  std::cout << "step:"<<step<<std::endl;
        // std::cout << "LocalCPs0:"<<LocalCPs[0].transpose()<<std::endl;
        // std::cout << "LocalCPs1:"<<LocalCPs[1].transpose() <<std::endl;
        // std::cout << "LocalCPs2:"<<LocalCPs[2].transpose() <<std::endl;


        //到达终点  将新建qn-3为最终点，这种写法目的是和下文保持一致
        if((cur_node->position-qn_3_goal).norm()<=step&&CheckDynamic(LocalCPs,qn_3_goal))
        {
            NodePtr end_node=new Node;
            end_node->parent=cur_node;
            end_node->step=step;
            end_node->position=qn_3qn_2qn_1qn[0];
            end_node->grid= swin_map_->WorldToGrid(qn_3qn_2qn_1qn[0]);
            
            retrievePartCPs(end_node);
            path=getPartCPs();
            for(int i=1;i<qn_3qn_2qn_1qn.size();i++)  path.push_back(qn_3qn_2qn_1qn[i]);  //补齐qn-2,qn-1 和qn

            return   Vectors2matrix(path);
        }

        //搜索已达到一定程度,将当前点作为qn-2 ，以最终速度，加速度为零 直接结束
        // if(cur_node->g_score>=5*delta_t_)
        if((cur_node->position-pos_start).norm()>=4)
        // if(cur_node->time_score>=std::min((6-(q0q1q2[2]-pos_start).norm())/max_vel_,(pos_start-pos_goal).norm()/max_vel_))
        {   
            retrievePartCPs(cur_node);
            path=getPartCPs();
            for(int i=0;i<degree_-1;i++)            path.push_back(path.back());//补齐qn-2,qn-1 和qn
            // std::cout << "use node num: " << use_node_num_ << std::endl;
            return Vectors2matrix(path);
        }

        //如果步长为0，直接跳过
        // if(step==0) continue;
       
       //扩展该节点
       std::vector<Eigen::Vector3d> q_nbrs;
        q_nbrs=NodeExpansion(cur_node,step);
        // q_nbrs=SafeNodeExpansion(LocalCPs,cur_node,step);
        for(int i=0;i<q_nbrs.size();i++)
        {
            Eigen::Vector3d q_nbr = q_nbrs[i];

            //如果扩展点Z轴过低，则跳过
            // if(q_nbr(2)<0.3) continue;

            //如果不在地图内，则跳过
            if( swin_map_->isInMap(q_nbr) == false )    continue;

            //如果邻居点离障碍物太近，则跳过
            if(swin_map_->getDistance(q_nbr)<=margin_)  continue;

            //如果不满足动力学约束，则跳过
            if(CheckDynamic(LocalCPs,q_nbr) == false)   continue;

            //如果在close_set内，则跳过
            Eigen::Vector3i temp_id = swin_map_->WorldToGrid(q_nbr);
            // NodePtr temp_node = expanded_nodes_.find(temp_id);
            NodePtr temp_node = expanded_nodes_.find(q_nbr);
            
            if (temp_node != NULL && temp_node->node_state == IN_CLOSE_SET)  continue;

            //计算代价
            // double temp_g_score = cur_node->g_score+delta_t_;
            double temp_time_score = cur_node->time_score+delta_t_;
            double temp_g_score = cur_node->g_score+(q_nbr-cur_node->position).norm();
            double temp_f_score = temp_g_score+lambda_heu_*getEuclHeu(q_nbr,qn_3_goal);   // getDiagHeu(q_nbr,qn_2_goal)
            //如果该点未在open_set中，直接加入
            if(temp_node==NULL)
            {
                temp_node = ctrl_node_pool_[use_node_num_++];
                temp_node->position = q_nbr;
                temp_node->grid = temp_id;
                temp_node->time_score=temp_time_score;
                temp_node->f_score = temp_f_score;
                temp_node->g_score = temp_g_score;
                temp_node->parent = cur_node;
                temp_node->step=step;
                temp_node->node_state = IN_OPEN_SET;
                open_set_.push(temp_node);
                expanded_nodes_.insert(temp_node->position,temp_node);
                // expanded_nodes_.insert(temp_node->grid,temp_node);

               if (use_node_num_ == allocate_num_)
                {
                    cout << "run out of memory." << endl;
                    return   Vectors2matrix(path);
                 }

            } 
            //如果该点在open_set中
            else if (temp_node->node_state == IN_OPEN_SET) 
            {
                if (temp_g_score < temp_node->g_score) 
                {
                    temp_node->position = q_nbr;
                    temp_node->time_score=temp_time_score;
                    temp_node->f_score = temp_f_score;
                    temp_node->g_score = temp_g_score;
                    temp_node->parent = cur_node;
                    temp_node->step=step;
                }
            }else  std::cout << "error type in searching: " << temp_node->node_state << std::endl;

        }
    }

  /* ---------- open set empty, no path ---------- */
  std::cout << "open set empty, 搜索失败"<< std::endl;
  std::cout << "use node num: " << use_node_num_ << std::endl;
  std::cout << "iter num: " << iter_num_ << std::endl;

  std::cout << "起点q2 为:"<<q0q1q2q3[3].transpose() <<std::endl;
//   std::cout << "起点q2 的邻居为:"<<q_nbrs.size() <<std::endl;
  return Vectors2matrix(path);
}


//相较于3次曲线，函数有改变,可以通用
Eigen::MatrixXd UUCtrlAstar_Four::RetrieveCtrlNodeStep()
{
  vector<double> steps;
  for (int i = 0; i < searched_ctrl_nodes_.size(); ++i) {
    steps.push_back(searched_ctrl_nodes_[i]->step);
  }
  //补齐qn-2,qn-1 和qn
  for(int i = 0; i < degree_-1; i++)
  {
      steps.push_back(-1.0);
  }
//   steps.push_back(-1.0);
//   steps.push_back(-1.0);
//   steps.push_back(-1.0);
 
  return Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> >(steps.data(),1,steps.size());
}

//Closed form solution,函数有改变
std::vector<Eigen::Vector3d> UUCtrlAstar_Four::GetCPsFromInitialState(Eigen::Vector3d pos_start,Eigen::Vector3d vel_start,Eigen::Vector3d acc_start)
{
    if(degree_==4)
    {
     std::vector<Eigen::Vector3d> q0q1q2q3;

     Eigen::Vector3d q0,q1,q2,q3;

     q0=11*acc_start*delta_t_*delta_t_/6+pos_start;
     q0q1q2q3.push_back(q0);

     q1=pos_start-vel_start*delta_t_-7*(acc_start*delta_t_*delta_t_)/18;
     q0q1q2q3.push_back(q1);

     q2=2*acc_start*delta_t_*delta_t_/9+vel_start*delta_t_+pos_start;
     q0q1q2q3.push_back(q2);

     q3=pos_start;
     q0q1q2q3.push_back(q3);

    std::cout<<"起点 ESDF:"<<swin_map_->getDistance(pos_start)<<std::endl;
    for(auto i:q0q1q2q3)  std::cout<<i.transpose()<<std::endl;
    
     return q0q1q2q3;
    }
    else if(degree_==5)
    {
     std::vector<Eigen::Vector3d> q0q1q2q3q4;

     Eigen::Vector3d q0,q1,q2,q3,q4;

     q0=55*acc_start*delta_t_*delta_t_/6+16*vel_start*delta_t_+pos_start;
     q0q1q2q3q4.push_back(q0);

     q1=pos_start-4*vel_start*delta_t_-11*(acc_start*delta_t_*delta_t_)/12;
     q0q1q2q3q4.push_back(q1);

     q2=2*acc_start*delta_t_*delta_t_/9+4*vel_start*delta_t_/3+pos_start;
     q0q1q2q3q4.push_back(q2);

     q3=pos_start;
     q0q1q2q3q4.push_back(q3);

     q4=pos_start;
     q0q1q2q3q4.push_back(q4);

     for(auto i:q0q1q2q3q4)
            std::cout<<i.transpose()<<std::endl;

     return q0q1q2q3q4;
    }
}
//函数有改变
std::vector<Eigen::Vector3d> UUCtrlAstar_Four::GetCPsFromFinalState(Eigen::Vector3d pos_goal,Eigen::Vector3d vel_goal,Eigen::Vector3d acc_goal)
{
    if(degree_==4)
    {
    std::vector<Eigen::Vector3d> qn_3qn_2qn_1qn;

    Eigen::Vector3d qn_3,qn_2,qn_1,qn;


    qn_3=11*acc_goal*delta_t_*delta_t_/6+pos_goal;
    qn_2=pos_goal-vel_goal*delta_t_-7*(acc_goal*delta_t_*delta_t_)/18;
    qn_1=2*acc_goal*delta_t_*delta_t_/9+vel_goal*delta_t_+pos_goal;
    qn=pos_goal;

    qn_3qn_2qn_1qn.push_back(qn_3);
    qn_3qn_2qn_1qn.push_back(qn_2);
    qn_3qn_2qn_1qn.push_back(qn_1);
    qn_3qn_2qn_1qn.push_back(qn);
    return qn_3qn_2qn_1qn;


    }
    else if(degree_==5)
    {
     std::vector<Eigen::Vector3d> qn_4qn_3qn_2qn_1qn;
     Eigen::Vector3d qn_4,qn_3,qn_2,qn_1,qn;

     qn_4=55*acc_goal*delta_t_*delta_t_/6+16*vel_goal*delta_t_+pos_goal;
     qn_4qn_3qn_2qn_1qn.push_back(qn_4);

     qn_3=pos_goal-4*vel_goal*delta_t_-11*(acc_goal*delta_t_*delta_t_)/12;
     qn_4qn_3qn_2qn_1qn.push_back(qn_3);

     qn_2=2*acc_goal*delta_t_*delta_t_/9+4*vel_goal*delta_t_/3+pos_goal;
     qn_4qn_3qn_2qn_1qn.push_back(qn_2);

     qn_1=pos_goal;
     qn_4qn_3qn_2qn_1qn.push_back(qn_1);

     qn=pos_goal;
     qn_4qn_3qn_2qn_1qn.push_back(qn);

     return qn_4qn_3qn_2qn_1qn;
    }
}

vector<Eigen::Vector3d> UUCtrlAstar_Four::NodeExpansion(const NodePtr& q_cur, const double step)
{
    Eigen::Vector3d current_pos=q_cur->position;

    std::vector<Eigen::Vector3d> nbrs;
    Eigen::Vector3d nbr(0, 0, 0);

    //保守扩展，防止优化时初始解越界
    double  dir_0_0 = step*0.9;
    double  dir_45_0 = (step/sqrt(2))*0.9;
    double  dir_45_45 = (step/sqrt(3))*0.9;

    for (int i = -1; i < 2; ++i) {
        for (int j = -1; j < 2; ++j) {
            for (int k = -1; k < 2; ++k) {
                switch (abs(i) + abs(j) + abs(k)) {
                case 0:continue;
                case 1:
                    nbr(0) = current_pos(0) + i * dir_0_0;
                    nbr(1) = current_pos(1) + j * dir_0_0;
                    nbr(2) = current_pos(2) + k * dir_0_0;
                    break;
                case 2:
                    nbr(0) = current_pos(0) + i * dir_45_0;
                    nbr(1) = current_pos(1) + j * dir_45_0;
                    nbr(2) = current_pos(2) + k * dir_45_0;
                    break;
                case 3:
                    nbr(0) = current_pos(0) + i * dir_45_45;
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

vector<Eigen::Vector3d> UUCtrlAstar_Four::RetrieveLocalCPs(const NodePtr& q_cur)
{
    vector<Eigen::Vector3d> localCPs;
    NodePtr q_temp=q_cur;
    
    for(int i=0; i<degree_;i++)
    {
        localCPs.push_back(q_temp->position);
        if(i<degree_-1) q_temp=q_temp->parent;
    }
    reverse(localCPs.begin(),localCPs.end());
    return localCPs;
}

//函数有改变,可以通用
bool UUCtrlAstar_Four::CheckDynamic(const vector<Eigen::Vector3d>& LocalCPs,const Eigen::Vector3d& q_cur )
{
    vector<Eigen::Vector3d> localCPs=LocalCPs;
    localCPs.push_back(q_cur);

    Eigen::MatrixXd temp_mat;
    temp_mat=Vectors2matrix(localCPs);
    UnclampedUniformBspline pos(temp_mat,degree_,delta_t_);

    Eigen::MatrixXd vel_ctrl_bs,acc_ctrl_bs;
    Eigen::MatrixXd vel_ctrl_mv;
    vel_ctrl_bs=pos.getDerivative().getControlPoint();
    // vel_ctrl_mv=vel_ctrl_bs*M.vel_bs2mv_;     
    acc_ctrl_bs=pos.getDerivative().getDerivative().getControlPoint();


    if( acc_ctrl_bs.col(degree_-2).maxCoeff() > max_acc_ ||acc_ctrl_bs.col(degree_-2).minCoeff() < -max_acc_) 
    {
        // std::cout << "[加速度] 动力学检查失败"<< std::endl;
        //  std::cout << "max_acc_:"<<max_acc_<<"min_acc_:"<<-max_acc_<< std::endl;
        // std::cout << "acc_ctrl_bs.col(1).maxCoeff()"<< acc_ctrl_bs.col(1).maxCoeff()<<"acc_ctrl_bs.col(1).minCoeff()"<<acc_ctrl_bs.col(1).minCoeff()<<std::endl;
        
        // for (auto i: localCPs) {
        //     std::cout << i .transpose()<<std::endl; ; 
        // }
        
        return false;//Not dynamically feasible
    }

    if (vel_ctrl_bs.col(degree_-1).maxCoeff() > max_vel_ ||vel_ctrl_bs.col(degree_-1).minCoeff() < -max_vel_ ) 
    {
        // std::cout << "[速度] 动力学检查失败"<< std::endl;
        // std::cout << "max_acc_:"<<max_vel_<<"min_acc_:"<<-max_vel_<< std::endl;
        // std::cout << "vel_ctrl_bs.col(2).maxCoeff()"<< vel_ctrl_bs.col(2).maxCoeff()<<",vel_ctrl_bs.col(2).minCoeff()"<<vel_ctrl_bs.col(2).minCoeff()<<std::endl;
        return false;//Not dynamically feasible
    }

    // if (vel_ctrl_mv.maxCoeff() > max_vel_ ||vel_ctrl_mv.minCoeff() < -max_vel_ ) 
    // {
    //     std::cout << "[速度] 动力学检查失败"<< std::endl;
    //     return false;//Not dynamically feasible
    // }


  return true;//Dynamically feasible
}

void UUCtrlAstar_Four::retrievePartCPs(NodePtr end_node)
{
  NodePtr cur_node = end_node;
  searched_ctrl_nodes_.push_back(cur_node);

  while (cur_node->parent != NULL) {
    cur_node = cur_node->parent;
    searched_ctrl_nodes_.push_back(cur_node);
  }

  reverse(searched_ctrl_nodes_.begin(), searched_ctrl_nodes_.end());
}

std::vector<Eigen::Vector3d> UUCtrlAstar_Four::getPartCPs() {
  vector<Eigen::Vector3d> path;
  for (int i = 0; i < searched_ctrl_nodes_.size(); ++i) {
    path.push_back(searched_ctrl_nodes_[i]->position);
  }
  return path;
}

Eigen::MatrixXd UUCtrlAstar_Four::Vectors2matrix(const vector<Eigen::Vector3d>& vec)
{

    Eigen::MatrixXd mat= Eigen::MatrixXd::Zero(3,vec.size());

    for (int i = 0; i < vec.size(); i++) 
    {
        for (int j = 0; j < 3; ++j)     mat(j,i)=vec[i][j];
    }

    return mat;

}

double UUCtrlAstar_Four::getDiagHeu(Eigen::Vector3d x1, Eigen::Vector3d x2)
{
  double heu = (x1 - x2).array().abs().sum() +  resolution_*(sqrt(3) - 3) * (x1 - x2).array().abs().minCoeff();
//   heu=heu/max_vel_;
  return tie_breaker_ * heu;
}

double UUCtrlAstar_Four::getEuclHeu(Eigen::Vector3d x1, Eigen::Vector3d x2)
{
     return tie_breaker_ * (x2 - x1).norm();
}

int UUCtrlAstar_Four::GetDegree()
{
    return degree_;
}

double UUCtrlAstar_Four::GetDelta_t()
{
    return delta_t_;
}

std::vector<Eigen::Vector3d>  UUCtrlAstar_Four::getVisitedNodes() 
{
    std::vector<NodePtr> visited;
    visited.assign(ctrl_node_pool_.begin(), ctrl_node_pool_.begin() + use_node_num_ - 1);

    std::vector<Eigen::Vector3d> visited_node;
    for (int i = 0; i < visited.size(); i++) {
        visited_node.push_back(visited[i]->position);
    }
    return visited_node;
}

std::vector<std::vector<Eigen::Vector3d>>  UUCtrlAstar_Four::getVisitedNodes_TimeKind() 
{
    std::vector<NodePtr> visited;
    visited.assign(ctrl_node_pool_.begin(), ctrl_node_pool_.begin() + use_node_num_ - 1);

    std::vector<std::vector<Eigen::Vector3d>>  visited_node_timekind(5);
    for (int i = 0; i < visited.size(); i++) 
    {
        if(visited[i]->time_score==0.0)
        {
            visited_node_timekind[0].push_back(visited[i]->position);
        }
        if(visited[i]->time_score==1.0*delta_t_)
        {
            visited_node_timekind[1].push_back(visited[i]->position);
        }
        else if(visited[i]->time_score==2.0*delta_t_)
        {
            visited_node_timekind[2].push_back(visited[i]->position);
        }
        else if(visited[i]->time_score==3.0*delta_t_)
        {
             visited_node_timekind[3].push_back(visited[i]->position);
        }
        else 
             visited_node_timekind[4].push_back(visited[i]->position);
    }
    return visited_node_timekind;
}






