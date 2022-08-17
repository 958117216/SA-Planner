#include"path_search/uu_dirc_ctrl_astar.h"

UUDircCtrlAstar::~UUDircCtrlAstar()
{
    for (int i = 0; i < astar_node_allocate_num_; i++) delete astar_node_pool_[i];
    for (int i = 0; i < ctrl_node_allocate_num_; i++) delete ctrl_node_pool_[i];
}

void UUDircCtrlAstar::initSearch(ros::NodeHandle& nh,const SWINMap::Ptr& map,const PlanningVisualization::Ptr& visualization)
{   
    nh.param("uu_dirc_ctrl_astar/astar_node_allocate_num_", astar_node_allocate_num_, -1);
    nh.param("uu_dirc_ctrl_astar/ctrl_node_allocate_num_", ctrl_node_allocate_num_, -1);
    nh.param("uu_dirc_ctrl_astar/resolution_astar", resolution_, -1.0);
    nh.param("uu_dirc_ctrl_astar/lambda_heu", lambda_heu_, -1.0);
    nh.param("uu_dirc_ctrl_astar/margin", margin_, -1.0);
    nh.param("uu_dirc_ctrl_astar/degree",degree_,3);
    nh.param("uu_dirc_ctrl_astar/max_vel",max_vel_,1.0);
    nh.param("uu_dirc_ctrl_astar/max_acc",max_acc_,1.0);
    nh.param("uu_dirc_ctrl_astar/delta_t",delta_t_initial_,0.5);
    nh.param("uu_dirc_ctrl_astar/TerminationCPNum",TerminationCPNum_,10);
    nh.param("uu_dirc_ctrl_astar/reduced_threshold",reduced_threshold_,2.0);
    nh.param("uu_dirc_ctrl_astar/reduced_vel",reduced_vel_,0.4);
    nh.param("uu_dirc_ctrl_astar/show_rviz",show_rviz_,true);

    tie_breaker_ = 1.0 + 1.0 / 10000;

    /* ---------- module  params ---------- */
    this->visualization_=visualization;
    this->swin_map_ = map;

    /* ---------- pre-allocated node ---------- */
    astar_node_pool_.resize(astar_node_allocate_num_);
    for (int i = 0; i < astar_node_allocate_num_; i++) {
    astar_node_pool_[i] = new AstarNode;
    }


    ctrl_node_pool_.resize(ctrl_node_allocate_num_);
    for (int i = 0; i < ctrl_node_allocate_num_; i++) {
    ctrl_node_pool_[i] = new CtrlNode;
    }


    astar_use_node_num_ = 0;
    ctrl_use_node_num_ = 0;
    iter_num_ = 0;
    cout << "DirectionCtrlA* 初始化完成 " << endl;
}

void UUDircCtrlAstar::reset()
{
    astar_node_expanded_nodes_.clear();
    std::priority_queue<AstarNodePtr, std::vector<AstarNodePtr>, AstarNodeComparator> empty_queue;
    astar_node_open_set_.swap(empty_queue);

    ctrl_node_expanded_nodes_.clear();
    std::priority_queue<CtrlNodePtr, std::vector<CtrlNodePtr>, CtrlNodeComparator> empty_queue0;
    ctrl_node_open_set_.swap(empty_queue0);

    searched_ctrl_nodes_.clear();

    for (int i = 0; i < astar_use_node_num_; i++) {
    AstarNodePtr node = astar_node_pool_[i];
    node->parent = NULL;
    node->node_state = NOT_EXPAND;
    }

    for (int i = 0; i < ctrl_use_node_num_; i++) {
    CtrlNodePtr node = ctrl_node_pool_[i];
    node->parent = NULL;
    node->node_state = NOT_EXPAND;
    }

    astar_use_node_num_ = 0;
    ctrl_use_node_num_ = 0;
    iter_num_ = 0;
}

template <typename T>
double UUDircCtrlAstar::getHeu(T x1,T x2,int type)
{
   double heu =0;


    double dx = fabs(x1(0) - x2(0));
    double dy = fabs(x1(1) - x2(1));
    double dz = fabs(x1(2) - x2(2));

    int diag = min(min(dx, dy), dz);
   switch (type)
   {
       //曼哈顿距离
   case 1: 
       heu = (x1-x2).array().abs().sum();
       break;
        //欧式距离
    case 2:
       heu = (x1-x2).norm();
       break;
       //对角距离
    case 3:
        dx -= diag;
        dy -= diag;
        dz -= diag;

        if (dx < 1e-4) {
        heu = 1.0 * sqrt(3.0) * diag + sqrt(2.0) * min(dy, dz) + 1.0 * abs(dy - dz);
        }
        if (dy < 1e-4) {
        heu = 1.0 * sqrt(3.0) * diag + sqrt(2.0) * min(dx, dz) + 1.0 * abs(dx - dz);
        }
        if (dz < 1e-4) {
        heu = 1.0 * sqrt(3.0) * diag + sqrt(2.0) * min(dx, dy) + 1.0 * abs(dx - dy);
        }
       break;
       //Dijkstra
   default:
       heu = 0;
       break;
   }
    return   tie_breaker_*heu;
}

vector<Eigen::Vector3i> UUDircCtrlAstar::First_AstarGraphSearch(vector<Eigen::Vector3d> pos_start,Eigen::Vector3d pos_goal)
{
    vector<Eigen::Vector3i> AstarPath;

    //确定起点和终点在栅格地图中的位置 
    Eigen::Vector3i start_grid,end_grid;
    start_grid=swin_map_->WorldToGrid(0.1667*pos_start[0]+0.6667*pos_start[1]+0.1667*pos_start[2]);
    end_grid = swin_map_->WorldToGrid(pos_goal);

    AstarNodePtr cur_node;

    for(int i=0;i<degree_-1;i++)
    {
        cur_node = astar_node_pool_[astar_use_node_num_++];
        if(i==0)  cur_node->parent=NULL;
        else  cur_node->parent=astar_node_pool_[(astar_use_node_num_-2)];
        cur_node->grid= swin_map_->WorldToGrid(pos_start[i]);
        cur_node->node_state=IN_CLOSE_SET;
        astar_node_expanded_nodes_.insert(cur_node->grid, cur_node);
    }
    /* ---------- initialize ---------- */
    cur_node = astar_node_pool_[astar_use_node_num_++];
    cur_node->parent = astar_node_pool_[(astar_use_node_num_-2)];
    cur_node->grid = swin_map_->WorldToGrid(pos_start[2]);
    cur_node->g_score = 0.0;
    cur_node->f_score = lambda_heu_ * getHeu(cur_node->grid, end_grid,3); 
    cur_node->node_state = IN_OPEN_SET;

    astar_node_open_set_.push(cur_node);
    astar_node_expanded_nodes_.insert(cur_node->grid, cur_node);

    AstarNodePtr neighbor = NULL;

    //大循环
    while(!astar_node_open_set_.empty())
    {
        cur_node=astar_node_open_set_.top();
        /* ---------- pop node and add to close set ---------- */
        astar_node_open_set_.pop();
        cur_node->node_state = IN_CLOSE_SET;
        iter_num_ += 1;

        //终止条件1
        if(cur_node->grid ==end_grid)
        {
            std::cout << "A*图搜索到达终点"<<std::endl;
            //返回Grid
            AstarPath=GetAstarResults(astar_node_pool_[2],cur_node);
            return AstarPath;

        }
        //终止条件2
        if((cur_node->grid-start_grid).norm()*resolution_>=5.5)
        {
            std::cout << "A*图搜索到达边界"<<std::endl;
            //返回Grid
            AstarPath=GetAstarResults(astar_node_pool_[2],cur_node);
            return AstarPath;
        }


        std::vector<Eigen::Vector3i>  neighborSets;
        std::vector<double>  edgeCostSets;

        AstarGetSucc(cur_node, neighborSets, edgeCostSets);

        for(int i=0;i<neighborSets.size();i++)
        {
            Eigen::Vector3i nbr = neighborSets[i];
            //如果在close_set内，则跳过
            AstarNodePtr temp_node = astar_node_expanded_nodes_.find(nbr);

            if (temp_node != NULL && temp_node->node_state == IN_CLOSE_SET)  continue;

            //计算代价
            // double vel_g= (cur_node->grid-cur_node->parent->parent->grid).dot((nbr-cur_node->grid).normalized()) / (2*delta_t_);
            double temp_g_score = cur_node->g_score+edgeCostSets[i];
            double temp_f_score = temp_g_score + lambda_heu_ * getHeu(nbr, end_grid,3); // getDiagHeu(q_nbr,qn_2_goal)
            //如果该点未在open_set中，直接加入
            if(temp_node==NULL)
            {
                temp_node = astar_node_pool_[astar_use_node_num_++];
                temp_node->position=swin_map_->GridToWorld(nbr);
                temp_node->grid = nbr;
                temp_node->f_score = temp_f_score;
                temp_node->g_score = temp_g_score;
                temp_node->parent = cur_node;
                temp_node->node_state = IN_OPEN_SET;
                astar_node_open_set_.push(temp_node);
                astar_node_expanded_nodes_.insert(temp_node->grid,temp_node);

                if (astar_use_node_num_ ==astar_node_allocate_num_)
                {
                    cout << "run out of memory." << endl;
                    return  AstarPath;
                }
            } 
            //如果该点在open_set中
            else if (temp_node->node_state == IN_OPEN_SET) 
            {
                if (temp_g_score < temp_node->g_score) 
                {
                    temp_node->parent = cur_node;//认贼作父
                    temp_node->f_score = temp_f_score;
                    temp_node->g_score = temp_g_score;
                }
            }else  std::cout << "error type in searching: " << temp_node->node_state << std::endl;

        }
    }

    /* ---------- open set empty, no path ---------- */
    cout << "first A* open set empty, no path!" << endl;
    cout << "first A*  use node num: " << astar_use_node_num_ << endl;
    cout << "first A* iter num: " << iter_num_ << endl;
    return AstarPath;
}

vector<Eigen::Vector3i> UUDircCtrlAstar::GetAstarResults(AstarNodePtr first_node,AstarNodePtr last_node)
{

    AstarNodePtr cur_node = last_node;
    vector<Eigen::Vector3i>  AstarResults;
    while (cur_node->parent != first_node)
    {
        AstarResults.push_back(cur_node->grid);
        cur_node = cur_node->parent;
    }
    AstarResults.push_back(cur_node->grid);//存起点

    reverse(AstarResults.begin(), AstarResults.end());//将乾坤逆转吧
    return AstarResults;
}

 void UUDircCtrlAstar::AstarGetSucc(AstarNodePtr currentPtr, vector<Eigen::Vector3i> & neighborSets, vector<double> & edgeCostSets)
{   
    neighborSets.clear();
    edgeCostSets.clear();

   //入口参数检测
    if(currentPtr == nullptr)  std::cout << "Error: Current pointer is null!" << endl;


    Eigen::Vector3i thisNode = currentPtr -> grid;
    Eigen::Vector3i offset;
    Eigen::Vector3i NeighborNode;

    for(int i = -1;i <= 1;++i ){
        for(int j = -1;j <= 1;++j ){
            for(int k = -1;k <= 1;++k){

                // 当前节点 continue
                if( i == 0 && j == 0 && k == 0) continue; 

                offset<<i,j,k;
                NeighborNode=thisNode+offset;

                // 邻居节点不在地图内 continue
                if( swin_map_->isInMap(NeighborNode)== false) 
                {
                    // std::cout << "邻居节点不在地图内 "<<NeighborNode.transpose()<<std::endl;
                    continue; 
                }
                //如果邻居点离障碍物太近，则跳过
                if(swin_map_->getDistance(NeighborNode)<=margin_)
                {
                    // std::cout << "如果邻居点离障碍物太近，则跳过 "<<std::endl;
                    continue; 
                }

                neighborSets.push_back(NeighborNode); 
                edgeCostSets.push_back(offset.norm()); 
            }
        }
    }
}

vector<Eigen::Vector3d> UUDircCtrlAstar::Second_ExtractAstarPath(vector<Eigen::Vector3i>& AstarPath)
{
    vector<Eigen::Vector3d> EasyAstarPath;
    Eigen::Vector3d temp;
    swin_map_->GridToWorld(AstarPath.front(),temp);
    EasyAstarPath.push_back(temp);

    for(int i=1;i<AstarPath.size()-1;i++)
    {
        if( (AstarPath[i]-AstarPath[i-1]) != (AstarPath[i+1]-AstarPath[i]) )
        {
            swin_map_->GridToWorld(AstarPath[i],temp);
            if(EasyAstarPath.empty())   EasyAstarPath.push_back(temp);
            else if((temp-EasyAstarPath.back()).norm()<0.4)
            {
                EasyAstarPath.back()=(temp+EasyAstarPath.back())/2;
            }
            else  EasyAstarPath.push_back(temp);
        }
    }
    swin_map_->GridToWorld(AstarPath.back(),temp);
    EasyAstarPath.push_back(temp);

    return EasyAstarPath;
}

vector<CtrlNodePtr> UUDircCtrlAstar::Third_Directional_Search(const int stage,vector<CtrlNodePtr>& q0q1q2_stage,const Eigen::Vector3d& goal_next,bool final_goal)
{
    //清空上一阶段搜索的残留
    ctrlSearchReset();
    // std::cout << "该阶段的起点:"<<q0q1q2_stage[degree_-1]->position.transpose()<<std::endl;       
    // std::cout << "该阶段的终点:"<<goal_next.transpose()<<std::endl;       
    //最终结果
    std::vector<CtrlNodePtr> stage_result;

    CtrlNodePtr cur_node;

    //目前仅考虑3阶B样条曲线
    for(int i=0;i<degree_-1;i++)
    {
        cur_node=q0q1q2_stage[i];
        cur_node->node_state= IN_CLOSE_SET;
        ctrl_node_expanded_nodes_.insert(cur_node->position,cur_node);//控制点的hash 以位置为值
    }
    cur_node=q0q1q2_stage[degree_-1];
    cur_node->node_state=IN_OPEN_SET;
    cur_node->time_score=0.0;
    cur_node->g_score=0.0;
    cur_node->f_score=lambda_heu_*getHeu(cur_node->position,goal_next,3);  
    ctrl_node_open_set_.push(cur_node);
    ctrl_node_expanded_nodes_.insert(cur_node->position,cur_node);

    /*开始大循环*/
    while(!ctrl_node_open_set_.empty())
    {
        cur_node = ctrl_node_open_set_.top();
        /* ---------- pop node and add to close set ---------- */
        ctrl_node_open_set_.pop();
        cur_node->node_state = IN_CLOSE_SET;
        iter_num_ += 1;

        //得到局部曲线的前degree_个控制点
        std::vector<Eigen::Vector3d> LocalCPs;
        LocalCPs=RetrieveLocalCPs(cur_node);
        

        
       //计算步长,主要是考虑起点q2是不是位于地图内
       if( swin_map_->isInMap(cur_node->position) == false )    
       {
         std::cout << "该点不在地图内!其数值为:"<<cur_node->position.transpose() <<std::endl;
        continue;
       }
        // double step = GetStep(swin_map_->getDistance(cur_node->position));
        double step = GetStepEASA(LocalCPs);
        // std::cout << "step:"<<step<<std::endl;
        
       //扩展该节点，并根据动力学约束调整步长
        std::vector<Eigen::Vector3d> q_nbrs;
        q_nbrs=DirectionSafeNodeExpansion(LocalCPs,cur_node,step,(goal_next-cur_node->position).normalized());

        //开始判断以下三种停止条件
        if(GetPathLength(cur_node)>=TerminationCPNum_)
        {
            stage_result=GetCtrlAstarResults(q0q1q2_stage[0],cur_node);
            std::cout << "搜索点已足够"<<std::endl;
            
            return   stage_result;
        }

        if(  (cur_node->position-goal_next).norm()<=step  &&   CheckDynamic(LocalCPs,goal_next)==true )
        {
            // std::cout << "step:"<<step<<std::endl;
            // std::cout << "LocalCPs0:"<<LocalCPs[0].transpose()<<std::endl;
            // std::cout << "LocalCPs1:"<<LocalCPs[1].transpose() <<std::endl;
            // std::cout << "LocalCPs2:"<<LocalCPs[2].transpose() <<std::endl;

            std::cout << "终点动力学已符合，抵达终点！"<<std::endl;
            CtrlNodePtr end_node= ctrl_node_pool_[ctrl_use_node_num_++];
            end_node->parent=cur_node;
            end_node->stage=stage;
            end_node->step=step;
            end_node->position=goal_next;
            
            stage_result=GetCtrlAstarResults(q0q1q2_stage[0],end_node);
            std::cout<<"stage_result size is "<<stage_result.size()<<std::endl;
            return   stage_result;
        }

        //当前节点与目标点很近，也会停止搜索；如果目标点是终点，这种停止不能保证最后一段轨迹的动力学可行性
        // if((cur_node->position-goal_next).norm()<=0.3)
        if((cur_node->position-goal_next).norm()<=step)
        {
            stage_result=GetCtrlAstarResults(q0q1q2_stage[0],cur_node);

            if(final_goal) 
            {
                std::cout << "强行停止!  最后一段轨迹动力学无法保证"<<std::endl;
            }  
            else std::cout << "终点足够近，该阶段的搜索已经结束"<<std::endl;
            
            return   stage_result;
        }




        // q_nbrs=SafeNodeExpansion( LocalCPs,cur_node, step);
        
        for(int i=0;i<q_nbrs.size();i++)
        {
            Eigen::Vector3d q_nbr = q_nbrs[i];

            //如果扩展点Z轴过低，则跳过
            // if(q_nbr(2)<0.3) continue;

            //如果不在地图内，则跳过
            if( swin_map_->isInMap(q_nbr) == false )    
            {
                std::cout << "扩张点不在地图内，其为："<<q_nbr.transpose()<<std::endl;
                continue;
            }

            //如果邻居点离障碍物太近，则跳过
            // if(swin_map_->getDistance(q_nbr)<=0)  continue;

            //如果不满足动力学约束，则跳过
            // if(CheckDynamic(LocalCPs,q_nbr) == false)   continue;

            //如果在close_set内，则跳过
            // CtrlNodePtr temp_node = ctrl_node_expanded_nodes_.find(q_nbr);
            CtrlNodePtr temp_node=NULL;
            // if (temp_node != NULL && temp_node->node_state == IN_CLOSE_SET)  continue;

            //计算代价
            double temp_time_score = cur_node->time_score+delta_t_;
            double temp_g_score = cur_node->g_score+(q_nbr-cur_node->position).norm();
            double temp_f_score = -temp_time_score+temp_g_score+lambda_heu_*getHeu(q_nbr,goal_next,3);  

            //如果该点未在open_set中，直接加入
            if(temp_node==NULL)
            {
                temp_node = ctrl_node_pool_[ctrl_use_node_num_++];
                temp_node->position = q_nbr;
                temp_node->time_score=temp_time_score;
                temp_node->f_score = temp_f_score;
                temp_node->g_score = temp_g_score;
                temp_node->parent = cur_node;
                temp_node->stage=stage;
                temp_node->step=step;
                temp_node->node_state = IN_OPEN_SET;
                ctrl_node_open_set_.push(temp_node);
                ctrl_node_expanded_nodes_.insert(temp_node->position,temp_node);


               if (ctrl_use_node_num_ == ctrl_node_allocate_num_ )
                {
                    std::vector<Eigen::Vector3d> visited_nodes;
                    visited_nodes=ThirdStage_getVisitedNodes();
                    visualization_->drawVisitedNode(visited_nodes,0.1,visualization_->Colors_Formula_[visualization_->Purple]);
                    cout << "run out of memory." << endl;
                    return   stage_result;
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
    std::cout << "ctrl_use_node_num_ : " << ctrl_use_node_num_ << std::endl;
    std::cout << "iter num: " << iter_num_ << std::endl;
    return stage_result;
}

void  UUDircCtrlAstar::ctrlSearchReset()
{
    ctrl_node_expanded_nodes_.clear();
    std::priority_queue<CtrlNodePtr, std::vector<CtrlNodePtr>, CtrlNodeComparator> empty_queue0;
    ctrl_node_open_set_.swap(empty_queue0);

    // for (int i = 0; i < ctrl_use_node_num_; i++) {
    // CtrlNodePtr node = ctrl_node_pool_[i];
    // node->parent = NULL;
    // node->node_state = NOT_EXPAND;
    // }
    // ctrl_use_node_num_ = 0;
    // iter_num_ = 0;
}

vector<Eigen::Vector3d> UUDircCtrlAstar::RetrieveLocalCPs(const CtrlNodePtr& q_cur)
{
    vector<Eigen::Vector3d> localCPs;
    CtrlNodePtr q_temp=q_cur;
    
    for(int i=0; i<degree_;i++)
    {
        localCPs.push_back(q_temp->position);
        if(i<degree_-1) q_temp=q_temp->parent;
    }
    reverse(localCPs.begin(),localCPs.end());
    return localCPs;
}

double UUDircCtrlAstar::GetStep(const double dist)
{
    double mini=reduced_vel_*delta_t_;
        if(dist<=margin_) return mini;
        else if(dist>margin_&&dist<=(margin_+max_vel_*delta_t_))
        {
            double k=(max_vel_-mini)/max_vel_*delta_t_;
            double b=mini-margin_*k;
            return (k*dist+b);
        }
        else return (max_vel_*delta_t_);
}

bool UUDircCtrlAstar::CheckDynamic(const vector<Eigen::Vector3d>& LocalCPs,const Eigen::Vector3d& q_cur )
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


    if( acc_ctrl_bs.col(1).maxCoeff() > (max_acc_+1e-4) ||acc_ctrl_bs.col(1).minCoeff() < (-max_acc_-1e-4)) 
    {
        // std::cout << "[加速度] 动力学检查失败"<< std::endl;
        //  std::cout << "max_acc_:"<<max_acc_<<"min_acc_:"<<-max_acc_<< std::endl;
        // std::cout << "acc_ctrl_bs.col(1).maxCoeff()"<< acc_ctrl_bs.col(1).maxCoeff()<<"acc_ctrl_bs.col(1).minCoeff()"<<acc_ctrl_bs.col(1).minCoeff()<<std::endl;
        
        // for (auto i: localCPs) {
        //     std::cout << i .transpose()<<std::endl; ; 
        // }
        
        return false;//Not dynamically feasible
    }

    if (vel_ctrl_bs.col(2).maxCoeff() > max_vel_+1e-4 ||vel_ctrl_bs.col(2).minCoeff() < -max_vel_-1e-4 ) 
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

Eigen::MatrixXd UUDircCtrlAstar::Vectors2matrix(const vector<Eigen::Vector3d>& vec)
{

    Eigen::MatrixXd mat= Eigen::MatrixXd::Zero(3,vec.size());

    for (int i = 0; i < vec.size(); i++) 
    {
        for (int j = 0; j < 3; ++j)     mat(j,i)=vec[i][j];
    }

    return mat;
}

vector<CtrlNodePtr> UUDircCtrlAstar::GetCtrlAstarResults(CtrlNodePtr first_node ,CtrlNodePtr last_node)
{
    CtrlNodePtr cur_node = last_node;
    vector<CtrlNodePtr>  AstarResults;
    while (cur_node  != first_node)
    {
        AstarResults.push_back(cur_node);
        cur_node = cur_node->parent;
    }
    AstarResults.push_back(cur_node);//存起点

    reverse(AstarResults.begin(), AstarResults.end());
    return AstarResults;
}

//指向性安全扩展
auto Getsign = [](double x) { return ((x > 0) ? 1 : ((x < 0) ? -1 : 0)); };
vector<Eigen::Vector3d> UUDircCtrlAstar::DirectionSafeNodeExpansion(const vector<Eigen::Vector3d>& LocalCPs,const CtrlNodePtr& q_cur, double& step,const Eigen::Vector3d direction)
{

    Eigen::MatrixXd localCPs=Vectors2matrix(LocalCPs);
    Eigen::Vector3d current_pos=q_cur->position;

    std::vector<Eigen::Vector3d> nbrs;
    Eigen::Vector3d nbr(0, 0, 0);

    //保守扩展，防止优化时初始解越界

    int x_direction,y_direction,z_direction;//仅可取-1，0，1

    Eigen::Vector3d g_direction;
    x_direction=Getsign(   (abs(direction(0))>1e-2)?direction(0):0  );
    y_direction=Getsign(   (abs(direction(1))>1e-2)?direction(1):0  );
    z_direction=Getsign(   (abs(direction(2))>1e-2)?direction(2):0  );
    g_direction=direction;

    // std::cout<<"归一化方向："<<g_direction.transpose()<<std::endl;
    double  dx,dy,dz;
    double  safe_step;

    // nbr=current_pos;
    // if(x_direction!=0)
    // {
    //     dx=x_direction*step;
    //     dx=AdjustStep(localCPs(0,0),localCPs(0,1),localCPs(0,2),dx);
    //     nbr(0) = current_pos(0) + dx;
    //     nbrs.push_back(nbr);
    // }

    // nbr=current_pos;
    // if(y_direction!=0)
    // {
    //     dy=y_direction*step;
    //     dy=AdjustStep(localCPs(1,0),localCPs(1,1),localCPs(1,2),dy);
    //     nbr(1) = current_pos(1) + dy;
    //     nbrs.push_back(nbr);
    // }

    // nbr=current_pos;
    // if(z_direction!=0)
    // {
    //     dz=z_direction*step;
    //     dz=AdjustStep(localCPs(2,0),localCPs(2,1),localCPs(2,2),dz);
    //     nbr(2) = current_pos(2) + dz;
    //     nbrs.push_back(nbr);
    // }

    // if(  (abs(x_direction)+abs(y_direction)+abs(z_direction)) !=1)  //行进方向与以上均不重合
    {
        nbr=current_pos;
        dx=step * g_direction(0);
        dy=step * g_direction(1);
        dz=step * g_direction(2);
        dx=AdjustStep(localCPs(0,0),localCPs(0,1),localCPs(0,2),dx);
        dy=AdjustStep(localCPs(1,0),localCPs(1,1),localCPs(1,2),dy);
        dz=AdjustStep(localCPs(2,0),localCPs(2,1),localCPs(2,2),dz);
        step=std::max(step,sqrt(dx*dx + dy*dy+ dz*dz));  //步长会改变
        // safe_step=std::min(std::min(abs(dx/ g_direction(0)),abs(dy/ g_direction(1))),abs(dz/ g_direction(2))); 
        // nbr(0) = current_pos(0) + safe_step * g_direction(0);
        // nbr(1) = current_pos(1) + safe_step * g_direction(1);
        // nbr(2) = current_pos(2) + safe_step * g_direction(2);
        nbr(0) = current_pos(0) + dx;
        nbr(1) = current_pos(1) + dy;
        nbr(2) = current_pos(2) + dz;
        nbrs.push_back(nbr);
    }
   
    return nbrs;

}



vector<Eigen::Vector3d> UUDircCtrlAstar::SafeNodeExpansion(const vector<Eigen::Vector3d>& LocalCPs,const CtrlNodePtr& q_cur, const double step)
    {
        Eigen::MatrixXd localCPs=Vectors2matrix(LocalCPs);
        Eigen::Vector3d current_pos=q_cur->position;

        std::vector<Eigen::Vector3d> nbrs;
        Eigen::Vector3d nbr(0, 0, 0);

    //保守扩展，防止优化时初始解越界
    const double  dir_0_0 = step;
    const double  dir_45_0 = (step/sqrt(2));
    const double  dir_45_45 = (step/sqrt(3));

        double  dx,dy,dz;
        double  safe_step;
        for (int i = -1; i < 2; ++i) {
            for (int j = -1; j < 2; ++j) {
                for (int k = -1; k < 2; ++k) {
                    switch (abs(i) + abs(j) + abs(k)) {
                        case 0:continue;
                        case 1:
                            dx=i * dir_0_0;
                            dy=j * dir_0_0;
                            dz=k * dir_0_0;

                            if(abs(i)==1)            dx=AdjustStep(localCPs(0,0),localCPs(0,1),localCPs(0,2),dx);
                            else if(abs(j)==1)   dy=AdjustStep(localCPs(1,0),localCPs(1,1),localCPs(1,2),dy);
                            else if(abs(k)==1)  dz=AdjustStep(localCPs(2,0),localCPs(2,1),localCPs(2,2),dz);
                            if((abs(dx)+abs(dy)+abs(dz)==0)) continue;
                            break;

                        case 2:
                            dx=i * dir_45_0;
                            dy=j * dir_45_0;
                            dz=k * dir_45_0;
                            if(i==0) 
                            {
                                dy=AdjustStep(localCPs(1,0),localCPs(1,1),localCPs(1,2),dy);
                                dz=AdjustStep(localCPs(2,0),localCPs(2,1),localCPs(2,2),dz);
                                safe_step=std::min(abs(dy),abs(dz))*sqrt(2); 
                            } 
                            else if(j==0)
                            {
                                dx=AdjustStep(localCPs(0,0),localCPs(0,1),localCPs(0,2),dx);
                                dz=AdjustStep(localCPs(2,0),localCPs(2,1),localCPs(2,2),dz);
                                safe_step=std::min(abs(dx),abs(dz))*sqrt(2); 
                            }  
                            else if(k==0)
                            {
                                dx=AdjustStep(localCPs(0,0),localCPs(0,1),localCPs(0,2),dx);
                                dy=AdjustStep(localCPs(1,0),localCPs(1,1),localCPs(1,2),dy);
                                safe_step=std::min(abs(dx),abs(dy))*sqrt(2); 
                            } 
                            if(safe_step==0)  continue;
                            dx=Getsign(dx) * safe_step/sqrt(2);//原为   i,j,k* safe_step/sqrt(2)
                            dy=Getsign(dy) * safe_step/sqrt(2);
                            dz=Getsign(dz)* safe_step/sqrt(2);
                            break;
                        case 3:
                            dx=i * dir_45_45;
                            dy=j * dir_45_45;
                            dz=k * dir_45_45;
                            dx=AdjustStep(localCPs(0,0),localCPs(0,1),localCPs(0,2),dx);
                            dy=AdjustStep(localCPs(1,0),localCPs(1,1),localCPs(1,2),dy);
                            dz=AdjustStep(localCPs(2,0),localCPs(2,1),localCPs(2,2),dz);
                            safe_step=std::min(std::min(abs(dx),abs(dy)),abs(dz))*sqrt(3); 
                            if(safe_step==0)  continue;
                            dx=Getsign(dx) * safe_step/sqrt(3);  //原为   i,j,k* safe_step/sqrt(3)
                            dy=Getsign(dy) * safe_step/sqrt(3);
                            dz=Getsign(dz) * safe_step/sqrt(3);
                            break;
                        default:continue;
                    }
                    nbr(0) = current_pos(0) + dx;
                    nbr(1) = current_pos(1) + dy;
                    nbr(2) = current_pos(2) + dz;
                    nbrs.push_back(nbr);
                }
            }
        }
        return nbrs;

    }


double UUDircCtrlAstar::AdjustStep(double q0,double q1,double q2,double step)
{
    // printf("q1: %f, q2: %f\n",q1,q2);
    if( ((step-q2+q1)/(delta_t_*delta_t_)) > (max_acc_+ 1e-4))//此种情况调整后的步长会变小
    { 
        // std::cout << "调整之前step"<<step;
        step=max_acc_*delta_t_*delta_t_+q2-q1;
        // std::cout << "调整之后step"<<step<<std::endl;
    }else if( ((step-q2+q1)/(delta_t_*delta_t_))  < (-max_acc_- 1e-4))//此种情况调整后的步长会变大
    {
        //  std::cout << "调整之前step"<<step;
        step=-max_acc_*delta_t_*delta_t_+q2-q1;
        // std::cout << "调整之后step"<<step<<std::endl;
    }else
    {
    //    std::cout << "不调整step"<<step<<std::endl;
        step=step;
    } 

    return step;
}



Eigen::MatrixXd UUDircCtrlAstar::Overall_Search(Eigen::Vector3d pos_start,Eigen::Vector3d vel_start,Eigen::Vector3d acc_start,
            Eigen::Vector3d pos_goal,Eigen::Vector3d vel_goal,Eigen::Vector3d acc_goal)
{
    /*建立变量*/
    delta_t_=delta_t_initial_;
    Eigen::Vector3d qn_2_goal; //以qn-2控制点为终点
    std::vector<Eigen::Vector3d> path;

    if((pos_start-pos_goal).norm()<0.4) 
    {
        std::cout << "终点距离起点太近!"<<std::endl;
        return Vectors2matrix(path);
    }

    /*由初始状态确定最初的p个控制点    以第p个控制点为起点*/
    std::vector<Eigen::Vector3d> q0q1q2;
    q0q1q2= GetCPsFromInitialState(pos_start,vel_start,acc_start);

    /*由初始状态确定时间间隔*/
    if(swin_map_->getDistance(q0q1q2[2])<margin_)
    {
        AdjustDeltaT(pos_start,vel_start,acc_start);
        /*由初始状态确定最初的p个控制点    以第p个控制点为起点*/
        q0q1q2= GetCPsFromInitialState(pos_start,vel_start,acc_start);
    }else   std::cout << "q2 是安全的,无需调整时间间隔"<<std::endl;
    
    q0q1q2_=q0q1q2;

    /*由最终状态确定最后的p个控制点    以qn-2控制点为终点*/
    std::vector<Eigen::Vector3d> qn_2qn_1qn;
    qn_2qn_1qn= GetCPsFromFinalState(pos_goal,vel_goal,acc_goal);
    qn_2_goal=qn_2qn_1qn[0];
    //  std::cout << "qn_2_goal:"<<qn_2_goal.transpose()<<std::endl;

    /*1. 第一步  以q2为起点 qn-2为终点进行A*搜索*/
    vector<Eigen::Vector3i> FirstStageRes;
    FirstStageRes=First_AstarGraphSearch(q0q1q2,qn_2_goal);

    //判断，一阶段搜索失败则直接退出
    if(FirstStageRes.size()==0)
    {
        std::cout << "A* 图搜索失败！"<<std::endl;
       return  Vectors2matrix(path);
    }
    vector<Eigen::Vector3d> FirstStagePath;
    for(int k=0;k<FirstStageRes.size();k++)
    {
        FirstStagePath.push_back(swin_map_->GridToWorld(FirstStageRes[k])); //栅格中心为world坐标
    }
    if(show_rviz_)    visualization_->drawGeometricPath(FirstStagePath,0.15,visualization_->Colors_Formula_[visualization_->Yellow]);//显示一阶段结果
    std::cout << "FirstStageRes"<<FirstStageRes.size()<<std::endl;

    /*2. 第二步 提取A*结果中的方向拐点*/
    SecondStageRes_.clear();//SecondStageRes_已改为类变量，方便优化模块提取，故每次使用前要清空
    SecondStageRes_=Second_ExtractAstarPath(FirstStageRes);
    if(show_rviz_)  visualization_->drawGeometricPath(SecondStageRes_,0.2,visualization_->Colors_Formula_[visualization_->Blue],10);//显示二阶段结果
    
    std::cout << "SecondStageRes_"<<SecondStageRes_.size()<<std::endl;


    /*3. 第三步 对每个拐点进行定向的步长扩展 */
    vector<CtrlNodePtr> q0q1q2_stage;
    vector<CtrlNodePtr> ThirdStageRes;

    CtrlNodePtr cur_node;
    
    //此处更新初始的三个节点
    for(int i=0;i<degree_;i++)
    {
        cur_node = ctrl_node_pool_[ctrl_use_node_num_++];
        if(i==0)  cur_node->parent=NULL;
        else  cur_node->parent=ctrl_node_pool_[(ctrl_use_node_num_-2)];
        cur_node->step=-1.0;
        cur_node->stage=0; //q1-q2 即初始段 位于起点到q2 生成的凸包中
        cur_node->position=q0q1q2[i];
        // std::cout << "q"<<i<<" is "<<q0q1q2[i].transpose()<<std::endl;
        ThirdStageRes.push_back(cur_node);
    }


    //存储上一阶段的后三个节点，作为下一阶段的初始节点 。   注意 someVector.back();  is the same as   *(someVector.end()-1);
    for(int i=1;i<SecondStageRes_.size();i++)
    {
        //存储上一阶段的后三个节点，作为下一阶段的初始节点
        vector<CtrlNodePtr>::iterator iterator=ThirdStageRes.end()-1;
        for(int j=0;j<degree_;j++)
        {
            q0q1q2_stage.push_back(*iterator);
            iterator--;
        }
        reverse(q0q1q2_stage.begin(),q0q1q2_stage.end());

        //ThirdStageRes 包含 q0,q1,q2,..qn-2
        if( i==SecondStageRes_.size()-1)  ThirdStageRes= Third_Directional_Search(i,q0q1q2_stage,SecondStageRes_[i],true);
        else ThirdStageRes= Third_Directional_Search(i,q0q1q2_stage,SecondStageRes_[i],false);

        if(ThirdStageRes.size()==0)
        {
            std::cout << "第三阶段扩张失败！"<<std::endl;
            return  Vectors2matrix(path);
        }

        if(show_rviz_)//显示三阶段的扩展点
        {
            std::vector<Eigen::Vector3d> visited_nodes;
            visited_nodes=ThirdStage_getVisitedNodes();
            visualization_->drawVisitedNode(visited_nodes,0.25,visualization_->Colors_Formula_[visualization_->Purple]);
        }

        std::cout << "第"<<i<<"段,轨迹搜索完毕！"<<std::endl;

        if(GetPathLength(ThirdStageRes.back())>=TerminationCPNum_)  break;
    }


    searched_ctrl_nodes_= GetCtrlAstarResults(ctrl_node_pool_[0],ThirdStageRes.back());

    std::cout << "searched_ctrl_nodes_ size is "<<searched_ctrl_nodes_.size()<<std::endl;
    path=CtrlNode2Path();
    for(int i=1;i<qn_2qn_1qn.size();i++)  path.push_back(path.back());  //补齐qn-1 和qn, qn-2=qn-1=qn



    return   Vectors2matrix(path);
}


void UUDircCtrlAstar::AdjustDeltaT(Eigen::Vector3d pos_start,Eigen::Vector3d vel_start,Eigen::Vector3d acc_start)
{
    // double distance_tolerance=swin_map_->getDistance(pos_start)-resolution_;


    // if((acc_start*delta_t_*delta_t_/3+vel_start*delta_t_).norm()<distance_tolerance)
    // {
    //     std::cout << "路径搜索起点是安全的,时间间隔无需调整!"<<std::endl;
    // }
    // else if(acc_start.norm()==0)//acc_start 为0意味着 此问题将不再是一元四次方程的数学形式
    // {
    //     delta_t_=sqrt((distance_tolerance)*(distance_tolerance)/(vel_start.squaredNorm()));
    // }
    // else
    // {
    //     double A=(acc_start(0)*acc_start(0)+acc_start(1)*acc_start(1)+acc_start(2)*acc_start(2))/9;  //(a01*a01+a02*a02+a03*a03)/9
    //     double B=2*(acc_start(0)*vel_start(0)+acc_start(1)*vel_start(1)+acc_start(2)*vel_start(2))/3;//2*(a01*v01+a02*v02+a03*v03)/3;
    //     double C=vel_start(0)*vel_start(0)+vel_start(1)*vel_start(1)+vel_start(2)*vel_start(2);//v01*v01+v02*v02+v03*v03;
    //     double D=0;
    //     double E=-(distance_tolerance)*(distance_tolerance);//-dis*dis   0.3为留出的一定余量

    //     int nroots;
    //     double ans[4];
    //     nroots=quartic(B/A,C/A,D/A,E/A,ans);
    //     if(nroots==0)
    //         printf("Equation has no real roots!\n");
    //     else
    //     {
    //         printf("Equation has %d real roots: ",nroots);
    //         for (int i = 0; i < nroots; i++)
    //         {
    //             if(ans[i]>0&&ans[i]<delta_t_)   delta_t_=ans[i];
    //             printf("%.16lf, ",ans[i]);
    //         }
    //      }
    // }
    // std::cout << "调整后的时间间隔为:"<<delta_t_<<std::endl;
    double distance_tolerance=swin_map_->getDistance(pos_start);

    if((acc_start*delta_t_*delta_t_/3+vel_start*delta_t_).norm()<distance_tolerance)
    {
        std::cout << "路径搜索起点是安全的,时间间隔无需调整!"<<std::endl;
    }
    else if(acc_start.norm()==0)//acc_start 为0意味着 此问题将不再是一元四次方程的数学形式
    {
        // delta_t_=sqrt((distance_tolerance-1.2*margin_)*(distance_tolerance-1.2*margin_)/(vel_start.squaredNorm()));
                delta_t_=sqrt((distance_tolerance)*(distance_tolerance)/(vel_start.squaredNorm()));
    }
    else
    {
        double A=(acc_start(0)*acc_start(0)+acc_start(1)*acc_start(1)+acc_start(2)*acc_start(2))/9;  //(a01*a01+a02*a02+a03*a03)/9
        double B=2*(acc_start(0)*vel_start(0)+acc_start(1)*vel_start(1)+acc_start(2)*vel_start(2))/3;//2*(a01*v01+a02*v02+a03*v03)/3;
        double C=vel_start(0)*vel_start(0)+vel_start(1)*vel_start(1)+vel_start(2)*vel_start(2);//v01*v01+v02*v02+v03*v03;
        double D=0;
        // double E=-(distance_tolerance-1.2*margin_)*(distance_tolerance-1.2*margin_);//-dis*dis   0.3为留出的一定余量
                double E=-(distance_tolerance)*(distance_tolerance);//-dis*dis   0.3为留出的一定余量

        int nroots;
        double ans[4];
        nroots=quartic(B/A,C/A,D/A,E/A,ans);
        if(nroots==0)
            printf("Equation has no real roots!\n");
        else
        {
            printf("Equation has %d real roots: ",nroots);
            for (int i = 0; i < nroots; i++)
            {
                if(ans[i]>0&&ans[i]<delta_t_)   delta_t_=ans[i];
                printf("%.16lf, ",ans[i]);
            }
         }
    }
    std::cout << "调整后的时间间隔为:"<<delta_t_<<std::endl;



}

double UUDircCtrlAstar::cubic(double b,double c,double d)
{
    double p=c-b*b/3.0;
    double q=2.0*b*b*b/27.0-b*c/3.0+d;

    if(p==0.0) return pow(q,1.0/3.0);
    if(q==0.0) return 0.0;

    double t=sqrt(fabs(p)/3.0);
    double g=1.5*q/(p*t);
    if(p>0.0)
        return -2.0*t*sinh(asinh(g)/3.0)-b/3.0;


    if(4.0*p*p*p+27.0*q*q<0.0)
        return 2.0*t*cos(acos(g)/3.0)-b/3.0;

    if(q>0.0)
        return -2.0*t*cosh(acosh(-g)/3.0)-b/3.0;

    return 2.0*t*cosh(acosh(g)/3.0)-b/3.0;
}

int UUDircCtrlAstar::quartic(double b,double c,double d,double e,double* ans)
{

    double p=c-0.375*b*b;
    double q=0.125*b*b*b-0.5*b*c+d;
    double m=cubic(p,0.25*p*p+0.01171875*b*b*b*b-e+0.25*b*d-0.0625*b*b*c,-0.125*q*q);
    if(q==0.0)
    {
        if(m<0.0) return 0;
        int nroots=0;
        double sqrt_2m=sqrt(2.0*m);
        if(-m-p>0.0)
        {
            double delta=sqrt(2.0*(-m-p));
            ans[nroots++]=-0.25*b+0.5*(sqrt_2m-delta);
            ans[nroots++]=-0.25*b-0.5*(sqrt_2m-delta);
            ans[nroots++]=-0.25*b+0.5*(sqrt_2m+delta);
            ans[nroots++]=-0.25*b-0.5*(sqrt_2m+delta);
        }

        if(-m-p==0.0)
        {
            ans[nroots++]=-0.25*b-0.5*sqrt_2m;
            ans[nroots++]=-0.25*b+0.5*sqrt_2m;
        }

        return nroots;
    }

    if(m<0.0) return 0;
    double sqrt_2m=sqrt(2.0*m);
    int nroots=0;
    if(-m-p+q/sqrt_2m>=0.0)
    {
        double delta=sqrt(2.0*(-m-p+q/sqrt_2m));
        ans[nroots++]=0.5*(-sqrt_2m+delta)-0.25*b;
        ans[nroots++]=0.5*(-sqrt_2m-delta)-0.25*b;
    }

    if(-m-p-q/sqrt_2m>=0.0)
    {
        double delta=sqrt(2.0*(-m-p-q/sqrt_2m));
        ans[nroots++]=0.5*(sqrt_2m+delta)-0.25*b;
        ans[nroots++]=0.5*(sqrt_2m-delta)-0.25*b;
    }

    return nroots;
}

std::vector<Eigen::Vector3d> UUDircCtrlAstar::GetCPsFromInitialState(Eigen::Vector3d pos_start,Eigen::Vector3d vel_start,Eigen::Vector3d acc_start)
{
     std::vector<Eigen::Vector3d> q0q1q2;

     Eigen::Vector3d q0,q1,q2;

     q0=acc_start*delta_t_*delta_t_/3-vel_start*delta_t_+pos_start;
     q0q1q2.push_back(q0);

     q1=pos_start-(acc_start*delta_t_*delta_t_)/6;
     q0q1q2.push_back(q1);

     q2=acc_start*delta_t_*delta_t_/3+vel_start*delta_t_+pos_start;
     q0q1q2.push_back(q2);
    
    //  std::cout<<"pos_start 的ESDF为:"<<   swin_map_->getDistance(pos_start)<<std::endl;
    //  std::cout<<"q2 和pos_start 之间的距离为:"<<  (q2-pos_start).norm()<<std::endl;
     return q0q1q2;
}

std::vector<Eigen::Vector3d> UUDircCtrlAstar::GetCPsFromFinalState(Eigen::Vector3d pos_goal,Eigen::Vector3d vel_goal,Eigen::Vector3d acc_goal)
{
    std::vector<Eigen::Vector3d> qn_2qn_1qn;

    Eigen::Vector3d qn_2,qn_1,qn;

    qn_2=acc_goal*delta_t_*delta_t_/3-vel_goal*delta_t_+pos_goal;
    qn_1=pos_goal-(acc_goal*delta_t_*delta_t_)/6;
    qn=acc_goal*delta_t_*delta_t_/3+vel_goal*delta_t_+pos_goal;

    qn_2qn_1qn.push_back(qn_2);
    qn_2qn_1qn.push_back(qn_1);
    qn_2qn_1qn.push_back(qn);
    return qn_2qn_1qn;
}

std::vector<Eigen::Vector3d> UUDircCtrlAstar::CtrlNode2Path()
{
  vector<Eigen::Vector3d> path;
  for (int i = 0; i < searched_ctrl_nodes_.size(); ++i) {
    path.push_back(searched_ctrl_nodes_[i]->position);
  }
  return path;
}

int UUDircCtrlAstar::GetDegree()
{
    return degree_;
}

double UUDircCtrlAstar::GetDelta_t()
{
    return delta_t_;
}

std::vector<Eigen::Vector3d>  UUDircCtrlAstar::ThirdStage_getVisitedNodes() 
{
    std::vector<CtrlNodePtr> visited;
    visited.assign(ctrl_node_pool_.begin(), ctrl_node_pool_.begin() + ctrl_use_node_num_ - 1);

    std::vector<Eigen::Vector3d> visited_node;
    for (int i = 0; i < visited.size(); i++) {
        visited_node.push_back(visited[i]->position);
    }
    return visited_node;
}


Eigen::MatrixXd UUDircCtrlAstar::RetrieveCtrlNodeStep()
{
  vector<double> steps;
  for (int i = 0; i < searched_ctrl_nodes_.size(); ++i) {
    steps.push_back(searched_ctrl_nodes_[i]->step);
  }
  
  steps.push_back(0.0);//补齐qn-1 和qn
  steps.push_back(0.0);
    
  return Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> >(steps.data(),1,steps.size());
}

int UUDircCtrlAstar::GetPathLength(CtrlNodePtr last_node)
{

    CtrlNodePtr cur_node = last_node;
    int length=0;
    while (cur_node  != NULL)
    {
       length++;
        cur_node = cur_node->parent;
    }
    return length+2; //加上qn-1和qn

}

double UUDircCtrlAstar::GetStepEASA(const vector<Eigen::Vector3d>& LocalCPs)
{
    Eigen::Vector3d pos;
    // pos=0.1667*LocalCPs[0]+0.6667*LocalCPs[1]+0.1667*LocalCPs[2];
    pos=LocalCPs[2];
    double dist=swin_map_->getDistance(pos);
    if(dist<reduced_threshold_)
    {
        Eigen::Vector3d qv0,qv1,vel;
        qv0=(LocalCPs[1]-LocalCPs[0])/delta_t_;
        qv1=(LocalCPs[2]-LocalCPs[1])/delta_t_;
        // vel=0.5*qv0+0.5*qv1;
        vel=qv1;
        Eigen::Vector3d dist_grad;
        dist_grad=-1*swin_map_->getGradient(pos);

        double vmin=reduced_vel_;
        double vmax=max_vel_;

        double k=vmax-vmin,b=vmin;
        
        if(dist_grad.norm()<1e-4||vel.norm()<1e-4)
        {
            return b*delta_t_;
        }
        else
        {
            // std::cout<<"dist_grad is "<<dist_grad.transpose()<<std::endl;
            // std::cout<<"vel is "<<vel.transpose()<<std::endl;
            // std::cout<<"cos thsta is "<<(vel/ vel.norm()).dot((dist_grad/ dist_grad.norm()))<<std::endl;
            double cos=vel.dot(dist_grad) / vel.norm() / dist_grad.norm();
            return k/(1+exp(10*cos))+b;
            // return (k*(vel.dot(dist_grad) / vel.norm() / dist_grad.norm())+b)*delta_t_;
        }

    }
    else    return (max_vel_*delta_t_);
}

//num_stage 表示几个多面体
vector<Eigen::Vector3d,Eigen::aligned_allocator<Eigen::Vector3d>> UUDircCtrlAstar::getDecompPath(int num_stage)
{

    vector<Eigen::Vector3d,Eigen::aligned_allocator<Eigen::Vector3d>>  path;

    path.push_back(q0q1q2_[1]);
    path.push_back(q0q1q2_[2]);
    for(int i=1;i<SecondStageRes_.size();i++) 
    {   
        if(path.size()>=num_stage+1)  break;
        path.push_back(SecondStageRes_[i]);
    }

    return path;
}

std::vector<int>  UUDircCtrlAstar::RetrieveCtrlNodeStage()
{
    vector<int> stage;
    for (int i = 0; i < searched_ctrl_nodes_.size(); ++i) {
        stage.push_back(searched_ctrl_nodes_[i]->stage);
    }

    stage.push_back(stage.back());//补齐qn-1 和qn
    stage.push_back(stage.back());

    return stage;
}