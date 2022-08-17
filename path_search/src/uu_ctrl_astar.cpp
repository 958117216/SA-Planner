#include"path_search/uu_ctrl_astar.h"

UUCtrlAstar::~UUCtrlAstar()
{
    for (int i = 0; i < allocate_num_; i++) delete ctrl_node_pool_[i];
}

void UUCtrlAstar::initSearch(ros::NodeHandle& nh,const SWINMap::Ptr& map,const PlanningVisualization::Ptr& visualization)
{
    nh.param("uu_ctrl_astar/resolution_astar", resolution_, -1.0);
    nh.param("uu_ctrl_astar/lambda_heu", lambda_heu_, -1.0);
    nh.param("uu_ctrl_astar/margin", margin_, -1.0);
    nh.param("uu_ctrl_astar/allocate_num", allocate_num_, -1);
    nh.param("uu_ctrl_astar/degree",degree_,3);
    nh.param("uu_ctrl_astar/max_vel",max_vel_,1.0);
    nh.param("uu_ctrl_astar/max_acc",max_acc_,1.0);
    nh.param("uu_ctrl_astar/delta_t",delta_t_,1.0);
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

double UUCtrlAstar::GetStep(const double dist)
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

void UUCtrlAstar::reset()
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

Eigen::MatrixXd UUCtrlAstar::search(Eigen::Vector3d pos_start,Eigen::Vector3d vel_start,Eigen::Vector3d acc_start,
            Eigen::Vector3d pos_goal,Eigen::Vector3d vel_goal,Eigen::Vector3d acc_goal)
{
    // std::cout << "最初地图版本:" << swin_map_->version_ << std::endl;
    /*建立变量*/
    Eigen::Vector3d qn_2_goal; //以qn-2控制点为终点
    NodePtr cur_node;
    std::vector<Eigen::Vector3d> path;

    if((pos_start-pos_goal).norm()<0.4) 
    {
        std::cout << "终点距离起点太近!"<<std::endl;
        return Vectors2matrix(path);
    }
    /*由初始状态确定时间间隔*/
    AdjustDeltaT(pos_start,vel_start,acc_start);

    /*由最终状态确定最后的p个控制点    以qn-2控制点为终点*/
    std::vector<Eigen::Vector3d> qn_2qn_1qn;
    qn_2qn_1qn= GetCPsFromFinalState(pos_goal,vel_goal,acc_goal);
    qn_2_goal=qn_2qn_1qn[0];
    //  std::cout << "qn_2_goal:"<<qn_2_goal.transpose()<<std::endl;


    /*由初始状态确定最初的p个控制点    以第p个控制点为起点*/
    std::vector<Eigen::Vector3d> q0q1q2;
    q0q1q2= GetCPsFromInitialState(pos_start,vel_start,acc_start);

    visualization_->drawSearchStart(q0q1q2[2],0.4,Eigen::Vector4d(1,0.5,0,1));

    for(int i=0;i<degree_-1;i++)
    {
        cur_node = ctrl_node_pool_[use_node_num_++];
        if(i==0)  cur_node->parent=NULL;
        else  cur_node->parent=ctrl_node_pool_[(use_node_num_-2)];
        cur_node->step=-1.0;
        cur_node->position=q0q1q2[i];
        cur_node->grid= swin_map_->WorldToGrid(cur_node->position);
        cur_node->node_state=IN_CLOSE_SET;
        expanded_nodes_.insert(cur_node->position,cur_node);
        // expanded_nodes_.insert(cur_node->grid,cur_node);
    }

    cur_node = ctrl_node_pool_[use_node_num_++];
    cur_node->parent=ctrl_node_pool_[(use_node_num_-2)];
    cur_node->step=-1.0;
    cur_node->position=q0q1q2[2];
    cur_node->grid= swin_map_->WorldToGrid(cur_node->position);
    cur_node->node_state=IN_OPEN_SET;
    cur_node->time_score=0.0;
    cur_node->g_score=0.0;
    cur_node->f_score=lambda_heu_*getDiagHeu(cur_node->position,qn_2_goal);                //getEuclHeu(cur_node->position,qn_2_goal);    
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
       if( swin_map_->isInMap(cur_node->position) == false )    
       {
         std::cout << "起点q2 不在地图内!其数值为:"<<q0q1q2[2].transpose() <<std::endl;
        continue;
       }
        double step = GetStep(swin_map_->getDistance(cur_node->grid));

        //  std::cout << "step:"<<step<<std::endl;
        // std::cout << "LocalCPs0:"<<LocalCPs[0].transpose()<<std::endl;
        // std::cout << "LocalCPs1:"<<LocalCPs[1].transpose() <<std::endl;
        // std::cout << "LocalCPs2:"<<LocalCPs[2].transpose() <<std::endl;


        //到达终点  将新建qn-2为最终点，这种写法目的是和下文保持一致
        if((cur_node->position-qn_2_goal).norm()<=step&&CheckDynamic(LocalCPs,qn_2_goal))
        {
            NodePtr end_node=new Node;
            end_node->parent=cur_node;
            end_node->step=step;
            end_node->position=qn_2qn_1qn[0];
            end_node->grid= swin_map_->WorldToGrid(qn_2qn_1qn[0]);
            
            retrievePartCPs(end_node);
            path=getPartCPs();
            for(int i=1;i<qn_2qn_1qn.size();i++)  path.push_back(qn_2qn_1qn[i]);  //补齐qn-1 和qn

            return   Vectors2matrix(path);
        }

        //搜索已达到一定程度,将当前点作为qn-2 ，以最终速度，加速度为零 直接结束
        // if(cur_node->time_score>=3*delta_t_)
        if((cur_node->position-q0q1q2[2]).norm()>=(5-(q0q1q2[2]-pos_start).norm())||((cur_node->position-qn_2_goal).norm()<=step))//||((cur_node->position-qn_2_goal).norm()<=0.4)
        // if(cur_node->time_score>=std::min((6-(q0q1q2[2]-pos_start).norm())/max_vel_,(pos_start-pos_goal).norm()/max_vel_))
        {   
            retrievePartCPs(cur_node);
            path=getPartCPs();
            path.push_back(path.back());//补齐qn-1 和qn
            path.push_back(path.back());    
            // std::cout << "use node num: " << use_node_num_ << std::endl;
            return Vectors2matrix(path);
        }

        //如果步长为0，直接跳过
        // if(step==0) continue;
       
       //扩展该节点
       std::vector<Eigen::Vector3d> q_nbrs;
        // q_nbrs=NodeExpansion(cur_node,step);
        q_nbrs=SafeNodeExpansion(LocalCPs,cur_node,step);

    //     Eigen::Vector3d goal_next;
    //    goal_next=pos_goal;
        
    //     q_nbrs=DirectionSafeNodeExpansion(LocalCPs,cur_node,step,(goal_next-cur_node->position).normalized());
        for(int i=0;i<q_nbrs.size();i++)
        {
            Eigen::Vector3d q_nbr = q_nbrs[i];

            //如果扩展点Z轴过低，则跳过
            // if(q_nbr(2)<0.3) continue;

            //如果不在地图内，则跳过
            if( swin_map_->isInMap(q_nbr) == false )    continue;

            //如果邻居点离障碍物太近，则跳过
            if(swin_map_->getDistance(q_nbr)<=0)  continue;

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
            double temp_f_score = temp_g_score+lambda_heu_*getEuclHeu(q_nbr,qn_2_goal);   // getDiagHeu(q_nbr,qn_2_goal)
            // double temp_f_score = lambda_heu_*getEuclHeu(q_nbr,qn_2_goal);   // getDiagHeu(q_nbr,qn_2_goal)
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
                    visualization_->drawVisitedNode(getVisitedNodes(),0.1,visualization_->Colors_Formula_[visualization_->Purple]);
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

  std::cout << "起点q2 为:"<<q0q1q2[2].transpose() <<std::endl;
  visualization_->drawVisitedNode(getVisitedNodes(),0.1,visualization_->Colors_Formula_[visualization_->Purple]);
//   std::cout << "起点q2 的邻居为:"<<q_nbrs.size() <<std::endl;
  return Vectors2matrix(path);
}

Eigen::MatrixXd UUCtrlAstar::RetrieveCtrlNodeStep()
{
  vector<double> steps;
  for (int i = 0; i < searched_ctrl_nodes_.size(); ++i) {
    steps.push_back(searched_ctrl_nodes_[i]->step);
  }
  
  steps.push_back(-1.0);//补齐qn-1 和qn
  steps.push_back(-1.0);
 
  return Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> >(steps.data(),1,steps.size());
}

void UUCtrlAstar::AdjustDeltaT(Eigen::Vector3d pos_start,Eigen::Vector3d vel_start,Eigen::Vector3d acc_start)
{
    double distance_tolerance=swin_map_->getDistance(pos_start);
    delta_t_=1.0;

    if((acc_start*delta_t_*delta_t_/3+vel_start*delta_t_).norm()<distance_tolerance)
    {
        std::cout << "路径搜索起点是安全的,时间间隔无需调整!"<<std::endl;
    }
    else if(acc_start.norm()==0)//acc_start 为0意味着 此问题将不再是一元四次方程的数学形式
    {
        delta_t_=sqrt((distance_tolerance-0.5)*(distance_tolerance-0.5)/(vel_start.squaredNorm()));
    }
    else
    {
        double A=(acc_start(0)*acc_start(0)+acc_start(1)*acc_start(1)+acc_start(2)*acc_start(2))/9;  //(a01*a01+a02*a02+a03*a03)/9
        double B=2*(acc_start(0)*vel_start(0)+acc_start(1)*vel_start(1)+acc_start(2)*vel_start(2))/3;//2*(a01*v01+a02*v02+a03*v03)/3;
        double C=vel_start(0)*vel_start(0)+vel_start(1)*vel_start(1)+vel_start(2)*vel_start(2);//v01*v01+v02*v02+v03*v03;
        double D=0;
        double E=-(distance_tolerance-0.5)*(distance_tolerance-0.5);//-dis*dis   0.3为留出的一定余量

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

double UUCtrlAstar::cubic(double b,double c,double d)
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

int UUCtrlAstar::quartic(double b,double c,double d,double e,double* ans)
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


std::vector<Eigen::Vector3d> UUCtrlAstar::GetCPsFromInitialState(Eigen::Vector3d pos_start,Eigen::Vector3d vel_start,Eigen::Vector3d acc_start)
{
     std::vector<Eigen::Vector3d> q0q1q2;

     Eigen::Vector3d q0,q1,q2;

     q0=acc_start*delta_t_*delta_t_/3-vel_start*delta_t_+pos_start;
     q0q1q2.push_back(q0);

     q1=pos_start-(acc_start*delta_t_*delta_t_)/6;
     q0q1q2.push_back(q1);

     q2=acc_start*delta_t_*delta_t_/3+vel_start*delta_t_+pos_start;
     q0q1q2.push_back(q2);
    
     std::cout<<"pos_start 的ESDF为:"<<   swin_map_->getDistance(pos_start)<<std::endl;
     std::cout<<"q2 和pos_start 之间的距离为:"<<  (q2-pos_start).norm()<<std::endl;
     return q0q1q2;
}

std::vector<Eigen::Vector3d> UUCtrlAstar::GetCPsFromFinalState(Eigen::Vector3d pos_goal,Eigen::Vector3d vel_goal,Eigen::Vector3d acc_goal)
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

std::vector<Eigen::Vector3d> UUCtrlAstar::GetCPsFromQn_2FinalState(Eigen::Vector3d qn_2,Eigen::Vector3d vel_goal,Eigen::Vector3d acc_goal)
{
    std::vector<Eigen::Vector3d> qn_1qn;

    Eigen::Vector3d qn_1,qn;
    qn_1=qn_2+delta_t_*vel_goal-(acc_goal*delta_t_*delta_t_)/2;
    qn=qn_2+2*delta_t_*vel_goal;


    qn_1qn.push_back(qn_1);
    qn_1qn.push_back(qn);
    return qn_1qn;

}

vector<Eigen::Vector3d> UUCtrlAstar::NodeExpansion(const NodePtr& q_cur, const double step)
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

vector<Eigen::Vector3d> UUCtrlAstar::RetrieveLocalCPs(const NodePtr& q_cur)
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

bool UUCtrlAstar::CheckDynamic(const vector<Eigen::Vector3d>& LocalCPs,const Eigen::Vector3d& q_cur )
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


    if( acc_ctrl_bs.col(1).maxCoeff() > max_acc_ ||acc_ctrl_bs.col(1).minCoeff() < -max_acc_) 
    {
        // std::cout << "[加速度] 动力学检查失败"<< std::endl;
        //  std::cout << "max_acc_:"<<max_acc_<<"min_acc_:"<<-max_acc_<< std::endl;
        // std::cout << "acc_ctrl_bs.col(1).maxCoeff()"<< acc_ctrl_bs.col(1).maxCoeff()<<"acc_ctrl_bs.col(1).minCoeff()"<<acc_ctrl_bs.col(1).minCoeff()<<std::endl;
        
        // for (auto i: localCPs) {
        //     std::cout << i .transpose()<<std::endl; ; 
        // }
        
        return false;//Not dynamically feasible
    }

    if (vel_ctrl_bs.col(2).maxCoeff() > max_vel_ ||vel_ctrl_bs.col(2).minCoeff() < -max_vel_ ) 
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

void UUCtrlAstar::retrievePartCPs(NodePtr end_node)
{
  NodePtr cur_node = end_node;
  searched_ctrl_nodes_.push_back(cur_node);

  while (cur_node->parent != NULL) {
    cur_node = cur_node->parent;
    searched_ctrl_nodes_.push_back(cur_node);
  }

  reverse(searched_ctrl_nodes_.begin(), searched_ctrl_nodes_.end());
}

std::vector<Eigen::Vector3d> UUCtrlAstar::getPartCPs() {
  vector<Eigen::Vector3d> path;
  for (int i = 0; i < searched_ctrl_nodes_.size(); ++i) {
    path.push_back(searched_ctrl_nodes_[i]->position);
  }
  return path;
}

Eigen::MatrixXd UUCtrlAstar::Vectors2matrix(const vector<Eigen::Vector3d>& vec)
{

    Eigen::MatrixXd mat= Eigen::MatrixXd::Zero(3,vec.size());

    for (int i = 0; i < vec.size(); i++) 
    {
        for (int j = 0; j < 3; ++j)     mat(j,i)=vec[i][j];
    }

    return mat;

}

double UUCtrlAstar::getDiagHeu(Eigen::Vector3d x1, Eigen::Vector3d x2)
{
  double heu = (x1 - x2).array().abs().sum() +  resolution_*(sqrt(3) - 3) * (x1 - x2).array().abs().minCoeff();
//   heu=heu/max_vel_;
  return tie_breaker_ * heu;
}

double UUCtrlAstar::getEuclHeu(Eigen::Vector3d x1, Eigen::Vector3d x2)
{
     return tie_breaker_ * (x2 - x1).norm();
}

int UUCtrlAstar::GetDegree()
{
    return degree_;
}

double UUCtrlAstar::GetDelta_t()
{
    return delta_t_;
}

std::vector<Eigen::Vector3d>  UUCtrlAstar::getVisitedNodes() 
{
    std::vector<NodePtr> visited;
    visited.assign(ctrl_node_pool_.begin(), ctrl_node_pool_.begin() + use_node_num_ - 1);

    std::vector<Eigen::Vector3d> visited_node;
    for (int i = 0; i < visited.size(); i++) {
        visited_node.push_back(visited[i]->position);
    }
    return visited_node;
}


std::vector<std::vector<Eigen::Vector3d>>  UUCtrlAstar::getVisitedNodes_TimeKind() 
{
    std::vector<NodePtr> visited;
    visited.assign(ctrl_node_pool_.begin(), ctrl_node_pool_.begin() + use_node_num_ - 1);

    std::vector<std::vector<Eigen::Vector3d>>  visited_node_timekind(4);
    for (int i = 0; i < visited.size(); i++) 
    {
        if(visited[i]->time_score==1.0*delta_t_)
        {
            visited_node_timekind[0].push_back(visited[i]->position);
        }
        else if(visited[i]->time_score==2.0*delta_t_)
        {
            visited_node_timekind[1].push_back(visited[i]->position);
        }
        else if(visited[i]->time_score==3.0*delta_t_)
        {
             visited_node_timekind[2].push_back(visited[i]->position);
        }
        else 
             visited_node_timekind[3].push_back(visited[i]->position);
    }
    return visited_node_timekind;
}

    double UUCtrlAstar::AdjustStep(double q0,double q1,double q2,double step)
    {
        // printf("q1: %f, q2: %f\n",q1,q2);
        if( ((step-q2+q1)/(delta_t_*delta_t_)) > max_acc_)
        { 
            // std::cout << "调整之前step"<<step;
            step=0.95*max_acc_*delta_t_*delta_t_+q2-q1;
            // std::cout << "调整之后step"<<step<<std::endl;
        }else if( ((step-q2+q1)/(delta_t_*delta_t_))  < -max_acc_)
        {
            //  std::cout << "调整之前step"<<step;
            step=-0.95*max_acc_*delta_t_*delta_t_+q2-q1;
            // std::cout << "调整之后step"<<step<<std::endl;
        }else
        {
        //    std::cout << "不调整step"<<step<<std::endl;
            step=step;
        } 
        
        return step;
    }


    auto Getsign = [](double x) { return ((x > 0) ? 1 : ((x < 0) ? -1 : 0)); };
    vector<Eigen::Vector3d> UUCtrlAstar::SafeNodeExpansion(const vector<Eigen::Vector3d>& LocalCPs,const NodePtr& q_cur, const double step)
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


        //指向性安全扩展
vector<Eigen::Vector3d> UUCtrlAstar::DirectionSafeNodeExpansion(const vector<Eigen::Vector3d>& LocalCPs,const NodePtr& q_cur, const double step,const Eigen::Vector3d direction)
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

    std::cout<<"归一化方向："<<g_direction.transpose()<<std::endl;
    double  dx,dy,dz;
    double  safe_step;

    nbr=current_pos;
    if(x_direction!=0)
    {
        dx=x_direction*step;
        dx=AdjustStep(localCPs(0,0),localCPs(0,1),localCPs(0,2),dx);
        nbr(0) = current_pos(0) + dx;
        nbrs.push_back(nbr);
    }

    nbr=current_pos;
    if(y_direction!=0)
    {
        dy=y_direction*step;
        dy=AdjustStep(localCPs(1,0),localCPs(1,1),localCPs(1,2),dy);
        nbr(1) = current_pos(1) + dy;
        nbrs.push_back(nbr);
    }

    nbr=current_pos;
    if(z_direction!=0)
    {
        dz=z_direction*step;
        dz=AdjustStep(localCPs(2,0),localCPs(2,1),localCPs(2,2),dz);
        nbr(2) = current_pos(2) + dz;
        nbrs.push_back(nbr);
    }

    if(  (abs(x_direction)+abs(y_direction)+abs(z_direction)) !=1)  //行进方向与以上均不重合
    {
        nbr=current_pos;
        dx=step * g_direction(0);
        dy=step * g_direction(1);
        dz=step * g_direction(2);
        dx=AdjustStep(localCPs(0,0),localCPs(0,1),localCPs(0,2),dx);
        dy=AdjustStep(localCPs(1,0),localCPs(1,1),localCPs(1,2),dy);
        dz=AdjustStep(localCPs(2,0),localCPs(2,1),localCPs(2,2),dz);
        // safe_step=std::min(std::min(abs(dx),abs(dy)),abs(dz))*sqrt(3); 
        nbr(0) = current_pos(0) + dx;
        nbr(1) = current_pos(1) + dy;
        nbr(2) = current_pos(2) + dz;
        nbrs.push_back(nbr);
    }
   
    return nbrs;

}


// Eigen::MatrixXd UUCtrlAstar::Directional_Search(Eigen::Vector3d pos_start,Eigen::Vector3d vel_start,Eigen::Vector3d acc_start,
//             Eigen::Vector3d pos_goal,Eigen::Vector3d vel_goal,Eigen::Vector3d acc_goal)
// {
//     //·定义终极变量
//    std::vector<Eigen::Vector3d> path;
//    Eigen::Vector3d qn_2_goal; //以qn-2控制点为终点
   
//     if((pos_start-pos_goal).norm()<0.4) 
//     {
//         std::cout << "终点距离起点太近!"<<std::endl;
//         return Vectors2matrix(path);
//     }

//     /*由初始状态确定时间间隔*/
//     AdjustDeltaT(pos_start,vel_start,acc_start);
//     /*由最终状态确定最后的p个控制点    以qn-2控制点为终点*/
//     std::vector<Eigen::Vector3d> qn_2qn_1qn;
//     qn_2qn_1qn= GetCPsFromFinalState(pos_goal,vel_goal,acc_goal);
//     qn_2_goal=qn_2qn_1qn[0];


//     //*1.A*获得路径点

//     //*2.精简A*获得方向点

//     //3.对每个方向点进行指向性扩张，直至终点
    
//     vector<Eigen::Vector3d> Directional_Points;
    
    
//     vector<Eigen::Vector3d>::iterator iterator_temp = Directional_Points.begin();//vector第一个点为起点
//     iterator_temp++;
//     Eigen::Vector3d goal_next=*iterator_temp;

//     NodePtr cur_node;
//     /*由初始状态确定最初的p个控制点    以第p个控制点为起点*/
//     std::vector<Eigen::Vector3d> q0q1q2;
//     q0q1q2= GetCPsFromInitialState(pos_start,vel_start,acc_start);

//     visualization_->drawSearchStart(q0q1q2[2],0.4,Eigen::Vector4d(1,0.5,0,1));

//     for(int i=0;i<degree_-1;i++)
//     {
//         cur_node = ctrl_node_pool_[use_node_num_++];
//         if(i==0)  cur_node->parent=NULL;
//         else  cur_node->parent=ctrl_node_pool_[(use_node_num_-2)];
//         cur_node->step=-1.0;
//         cur_node->position=q0q1q2[i];
//         cur_node->grid= swin_map_->WorldToGrid(cur_node->position);
//         cur_node->node_state=IN_CLOSE_SET;
//         expanded_nodes_.insert(cur_node->position,cur_node);
//         // expanded_nodes_.insert(cur_node->grid,cur_node);
//     }

//     cur_node = ctrl_node_pool_[use_node_num_++];
//     cur_node->parent=ctrl_node_pool_[(use_node_num_-2)];
//     cur_node->step=-1.0;
//     cur_node->position=q0q1q2[2];
//     cur_node->grid= swin_map_->WorldToGrid(cur_node->position);
//     cur_node->node_state=IN_OPEN_SET;
//     cur_node->time_score=0.0;
//     cur_node->g_score=0.0;
//     cur_node->f_score=lambda_heu_*getDiagHeu(cur_node->position,qn_2_goal);                //getEuclHeu(cur_node->position,qn_2_goal);    
//     open_set_.push(cur_node);
//     expanded_nodes_.insert(cur_node->position,cur_node);
//     // expanded_nodes_.insert(cur_node->grid,cur_node);

//     /*开始大循环*/
//     while(!open_set_.empty())
//     {
//         cur_node = open_set_.top();
//         /* ---------- pop node and add to close set ---------- */
//         open_set_.pop();
//         cur_node->node_state = IN_CLOSE_SET;
//         iter_num_ += 1;

//         //得到局部曲线的前degree_个控制点
//         std::vector<Eigen::Vector3d> LocalCPs;
//         LocalCPs=RetrieveLocalCPs(cur_node);
        
//        //计算步长,主要是考虑起点q2是不是位于地图内
//     //    if( swin_map_->isInMap(cur_node->position) == false )    
//     //    {
//     //      std::cout << "起点q2 不在地图内!其数值为:"<<q0q1q2[2].transpose() <<std::endl;
//     //     continue;
//     //    }
//         double step = GetStep(swin_map_->getDistance(cur_node->grid));

//         //  std::cout << "step:"<<step<<std::endl;
//         // std::cout << "LocalCPs0:"<<LocalCPs[0].transpose()<<std::endl;
//         // std::cout << "LocalCPs1:"<<LocalCPs[1].transpose() <<std::endl;
//         // std::cout << "LocalCPs2:"<<LocalCPs[2].transpose() <<std::endl;


//         //到达终点  将新建qn-2为最终点，这种写法目的是和下文保持一致
//         if((cur_node->position-goal_next).norm()<=step&&CheckDynamic(LocalCPs,goal_next))
//         {
//             if(iterator_temp==Directional_Points.end())
//             {
//                 NodePtr end_node=new Node;
//                 end_node->parent=cur_node;
//                 end_node->step=step;
//                 end_node->position=qn_2qn_1qn[0];
//                 end_node->grid= swin_map_->WorldToGrid(qn_2qn_1qn[0]);
//                 retrievePartCPs(end_node);
//                 path=getPartCPs();
//                 for(int i=1;i<qn_2qn_1qn.size();i++)  path.push_back(qn_2qn_1qn[i]);  //补齐qn-1 和qn

//                 return   Vectors2matrix(path);
//             }
//             else
//             {
//                 iterator_temp++;
//                 Eigen::Vector3d goal_next=*iterator_temp;


//             }
//         }
        
//        //扩展该节点
//        std::vector<Eigen::Vector3d> q_nbrs;
//         q_nbrs=DirectionSafeNodeExpansion(LocalCPs,cur_node,step,(goal_next-cur_node->position).normalized());
//         for(int i=0;i<q_nbrs.size();i++)
//         {
//             Eigen::Vector3d q_nbr = q_nbrs[i];

//             //如果不在地图内，则跳过
//             if( swin_map_->isInMap(q_nbr) == false )    continue;

//             //如果邻居点离障碍物太近，则跳过
//             if(swin_map_->getDistance(q_nbr)<=0)  continue;

//             //如果不满足动力学约束，则跳过
//             if(CheckDynamic(LocalCPs,q_nbr) == false)   continue;

//             //如果在close_set内，则跳过
//             Eigen::Vector3i temp_id = swin_map_->WorldToGrid(q_nbr);
//             // NodePtr temp_node = expanded_nodes_.find(temp_id);
//             NodePtr temp_node = expanded_nodes_.find(q_nbr);
            
//             if (temp_node != NULL && temp_node->node_state == IN_CLOSE_SET)  continue;

//             //计算代价
//             // double temp_g_score = cur_node->g_score+delta_t_;
//             double temp_time_score = cur_node->time_score+delta_t_;
//             double temp_g_score = cur_node->g_score+(q_nbr-cur_node->position).norm();
//             double temp_f_score = temp_g_score+lambda_heu_*getEuclHeu(q_nbr,qn_2_goal);   // getDiagHeu(q_nbr,qn_2_goal)
//             // double temp_f_score = lambda_heu_*getEuclHeu(q_nbr,qn_2_goal);   // getDiagHeu(q_nbr,qn_2_goal)
//             //如果该点未在open_set中，直接加入
//             if(temp_node==NULL)
//             {
//                 temp_node = ctrl_node_pool_[use_node_num_++];
//                 temp_node->position = q_nbr;
//                 temp_node->grid = temp_id;
//                 temp_node->time_score=temp_time_score;
//                 temp_node->f_score = temp_f_score;
//                 temp_node->g_score = temp_g_score;
//                 temp_node->parent = cur_node;
//                 temp_node->step=step;
//                 temp_node->node_state = IN_OPEN_SET;
//                 open_set_.push(temp_node);
//                 expanded_nodes_.insert(temp_node->position,temp_node);
//                 // expanded_nodes_.insert(temp_node->grid,temp_node);

//                if (use_node_num_ == allocate_num_)
//                 {
//                     cout << "run out of memory." << endl;
//                     visualization_->drawVisitedNode(getVisitedNodes(),0.1,visualization_->Colors_Formula_[visualization_->Purple]);
//                     return   Vectors2matrix(path);
//                  }

//             } 
//             //如果该点在open_set中
//             else if (temp_node->node_state == IN_OPEN_SET) 
//             {
//                 if (temp_g_score < temp_node->g_score) 
//                 {
//                     temp_node->position = q_nbr;
//                     temp_node->time_score=temp_time_score;
//                     temp_node->f_score = temp_f_score;
//                     temp_node->g_score = temp_g_score;
//                     temp_node->parent = cur_node;
//                     temp_node->step=step;
//                 }
//             }else  std::cout << "error type in searching: " << temp_node->node_state << std::endl;

//         }
//     }

//   /* ---------- open set empty, no path ---------- */
//   std::cout << "open set empty, 搜索失败"<< std::endl;
//   std::cout << "use node num: " << use_node_num_ << std::endl;
//   std::cout << "iter num: " << iter_num_ << std::endl;

//   std::cout << "起点q2 为:"<<q0q1q2[2].transpose() <<std::endl;
//   visualization_->drawVisitedNode(getVisitedNodes(),0.1,visualization_->Colors_Formula_[visualization_->Purple]);
// //   std::cout << "起点q2 的邻居为:"<<q_nbrs.size() <<std::endl;
//   return Vectors2matrix(path);
// }

// vector<NodePtr> UUCtrlAstar::Stage_DirectionalExpand(vector<NodePtr>& q0q1q2_stage,const Eigen::Vector3d& goal_next,bool final_goal)
// {
//     //分段搜索之前应该清空open set和close set
//     expanded_nodes_.clear();
//     std::priority_queue<NodePtr, std::vector<NodePtr>, NodeComparator0> empty_queue;
//     open_set_.swap(empty_queue);

//     NodePtr cur_node;//当前节点

//     for(int i=0;i<degree_-1;i++)
//     {
//         cur_node = q0q1q2_stage[i];
//         expanded_nodes_.insert(cur_node->position,cur_node);
//     }

//     cur_node = q0q1q2_stage[2];
//     cur_node->node_state=IN_OPEN_SET;
//     cur_node->g_score=0.0;
//     cur_node->f_score=lambda_heu_*getDiagHeu(cur_node->position,goal_next);                //getEuclHeu(cur_node->position,qn_2_goal);    
//     open_set_.push(cur_node);
//     expanded_nodes_.insert(cur_node->position,cur_node);

//  while(!open_set_.empty())
//     {
//         cur_node = open_set_.top();
//         /* ---------- pop node and add to close set ---------- */
//         open_set_.pop();
//         cur_node->node_state = IN_CLOSE_SET;
//         iter_num_ += 1;

//         //得到局部曲线的前degree_个控制点
//         std::vector<Eigen::Vector3d> LocalCPs;
//         LocalCPs=RetrieveLocalCPs(cur_node);

//         double step = GetStep(swin_map_->getDistance(cur_node->grid));

//         //到达终点  将新建qn-2为最终点，这种写法目的是和下文保持一致
//         if((cur_node->position-goal_next).norm()<=step&&CheckDynamic(LocalCPs,goal_next))
//         {
//             if(final_goal)
//             {
//                 NodePtr end_node=new Node;
//                 end_node->parent=cur_node;
//                 end_node->step=step;
//                 end_node->position=goal_next;
//                 end_node->grid= swin_map_->WorldToGrid(goal_next);
//             }
//         }
        
//        //扩展该节点
//        std::vector<Eigen::Vector3d> q_nbrs;
//         q_nbrs=DirectionSafeNodeExpansion(LocalCPs,cur_node,step,(goal_next-cur_node->position).normalized());
        
//         for(int i=0;i<q_nbrs.size();i++)
//         {
//             Eigen::Vector3d q_nbr = q_nbrs[i];

//             //如果不在地图内，则跳过
//             if( swin_map_->isInMap(q_nbr) == false )    continue;

//             //如果邻居点离障碍物太近，则跳过
//             if(swin_map_->getDistance(q_nbr)<=0)  continue;

//             //如果不满足动力学约束，则跳过
//             if(CheckDynamic(LocalCPs,q_nbr) == false)   continue;

//             //如果在close_set内，则跳过
//             Eigen::Vector3i temp_id = swin_map_->WorldToGrid(q_nbr);
//             NodePtr temp_node = expanded_nodes_.find(q_nbr);
            
//             if (temp_node != NULL && temp_node->node_state == IN_CLOSE_SET)  continue;

//             //计算代价
//             double temp_time_score = cur_node->time_score+delta_t_;
//             double temp_g_score = cur_node->g_score+(q_nbr-cur_node->position).norm();
//             double temp_f_score = temp_g_score+lambda_heu_*getEuclHeu(q_nbr,qn_2_goal);   // getDiagHeu(q_nbr,qn_2_goal)
//             //如果该点未在open_set中，直接加入
//             if(temp_node==NULL)
//             {
//                 temp_node = ctrl_node_pool_[use_node_num_++];
//                 temp_node->position = q_nbr;
//                 temp_node->grid = temp_id;
//                 temp_node->time_score=temp_time_score;
//                 temp_node->f_score = temp_f_score;
//                 temp_node->g_score = temp_g_score;
//                 temp_node->parent = cur_node;
//                 temp_node->step=step;
//                 temp_node->node_state = IN_OPEN_SET;
//                 open_set_.push(temp_node);
//                 expanded_nodes_.insert(temp_node->position,temp_node);

//                if (use_node_num_ == allocate_num_)
//                 {
//                     cout << "run out of memory." << endl;
//                  }

//             } 
//             //如果该点在open_set中
//             else if (temp_node->node_state == IN_OPEN_SET) 
//             {
//                 if (temp_g_score < temp_node->g_score) 
//                 {
//                     temp_node->position = q_nbr;
//                     temp_node->time_score=temp_time_score;
//                     temp_node->f_score = temp_f_score;
//                     temp_node->g_score = temp_g_score;
//                     temp_node->parent = cur_node;
//                     temp_node->step=step;
//                 }
//             }else  std::cout << "error type in searching: " << temp_node->node_state << std::endl;

//         }

//     }

// }




