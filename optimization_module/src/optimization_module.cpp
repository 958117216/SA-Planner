//
// Created by gyl on 2022/3/27.
//

#include "optimization_module/optimization_module.h"


void optimization::q2x(const Eigen::MatrixXd &q, std::vector<double> &x)
{
    x.clear();

    // std::cout << "q has size= " << q.size() << std::endl;
    // std::cout << "n has size= " << n.size() << std::endl;

    for (int i = 3; i <= (variable_num_/q_dim_)+2; i++)  //N_-2
    {
        for (int j = 0; j < q_dim_; j++)
        {
            x.push_back(q(j,i));
        }
    }

}

template <class T>
void optimization::x2q(const T &x,Eigen::MatrixXd &q)
{
    // q=Eigen::MatrixXd::Zero(q_dim_,N_+1);
    q.resize(q_dim_,N_+1);
    q.col(0)=q0_; // Not a decision variable
    q.col(1)=q1_; // Not a decision variable
    q.col(2)=q2_; // Not a decision variable

    // Control points (3x1)
    for (int i = 3; i <= (variable_num_/q_dim_)+2; i++) //N_-2
    {
        for(int j=0;j<q_dim_;j++)
        {
            q(j,i)=x[q_dim_*(i-3)+j];
        }
    }
    for (int i = (variable_num_/q_dim_)+3; i <= N_; i++) //将末尾填入 Not a decision variable
    {
        q.col(i)=q.col(i-1);
    }

}

/*轨迹优化部分*/
Eigen::MatrixXd optimization::traj_optimization(const Eigen::MatrixXd &ctl_points,const Eigen::RowVectorXd &step,const Eigen::MatrixXd &corridors,
                                                const double &delta_t,const Eigen::MatrixXd &pos_goal)
{
    //从初始解获得问题信息
    q_dim_=ctl_points.rows();
    q0_=ctl_points.col(0);
    q1_=ctl_points.col(1);
    q2_=ctl_points.col(2);
    q_=ctl_points;
    delta_t_=delta_t;
    N_=ctl_points.cols()-1;
    num_of_segments_=ctl_points.cols()-3;
    traj_goal_=pos_goal;
    variable_num_=q_dim_*(N_+1-5);
    q2x(q_,x_);
    // inequality_num_=14*N_-35;
    inequality_num_=13*N_-31;//-(N_-4);
    Vmax_=2.0;
    Amax_=2.0;
    initial_FixedStep_=step;
    initial_Fix_q_=ctl_points;
    corridors_=corridors;

    // nlopt::opt opt(nlopt::LD_SLSQP, variable_num_);

    nlopt::opt opt(nlopt::AUGLAG, variable_num_);
    nlopt::opt local_opt(nlopt::LD_LBFGS, variable_num_);
    opt.set_maxeval(500);
    // local_opt.set_maxtime(0.05);
    local_opt.set_maxeval(500);
    // local_opt.set_xtol_rel(0.000000001);
    // local_opt.set_ftol_rel(0.0000000000001);

    opt.set_local_optimizer(local_opt);
    // opt.set_maxtime(0.05);
    // opt.set_xtol_rel(0.000000001);
    // opt.set_ftol_rel(0.000000000001);
    
    //*常规变量的上下限
    std::vector<double> lb(variable_num_), ub(variable_num_);
    const double   bound = 2.0;   //已修改  原为10.0
    for (int i = 0; i < variable_num_; ++i)
    {
        lb[i] = x_[i] - bound;
        ub[i] = x_[i] + bound;
    }
    opt.set_lower_bounds(lb);
    opt.set_upper_bounds(ub);

    //*引入Corridor 常规变量的上下限 
    //   //采用飞行走廊硬约束
    //   vector<double> ESDF_results_part(variable_num_);
    //  //对控制点的ESDF信息进行评估
    //   Eigen::Vector3d temp_points;
    //   vector<double> ESDF_results;
    //   double temp_esdf;

    //     for(int i = 0; i <q_.cols();i++)
    //     {
    //         temp_points=q_.col(i);
    //         temp_esdf=map_module_->getDistance(temp_points)-1e-5;
    //         ESDF_results.push_back(temp_esdf);
    //     }
    //     //根据优化的变量数量,读取相应的飞行走廊大小  ESDF_results_part
    //     for (int i = 3; i <=N_-2; ++i) {
    //         for (int j = 0; j < 3; j++) {
    //             ESDF_results_part[3 * (i - 3) + j]=ESDF_results[i];
    //         }
    //     }
    //     std::vector<double> lb(variable_num_), ub(variable_num_);
    //     for (int i = 0; i < variable_num_; ++i)
    //     {
    //         if(ESDF_results_part[i]>=0.3) ESDF_results_part[i]-=0.3;
    //         lb[i] = x_[i] - ESDF_results_part[i];
    //         ub[i] = x_[i] + ESDF_results_part[i];
    //     }
    //     opt.set_lower_bounds(lb);
    //     opt.set_upper_bounds(ub);


    //*引入Corridor 常规变量的上下限 V2.0
        // std::vector<double> lb(variable_num_), ub(variable_num_);
        // for (int i = 0; i < variable_num_; ++i)
        // {
        //     int row=i%3;
        //     int col=i/3;
        //     lb[i] =corridors_(row,col);
        //     if( x_[i]<lb[i]) x_[i]=lb[i];
        //     ub[i] = corridors_(row+3,col);
        //     if(x_[i]>ub[i]) x_[i]=ub[i];
        //     // lb[i] = min(corridors_(row,col), x_[i]);
        //     // ub[i] = max(corridors_(row+3,col), x_[i]);
        // }
        // opt.set_lower_bounds(lb);
        // opt.set_upper_bounds(ub);


    std::vector<double> tol_constraints;
    for (int i = 0; i < inequality_num_; i++)
    {
        tol_constraints.push_back(1e-6);
    }

    opt.add_inequality_mconstraint(optimization::traj_multiconstraints, this, tol_constraints);
    opt.set_min_objective(optimization::traj_objfunc,this);

    std::cout<<"优化之前的代价:"<<GetTraj_objfunc(initial_Fix_q_)<<std::endl;
    best_cost_=GetTraj_objfunc(initial_Fix_q_);

    try{
        double        final_cost;
        nlopt::result result = opt.optimize(x_, final_cost);
    } catch (std::exception& e) {
        std::cout << e.what() << std::endl;
    }

    x2q(x_,q_);
    std::cout<<"优化之后的代价:"<<GetTraj_objfunc(q_)<<std::endl;
    std::cout<<"优化2.0之后的代价:"<<GetTraj_objfunc(best_q_)<<std::endl;
    
    // return q_;
    return best_q_;
}

double optimization::traj_objfunc(const std::vector<double> &x, std::vector<double> &grad, void *func_data)
{
    optimization* opt = reinterpret_cast<optimization*>(func_data);
    Eigen::MatrixXd q;
    opt->x2q(x, q);
    return opt->computeTraj_objfunc(q,grad);
}

double optimization::computeTraj_objfunc(const Eigen::MatrixXd &q, std::vector<double> &grad)
{
    double cost=0.0;
    Eigen::Matrix4d  pos_bs_basis_matrix;
    pos_bs_basis_matrix<<
        -0.166667, 0.500000, -0.500000, 0.166667,
        0.500000, -1.000000, 0.000000, 0.666667,
        -0.500000, 0.500000, 0.500000, 0.166667,
        0.166667, 0.000000, 0.000000, 0.000000;
    Eigen::Matrix<double, 4, 1> tmp;
    tmp << 6.0, 0.0, 0.0, 0.0;
    Eigen::Matrix<double, 4, 1> jerk_bs_basis_matrix=pos_bs_basis_matrix*tmp;

    //标量对向量求导
    Eigen::MatrixXd partial_derivative=Eigen::MatrixXd::Zero(q_dim_,N_+1);

    //Jerk^2
    for (int i = 0; i <num_of_segments_; i++)
    {
        Eigen::Matrix<double, 3, 4> Q;
        Q.col(0) = q.col(i);
        Q.col(1) = q.col(i+1);
        Q.col(2) = q.col(i+2);
        Q.col(3) = q.col(i+3);

        // cost += (Q * jerk_bs_basis_matrix).squaredNorm();
        cost+=1*jerk_bs_basis_matrix.transpose()*Q.transpose()*Q*jerk_bs_basis_matrix;
        // I only need to do this if grad!=NULL
        partial_derivative.block<3,4>(0,i)+= 1*2*Q*jerk_bs_basis_matrix*jerk_bs_basis_matrix.transpose();
    }
    //qn-2与终点 的代价
    cost+=10*(q.col(N_-2)-traj_goal_).transpose()*(q.col(N_-2)-traj_goal_);
    partial_derivative.block<3,1>(0,N_-2)+=10*2*(q.col(N_-2)-traj_goal_);

    //距离
    for (int k = 3; k <= N_-2; ++k)
    {
         Eigen::Vector3d qk=q.col(k);
         double dist=map_module_->getDistance(qk); 
         Eigen::Vector3d gradient=map_module_->getGradient(qk);
         map_module_->evaluateEDTWithGrad(qk,dist,gradient);
         if(dist<=0.3)
         {
            cost += 10*pow(dist - 0.3, 2);
            partial_derivative.block<3,1>(0,k)+=10*2.0 * (dist - 0.3) *gradient;
         }
    }


    if (!grad.empty()) {
        q2x(partial_derivative, grad);
    }
    // std::cout<<"优化中,代价:"<<cost<<std::endl;

    if(best_cost_>cost)
    {
        best_cost_=cost;
        best_q_=q;
    }


    return cost;
}

void optimization::traj_multiconstraints(unsigned int m, double *result, unsigned int n, const double *x, double *grad,
                                         void *f_data){
    optimization* opt = reinterpret_cast<optimization*>(f_data);
    Eigen::MatrixXd q;
    opt->x2q(x, q);
    opt->computeTraj_multiconstraints(n,q,m,result,grad);
}

void optimization::computeTraj_multiconstraints(unsigned n, const Eigen::MatrixXd &q, unsigned m, double *result, double* grad)
{
    conversion_BS2MV BS2MV(delta_t_);
    int index_result=0;
    int index_grad=0;

    if (grad)
    {
        for (int i = 0; i < n * m; i++)
        {
            grad[i] = 0.0;
        }
    }

    //*速度不等式约束  每段轨迹有3（维度）*3个速度控制点，总共有9*num_of_segments_=9*(N_-2)个约束
    for (int i = 0; i < num_of_segments_; ++i)
    {
        Eigen::Matrix<double, 3, 4> Q_BS;
        Eigen::Matrix<double, 3, 3> V_MV;
        Eigen::Matrix<double, 3, 3> vel_constraints;
        Q_BS.col(0) = q.col(i);
        Q_BS.col(1) = q.col(i+1);
        Q_BS.col(2) = q.col(i+2);
        Q_BS.col(3) = q.col(i+3);

        V_MV=Q_BS*BS2MV.bs_pos2vel_*BS2MV.vel_bs2mv_;
        vel_constraints=V_MV.cwiseProduct(V_MV)-Vmax_*Vmax_*Eigen::Matrix<double, 3, 3>::Ones();

        for (int n = 0; n < 3; ++n)
        {
            for (int m = 0; m < 3; ++m)  //维度
            {
                result[index_result++]=vel_constraints(m,n);
            }
        }

        if (grad)
        {
            //标量对向量求导
            Eigen::MatrixXd  f_PartialDerivative_x=Eigen::MatrixXd::Zero(q_dim_,N_+1);;

            std::vector<double> vector_grad;
            for (int n = 0; n < 3; ++n)
            {
                for (int m = 0; m < 3; ++m)
                {
                    Eigen::Vector3d a=Eigen::Vector3d::Zero();
                    Eigen::Vector3d b=Eigen::Vector3d::Zero();
                    a(m)=1; b(n)=1;
                    f_PartialDerivative_x.block<3,4>(0,i)=(BS2MV.bs_pos2vel_*BS2MV.vel_bs2mv_*
                            ((a*b.transpose()).cwiseProduct(2*Q_BS*BS2MV.bs_pos2vel_*BS2MV.vel_bs2mv_)).transpose()).transpose();
                    q2x(f_PartialDerivative_x,vector_grad);

                    std::copy(vector_grad.begin(), vector_grad.end(), &grad[index_grad]);
                    index_grad+=vector_grad.size();
                }
            }


        }
    }

    //*加速度不等式约束  只需要考虑 每段轨迹的3（维度）*1个（第二个）加速度控制点，总共有3*（num_of_segments_-1）=3*(N_-3)个约束 （最后一段轨迹不考虑）
    for (int i = 0; i < num_of_segments_-1; ++i)
    {

        Eigen::Matrix<double, 3, 4> Q_BS;
        Eigen::Matrix<double, 3, 2> Acc_BS;
        Eigen::Matrix<double, 3, 1> acc_constraints;
        Q_BS.col(0) = q.col(i);
        Q_BS.col(1) = q.col(i + 1);
        Q_BS.col(2) = q.col(i + 2);
        Q_BS.col(3) = q.col(i + 3);

        Acc_BS=Q_BS*BS2MV.bs_pos2vel_*BS2MV.bs_vel2acc_;
        acc_constraints= (Acc_BS.cwiseProduct(Acc_BS)).block<3,1>(0,1)-Amax_*Amax_*Eigen::Matrix<double, 3, 1>::Ones();

        for (int n = 0; n < 1; ++n)
        {
            for (int m = 0; m < 3; ++m)  //维度
            {
                result[index_result++]=acc_constraints(m,n);
            }
        }

        if (grad)
        {
            //标量对向量求导
            Eigen::MatrixXd  f_PartialDerivative_x=Eigen::MatrixXd::Zero(q_dim_,N_+1);;

            std::vector<double> vector_grad;
            for (int n = 0; n < 1; ++n)
            {
                for (int m = 0; m < 3; ++m) //维度
                {
                    Eigen::Vector3d a=Eigen::Vector3d::Zero();
                    Eigen::Vector2d b=Eigen::Vector2d::Zero();
                    a(m)=1; b(n)=1;
                    f_PartialDerivative_x.block<3,4>(0,i)=(BS2MV.bs_pos2vel_*BS2MV.bs_vel2acc_*
                                                           ((a*b.transpose()).cwiseProduct(2*Q_BS*BS2MV.bs_pos2vel_*BS2MV.bs_vel2acc_)).transpose()).transpose();
                    q2x(f_PartialDerivative_x,vector_grad);

                    std::copy(vector_grad.begin(), vector_grad.end(), &grad[index_grad]);
                    index_grad+=vector_grad.size();
                }
            }


        }


    }

    if(0)
    {
        //*两两控制点之间的步长约束   从（q2，q3）开始-到（qn-3，qn-2）结束，共n-4=N_-4个约束
        for (int k = 3; k <= N_-2; ++k)
        {
            Eigen::Matrix<double, 3, 2> Q;
            Q.col(0) = q.col(k-1);
            Q.col(1) = q.col(k);

            Eigen::Vector2d A;
            A<<-1,1;
            result[index_result++]=A.transpose()*Q.transpose()*Q*A-initial_FixedStep_[k]*initial_FixedStep_[k];

            if(grad)
            {
                //标量对向量求导
                Eigen::MatrixXd  f_PartialDerivative_x=Eigen::MatrixXd::Zero(q_dim_,N_+1);;

                std::vector<double> vector_grad;

                f_PartialDerivative_x.block<3,2>(0,k-1)=2*Q*A*A.transpose();
                q2x(f_PartialDerivative_x,vector_grad);

                std::copy(vector_grad.begin(), vector_grad.end(), &grad[index_grad]);
                index_grad+=vector_grad.size();
            }
        }
    
       
       
        //*碰撞约束 从q3到qn-2，共N_-4个约束
        for (int k = 3; k <= N_-2; ++k)
        {
            Eigen::Matrix<double, 3, 1> Q;
            Q.col(0) = q.col(k);


            result[index_result++]=(Q-initial_Fix_q_.col(k-1)).transpose()*(Q-initial_Fix_q_.col(k-1))-initial_FixedStep_[k]*initial_FixedStep_[k];

            if(grad)
            {
                //标量对向量求导
                Eigen::MatrixXd  f_PartialDerivative_x=Eigen::MatrixXd::Zero(q_dim_,N_+1);;

                std::vector<double> vector_grad;

                f_PartialDerivative_x.block<3,1>(0,k)=2*(Q-initial_Fix_q_.col(k-1));
                q2x(f_PartialDerivative_x,vector_grad);

                std::copy(vector_grad.begin(), vector_grad.end(), &grad[index_grad]);
                index_grad+=vector_grad.size();

            }

        }          

    }

    //*两两控制点之间的步长约束v2.0版   从（q2，q3）开始-到（qn-3，qn-2）结束，共n-4=N_-4个约束
    if(1)
    {
         for (int k = 3; k <= N_-2; ++k)
    {
        Eigen::Matrix<double, 3, 2> Q;
        Q.col(0) = q.col(k-1);
        Q.col(1) = q.col(k);

        Eigen::Vector3d qk_1=q.col(k-1);
        //此处是否要加判断是否在地图内
        double dist=map_module_->getDistance(qk_1); 
        // double step=2/(1+exp(1.3-dist));
        double step;

        if(dist <=0.3)
        {
            step=0;
        }else if(dist>2.3)
        {
            step=2;
        }else
        {
            step=dist-0.3;
        }


        Eigen::Vector2d A;
        A<<-1,1;
        result[index_result++]=A.transpose()*Q.transpose()*Q*A-step*step;

        if(grad)
        {
            //标量对向量求导
            Eigen::MatrixXd  f_PartialDerivative_x=Eigen::MatrixXd::Zero(q_dim_,N_+1);
            std::vector<double> vector_grad;

            if(0.3<dist&&dist<=2.3)
            {
                Eigen::Vector3d gradient=map_module_->getGradient(qk_1);
                f_PartialDerivative_x.block<3,1>(0,k-1)= -2*(Q.col(1)- Q.col(0))-2*step*(gradient.normalized());
            }else
            {
                f_PartialDerivative_x.block<3,1>(0,k-1)= -2*(Q.col(1)- Q.col(0));
            }
            
            f_PartialDerivative_x.block<3,1>(0,k)=2*(Q.col(1)- Q.col(0));


            q2x(f_PartialDerivative_x,vector_grad);

            std::copy(vector_grad.begin(), vector_grad.end(), &grad[index_grad]);
            index_grad+=vector_grad.size();
        }
    }
    }
    
}

double optimization::GetTraj_objfunc(const Eigen::MatrixXd &q) {
    std::vector<double> empty_grad;
    return computeTraj_objfunc(q,empty_grad);

}



/*Yaw优化部分*/
Eigen::MatrixXd optimization::yaw_optimization(const Eigen::MatrixXd &ctl_points,const double &delta_t,const vector<double> &waypoints)
{
    //从初始解获得问题信息
    q_dim_=ctl_points.rows();
    q0_=ctl_points.col(0);
    q1_=ctl_points.col(1);
    q2_=ctl_points.col(2);
    q_=ctl_points;
    delta_t_=delta_t;
    N_=ctl_points.cols()-1;
    num_of_segments_=ctl_points.cols()-3;
    waypoints_=waypoints;


    variable_num_=q_dim_*(N_+1-6);

    q2x(q_,x_);


    nlopt::opt opt(nlopt::LD_MMA, variable_num_);

    opt.set_min_objective(optimization::yaw_objfunc,this);
    opt.set_maxeval(500);
    opt.set_maxtime(0.005);
    opt.set_xtol_rel(1e-5);

    try{
        double        final_cost;
        nlopt::result result = opt.optimize(x_, final_cost);
    } catch (std::exception& e) {
        std::cout << e.what() << std::endl;
    }

    x2q(x_,q_);
    
    return q_;
    // return best_q_;
}

double optimization::yaw_objfunc(const std::vector<double> &x, std::vector<double> &grad, void *func_data)
{
    optimization* opt = reinterpret_cast<optimization*>(func_data);
    Eigen::MatrixXd q;
    opt->x2q(x, q);
    return opt->computeYaw_objfunc(q,grad);
}

double optimization::computeYaw_objfunc(const Eigen::MatrixXd &q, std::vector<double> &grad)
{
    double cost=0.0;
    Eigen::Matrix4d  pos_bs_basis_matrix;
    pos_bs_basis_matrix<<
        -0.166667, 0.500000, -0.500000, 0.166667,
        0.500000, -1.000000, 0.000000, 0.666667,
        -0.500000, 0.500000, 0.500000, 0.166667,
        0.166667, 0.000000, 0.000000, 0.000000;
    Eigen::Matrix<double, 4, 1> tmp;
    tmp << 6.0, 0.0, 0.0, 0.0;
    Eigen::Matrix<double, 4, 1> jerk_bs_basis_matrix=pos_bs_basis_matrix*tmp;

    //标量对向量求导
    Eigen::MatrixXd partial_derivative=Eigen::MatrixXd::Zero(q_dim_,N_+1);

    //Jerk^2
    for (int i = 0; i <num_of_segments_; i++)
    {
        Eigen::Matrix<double, 1, 4> Q;  //Attention!!
        Q.col(0) = q.col(i);
        Q.col(1) = q.col(i+1);
        Q.col(2) = q.col(i+2);
        Q.col(3) = q.col(i+3);

        // cost += (Q * jerk_bs_basis_matrix).squaredNorm();
        cost+=1*jerk_bs_basis_matrix.transpose()*Q.transpose()*Q*jerk_bs_basis_matrix;
        // I only need to do this if grad!=NULL
        partial_derivative.block<1,4>(0,i)+= 1*2*Q*jerk_bs_basis_matrix*jerk_bs_basis_matrix.transpose();
    }


    Eigen::Matrix<double, 4, 1> first_point_matrix;
    first_point_matrix<<1.0/6,2.0/3,1.0/6,0.0;

    //(q-waypoint)^2
    for (int i = 1; i <num_of_segments_; i++)
    {
        Eigen::Matrix<double, 1, 4> Q;  //Attention!!
        Q.col(0) = q.col(i);
        Q.col(1) = q.col(i+1);
        Q.col(2) = q.col(i+2);
        Q.col(3) = q.col(i+3);

        // cost += (Q * jerk_bs_basis_matrix).squaredNorm();
        cost+=10*(Q*first_point_matrix-waypoints_[i])*(Q*first_point_matrix-waypoints_[i]);
        // I only need to do this if grad!=NULL
        partial_derivative.block<1,4>(0,i) += 10*(2*(Q*first_point_matrix-waypoints_[i])*first_point_matrix).transpose();
    }



    if (!grad.empty()) {
        q2x(partial_derivative, grad);
    }

    // if(best_cost_>cost)
    // {
    //     best_cost_=cost;
    //     best_q_=q;
    // }

    return cost;
}





