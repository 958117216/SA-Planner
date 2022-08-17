#include "bspline_module/unclamped_uniform_bspline.h"


UnclampedUniformBspline::UnclampedUniformBspline(const Eigen::MatrixXd& points, const int& degree, const double& interval)
{
    control_points_=points;
    p_=degree;
    interval_=interval;

    n_ = points.cols()- 1;
    m_ = n_ + p_ + 1;

    u_ = Eigen::VectorXd::Zero(m_ + 1);

    //unclamped 
    for (int i = 0; i <= m_; ++i) 
    {
        if (i <= p_) {
        u_(i) = double(-p_ + i) * interval_;
        } else if (i > p_ && i <= m_ - p_) {
        u_(i) = u_(i - 1) + interval_;
        } else if (i > m_ - p_) {
        u_(i) = u_(i - 1) + interval_;
        }
    }
}

void UnclampedUniformBspline::setKnot(const Eigen::VectorXd& knot) { this->u_ = knot; }

Eigen::VectorXd UnclampedUniformBspline::getKnot() { return this->u_; }

void UnclampedUniformBspline::getTimeSpan(double& um, double& um_p) {  um   = u_(p_);  um_p = u_(m_ - p_); }

Eigen::MatrixXd UnclampedUniformBspline::getControlPoint() { return control_points_; }

std::vector<Eigen::VectorXd> UnclampedUniformBspline::calKnotPoints() 
{
    std::vector<Eigen::VectorXd> KnotPoints;

    for(int i=p_; i<=m_+1-p_; i++)
    {
        KnotPoints.push_back(evaluateDeBoor(u_(i)));
    }
   return KnotPoints; 

}


std::pair<Eigen::VectorXd, Eigen::VectorXd> UnclampedUniformBspline::getHeadTailPts() {
  Eigen::VectorXd head = evaluateDeBoor(u_(p_));
  Eigen::VectorXd tail = evaluateDeBoor(u_(m_ - p_));
  return std::make_pair(head, tail);
}

Eigen::VectorXd UnclampedUniformBspline::evaluateDeBoor(const double& u) {

  double ub = std::min(std::max(u_(p_), u), u_(m_ - p_));//确保u在定义域内

  // determine which [ui,ui+1] lay in
  int k = p_;
  while (true) {
    if (u_(k + 1) >= ub) break;
    ++k;
  }

  /* deBoor's alg */ //取决定该段曲线的四个控制点
  std::vector<Eigen::VectorXd> d;
  for (int i = 0; i <= p_; ++i) {
    d.push_back(control_points_.col(k - p_ + i));
    // cout << d[i].transpose() << endl;
  }

  for (int r = 1; r <= p_; ++r) {
    for (int i = p_; i >= r; --i) {
      double alpha = (ub - u_[i + k - p_]) / (u_[i + 1 + k - r] - u_[i + k - p_]);
      // cout << "alpha: " << alpha << endl;
      d[i] = (1 - alpha) * d[i - 1] + alpha * d[i];
    }
  }

  return d[p_];
}

Eigen::VectorXd UnclampedUniformBspline::evaluateDeBoorT(const double& t) {
  return evaluateDeBoor(t + u_(p_));
}

Eigen::MatrixXd UnclampedUniformBspline::getDerivativeControlPoints() {
  // The derivative of a b-spline is also a b-spline, its order become p_-1
  // control point Qi = p_*(Pi+1-Pi)/(ui+p_+1-ui+1)
  Eigen::MatrixXd ctp = Eigen::MatrixXd::Zero(control_points_.rows(), control_points_.cols()-1);
  for (int i = 0; i < ctp.cols(); ++i) {
    ctp.col(i) = p_ * (control_points_.col(i + 1) - control_points_.col(i)) / (u_(i + p_ + 1) - u_(i + 1));
  }
  return ctp;
}

UnclampedUniformBspline UnclampedUniformBspline::getDerivative()
{

  Eigen::MatrixXd   ctp = getDerivativeControlPoints();
  UnclampedUniformBspline derivative(ctp, p_ - 1, interval_);

  /* 非均匀B样条需要重置节点向量，因为初始化默认是均匀B样条 */
  Eigen::VectorXd knot(u_.rows() - 2);
  knot = u_.segment(1, u_.rows() - 2);
  derivative.setKnot(knot);

  return derivative;
}

double UnclampedUniformBspline::getTimeSum()
{
    double um,um_p;
    getTimeSpan(um,um_p);
    return um_p-um;
}

double UnclampedUniformBspline::getLength(const double& res = 0.01)
{
    double          length = 0.0;
    double          dur    = getTimeSum();
    Eigen::VectorXd p_l    = evaluateDeBoorT(0.0), p_n;
    for (double t = res; t <= dur + 1e-4; t += res) {
        p_n = evaluateDeBoorT(t);
        length += (p_n - p_l).norm();
        p_l = p_n;
    }
    return length;
}

double UnclampedUniformBspline::getJerk()
{
  UnclampedUniformBspline jerk_traj = getDerivative().getDerivative().getDerivative();

  Eigen::VectorXd times     = jerk_traj.getKnot();
  Eigen::MatrixXd ctrl_pts  = jerk_traj.getControlPoint();
  int             dimension = ctrl_pts.rows();

  double jerk = 0.0;
  for (int i = 0; i < ctrl_pts.cols(); ++i) {
    for (int j = 0; j < dimension; ++j) {
      jerk += (times(i + 1) - times(i)) * ctrl_pts(i, j) * ctrl_pts(i, j);
    }
  }

  return jerk;
}

void   UnclampedUniformBspline::getMeanAndMaxVel(double& mean_v, double& max_v)
{
  UnclampedUniformBspline vel = getDerivative();
  double            tm, tmp;
  vel.getTimeSpan(tm, tmp);

  double max_vel = -1.0, mean_vel = 0.0;
  int    num = 0;
  for (double t = tm; t <= tmp; t += 0.01) {
    Eigen::VectorXd vxd = vel.evaluateDeBoor(t);
    double          vn  = vxd.norm();

    mean_vel += vn;
    ++num;
    if (vn > max_vel) {
      max_vel = vn;
    }
  }

  mean_vel = mean_vel / double(num);
  mean_v   = mean_vel;
  max_v    = max_vel;
}

void   UnclampedUniformBspline::getMeanAndMaxAcc(double& mean_a, double& max_a)
{
  UnclampedUniformBspline acc = getDerivative().getDerivative();
  double            tm, tmp;
  acc.getTimeSpan(tm, tmp);

  double max_acc = -1.0, mean_acc = 0.0;
  int    num = 0;
  for (double t = tm; t <= tmp; t += 0.01) {
    Eigen::VectorXd axd = acc.evaluateDeBoor(t);
    double          an  = axd.norm();

    mean_acc += an;
    ++num;
    if (an > max_acc) {
      max_acc = an;
    }
  }

  mean_acc = mean_acc / double(num);
  mean_a   = mean_acc;
  max_a    = max_acc;
}

double UnclampedUniformBspline::getInterval() { return interval_; }


void UnclampedUniformBspline::setPhysicalLimits(const double& vel, const double& acc) {
  limit_vel_   = vel; 
  limit_acc_   = acc;
  limit_ratio_ = 100;
  // limit_vel_   = 1;   //0.2
  // limit_acc_   = 0.5;   //0.2
  // limit_ratio_ = 100;
 }


bool UnclampedUniformBspline::checkFeasibility(bool show) {
  bool fea = true;
  // SETY << "[Bspline]: total points size: " << control_points_.rows() << endl;

  Eigen::MatrixXd P         = control_points_;
  int             dimension = control_points_.rows();

  /* check vel feasibility and insert points */
  double max_vel = -1.0;
  for (int i = 0; i < P.cols() - 1; ++i) {
    Eigen::VectorXd vel = p_ * (P.col(i + 1) - P.col(i)) / (u_(i + p_ + 1) - u_(i + 1));

    if (fabs(vel(0)) > limit_vel_ + 1e-4 || fabs(vel(1)) > limit_vel_ + 1e-4 ||
        fabs(vel(2)) > limit_vel_ + 1e-4) {

      if (show) std::cout << "[Check]: Infeasible vel " << i << " :" << vel.transpose() << std::endl;
      fea = false;

      for (int j = 0; j < dimension; ++j) {
        max_vel = std::max(max_vel, fabs(vel(j)));
      }
    }
  }

  /* acc feasibility */
  double max_acc = -1.0;
  for (int i = 0; i < P.cols() - 2; ++i) {

    Eigen::VectorXd acc = p_ * (p_ - 1) *
        ((P.col(i + 2) - P.col(i + 1)) / (u_(i + p_ + 2) - u_(i + 2)) -
         (P.col(i + 1) - P.col(i)) / (u_(i + p_ + 1) - u_(i + 1))) /
        (u_(i + p_ + 1) - u_(i + 2));

    if (fabs(acc(0)) > limit_acc_ + 1e-4 || fabs(acc(1)) > limit_acc_ + 1e-4 ||
        fabs(acc(2)) > limit_acc_ + 1e-4) {

      if (show) std::cout << "[Check]: Infeasible acc " << i << " :" << acc.transpose() << std::endl;
      fea = false;

      for (int j = 0; j < dimension; ++j) {
        max_acc = std::max(max_acc, fabs(acc(j)));
      }
    }
  }

  double ratio = std::max(max_vel / limit_vel_, sqrt(fabs(max_acc) / limit_acc_));

  return fea;
}

            //重新分配时间
bool UnclampedUniformBspline::reallocateTime(bool show) {
  // SETY << "[Bspline]: total points size: " << control_points_.rows() << endl;
  // cout << "origin knots:\n" << u_.transpose() << endl;
  bool fea = true;

  Eigen::MatrixXd P         = control_points_;
  int             dimension = control_points_.rows();

  double max_vel, max_acc;

  /* check vel feasibility and insert points */
  for (int i = 0; i < P.cols() - 1; ++i) {
    Eigen::VectorXd vel = p_ * (P.col(i + 1) - P.col(i)) / (u_(i + p_ + 1) - u_(i + 1));

    if (fabs(vel(0)) > limit_vel_ + 1e-4 || fabs(vel(1)) > limit_vel_ + 1e-4 ||
        fabs(vel(2)) > limit_vel_ + 1e-4) {

      fea = false;
      if (show) std::cout << "[Realloc]: Infeasible vel " << i << " :" << vel.transpose() << std::endl;

      max_vel = -1.0;
      for (int j = 0; j < dimension; ++j) {
        max_vel = std::max(max_vel, fabs(vel(j)));
      }

      double ratio = max_vel / limit_vel_ + 1e-4;
      if (ratio > limit_ratio_) ratio = limit_ratio_;

      double time_ori = u_(i + p_ + 1) - u_(i + 1);
      double time_new = ratio * time_ori;
      double delta_t  = time_new - time_ori;
      double t_inc    = delta_t / double(p_);

      //时间放大部分，在 u_(i + 1)到u_(i + p_ + 1)之间的间隔均匀平分
      for (int j = i + 2; j <= i + p_ + 1; ++j) {
        u_(j) += double(j - i - 1) * t_inc;
        if (j <= 5 && j >= 1) {
          // cout << "vel j: " << j << endl;
        }
      }

     //时间放大部分，在 u_(i + p_ + 1)之后的节点依次增加，致使以后的节点间隔不变
      for (int j = i + p_ + 2; j < u_.rows(); ++j) {
        u_(j) += delta_t;
      }
    }
  }

  /* acc feasibility */
  for (int i = 0; i < P.cols() - 2; ++i) {

    Eigen::VectorXd acc = p_ * (p_ - 1) *
        ((P.col(i + 2) - P.col(i + 1)) / (u_(i + p_ + 2) - u_(i + 2)) -
         (P.col(i + 1) - P.col(i)) / (u_(i + p_ + 1) - u_(i + 1))) /
        (u_(i + p_ + 1) - u_(i + 2));

    if (fabs(acc(0)) > limit_acc_ + 1e-4 || fabs(acc(1)) > limit_acc_ + 1e-4 ||
        fabs(acc(2)) > limit_acc_ + 1e-4) {

      fea = false;
      if (show) std::cout << "[Realloc]: Infeasible acc " << i << " :" << acc.transpose() << std::endl;

      max_acc = -1.0;
      for (int j = 0; j < dimension; ++j) {
        max_acc = std::max(max_acc, fabs(acc(j)));
      }

      double ratio = sqrt(max_acc / limit_acc_) + 1e-4;
      if (ratio > limit_ratio_) ratio = limit_ratio_;
      // cout << "ratio: " << ratio << endl;

      double time_ori = u_(i + p_ + 1) - u_(i + 2);
      double time_new = ratio * time_ori;
      double delta_t  = time_new - time_ori;
      double t_inc    = delta_t / double(p_ - 1);

      if (i == 1 || i == 2) {
        // cout << "acc i: " << i << endl;
        for (int j = 2; j <= 5; ++j) {
          u_(j) += double(j - 1) * t_inc;
        }

        for (int j = 6; j < u_.rows(); ++j) {
          u_(j) += 4.0 * t_inc;
        }

      } else {

        for (int j = i + 3; j <= i + p_ + 1; ++j) {
          u_(j) += double(j - i - 2) * t_inc;
          if (j <= 5 && j >= 1) {
            // cout << "acc j: " << j << endl;
          }
        }

        for (int j = i + p_ + 2; j < u_.rows(); ++j) {
          u_(j) += delta_t;
        }
      }
    }
  }

  return fea;
}






