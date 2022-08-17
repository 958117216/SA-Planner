#pragma once

#include <Eigen/Eigen>
#include <algorithm>
#include <iostream>
#include <vector>
#include <ros/ros.h>

class UnclampedUniformBspline {
private:

  Eigen::MatrixXd control_points_;//控制点 [v0,..vn]

  int             p_, n_, m_;  // p degree, n+1 control points, m = n+p+1
  Eigen::VectorXd u_;          // knots vector
  double          interval_;   // knot span  delta_t_

  Eigen::MatrixXd getDerivativeControlPoints();

  double limit_vel_, limit_acc_, limit_ratio_;  // physical limits and time adjustment ratio
  
public:
  //初始化
  UnclampedUniformBspline(){}
  ~UnclampedUniformBspline(){}
  
  UnclampedUniformBspline(const Eigen::MatrixXd& points, const int& degree, const double& interval);
  // get / set basic bspline info
  void                                   setKnot(const Eigen::VectorXd& knot);
  Eigen::VectorXd                        getKnot();
  Eigen::MatrixXd                        getControlPoint();
  std::vector<Eigen::VectorXd> calKnotPoints();
  double                                 getInterval();
  void                                   getTimeSpan(double& um, double& um_p);
  std::pair<Eigen::VectorXd, Eigen::VectorXd> getHeadTailPts();


  // compute position / derivative
  Eigen::VectorXd   evaluateDeBoor(const double& u);   // use u \in [up, u_mp]
  Eigen::VectorXd   evaluateDeBoorT(const double& t);  // use t \in [0, duration]
  UnclampedUniformBspline getDerivative();


  /* check feasibility, adjust time */

  void   setPhysicalLimits(const double& vel, const double& acc);
  bool   checkFeasibility(bool show = false);
  bool   reallocateTime(bool show = false);

  /* for performance evaluation */
  double getTimeSum();
  double getLength(const double& res);
  double getJerk();
  void   getMeanAndMaxVel(double& mean_v, double& max_v);
  void   getMeanAndMaxAcc(double& mean_a, double& max_a);

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};


