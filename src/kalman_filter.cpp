#include "kalman_filter.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

// Please note that the Eigen library does not initialize 
// VectorXd or MatrixXd objects with zeros upon creation.

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_ = R_in;
  Q_ = Q_in;
}

void KalmanFilter::Predict() {
  /**
  DONE:
    * predict the state
  */
   
   // Lesson 7 Kalman Filter equations 
   // x(t) = F(t) * x(t-1) + B(t) * u(t) + w(t) 
   
    x_ = F_ * x_;
    MatrixXd Ft = F_.transpose();
    P_ = F_ * P_ * Ft + Q_;

}

void KalmanFilter::Update(const VectorXd &z) {
  /**
  DONE:
    * update the state by using Kalman Filter equations
  */
   //measurement update step
    VectorXd z_pred = H_ * x_;
    VectorXd y = z - z_pred;
    MatrixXd Ht = H_.transpose();
    //moved PHt calculation here -- to remove the repetition calculation later on
    MatrixXd PHt = P_ * Ht;
    MatrixXd S = H_ * PHt + R_;
    MatrixXd Si = S.inverse();
    MatrixXd K = PHt * Si;

    //new state
    x_ = x_ + (K * y);
    long x_size = x_.size();
    MatrixXd I = MatrixXd::Identity(x_size, x_size);
    P_ = (I - K * H_) * P_;  

}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  DONE:
    * update the state by using Extended Kalman Filter equations
  */

    float px = x_(0);
    float py = x_(1);
    float vx = x_(2);
    float vy = x_(3);


    float rho = sqrt(px * px + py * py);

    float theta = atan2(py, px);
    float ro_dot = (px * vx + py * vy) / rho;
    
    //measurement update 
    VectorXd z_pred(3);
    z_pred << rho, theta, ro_dot;
    VectorXd y = z - z_pred;

    //Code Review comment : Perform normalization when you subtract 2 angles 

    // Why ? because if you don't angle may not be in the range of -pi to +pi
   
    while ( y(1) > +3.1416) {
	y(1) -= 2 * 3.1416;
     } 

   while ( y(1) < - 3.1416) {
       y(1) += 2 * 3.1416;
     }

    MatrixXd Ht = H_.transpose();
    MatrixXd S = H_ * P_ * Ht + R_;
    MatrixXd Si = S.inverse();
    MatrixXd PHt = P_ * Ht;
    MatrixXd K = PHt * Si;

    //estimate new state
    x_ = x_ + ( K * y );
    MatrixXd Ft = F_.transpose();
    P_ = F_ * P_ * Ft + Q_;
}
