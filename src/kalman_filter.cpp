#include "kalman_filter.h"
#include <iostream>

using Eigen::MatrixXd;
using Eigen::VectorXd;

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
  TODO:
    * predict the state
  */
	x_ = F_*x_;
	P_ = F_*P_*F_.transpose()+Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Kalman Filter equations
  */
	VectorXd y = z-H_*x_;
	MatrixXd S = H_*P_*H_.transpose()+R_;
	MatrixXd K = P_*H_.transpose()*S.inverse();
	MatrixXd I = MatrixXd::Identity(H_.cols(),H_.cols());

	// new state
	x_ = x_+K*y;
	P_ = (I-K*H_)*P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */
	float px = x_(0);
	float py = x_(1);
	float vx = x_(2);
	float vy = x_(3);

	VectorXd hx = VectorXd(3);
	hx(0) = sqrt(px*px+py*py);
	hx(1) = atan2(py,px);
	hx(2) = (px*vx+py*vy)/hx(0);

	VectorXd y = z-hx;

	// Make sure that the resulting angle, phi, is between -pi and pi
	y(1) = fmod((y(1)+M_PI),(2*M_PI)) - M_PI;

	MatrixXd S = H_*P_*H_.transpose()+R_;
	MatrixXd K = P_*H_.transpose()*S.inverse();
	MatrixXd I = MatrixXd::Identity(H_.cols(),H_.cols());

//	// new state
	x_ = x_+K*y;
	P_ = (I-K*H_)*P_;
}
