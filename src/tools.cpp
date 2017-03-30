#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
  TODO:
    * Calculate the RMSE here.
  */
	VectorXd rmse(4);
	rmse << 0,0,0,0;

	// check the validity of the following inputs:
	//  * the estimation vector size should not be zero
	//  * the estimation vector size should equal ground truth vector size
	if (estimations.size()==0)
	{
		std::cout << "CalculateRMSE: No measurements given" << std::endl;
		return rmse;
	}
	if (estimations.size()!=ground_truth.size())
	{
		std::cout << "CalculateRMSE: Measurements and ground_truth not same size" << std::endl;
		return rmse;
	}

	//accumulate squared residuals
	for(int i=0; i < estimations.size(); ++i){
		VectorXd res = estimations[i]-ground_truth[i];
		rmse += (res.array()*res.array()).matrix();
	}

	//calculate the mean
	rmse /= estimations.size();

	//calculate the squared root
	rmse = rmse.array().sqrt().matrix();

	//return the result
	return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
  TODO:
    * Calculate a Jacobian here.
  */
	MatrixXd Hj = MatrixXd::Zero(3,4);
	//recover state parameters
	float px = x_state(0);
	float py = x_state(1);
	float vx = x_state(2);
	float vy = x_state(3);

	float range_squared = px*px+py*py;
	float range = sqrt(range_squared);
	float range_tripled = range*range_squared;

	//check division by zero
	if (range < 0.0001)
	{
		std::cout << "CalculateJacobian () - Error - Division by Zero" << std::endl;
		return Hj; // Return zero matrix
	}
	//compute the Jacobian matrix

	Hj(0,0) = px/range;
	Hj(0,1) = py/range;
	Hj(1,0) = -py/range_squared;
	Hj(1,1) = px/range_squared;
	Hj(2,0) = py*(vx*py-vy*px)/range_tripled;
	Hj(2,1) = px*(vy*px-vx*py)/range_tripled;
	Hj(2,2) = px/range;
	Hj(2,3) = py/range;


	return Hj;
}
