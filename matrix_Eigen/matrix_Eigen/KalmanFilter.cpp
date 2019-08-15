/**
* Implementation of KalmanFilter class.
*
* @author: Hayk Martirosyan
* @date: 2014.11.15
*/

#include <iostream>
#include <stdexcept>

#include "KalmanFilter.h"

/*
B*uk - kontrol signal - B=0
A - 
H -
Q -
R - noise i the environment
P - error covariance




*/




KalmanFilter::KalmanFilter(
	double dt,
	const Eigen::MatrixXd& A,
	const Eigen::VectorXd& B,
	const Eigen::MatrixXd& H,
	const Eigen::MatrixXd& Q,
	const Eigen::MatrixXd& R,
	const Eigen::MatrixXd& P)
	: A(A), B(B), H(H), Q(Q), R(R), P0(P),
	m(H.rows()), n(A.rows()), dt(dt), initialized(false),
	I(n, n), x_hat(n), x_hat_new(n)
{
	I.setIdentity();
}

KalmanFilter::KalmanFilter() {}

void KalmanFilter::init(double t0, const Eigen::VectorXd& x0) {
	x_hat = x0;
	P = P0;
	this->t0 = t0;
	t = t0;
	initialized = true;
}
void KalmanFilter::initU(const double a) {
	x_hat[2] = a;
	
}


void KalmanFilter::init() {
	x_hat.setZero();
	P = P0;
	t0 = 0;
	t = t0;
	initialized = true;
}

void KalmanFilter::update(const Eigen::VectorXd& y) {

	if (!initialized)
		throw std::runtime_error("Filter is not initialized!");

	x_hat_new = A * x_hat + B; //B added 2cool
	P = A * P * A.transpose() + Q;
	K = P * H.transpose() * (H * P * H.transpose() + R).inverse();
	x_hat_new += K * (y - H * x_hat_new);
	P = (I - K * H) * P;
	x_hat = x_hat_new;

	t += dt;
}

void KalmanFilter::update(const Eigen::VectorXd & y, double dt, const Eigen::MatrixXd A) {

	this->A = A;
	this->dt = dt;
	update(y);
}
