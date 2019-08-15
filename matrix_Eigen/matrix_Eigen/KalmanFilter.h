#pragma once
/**
* Kalman filter implementation using Eigen. Based on the following
* introductory paper:
*
*     http://www.cs.unc.edu/~welch/media/pdf/kalman_intro.pdf
*
* @author: Hayk Martirosyan
* @date: 2014.11.15
*/

#pragma once
#include "../../eigen/Eigen/Dense"
using namespace Eigen;


class KalmanFilter {

public:

	/**
	* Create a Kalman filter with the specified matrices.
	*   A - System dynamics matrix
	*   H - Output matrix
	*   Q - Process noise covariance
	*   R - Measurement noise covariance
	*   P - Estimate error covariance
	*/
	KalmanFilter(
		double dt,
		const Eigen::MatrixXd& A,
		const Eigen::VectorXd& B,
		const Eigen::MatrixXd& H,
		const Eigen::MatrixXd& Q,
		const Eigen::MatrixXd& R,
		const Eigen::MatrixXd& P
	);

	/**
	* Create a blank estimator.
	*/
	KalmanFilter();

	/**
	* Initialize the filter with initial states as zero.
	*/
	void init();

	/**
	* Initialize the filter with a guess for initial states.
	*/
	void init(double t0, const Eigen::VectorXd& x0);

	/**
	* Update the estimated state based on measured values. The
	* time step is assumed to remain constant.
	*/
	void update(const Eigen::VectorXd& y);

	/**
	* Update the estimated state based on measured values,
	* using the given time step and dynamics matrix.
	*/
	void update(const Eigen::VectorXd& y, double dt, const Eigen::MatrixXd A);

	/**
	* Return the current state and time.
	*/
	Eigen::VectorXd state() { return x_hat; };
	double time() { return t; };
	void initU(const double x0);

	Eigen::VectorXd B;

private:

	// Matrices for computation
	Eigen::MatrixXd A, H, Q, R, P, K, P0;

	// System dimensions
	int m, n;

	// Initial and current time
	double t0, t;

	// Discrete time step
	double dt;

	// Is the filter initialized?
	bool initialized;

	// n-size identity
	Eigen::MatrixXd I;

	// Estimated states
	Eigen::VectorXd x_hat, x_hat_new;
};
