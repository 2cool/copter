#pragma once
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
	void base(int const b);
	KalmanFilter(
		const Eigen::Matrix3f& A,
		const Eigen::Vector3f& B,
		const Eigen::RowVector3f& H,
		const Eigen::Matrix3f& Q,
		const Eigen::MatrixXf& R,
		const Eigen::Matrix3f& P
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
	void init(const Eigen::Vector3f& x0);

	/**
	* Update the estimated state based on measured values. The
	* time step is assumed to remain constant.
	*/
	void update(const Eigen::VectorXf& y);

	/**
	* Update the estimated state based on measured values,
	* using the given time step and dynamics matrix.
	*/
	//void update(const Eigen::VectorXf& y, const Eigen::MatrixXf A);
	void update();
	/**
	* Return the current state and time.
	*/
	Eigen::Vector3f state() { return x_hat; };

	void initU(const double x0);

	Eigen::Vector3f B;
	Eigen::Matrix3f getP() { return P; }
	Eigen::Vector3f getK() { return K; }

	Eigen::MatrixXf R;
	Eigen::Matrix3f A, Q, P, P0;
	RowVector3f H;
	Vector3f K;


private:

	// Matrices for computation


	// System dimensions
	int m, n;

	// Initial and current time


	// Discrete time step


	// Is the filter initialized?
	bool initialized;

	// n-size identity
	Eigen::Matrix3f I;

	// Estimated states
	Eigen::Vector3f x_hat, x_hat_new;
};
