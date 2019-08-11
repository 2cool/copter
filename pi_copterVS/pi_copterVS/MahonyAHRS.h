

	



//=====================================================================================================
// MahonyAHRS.h
//=====================================================================================================
//
// Madgwick's implementation of Mayhony's AHRS algorithm.
// See: http://www.x-io.co.uk/node/8#open_source_ahrs_and_imu_algorithms
//
// Date			Author			Notes
// 29/09/2011	SOH Madgwick    Initial release
// 02/10/2011	SOH Madgwick	Optimised for reduced CPU load
//
//=====================================================================================================
#ifndef MahonyAHRS_h
#define MahonyAHRS_h


#ifndef _QUAT__
#define _QUAT__
struct quat {
	float w = 1, x = 0, y = 0, z = 0;
};
#endif
class MahonyAHRS
{
public:
//----------------------------------------------------------------------------------------------------
// Variable declaration

//float twoKp;			// 2 * proportional gain (Kp)
//float twoKi;			// 2 * integral gain (Ki)
 //float q0, q1, q2, q3;	// quaternion of sensor frame relative to auxiliary frame
 quat q;
//---------------------------------------------------------------------------------------------------
// Function declarations

	void AHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz, float dt);

private:
	void AHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az,float dt);
	float invSqrt(float x);
};

extern MahonyAHRS MAH;
#endif


//=====================================================================================================
// End of file
//=====================================================================================================
