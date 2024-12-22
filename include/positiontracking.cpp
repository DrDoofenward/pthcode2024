//positiontracking.cpp is 2nd in the conga line
#include "setup.cpp"

//adding math tingys
#include <cmath>

#define PI 3.14159265


/**
 * postracking is the core for all of the position tracking functions on the robot.
 * While easily accessable from 
 */
class postracking {																				//position tracking
	//specifically public for totaldistance
	public:
		double totaldistance;
	
	//private holds all of the data and functions that arent needed outside of the class
	private:
		//filter constraints
		const double GPS_CORRECTION_RATE = 20; //gps correction interval (ms)
		const double ALPHA = 0.95; //complementary filter tuning factor

		//stucture for all of the position values
		struct position {
			double xPos = 0;
			double yPos = 0;
			double theta = 0;
		} encU,gpsS;

		//create a structure for each motor, listing its last value and delta value
		struct extradata {
			double last = 0;
			double delta = 0;
			
		} leftENC, rightENC, thetaIMU;

		//extra values for position tracking and autonomous function
		double distance;

		//updates the IMU position
		void updateIMUpos() {
			//get the theta
			encU.theta = inertial.get_heading();
			//calculated total change in distance
			distance = ((((driveLB.get_position())-leftENC.last)+((driveRB.get_position())-rightENC.last))/2)/ENCadjustment;
			totaldistance += distance;
			//getting the x and y value
			encU.xPos = FAPedX + distance*(sin((encU.theta*PI)/180));
			encU.yPos = FAPedY - distance*(cos((encU.theta*PI)/180));

			//making sure X and Y are not going to nan or inf, bugging the code
			if ((std::isnan(encU.xPos)) || (std::isinf(encU.xPos)) ) encU.xPos = 0;
			if ((std::isnan(encU.yPos)) || (std::isinf(encU.yPos))) encU.yPos = 0;

			//update the motor values
			leftENC.delta = driveLB.get_position() - leftENC.last; //left motor
			leftENC.last = driveLB.get_position();
			rightENC.delta = driveRB.get_position() - rightENC.last; //right motor
			rightENC.last = driveRB.get_position();
			//update inertial as well
			thetaIMU.delta = inertial.get_heading() - thetaIMU.last; //rotation
			thetaIMU.last = inertial.get_heading();
		}

		//updates the GPS position
		void updateGPSpos() {
			gpsS.xPos = gps.get_x_position();
			gpsS.yPos = gps.get_y_position();
			gpsS.theta = gps.get_heading();
		}

	//public section that holds functions called outside of the class
	public:
		void getAbsolutePosition() {
			updateIMUpos();
			updateGPSpos();
			//simple complementary filter (because i dont want to code a kalman filter atm)
			//FAPedX = ALPHA * encU.xPos + (1 - ALPHA) * gpsS.xPos;
    		//FAPedY = ALPHA * encU.yPos + (1 - ALPHA) * gpsS.yPos;
    		//FAPedTheta = ALPHA * encU.theta + (1 - ALPHA) * gpsS.theta;
			//test code for the inertial sensor specifically
			FAPedX = encU.xPos;
    		FAPedY = encU.yPos;
    		FAPedTheta = encU.theta;

		}

};

//creating a task function for running the position system
postracking posTracking;
void activatePositionTracking() {																//function to activate all position tracking functionality
	while (inertial.is_calibrating()) {
		pros::delay(20);
	}
	// posTracking.driftOffSet = inertial.get_accel().x;
	while (true) {
		posTracking.getAbsolutePosition();
		pros::delay(20);
	}
}
