//positiontracking.cpp is 2nd in the conga line
#include "setup.cpp"

#define PI 3.14159265

/** PID SYSTEM
 * PID, or Proportional, Integral, Derivative, is a feedback controller that uses
 * past, present, and future error to determine how much and how quickly to correct
 * a process, the controller uses three times, P, I, and D, that are equivilant to
 * magnitude, duration, and rate of change of the error. 
 */
class PIDSystem {																				//PID system class and tunable values

	private:

		//TURNING
		//variables stored for the loop
		double Tprevious_error = 0;
		double Tintegral = 0;
		double Tintegral_limit = 50;

		//PID values for turning
		double TkP = 0.5;
		double TkI = 0.04;
		double TkD = 0.2;

		//DISTANCE PID
		//variables stored for the loop
		double Dprevious_error = 0;
		double Dintegral = 0;
		double Dintegral_limit = 50;

		//PID values for turning
		double DkP = 6;
		double DkI = 0;
		double DkD = 3;

	//functions for the system are stored here
	public:
 		double distancePID(double target, double current) {										//PID code for forward/backward velocity
    		double error = target - current;

    		//proportional equation
    		double proportional = error * DkP;
    
    		//integral equation
    		Dintegral += error;
			//if integral error gets too large, change it to the max (prevents windup)
			if (Dintegral > Dintegral_limit) Dintegral = Dintegral_limit;
    		if (Dintegral < -Dintegral_limit) Dintegral = -Dintegral_limit;
    		double integral_part = Dintegral * DkI;
    
    		//derivative equation
    		double derivative = (error - Dprevious_error) * DkD;
    		Dprevious_error = error;
    
    		//combine the numbers and return the output
			
			/*take together the proportional and the derivative then smash together those
			 *two different expressions with inertial to create and push out an added number
			 * 想像上のテクニック: 出力
			 */
    		double output = proportional + integral_part + derivative;
    
    		return output;
		}

		double turnPID(double target, double current) {											//PID code for turning velocity
    		double error = target - current;

			//if statements for fixing zero turns (for turning)
			if (error > 180) {
   				error -= 360;
			}
			if (error < -180) {
   				error += 360;
			}

    		//proportional equation
    		double proportional = error * TkP;
    
    		//integral equation
    		Tintegral += error;
			//if integral error gets too large, change it to the max (prevents windup)
			if (Tintegral > Tintegral_limit) Tintegral = Tintegral_limit;
    		if (Tintegral < -Tintegral_limit) Tintegral = -Tintegral_limit;
    		double integral_part = Tintegral * TkI;
    
    		//derivative equation
    		double derivative = (error - Tprevious_error) * TkD;
    		Tprevious_error = error;
    
    		//combine the numbers and return the output
			
			/*take together the proportional and the derivative then smash together those
			 *two different expressions with inertial to create and push out an added number
			 * 想像上のテクニック: 出力
			 */
    		double output = proportional + integral_part + derivative;
    
    		return output;
		}

		//small function for reseting the loop for reusability
		void resetvariables() {																	//function resets values in the PID system
			Tprevious_error = 0;
			Tintegral = 0;
			Dprevious_error = 0;
			Dintegral = 0;
		}
};
PIDSystem PID;

/**
 * postracking is the core for all of the position tracking functions on the robot.
 * While easily accessable from 
 */
class postracking {																				//position tracking (likely will be rewritten)
	//public section specifically for public variables
	public:
	
		//finding out if the GPS will be enabled
		bool gpsEnabled = true;
		//function to toggle to GPS on and off
		void toggleGPS() {
			if (gpsEnabled) { gpsEnabled = false;
			} else {gpsEnabled = true;}
		}

		//extra values for position tracking and autonomous function
		double totaldistance;

	//private holds all of the functions and variables that does not need to be called outside of the class
	private:
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
		//double driftIMU;

		//function that updates the last and delta values for each motor
		void updatevalues() {
			//motors
			leftENC.delta = driveLB.get_position() - leftENC.last; //left motor
			leftENC.last = driveLB.get_position();
			rightENC.delta = driveRB.get_position() - rightENC.last; //right motor
			rightENC.last = driveRB.get_position();
			//inertial
			thetaIMU.delta = inertial.get_heading() - thetaIMU.last; //rotation
			thetaIMU.last = inertial.get_heading();

			//if the gps is enabled, add those values as well
			if (gpsEnabled) {
				gpsS.theta = gps.get_heading();
				gpsS.xPos = gps.get_x_position();
				gpsS.yPos = gps.get_y_position();
			}
		}

		//functions 
		void updateIMUpos() {
			//get the theta
			encU.theta = inertial.get_heading();
			//calculated total change in distance
			distance = ((((driveLB.get_position())-leftENC.last)+((driveRB.get_position())-rightENC.last))/2)/ENCadjustment;
			totaldistance += distance;
			//getting the x and y value
			encU.xPos += distance*(cos((encU.theta*PI)/180));
			encU.yPos += distance*(sin((encU.theta*PI)/180));

			//making sure X and Y are not going to nan or inf, bugging the code
			if ((std::isnan(encU.xPos)) || (std::isinf(encU.xPos)) ) encU.xPos = 0;
			if ((std::isnan(encU.yPos)) || (std::isinf(encU.yPos))) encU.yPos = 0;
			//update the motor values
			updatevalues();
		}
	//public section that holds functions called outside of the class
	public:
		void getAbsolutePosition() {
			updateIMUpos();
			//temporary solution to keep position tracking functioning
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
