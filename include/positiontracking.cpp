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
		//stucture for all of the position values
		struct position {
			double xPos = 0;
			double yPos = 0;
			double theta = 0;
		} current,last,gpsS;

		//drift offset value
		// double driftOffSet;

		//finding out if the GPS will be enabled
		bool gpsEnabled = true;
		//function to toggle to GPS on and off
		void toggleGPS() {
			if (gpsEnabled) {
				gpsEnabled = false;
			} else {gpsEnabled = true;}
		}

		//extra values for position tracking and autonomous function
		double totaldistance;

	//private holds all of the functions and variables that does not need to be called outside of the class
	private:
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

		//outdates some values, and sets them as last values	
		void outdatevalues() {
			last.xPos = current.xPos;
			last.yPos = current.yPos;
			last.theta = current.theta;
			
		}

	//public section that holds functions called outside of the class
	public:
		//functions 
		void updatepos() {
			//get the theta
			current.theta = inertial.get_heading();
			//get the drift
			// driftIMU = inertial.get_accel().x;
			//older values are placed here
			outdatevalues();
			//calculated total change in distance
			distance = ((((driveLB.get_position())-leftENC.last)+((driveRB.get_position())-rightENC.last))/2)/ENCadjustment;
			totaldistance += distance;
			//getting the x and y value
			current.xPos += distance*(cos((current.theta*PI)/180));
			current.yPos += distance*(sin((current.theta*PI)/180));
			//accounting for drift
			// current.xPos += driftIMU*(cos(((current.theta+90)*PI)/180));
			// current.yPos += driftIMU*(sin(((current.theta+90)*PI)/180));

			//X Nan
			if ((std::isnan(current.xPos)) || (std::isinf(current.xPos)) ) {
				current.xPos = 0; }
			//Y Nan
			if ((std::isnan(current.yPos)) || (std::isinf(current.yPos))) {
				current.yPos = 0; }
			//update the motor values
			updatevalues();

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
		posTracking.updatepos();
		pros::delay(20);
	}
}
