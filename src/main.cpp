#include "main.h"
#include "pros/misc.h"
#include "pros/motors.h"
#include "pros/motors.hpp"
#include "pros/rtos.hpp"


//including seperate files in a conga line style
//main.cpp is the last of the conga line and runs everything
#include "custombraindisplay.cpp"
#include <cmath>


//sensors and motors declared in setup.cpp

//core function to move the intake, has 2 variables to control it much like assignDrivetrainVelocity
void moveIntake(bool reverse, int velocity) {																	//intake system
	if (reverse) { intake.move_velocity(velocity*-5);
	} else { intake.move_velocity(velocity*5);}
}

class mogoSystem {																								//mogo piston mech system
	private:
		//internal variable for toggling the mogo
		bool mogoToggled = false;

	public:
		//toggle to toggle the mogo mech on and off
		void toggle() {
			if (mogoToggled) { 
				mogoMech.set_value(false);
				mogoToggled = false;
			} else { 
				mogoMech.set_value(true);
				mogoToggled = true; }
		}

};

mogoSystem mogo;

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

		//PID values for distance
		double DkP = 0.25;
		double DkI = 0.03;
		double DkD = 0.2;

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
			if (error > 180) error -= 360;
			if (error < -180) error += 360;
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
 * The drivetrain class holds all functions that directly affiliate with control
 * of the drivetrain, wether its both autonomous or driver control. 
 */
class drivetrainf {
	private:


	//all functions that are needed to be called will be placed in the public space
	public:
		/**
 		* The main drive function is the main function to control the drivetrain, it
 		* takes a few variables and does some math and assigns velocities to each of
 		* the 6 drivetrain motors. Used for both autonomous and driver control functions
 		*/
		void assignDrivetrainVelocity(int forward_vel, int turn_vel) {												//assign drive velocity
			driveLeft.move_velocity(5*((forward_vel*-1)+turn_vel));
			driveRight.move_velocity(5*((forward_vel*-1)-turn_vel));
		}

		//turntoheading is used to turn to a specific heading in degrees
		void turnToHeading(double heading) {																		//turn to heading
			int passlimit = 1;
			
			//while statement that runs til heading is accurate to about 1 degree of error
			while (passlimit > 0) {
				//run the pid loop
				double velocity = PID.turnPID(heading, FAPedTheta);
				//assign drive velocity
				assignDrivetrainVelocity(0, velocity);
				//if make it to assigned heading more then 3 times, let it pass
				if ((FAPedTheta >= heading-0.5) && (FAPedTheta <= heading+0.5)) {
					passlimit -= 1;
				}
				//delay for no overflow
				pros::delay(20);
			}
			assignDrivetrainVelocity(0, 0);
			PID.resetvariables();
			pros::delay(20);
		};

		//placeholder for a moveforward
		void moveDistanceL(double distance, double heading) {														//move distance (locked)
			int passlimit = 1;
			double error = posTracking.totaldistance+distance;
			//while statement that runs til heading is accurate to about 1 degree of error
			while (passlimit > 0) {
				//run the pid loop for both the distance and heading
				double forwardvelocity = PID.distancePID(error, posTracking.totaldistance);
				double turnvelocity = PID.turnPID(heading, FAPedTheta);
				//assign drive velocity
				assignDrivetrainVelocity(forwardvelocity, turnvelocity);
				//pros::screen::print(TEXT_SMALL, 5,"%s","balls: " + (std::to_string(error-posTracking.totaldistance))); test thing
				//if make it to assigned heading more then 3 times, let it pass
				if ((error >= posTracking.totaldistance-3) && (error <= posTracking.totaldistance+3)) {
					passlimit -= 1;
				}
				//delay for no overflow
				pros::delay(20);
			}
			assignDrivetrainVelocity(0, 0);
			PID.resetvariables();
			pros::delay(20);
		};

		//placeholder for a gotocoordinate
		void goToCoordinate(double x, double y) {																//go to a coordinate
			int passlimit = 1;
			//do initial turn
			double heading = (atan2(y-FAPedY,FAPedX-x)*180/PI)-90;
			if (heading < 0) heading +=360;
			pros::screen::print(TEXT_SMALL, 5,"%s","balls: " + (std::to_string(heading)));
			turnToHeading(heading);
			//find out the distance between current and set position
			double distanceX = FAPedX - x;
			double distanceY = FAPedY - y;
			//pythagorean theoremmaxxing
			double distance = sqrt(pow(distanceX, 2) + pow(distanceY, 2));
			moveDistanceL(distance, heading);
			assignDrivetrainVelocity(0, 0);
			PID.resetvariables();
			pros::delay(20);
		};

		
};
//assigns a nickname to the drive
drivetrainf drive;


/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {																								//built in initialization function
	//wait for robot to finish startup
	pros::delay(100);
	//initial coloring
	pros::screen::set_pen(COLOR_BLACK);
	pros::screen::fill_rect(0, 0, 480, 240);
	//coloring each side and labeling zero
	pros::screen::set_pen(COLOR_RED);
	pros::screen::fill_rect(180, 20, 200, 220);
	pros::screen::set_pen(COLOR_BLUE);
	pros::screen::fill_rect(440, 20, 460, 220);
	pros::delay(20);
	//initialize brain sequences
	pros::Task motortemps(taskTempDisplay);
	//initiate position tracking
	pros::Task realPosition(activatePositionTracking);

}

//runs while the robot is in the disabled state via the field managment system
void disabled() {																								//built in disabled function
}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void autonomous() {																							//autonomous executor
	//prevents the autonomous from starting without inertial calibration completing
	while (inertial.is_calibrating()) {
		pros::delay(20);
	}
	//tune the drivetrain values
	driveLeft.set_brake_modes(pros::E_MOTOR_BRAKE_COAST);
	driveRight.set_brake_modes(pros::E_MOTOR_BRAKE_COAST);
	//declare the current position
	FAPedX = 0;
	FAPedY = 0;
	FAPedTheta = 0;
	pros::delay(20);
	//testing the coordinate system
	drive.goToCoordinate(600, 0);
}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
void opcontrol() {																							//driver control executor
	//initialize the controller
	pros::Controller master(pros::E_CONTROLLER_MASTER);

	//initialize any driver control values
	bool mogoPressing = false;

	while (true) {

		//tune the drivetrain values
		driveLeft.set_brake_modes(pros::E_MOTOR_BRAKE_COAST);
		driveRight.set_brake_modes(pros::E_MOTOR_BRAKE_COAST);

		//finding the joystick value and assigning velocities to the motors
		int turnVel = (master.get_analog(ANALOG_RIGHT_X))/drivetrainTurnGoverner;
		int forwardVel = master.get_analog(ANALOG_LEFT_Y);
		drive.assignDrivetrainVelocity(forwardVel, turnVel);
		
		//driver control intake integration
		if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
			moveIntake(false, 500);
		} else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
			moveIntake(true, 500);
		}else {moveIntake(false, 0);}

		//driver control mogo mech integration
		if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R1) && (mogoPressing == false)) {
			mogoPressing = true;
			mogo.toggle();
		} else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R1) == false) {
			mogoPressing = false;
		}
		pros::delay(20);
	}
}
