#include "main.h"
#include "pros/llemu.hpp"
#include "pros/misc.h"
#include "pros/motors.h"
#include "pros/motors.hpp"
#include "pros/rtos.hpp"
#include <cmath>

#define PI 3.14159265

//tuneable values for certain systems
int drivetrainTurnGoverner = 2;
double ENCadjustment = 37.5;

//assigning the master controller
pros::Controller master(pros::E_CONTROLLER_MASTER);

//Creating the drivetrain motors and assigning their groups
pros::Motor driveLF(17,pros::E_MOTOR_GEARSET_06, true);
pros::Motor driveLB(16,pros::E_MOTOR_GEARSET_06, true);
pros::Motor driveLT(15,pros::E_MOTOR_GEARSET_06, false);

pros::Motor_Group driveLeft ({driveLF,driveLB,driveLT});

pros::Motor driveRF(20,pros::E_MOTOR_GEARSET_06, false);
pros::Motor driveRB(19,pros::E_MOTOR_GEARSET_06, false);
pros::Motor driveRT(18,pros::E_MOTOR_GEARSET_06, true);

pros::Motor_Group driveRight ({driveRF,driveRB,driveRT});

//assigning other motors
pros::Motor intake(14,pros::E_MOTOR_GEARSET_06,true);
pros::Motor wallstake(2,pros::E_MOTOR_GEARSET_18,false);

//assigning pneumatics
pros::ADIDigitalOut mogoMech ('B');

//assigning sensors
pros::IMU inertial (13);
pros::GPS gps (11);
pros::Vision vision (12);



//core function to move the intake, has 2 variables to control it much like assignDrivetrainVelocity
void moveIntake(bool reverse, int velocity) {
	if (reverse) { intake.move_velocity(velocity*-5);
	} else { intake.move_velocity(velocity*5);}
}

class mogoSystem {
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
class PIDSystem {

	private:
		//variables stored for the loop
		double previous_error = 0;
		double integral = 0;

	//functions for the system are stored here
	public:
 		double PID(double target, double current, double kP, double kI, double kD) {
    		double error = target - current;
    
    		//proportional equation
    		double proportional = error * kP;
    
    		//integral equation
    		integral += error;
    		double integral_part = integral * kI;
    
    		//derivative equation
    		double derivative = (error - previous_error) * kD;
    		previous_error = error;
    
    		//combine the numbers and return the output
			
			/*take together the proportional and the derivative then smash together those
			 *two different expressions with inertial to create and push out an added number
			 * 想像上のテクニック: 出力
			 */
    		double output = proportional + integral_part + derivative;
    
    		return output;
		}

		//small function for reseting the loop for reusability
		void resetvariables() {
			previous_error = 0;
			integral = 0;
		}
};
PIDSystem PID;


/**
 * postracking is the core for all of the position tracking functions on the robot.
 * While easily accessable from 
 */
class postracking {
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
			
			//position values
			pros::lcd::print(3, "x position: %f", current.xPos);
			pros::lcd::print(4, "y position: %f", current.yPos);
			pros::lcd::print(5, "drift offset unavailable");
			pros::lcd::print(6, "theta: %f", current.theta);
			//deal with NAN's
			//X Nan
			if ((std::isnan(current.xPos)) || (std::isinf(current.xPos)) ) {
				current.xPos = 0;
				pros::lcd::print(3, "X is NAN"); }
			//Y Nan
			if ((std::isnan(current.yPos)) || (std::isinf(current.yPos))) {
				current.yPos = 0;
				pros::lcd::print(4, "Y is NAN"); }
			//update the motor values
			updatevalues();


		}

};

//creating a task function for running the position system
postracking posTracking;
void activatePositionTracking() {
	while (inertial.is_calibrating()) {
		pros::delay(20);
	}
	// posTracking.driftOffSet = inertial.get_accel().x;
	while (true) {
		posTracking.updatepos();
		pros::delay(20);
	}
}

/**
 * The drivetrain class holds all functions that directly affiliate with control
 * of the drivetrain, wether its both autonomous or driver control. 
 */
class drivetrainf {
	private:
		//PID values for turning
		double TkP = 0.1;
		double TkI = 0.1;
		double TkD = 0.1;


	//all functions that are needed to be called will be placed in the public space
	public:
		/**
 		* The main drive function is the main function to control the drivetrain, it
 		* takes a few variables and does some math and assigns velocities to each of
 		* the 6 drivetrain motors. Used for both autonomous and driver control functions
 		*/
		void assignDrivetrainVelocity(int forward_vel, int turn_vel) {
			driveLeft.move_velocity(5*(forward_vel+turn_vel));
			driveRight.move_velocity(5*(forward_vel-turn_vel));
		}

		//placeholder for a turntoheading
		void turnToHeading(double heading) {
			//while statement that runs til heading is accurate to about 1 degree of error
			while (((posTracking.current.theta >= heading-1) && (posTracking.current.theta <= heading+1))==false) {
				//run the pid loop
				double velocity = PID.PID(heading, posTracking.current.theta, TkP, TkI, TkD);
				//assign drive velocity
				assignDrivetrainVelocity(0, velocity);
				//delay for no overflow
				pros::delay(20);
			}
		};

		//placeholder for a moveforward
		void moveDistance() {};

		//placeholder for a gotocoordinate
		void goToCoordinatePre() {};

		void goToCoordinateSpe() {};

		
};

//assigns a nickname to the drive
drivetrainf drive;

/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */
void on_center_button() {
	static bool pressed = false;
	pressed = !pressed;
	if (pressed) {
		pros::lcd::set_text(2, "I was pressed!");
	} else {
		pros::lcd::clear_line(2);
	}
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	pros::lcd::initialize();
	pros::lcd::set_text(1, "Hello PROS User!");

	pros::lcd::register_btn1_cb(on_center_button);
	
	//initiate position tracking
	pros::delay(500);
	pros::Task realPosition(activatePositionTracking);
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {
}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}

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
void autonomous() {

	//tune the drivetrain values
	driveLeft.set_brake_modes(pros::E_MOTOR_BRAKE_BRAKE);
	driveRight.set_brake_modes(pros::E_MOTOR_BRAKE_BRAKE);
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
void opcontrol() {
	//initialize the controller
	pros::Controller master(pros::E_CONTROLLER_MASTER);

	//initialize any driver control values
	bool mogoPressing = false;

	while (true) {
		pros::lcd::print(0, "%d %d %d", (pros::lcd::read_buttons() & LCD_BTN_LEFT) >> 2,
		                 (pros::lcd::read_buttons() & LCD_BTN_CENTER) >> 1,
		                 (pros::lcd::read_buttons() & LCD_BTN_RIGHT) >> 0);

		//tune the drivetrain values
		driveLeft.set_brake_modes(pros::E_MOTOR_BRAKE_COAST);
		driveRight.set_brake_modes(pros::E_MOTOR_BRAKE_COAST);

		//finding the joystick value and assigning velocities to the motors
		int turnVel = (master.get_analog(ANALOG_LEFT_X))/drivetrainTurnGoverner;
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
