//setup is the first file in the conga line, and sets up all of the hardware related functionality
#include "main.h"

//assigning the master controller
pros::Controller master(pros::E_CONTROLLER_MASTER);

//Creating the drivetrain motors and assigning their groups
pros::Motor driveLF(5,pros::E_MOTOR_GEARSET_06, true);
pros::Motor driveLB(6,pros::E_MOTOR_GEARSET_06, true);
pros::Motor driveLT(7,pros::E_MOTOR_GEARSET_06, false);

pros::Motor_Group driveLeft ({driveLF,driveLB,driveLT});

pros::Motor driveRF(2,pros::E_MOTOR_GEARSET_06, false);
pros::Motor driveRB(3,pros::E_MOTOR_GEARSET_06, false);
pros::Motor driveRT(4,pros::E_MOTOR_GEARSET_06, true);

pros::Motor_Group driveRight ({driveRF,driveRB,driveRT});

//assigning other motors
pros::Motor intake(8,pros::E_MOTOR_GEARSET_06,true);
pros::Motor wallstake(9,pros::E_MOTOR_GEARSET_18,false);

//assigning pneumatics
pros::ADIDigitalOut mogoMech ('B');

//assigning sensors
pros::IMU inertial (1);
pros::GPS gps (11);
pros::Vision vision (21);

//tuneable values for certain systems
const int drivetrainTurnGoverner = 2;
const double ENCadjustment = 1.57;

//double values for storing the final absolute position on the robot
double FAPedX;
double FAPedY;
double FAPedTheta;