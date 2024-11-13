#include "main.h"

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
pros::Vision vision (1);
