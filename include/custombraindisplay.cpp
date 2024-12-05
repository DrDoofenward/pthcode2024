//custombraindisplay is 3rd in the conga line, and holds all functionality for the brain
#include "positiontracking.cpp"
//extra included files for the brain
#include "pros/colors.h"
#include "pros/rtos.hpp"
#include "pros/screen.h"
#include "pros/screen.hpp"
#include <string>

/*requirements
 * 1. 8 Text lines for detecting motor temperature
 * 2. Text lines for position tracking
 * 3. Auton (and team) Selector
 * 4. Display showing the robots position on the field
 */

void drawbackground () {
pros::screen::set_pen(COLOR_YELLOW);
pros::screen::draw_rect(0, 0, 480, 240); }

//class for placing a robot on a 2d plane for 
class positiondisplay {
    private:
        //draws another display so it overlaps the old display
        void drawfield() {
            //overlaps the old display
            pros::screen::set_pen(COLOR_GREEN);
            pros::screen::draw_rect(240, 20, 200, 200);
            //draws the new display
            pros::screen::set_pen(COLOR_WHITE);
            //gotta do dis later
        }

        //draws a square that represents the robot so we know where the robot believes its positioned
        void drawrobot() {}

    public:
        //updates the robots current position
        void updateposition(double X, double Y, double theta) {}
};




//sets the pen color relating to where a value lies on a low-high bar
void setColorStatus(double value, double low, double high, bool inverted) {
    double ygreenpoint =  high*0.2;
    double yellowpoint =  high*0.4;
    double orangepoint =  high*0.6;
    double redpoint =  high*0.8;
    if (value <= ygreenpoint) {pros::screen::set_pen(COLOR_GREEN);}
    if (value >= ygreenpoint) {pros::screen::set_pen(COLOR_YELLOW_GREEN);}
    if (value >= yellowpoint) {pros::screen::set_pen(COLOR_YELLOW);}
    if (value >= orangepoint) {pros::screen::set_pen(COLOR_ORANGE);}
    if (value >= redpoint) {pros::screen::set_pen(COLOR_RED);}
}

//function to print the motors value
void printMotorTemp(pros::Motor motor,std::string name,int line) {
    int motortemp = motor.get_temperature();
    setColorStatus(motortemp, 0, 100, false);
    pros::screen::print(TEXT_SMALL, line,"%s",name + ": " + (std::to_string(motortemp)));
}

//task function that constantly sets all of the temps every .5 seconds
void taskTempDisplay () {
    pros::screen::set_pen(COLOR_WHITE);
    pros::delay(20);
    pros::screen::print(TEXT_SMALL, 1,"%s","Motor Temps");
    while (true) {
        printMotorTemp(driveLB, "driveLeft", 2);
        printMotorTemp(driveRB, "driveRight", 3);
        printMotorTemp(intake, "intakeMotor", 4);
        pros::delay(500);
    }
}