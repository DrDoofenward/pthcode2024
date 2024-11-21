//custombraindisplay is 3rd in the conga line, and holds all functionality for the brain
#include "positiontracking.cpp"
#include "pros/colors.h"
#include "pros/screen.h"
#include "pros/screen.hpp"
#include <string>

/*requirements
 * 1. 8 Text lines for detecting motor temperature
 * 2. Text lines for position tracking
 * 3. Auton (and team) Selector
 * 4. Display showing the robots position on the field
 */

//test function
void fas() {
    int i = 1;
    pros::screen::print(TEXT_MEDIUM, 1,"%s","asdasd" + (std::to_string(i))); }

//sets the pen color relating to where a value lies on a low-high bar
void setColorStatus(double value, double low, double high, bool inverted) {
    double ygreenpoint =  high*0.2;
    double yellowpoint =  high*0.4;
    double orangepoint =  high*0.6;
    double redpoint =  high*0.8;
    if (value >= ygreenpoint) {pros::screen::set_pen(COLOR_YELLOW_GREEN);}
    if (value >= yellowpoint) {pros::screen::set_pen(COLOR_YELLOW);}
    if (value >= orangepoint) {pros::screen::set_pen(COLOR_ORANGE);}
    if (value >= redpoint) {pros::screen::set_pen(COLOR_RED);}
    else {pros::screen::set_pen(COLOR_GREEN);}
}

//test function to print the motors value
void printMotorTemp(pros::Motor motor,std::string name,int line) {
    double motortemp = motor.get_temperature();
    setColorStatus(motortemp, 0, 100, false);
    pros::screen::print(TEXT_MEDIUM, 1,"%s",name + " temperature: " + (std::to_string(motortemp)));
}

