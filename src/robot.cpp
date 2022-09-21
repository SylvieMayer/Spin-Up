#include "main.h"

/*
    This file is for robot configuration
    Please refrain from creating functions here
    Variables are ok
    -Zeke
*/

pros::Controller master(pros::E_CONTROLLER_MASTER);


//motors 
/* 
    please update drive motors at somepoint with groups and reversals
    ports are temporary until construction completes
    -Zeke
*/ 


pros::Motor flywheel(17, true);
pros::Motor intake(14);

pros::Motor leftDrive1(11, false);
pros::Motor leftDrive2(12, false);
pros::Motor leftDrive3(13, false);
pros::Motor rightDrive1(15, true);
pros::Motor rightDrive2(16, true);
pros::Motor rightDrive3(20, true);

pros::Motor_Group leftDrive({leftDrive1, leftDrive2, leftDrive3});
pros::Motor_Group rightDrive({rightDrive1, rightDrive2, rightDrive3});

//sensors
