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

pros::Motor leftDrive1(1);
pros::Motor leftDrive2(2);
pros::Motor leftDrive3(3);
pros::Motor rightDrive1(4);
pros::Motor rightDrive2(5);
pros::Motor rightDrive3(6);
pros::Motor flywheel(11);
pros::Motor intake(12);

//sensors
