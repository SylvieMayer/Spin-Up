#pragma once
#include "main.h"
#include "pros/motors.hpp"

extern pros::Controller master;

//motors
extern pros::Motor leftDrive1;
extern pros::Motor leftDrive2;
extern pros::Motor leftDrive3;
extern pros::Motor rightDrive1;
extern pros::Motor rightDrive2;
extern pros::Motor rightDrive3;
extern sylib::Motor flywheel;
extern pros::Motor intake;

extern pros::Motor_Group leftDrive;
extern pros::Motor_Group rightDrive;
//sensors


//functions
void drive(double left, double right);
void flywheelCont();
void intakeCont();

//variables
extern int flywheelRPMTarget;