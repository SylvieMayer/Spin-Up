#pragma once
#include "main.h"
#include "pros/adi.hpp"
#include "pros/distance.hpp"
#include "pros/imu.hpp"
#include "pros/misc.hpp"
#include "pros/motors.hpp"
#include "pros/optical.hpp"
#include "pros/rotation.hpp"

extern pros::Controller master;
extern pros::Controller partner;

//motors
extern pros::Motor leftDrive1;
extern pros::Motor leftDrive2;
extern pros::Motor leftDrive3;
extern pros::Motor rightDrive1;
extern pros::Motor rightDrive2;
extern pros::Motor rightDrive3;


extern sylib::Motor flywheel;

extern sylib::Addrled trackLighting;
extern sylib::Addrled chassisLighting1;
extern sylib::Addrled chassisLighting2;

extern pros::ADILineSensor frisbeeTrackSensor;
extern pros::ADIDigitalOut stringShooter;
extern pros::ADIDigitalOut angler;
extern pros::Distance indexerSensor;
extern pros::IMU imu;
extern pros::Optical rollerSensor;

extern pros::Rotation leftRot;
extern pros::Rotation rightRot;


extern pros::Motor intake;

extern pros::Motor_Group leftDrive;
extern pros::Motor_Group rightDrive;
//sensors


//functions
void drive(double left, double right);
void flywheelCont();
void intakeCont();
int frisbeeDetect();
void odomControlLoop(void * param);
void turnToAngle();
int getFrisbeesInIntake();
int getRollerColor();
void spinCloseRoller();
void farSideHalfWP();

//variables
extern int flywheelRPMTarget;

extern int frisbeeTrackLightingInitial;

extern sylib::MedianFilter hueFilter;
extern sylib::MedianFilter distanceFilter;