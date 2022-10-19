#include "main.h"
#include <cstdint>
#include <optional>
#include <vector>

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


sylib::SpeedControllerInfo flywheelController (
        [](double rpm){return std::pow(M_E, (-0.001*rpm* 3600 / 3600 + 1)) + 3.125;}, // kV function
        10, // kP
        0.001, // kI
        0, // kD
        0, // kH
        true, // anti-windup enabled
        50, // anti-windup range
        true, // p controller bounds threshold enabled
        50, // p controller bounds cutoff enabled
        0.01, // kP2 for when over threshold
        50, // range to target to apply max voltage
        false, // coast down enabled
        0,  // coast down theshhold
        1 // coast down constant
);

sylib::Motor flywheel(17, 3600,true, flywheelController);
sylib::Addrled trackLighting(1,1,24);
sylib::Addrled chassisLighting1(1,3,64);
sylib::Addrled chassisLighting2(22,3,64);

pros::Motor intake(14);

pros::Motor leftDrive1(11, false);
pros::Motor leftDrive2(12, false);
pros::Motor leftDrive3(13, false);
pros::Motor rightDrive1(15, true);
pros::Motor rightDrive2(16, true);
pros::Motor rightDrive3(20, true);

pros::ADILineSensor frisbeeTrackSensor(2);

pros::Motor_Group leftDrive({leftDrive1, leftDrive2, leftDrive3});
pros::Motor_Group rightDrive({rightDrive1, rightDrive2, rightDrive3});

//sensors
