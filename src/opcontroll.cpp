#include "config.h"
#include "main.h"
#include "pros/misc.h"
#include "pros/rtos.hpp"
#include <array>
#include <cmath>
#include <cstddef>
#include <iostream>
#include <string>
#include <queue>
/*
    here is where driver control functions will go
    flywheel code is in a separate function because it will likely be complex
*/
double real_left = 0;
double real_right = 0;
double deadzone = 5;
int flywheelRPMTarget = 0;


void drive(double left, double right){
    left = left*100;
    right = right*100;
   
    if (std::abs(left) > deadzone){
      real_left=-left*1.05 + 5;
      leftDrive.move_voltage(real_left*120);
    }
    else {
      leftDrive.move_velocity(0);
    }

    if (std::abs(right) > deadzone){
      real_right=-right*1.05 + 5;
      rightDrive.move_voltage(real_right*120);
    }
    else{
      rightDrive.move_velocity(0);
    }
}
std::uint32_t oldTime = 0;
double realVelocity;
double kP = 5;
double kI = 0.005;
double dT = 0;
double error;
double flywheelCurrentLimit;
double power;
double sma;
double ema;
double median;
double raw;
double kV = 3.45;
double rawPWMValue;


bool frisbeeInTrack = false;
bool frisbeeInTrackPrevious = false;
bool frisbeeLeftTrack = false;
bool frisbeeEnteredTrack = false;
int frisbeeEnteredTrackStartTime = 0;
int frisbeesInIntakeAtShootTime = 0;

int getFrisbeeState(){
    if(frisbeeEnteredTrack){
        return 1;
    }
    else if(frisbeeLeftTrack){
        return 2;
    }
    else if(frisbeeInTrack){
        return 3;
    }
    else{
        return 0;
    }
}

bool sensorCalibrated = false;
int frisbeeTrackLightingInitial = 0;

int frisbeeDetect(){
    static sylib::hsv pulseColor = sylib::hsv();
    pulseColor.s = 1;
    pulseColor.v = 1;

    int lightReading = frisbeeTrackSensor.get_value();
	if(!sensorCalibrated){
        if(lightReading > 1){
            frisbeeTrackLightingInitial = lightReading;
            sensorCalibrated = true;
        }
        return 0;
    }
	
    
    if(lightReading <= frisbeeTrackLightingInitial*0.65){
        frisbeeInTrack = true;
        if(!frisbeeInTrackPrevious){
            frisbeeEnteredTrack = true;
            pulseColor.h = std::rand() % 360;
            trackLighting.pulse(sylib::Addrled::hsv_to_rgb(pulseColor), 2, 35);
            frisbeeEnteredTrackStartTime = sylib::millis();
            frisbeesInIntakeAtShootTime = getFrisbeesInIntake() - 1;
        }
        else{
            frisbeeEnteredTrack = false;
        }
    }
    else{
        frisbeeInTrack = false;
        if(!frisbeeInTrackPrevious){
            frisbeeEnteredTrack = false;
        }
        else{
            frisbeeEnteredTrack = true;
        }
    }

    frisbeeInTrackPrevious = frisbeeInTrack;

    if(frisbeeEnteredTrack){
        return 1;
    }
    else if(frisbeeLeftTrack){
        return 2;
    }
    else if(frisbeeInTrack){
        return 3;
    }
    else{
        return 0;
    }
}

void flywheelCont()
{
    //update button placement later
    if(master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)){
        //shift state
        if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP)){
            angler.set_value(true);
        }
        if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN)){
            angler.set_value(false);
        }


        if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)){
            flywheelRPMTarget = 0;
            flywheel.set_velocity_custom_controller(flywheelRPMTarget); 
            return;
        }
        else if(master.get_digital(pros::E_CONTROLLER_DIGITAL_A)){
            
            if(std::abs(flywheel.get_velocity_error()) < 100){
                flywheel.set_voltage(5000);
            }
            return;
        }

        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B))
        {
            flywheelRPMTarget = 3000;
        }
        else if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X))
        {
            flywheelRPMTarget = 3600;
        }
        else if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y))
        {
            flywheelRPMTarget = 3300;
        }
    }
    else{
        //unshifted state
        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A))
        {
            flywheelRPMTarget = 0;
        }
        else if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B))
        {
            flywheelRPMTarget = 2400;
        }
        else if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X))
        {
            flywheelRPMTarget = 2800;
        }
        else if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y))
        {
            flywheelRPMTarget = 2600;
        }
        else if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R1)){
            flywheelRPMTarget = 5000;
        }
        else if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT)){
            flywheelRPMTarget += 50;
        }
        else if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT)){
            flywheelRPMTarget -= 50;
        }
    }
    
    if(flywheelRPMTarget > -10){
        frisbeeDetect();

        if((sylib::millis() - frisbeeEnteredTrackStartTime <= 750) && flywheelRPMTarget > 0){
            flywheel.set_voltage(-12000);
        }
        else{
            flywheel.set_velocity_custom_controller(flywheelRPMTarget); 
        }
    }

    if(partner.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP)){
        angler.set_value(true);
    }
    if(partner.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN)){
        angler.set_value(false);
    }
}

void intakeCont()
{
        
    if(getFrisbeesInIntake() == 3){
        trackLighting.set_all(0x730f00);
    }
    else if(getFrisbeesInIntake() == 2){
        trackLighting.set_all(0x6b7300);
    }
    else if(getFrisbeesInIntake() == 1){
        trackLighting.set_all(0x047500);
    }
    else if(getFrisbeesInIntake() == 0){
        trackLighting.set_all(0x007575);
    }
    if(partner.get_digital(pros::E_CONTROLLER_DIGITAL_R1)){
        intake.move_velocity(100);
    }
    else if(partner.get_digital(pros::E_CONTROLLER_DIGITAL_R2)){
        intake.move_velocity(-100);
    }
    else if(partner.get_digital(pros::E_CONTROLLER_DIGITAL_L1)){
        intake.move_voltage(12000);
    }
    else if(partner.get_digital(pros::E_CONTROLLER_DIGITAL_L2)){
        intake.move_voltage(-12000);
    }
    else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L1)){
        intake.move_velocity(200);
        if(getFrisbeesInIntake() == 3){
            master.rumble(".");
        }
    } 
    else if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L2)){
        shootSingleFrisbee();
        // if(getFrisbeesInIntake() > 0 && getFrisbeesInIntake() == frisbeesInIntakeAtShootTime && sylib::millis() - frisbeeEnteredTrackStartTime >= 200 && sylib::millis() - frisbeeEnteredTrackStartTime < 400){
        //     intake.move_velocity(200);
        // }
        // else{
        //     intake.move_velocity(-200);
        // }
        
    } 
    else{
        intake.move_velocity(0);
    }
    
}