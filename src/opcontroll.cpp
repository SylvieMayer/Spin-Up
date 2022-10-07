#include "config.h"
#include "main.h"
#include "pros/misc.h"
#include "pros/rtos.hpp"
#include "sylib/sylib.hpp"
#include <array>
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
int oldFlywheelRPMTarget = 0;



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
double kI = 0.02;
double dT = 0;
double integral = 0;
double error;
double oldError = 0;
double derivative;
double flywheelCurrentLimit;
double power;
double sma;
double ema;
double median;
double raw;

sylib::SylviesPogVelocityEstimator flywheelVelocity(&flywheel, 3600);

double actualCurrentLimit(double temperature){
    double currentLimit;
    if(temperature>=70){
        currentLimit = 0;
    }
    else if(temperature >= 65){
        currentLimit = 312.5;
    }
    else if(temperature >= 60){
        currentLimit = 625;
    }
    else if(temperature >= 55){
        currentLimit = 1250;
    }
    else{
        currentLimit = 2500;
    }
    return currentLimit;
}

void flywheel_speed_set(){
    dT = pros::millis() - oldTime;
    oldTime = pros::millis();
    realVelocity = flywheelVelocity.getVelocity();
    // printf("c\n");
    sma = flywheelVelocity.getSmaFilteredVelocity();
    ema = flywheelVelocity.getEmaFilteredVelocity();
    median = flywheelVelocity.getMedianFilteredVelocity();
    raw = flywheelVelocity.getRawVelocity();
    error = flywheelRPMTarget - realVelocity;
    if (flywheelRPMTarget != oldFlywheelRPMTarget){
        integral = 0;
    }
    oldFlywheelRPMTarget = flywheelRPMTarget;
    integral = integral + error*dT;
    oldError = error;
    flywheelCurrentLimit = actualCurrentLimit(flywheel.get_temperature());
    power = kP * error + kI * integral;
    if(power < 0){
        power = 0;
    }
    if(power > 12000){
        power = 12000;
    }
    pros::lcd::set_text(0, std::to_string(realVelocity));
    pros::lcd::set_text(1, std::to_string(flywheelRPMTarget));
    pros::lcd::set_text(2, std::to_string(power));
    pros::lcd::set_text(3, std::to_string(error));
    pros::lcd::set_text(4, std::to_string(integral));
    pros::lcd::set_text(5, std::to_string(flywheelCurrentLimit));
    pros::lcd::set_text(6, std::to_string(flywheel.get_temperature()));
    pros::lcd::set_text(7, std::to_string(std::abs(error)));
    //                                    0              1   2   3   4            5                 6     7                           8
    printf("%d|%f|%f|%f|%f|%d|%f|%d|%f\n",pros::millis(),raw,sma,median,realVelocity,flywheelRPMTarget,error,flywheel.get_current_draw(),flywheel.get_actual_velocity()*18); 
    flywheel.move_voltage(8000);
}

void flywheelCont()
{
    //update button placement later
    if(master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)){
        flywheelRPMTarget = 5000;
    }
    else{
        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A))
        {
            flywheelRPMTarget = 0;
        }
        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B))
        {
            flywheelRPMTarget = 2700;
        }
        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X))
        {
            flywheelRPMTarget = 3150;
        }
        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y))
        {
            flywheelRPMTarget = 3600;
        }
    }
    flywheel_speed_set();
}

void intakeCont()
{
    if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L1))
    {
        intake.move_velocity(200);
    } else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L2))
    {
        intake.move_velocity(-200);
    } else 
    {
        intake.move_velocity(0);
    }
    
}