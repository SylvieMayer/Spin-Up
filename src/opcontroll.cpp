#include "config.h"
#include "main.h"
#include "pros/misc.h"
#include "pros/rtos.hpp"
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

// sylib::SylviesPogVelocityEstimator flywheelVelocity(&flywheel, 3600);

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
	std::cout << "flywheel \n";
    dT = pros::millis() - oldTime;
    oldTime = pros::millis();
    // realVelocity = flywheelVelocity.getVelocity();
    // sma = flywheelVelocity.getSmaFilteredVelocity();
    // ema = flywheelVelocity.getEmaFilteredVelocity();
    // median = flywheelVelocity.getMedianFilteredVelocity();
    // raw = flywheelVelocity.getRawVelocity();
    error = flywheelRPMTarget - realVelocity;
    flywheelCurrentLimit = actualCurrentLimit(flywheel.get_temperature());
    // rawPWMValue = vexDeviceMotorPwmGet(vexDeviceGetByIndex(16));
    
    // if(error >= 300){
    //     power = 12000;
    // }
    // else{
    //     power = flywheelRPMTarget*kV + integral*kI;
    // }
    // // power = kP * error + kI * integral;
    // if(power < 0){
    //     power = 0;
    // }
    // else if(power > 12000){
    //     power = 12000;
    // }
    int timeMultiplier = 2500;
    if(pros::millis() <= timeMultiplier*1){
        power = 10;
    }
    else if(pros::millis() <= timeMultiplier*2){
        power = 20;
    }
    else if(pros::millis() <= timeMultiplier*3){
        power = 30;
    }
    else if(pros::millis() <= timeMultiplier*4){
        power = 40;
    }
    else if(pros::millis() <= timeMultiplier*5){
        power = 50;
    }
    else if(pros::millis() <= timeMultiplier*6){
        power = 60;
    }
    else if(pros::millis() <= timeMultiplier*7){
        power = 70;
    }
    else if(pros::millis() <= timeMultiplier*8){
        power = 80;
    }
    else if(pros::millis() <= timeMultiplier*9){
        power = 90;
    }
    else if(pros::millis() <= timeMultiplier*10){
        power = 100;
    }
    else if(pros::millis() <= timeMultiplier*11){
        power = 0;
    }
    else{
        exit(0);
    }
    pros::lcd::set_text(0, std::to_string(realVelocity));
    pros::lcd::set_text(1, std::to_string(flywheelRPMTarget));
    pros::lcd::set_text(2, std::to_string(power));
    pros::lcd::set_text(3, std::to_string(error));
    pros::lcd::set_text(5, std::to_string(flywheelCurrentLimit));
    pros::lcd::set_text(6, std::to_string(flywheel.get_temperature()));
    pros::lcd::set_text(7, std::to_string(std::abs(error)));
    //                                                0              1   2   3      4            5                 6     7                           8      9                                            10                                       11                                  12
    // printf("%d|%f|%f|%f|%f|%d|%f|%d|%f|%f|%d|%d|%f\n",pros::millis(),raw,sma,median,realVelocity,flywheelRPMTarget,error,flywheel.get_current_draw(),power, rawPWMValue, flywheel.get_voltage(), pros::battery::get_voltage(), flywheelVelocity.getMaxFilteredAcceleration()); 
    // if(pros::millis() <= 15000){
    //     flywheel.move_voltage(6000 +(sin(pros::millis()*6.14/5000)) * 2000);
    // }
    // else if(pros::millis() <= 25000){
    //     flywheel.move_voltage(9000);

    // }
    // else if(pros::millis() <= 35000){
    //             flywheel.move_voltage(4000 +(sin(pros::millis()*6.14/2500)) * 3000);
    // }
    // else if(pros::millis() <= 50000){
    //     flywheel.move_voltage(0);   
    // }

    // flywheel.move_voltage(power);  
    // vexDeviceMotorPwmSet(vexDeviceGetByIndex(16),power);
    // std::cout << "Value: " <<vexDeviceLedRgbGet(vexDeviceGetByIndex(16)) << "\n";
}

void flywheelCont()
{
    //update button placement later
    // if(master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)){
    //     flywheelRPMTarget = 5000;
    // }
    // else{
    //     if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A))
    //     {
    //         flywheelRPMTarget = 0;
    //     }
    //     if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B))
    //     {
    //         flywheelRPMTarget = 2700;
    //     }
    //     if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X))
    //     {
    //         flywheelRPMTarget = 3150;
    //     }
    //     if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y))
    //     {
    //         flywheelRPMTarget = 3600;
    //     }
    // }
    // if(pros::millis() <= 15000){
    //     flywheelRPMTarget = 2000;
    // }
    // else if(pros::millis() <= 25000){
    //     flywheelRPMTarget = 3000;
    // }
    // else if(pros::millis() <= 35000){
    //     flywheelRPMTarget = 2750;
    // }
    // else if(pros::millis() <= 50000){
    //     flywheelRPMTarget = 3600;
    // }
    // else{
    //     flywheelRPMTarget = 0;
    // }
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