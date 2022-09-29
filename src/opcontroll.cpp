#include "config.h"
#include "main.h"
#include "pros/misc.h"
/*
    here is where driver controll functions will go
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
double currentSpeed;
double speedDifference;
double power;

void flywheel_speed_set(){
    currentSpeed = flywheel.get_actual_velocity();
    pros::lcd::set_text(1, std::to_string(currentSpeed));

    speedDifference = flywheelRPMTarget - currentSpeed;
    if (flywheelRPMTarget > 0){
        if (flywheelRPMTarget == 500){
            power = 12000;
        }
        if(speedDifference>=0){
            power = 11500;
        }
        else{
            power = 7000;
        }
    }
    else{
        power = 0;
    }
    flywheel.move_voltage(power);
}

void flywheelCont()
{
    //update button placement later
    if(master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)){
        flywheelRPMTarget = 500;
    }
    else{
        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A))
        {
            flywheelRPMTarget = 0;
        }
        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B))
        {
            flywheelRPMTarget = 150;
        }
        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X))
        {
            flywheelRPMTarget = 100;
        }
        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y))
        {
            flywheelRPMTarget = 200;
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