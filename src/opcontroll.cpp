#include "main.h"
/*
    here is where driver controll functions will go
    flywheel code is in a separate function because it will likely be complex
*/
double real_left = 0;
double real_right = 0;
double deadzone = 5;
void drive(double left, double right)
{
  left = left*100;
  right = right*100;
  if (abs(left) > deadzone)
    {
      real_left=-left*1.05 + 5;
      leftDrive1.move_voltage(real_left*120);
      leftDrive2.move_voltage(real_left*120);
      leftDrive3.move_voltage(real_left*120);
    }
    else{
      leftDrive1.move_velocity(0);
      leftDrive2.move_velocity(0);
      leftDrive3.move_velocity(0);
   }
   if (abs(right) > deadzone)
   {
      real_right=-right*1.05 + 5;
      rightDrive1.move_voltage(-real_right*120);
      rightDrive2.move_voltage(-real_right*120);
      rightDrive3.move_voltage(-real_right*120);
   }
   else
   {
      rightDrive1.move_velocity(0);
      rightDrive2.move_velocity(0);
      rightDrive3.move_velocity(0);
   }
}
int flywheelRPMTarget = 0;
void flywheelCont()
{
    //update button placement later
    if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A))
    {
        flywheelRPMTarget = 50;
    }
    if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B))
    {
        flywheelRPMTarget = 100;
    }
    if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X))
    {
        flywheelRPMTarget = 150;
    }
    if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y))
    {
        flywheelRPMTarget = 200;
    }
    flywheel.move_velocity(flywheelRPMTarget); 
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