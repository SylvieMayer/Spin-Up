#include "config.h"
#include "main.h"
#include "pros/misc.h"
#include "pros/rtos.hpp"
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
extern "C" int32_t vexDeviceGetTimestampByIndex(int32_t index);
extern "C" uint32_t vexSystemTimeGet(void);
extern "C" uint64_t vexSystemHighResTimeGet(void);
extern "C" void vPortExitCritical();
extern "C" void vPortEnterCritical();


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
double currentTicks;
double speedDifference;
double power;
double ema = 0;
double kA = 0.15;
double kA2 = 0.03;
double kV = 3.35;
std::uint32_t currentTime = 0;
double oldTicks = 0;
double dP;
double realVelocity;
double kT = 50;
double internalVelocityMeasure;
double kP = 5;
double kI = 0.02;
double kD = 200;
double integral = 0;
double error;
double oldError = 0;
double derivative;
double ema_error;
double flywheelCurrentLimit = 2500;
double actualCurrent = 0;
const int dT = 1;
const int queueMaxSize = 10;
double velTotal = 0;
std::queue<double> velQueue;
double sma_velocity = 0;
double raw_ema = 0;
int32_t flywheelMotorTimestamp = 0;
int32_t prev_flywheelMotorTimestamp = 0;
int32_t timestampDiff = 0;
int32_t oldDiff = 0;
double rawTicks = 0;
double rawTicksMotorTime = 0;
uint32_t systemTime = 0;

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
    vPortEnterCritical();
    currentTime = pros::millis();
    currentTicks = flywheel.get_position();
    flywheelMotorTimestamp = vexDeviceGetTimestampByIndex(16);
    systemTime = vexSystemTimeGet();
    // rawTicksMotorTime = flywheel.get_raw_position(&systemTime);
    // rawTicks = flywheel.get_raw_position(&currentTime);
    vPortExitCritical();
    timestampDiff = flywheelMotorTimestamp- prev_flywheelMotorTimestamp;
    prev_flywheelMotorTimestamp = flywheelMotorTimestamp;
    internalVelocityMeasure = flywheel.get_actual_velocity();
    dP = currentTicks - oldTicks;
    oldTicks = currentTicks;
    realVelocity = (dP/kT)/dT * 60000;
    velTotal += realVelocity;
    if(velQueue.size() >= queueMaxSize){
        velTotal -= velQueue.front();
        velQueue.pop();
    }
    velQueue.push(realVelocity);
    sma_velocity = velTotal/velQueue.size();
    ema = sma_velocity*kA + ema*(1-kA);
    raw_ema = realVelocity*kA + raw_ema*(1-kA);
    error = flywheelRPMTarget - sma_velocity;
    if (flywheelRPMTarget != oldFlywheelRPMTarget){
        integral = 0;
    }
    oldFlywheelRPMTarget = flywheelRPMTarget;
    integral = integral + error*dT;
    ema_error = std::abs(error)*kA2 + ema_error*(1-kA2);
    derivative = (error-oldError)/dT;
    oldError = error;
    flywheelCurrentLimit = actualCurrentLimit(flywheel.get_temperature());
    actualCurrent = flywheel.get_current_draw();
    power = kP * error + kI * integral;
    if(power < 0){
        power = 0;
    }
    if(power > 12000){
        power = 12000;
    }
    pros::lcd::set_text(0, std::to_string(ema));
    pros::lcd::set_text(1, std::to_string(flywheelRPMTarget));

    pros::lcd::set_text(2, std::to_string(power));
    pros::lcd::set_text(3, std::to_string(ema_error));
    pros::lcd::set_text(4, std::to_string(integral));
    pros::lcd::set_text(5, std::to_string(flywheelCurrentLimit));
    pros::lcd::set_text(6, std::to_string(flywheel.get_temperature()));
    pros::lcd::set_text(7, std::to_string(std::abs(error)));
    int32_t diffTime = (currentTime-flywheelMotorTimestamp);
    int32_t diffyDiff = diffTime - oldDiff;
    oldDiff = diffTime;//                                                0           1            2   3         4                 5     6             7            8                          9       10 11                     12            13       14        15           16                 17       18         
    printf("%d|%f|%f|%f|%d|%f|%f|%f|%f|%f|%f|%d|%d|%d|%d|%f|%d|%f|%f \n",currentTime,sma_velocity,ema,ema_error,flywheelRPMTarget,error,actualCurrent,realVelocity,internalVelocityMeasure*18,raw_ema,dP,flywheelMotorTimestamp,timestampDiff,diffTime,diffyDiff,currentTicks,vexSystemTimeGet()); 
    // printf("%d|%d|%d \n",currentTime,flywheelMotorTimestamp,vexSystemTimeGet());
    flywheel.move_voltage(power);
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