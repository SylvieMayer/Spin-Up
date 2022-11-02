#include "main.h"
#include "config.h"
#include "pros/adi.hpp"
#include "pros/misc.h"
#include "pros/rtos.hpp"
#include "sylib/system.hpp"
#include <cmath>
#include <cstdint>
#include <string>
#include <vector>
/*
	IMPORTANT
	Please avoid creating functions in this folder

	When using brackets please line them up as in lines 14-23
	This is to aid readablility
	
	Thanks -Zeke
*/


/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
const int autonRoutine = 1;


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

// const int CHASSIS_COLOR_START = 0x440044;
// const int CHASSIS_COLOR_END = 0x004444;
const int CHASSIS_COLOR_START = 0xFF00FF;
const int CHASSIS_COLOR_END = 0x00FFFF;

void chassis_light_default(){
	chassisLighting1.gradient(CHASSIS_COLOR_START, CHASSIS_COLOR_END, 0, 0, true, true);
	chassisLighting2.gradient(CHASSIS_COLOR_START, CHASSIS_COLOR_END, 0, 0, false, true);
	chassisLighting1.cycle(*chassisLighting1, 15, 0, true);
	chassisLighting2.cycle(*chassisLighting2, 15);
}

const double current_draw_cutoff = 50;

void chassis_light_control(){
	static int leftCurrentDraw;
	static int rightCurrentDraw;

	static int leftCurrentLimit;
	static int rightCurrentLimit;

	static double leftSpeed;
	static double rightSpeed;

	static std::uint32_t shift_amount;

	static double current_draw_speed_ratio;

	leftCurrentDraw = (leftDrive.get_current_draws()[0]+leftDrive.get_current_draws()[1]+leftDrive.get_current_draws()[2])/3;
	rightCurrentDraw = (rightDrive.get_current_draws()[0]+rightDrive.get_current_draws()[1]+rightDrive.get_current_draws()[2])/3;

	leftCurrentLimit = (actualCurrentLimit(leftDrive1.get_temperature()) +
					    actualCurrentLimit(leftDrive2.get_temperature()) + 
						actualCurrentLimit(leftDrive3.get_temperature()))/3;

	rightCurrentLimit = (actualCurrentLimit(rightDrive1.get_temperature()) +
					     actualCurrentLimit(rightDrive2.get_temperature()) + 
						 actualCurrentLimit(rightDrive3.get_temperature()))/3;

	leftSpeed = std::abs((leftDrive.get_actual_velocities()[0] + leftDrive.get_actual_velocities()[1]+leftDrive.get_actual_velocities()[2])/3);
	rightSpeed = std::abs((rightDrive.get_actual_velocities()[0] + rightDrive.get_actual_velocities()[1]+rightDrive.get_actual_velocities()[2])/3);


	current_draw_speed_ratio = std::abs((2500*(double)(leftCurrentDraw+rightCurrentDraw)/(double)(leftCurrentLimit+rightCurrentLimit))/(std::abs(((leftSpeed+rightSpeed)+1))/2));

	shift_amount = (std::uint32_t)(((current_draw_speed_ratio-current_draw_cutoff)));

	if(current_draw_speed_ratio > current_draw_cutoff){
		chassisLighting1.color_shift(shift_amount, -shift_amount, -shift_amount);
		chassisLighting2.color_shift(shift_amount, -shift_amount, -shift_amount);
	}
	else{
		chassisLighting1.color_shift(0, 0, 0);
		chassisLighting2.color_shift(0, 0, 0);
	}
}


void initialize(){
	sylib::initialize();
	chassis_light_default();
	stringShooter.set_value(false);
	leftRot.reset_position();
	rightRot.reset_position();
	imu.reset();
	rollerSensor.set_led_pwm(100);
	pros::Task odomTask(odomControlLoop);

}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {

	chassisLighting1.set_all(0xfa8b02);
	chassisLighting2.set_all(0xfa8b02);
	trackLighting.set_all(0x000000);
	flywheel.stop();
}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {
	if(autonRoutine == 1){
		chassisLighting1.set_all(0xab03ff);
		chassisLighting2.set_all(0xab03ff);
		trackLighting.set_all(0x000000);
	}
	else if(autonRoutine == 2){
		chassisLighting1.set_all(0x00fcd2);
		chassisLighting2.set_all(0x00fcd2);
		trackLighting.set_all(0x000000);
	}
	else if(autonRoutine == 3){
		chassisLighting1.set_all(0xff55d2);
		chassisLighting2.set_all(0xff55d2);
		trackLighting.set_all(0x000000);
	}
}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */



void autonomous() {
	chassisLighting1.set_all(0x00ff00);
	chassisLighting2.set_all(0x00ff00);
	trackLighting.set_all(0x000000);
	if(autonRoutine==1){
		farSideHalfWP();
	}
	else if(autonRoutine == 2){
		spinCloseRoller();
	}
	else if(autonRoutine == 3){
		skillsAuto();
	}
	else{
		trackLighting.set_all(0xee03ff);
	}
	
}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */



void opcontrol() {
	
	uint32_t control_ticks = 0;
	int flyVel = 0;
	int flyVelTarget = 0;
	int flyVelError = 0;
	chassis_light_default();
	angler.set_value(true);
	flywheelRPMTarget = 2250;
	uint32_t clock = sylib::millis();
	while (true){
		control_ticks++;
		
		if(master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)){ // SHIFT KEY
		if(master.get_digital(pros::E_CONTROLLER_DIGITAL_LEFT)){
			leftDrive.move_velocity(125);
			rightDrive.move_velocity(-125);
		}
		else if(master.get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT)){
			leftDrive.move_velocity(-125);
			rightDrive.move_velocity(125);
		}
		else{
			drive(master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y), master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y));
		}
		}
		else{
			if(master.get_digital(pros::E_CONTROLLER_DIGITAL_UP)){
				leftDrive.move_velocity(-125);
				rightDrive.move_velocity(-125);
			}
			else if(master.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN)){
        		leftDrive.move_velocity(125);
        		rightDrive.move_velocity(125);
			}
			else{
				drive(master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y), master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y));
			}
		}
		
		
		if(partner.get_digital(pros::E_CONTROLLER_DIGITAL_LEFT) &&
		   partner.get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT) && 
		   partner.get_digital(pros::E_CONTROLLER_DIGITAL_A) && 
		   partner.get_digital(pros::E_CONTROLLER_DIGITAL_Y)){
			
			if(master.get_digital(pros::E_CONTROLLER_DIGITAL_L1) &&
		   	   master.get_digital(pros::E_CONTROLLER_DIGITAL_R1) && 
		       master.get_digital(pros::E_CONTROLLER_DIGITAL_L2) && 
		       master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)){

				stringShooter.set_value(true);
		   }
		}
		else{
			flywheelCont();
			intakeCont();
		}
		
		chassis_light_control();

		flyVel = (int)flywheel.get_velocity();
		flyVelTarget = (int)flywheel.get_velocity_target();
		flyVelError = flyVelTarget-flyVel;

		if((control_ticks+6) % 12 == 0){
			if(std::abs(flyVelError) > 50 && std::abs(flyVelTarget) < 3900 && sylib::millis() > 2000){
				master.rumble("-");
			}
		}
		else if ((control_ticks) % 12 == 0) {
				master.set_text(0,0,std::to_string(flyVel) + " | " + std::to_string(flyVelTarget)+ " | " + std::to_string(flyVelError) + "    ");
		}
		if(control_ticks%1 == 0){
			
			printf("%d,%f,%f,%d\n", sylib::millis(), flywheel.get_velocity_target(), flywheel.get_velocity(),flywheel.get_applied_voltage());
		}
		// odomControlLoop();
		sylib::delay_until(&clock,10);
	}
}

