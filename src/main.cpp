#include "main.h"
#include "config.h"
#include "pros/adi.hpp"
#include "pros/rtos.hpp"
#include <cmath>
#include <cstdint>
#include <vector>
/*
	IMPORTANT
	Please avoid creating functions in this folder

	When using brackets please line them up as in lines 14-23
	This is to aid readablility
	
	Thanks -Zeke
*/
double initStartTime = 0;
/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */
void on_center_button() 
{
	static bool pressed = false;
	pressed = !pressed;
	if (pressed) {
		pros::lcd::set_text(2, "I was pressed!");
	} else {
		pros::lcd::clear_line(2);
	}
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() 
{
	sylib::SylibDaemon::startSylibDaemon();
	initStartTime = pros::millis();
	pros::lcd::initialize();
	pros::lcd::set_text(1, "Hello PROS User!");
	pros::lcd::register_btn1_cb(on_center_button);

	//pros::Task getFlywheelRPM(getFlywheelRPMfn);
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}

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
void autonomous() {}

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
	uint32_t mainTime = sylib::millis();
	// sylib::Motor flywheel(17, 3600,true);
	// sylib::Device a(1);
	int tick = 0;
	double gainCalculated = 0;
	double valPrint = 0;
	auto gainSma = sylib::EMAFilter();
	// valPrint = gainSma.filter(3.2, 1);
	printf("main start\n");
	// led1.set_all(0xFF0000);
	// led1.set_pixel(0x00FF00, 3);
	std::vector<std::uint32_t> lights;
	lights.resize(16);
	// led1.set_all(0x444444);
	while (true){
		tick++;
		drive(master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y), master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y));
		flywheelCont();
		for(int i = 0; i < 16; i++){
			lights[i] = sylib::millis() + i*0x111111;
		}
		// led1.set_buffer(lights);
		if(master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)){
        	flywheelRPMTarget = 5000;
    	}
		else{
			if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)){
            	flywheelRPMTarget = 0;
        	}
        	if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)){
            	flywheelRPMTarget = 2700;
        	}
        	if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)){
            	flywheelRPMTarget = 3150;
        	}
        	if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y)){
            	flywheelRPMTarget = 3600;
        	}
    	}
    	flywheel.set_velocity_custom_controller(flywheelRPMTarget);
		if(sylib::millis() <= 20000){
			flywheel.set_velocity_custom_controller(3200);
			flywheelRPMTarget = 3200;
		}
		else if(sylib::millis() <= 40000){
			flywheel.set_velocity_custom_controller(3600);
			flywheelRPMTarget = 3600;
		}
		else if(sylib::millis() <= 60000){
			flywheel.set_velocity_custom_controller(2700);
			flywheelRPMTarget = 2700;
		}
		else if(sylib::millis() <= 70000){
			flywheel.stop();
		}
		else if(sylib::millis() <= 90000){
			flywheel.set_velocity(3200);
			flywheelRPMTarget = 3200;
		}
		else if(sylib::millis() <= 110000){
			flywheel.set_velocity(3600);
			flywheelRPMTarget = 3600;
		}
		else if(sylib::millis() <= 130000){
			flywheel.set_velocity(2700);
			flywheelRPMTarget = 2700;
		}
		else{
			flywheel.stop();

		}
		// gainCalculated = flywheel.get_applied_voltage()/flywheel.get_velocity();
		// if (std::abs(gainCalculated) > 4.5 || std::abs(gainCalculated) < 2.5) {
		// }
		// else if(std::isnan(gainCalculated)){
		// }
		// else{
		// 	// valPrint = gainSma.filter(gainCalculated, 0.005);
		// }
		gainCalculated = gainSma.filter(std::abs(flywheel.get_velocity_error()),0.05);

		if(gainCalculated > 300){
			valPrint = 0;
		}
		else{
			valPrint = gainCalculated;
		}
		// // flywheelRPMTarget = 3800;
		// // flywheel.set_velocity_target(flywheelRPMTarget);
		printf("%d|%f|%f|%d|%d|%d|%f|%f|%d\n", sylib::millis(), flywheel.get_velocity(), flywheel.get_velocity_motor_reported(), flywheelRPMTarget, flywheel.get_p_voltage(), flywheel.get_i_voltage(), flywheel.get_velocity_error(), 15*valPrint, flywheel.get_applied_voltage()/5);
		sylib::delay_until(&mainTime,10);
	}
}
		// printf("created main task loop\n");

