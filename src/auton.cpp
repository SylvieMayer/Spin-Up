#include "main.h"
#include "config.h"
#include <cmath>
/* 
    auton and auton specific functions will go here
    feel free to make additional files if necessary
*/


const double TRACKING_WIDTH = 10.47;
const double WHEEL_DIAMETER = 3.25;
double theta = 0;
double theta_target = 0;
int current_left = 0;
int current_right = 0;
double current_avg = 0;

void driveDistance(double target_distance){
	double start_avg = (current_left+current_right)/2.0;

	current_avg = (current_left+current_right)/2.0 - start_avg;

	double error = target_distance - current_avg;

	double prevTime = sylib::millis();
	bool endMovement = false;
	bool waitingForEnd = true;
	int endMovementTime = 0;

	double prevError = error;
	double integral = 0;
	double power = 0;
	const double kP = 1;
	const double kI = 0.0;
	const double kD = 0.0;

	const double kP2 = 3;

	double theta_error = 0;

	while(!endMovement){
		theta_error = std::fmod(theta_target - theta, 360);
		current_avg = (current_left+current_right)/2.0 - start_avg;
		error = target_distance - current_avg;

		power = error * kP;
		power += ((error-prevError)/(sylib::millis()-prevTime)) * kD;
		integral += (error * (sylib::millis() - prevTime));
		power += integral * kI;

		prevError = error;
		prevTime = sylib::millis();

		if(power > 150){
			power = 150;
		}
		else if(power < -150){
			power = -150;
		}

		leftDrive.move_velocity(-power-theta_error*kP2);
		rightDrive.move_velocity(-power + theta_error*kP2);

		if(std::abs(error) < 0.5 && waitingForEnd){
			waitingForEnd = false;
			endMovementTime = sylib::millis();
		}
		else{
			waitingForEnd = true;
		}
		if(!waitingForEnd && sylib::millis() - endMovementTime > 500){
			endMovement = true;
		}

		sylib::delay(10);

	}
	leftDrive.move_velocity(0);
	rightDrive.move_velocity(0);
	
}

void turnToAngle(double angle){
	theta_target = angle;
	double error = std::fmod(theta_target - theta, 360);
	double prevError = error;
	double integral = 0;
	double power = 0;
	const double kP = 1;
	const double kI = 0.0;
	const double kD = 0.0;
	double prevTime = sylib::millis();
	bool endMovement = false;
	bool waitingForEnd = true;
	int endMovementTime = 0;


	while(!endMovement){
		error = std::fmod(theta_target - theta, 360);
		power = error * kP;
		power += ((error-prevError)/(sylib::millis()-prevTime)) * kD;
		integral += (error * (sylib::millis() - prevTime));
		power += integral * kI;

		prevError = error;
		prevTime = sylib::millis();

		if(power > 100){
			power = 100;
		}
		else if(power < -100){
			power = -100;
		}

		leftDrive.move_velocity(-power);
		rightDrive.move_velocity(power);

		if(std::abs(error) < 0.5 && waitingForEnd){
			waitingForEnd = false;
			endMovementTime = sylib::millis();
		}
		else{
			waitingForEnd = true;
		}
		if(!waitingForEnd && sylib::millis() - endMovementTime > 500){
			endMovement = true;
		}

		sylib::delay(10);
	}
	leftDrive.move_velocity(0);
	rightDrive.move_velocity(0);
}

void odomControlLoop(void * param){
	static int ticks = 0;

	int start_left = leftRot.get_position()*WHEEL_DIAMETER*M_PI/86450;
    int start_right = rightRot.get_position()*WHEEL_DIAMETER*M_PI/86450;

	while(true){
		ticks++;

		current_left = leftRot.get_position()*WHEEL_DIAMETER*M_PI/86450;
		current_right = rightRot.get_position()*WHEEL_DIAMETER*M_PI/86450;
		
		theta = (((current_left-start_left) - (current_right-start_right))/TRACKING_WIDTH);

        if(ticks%5 == 1){
			printf("%d,%f\n",sylib::millis(), theta*180/M_PI);
		}
		
		sylib::delay(10);
	}
}

void auton1(){
    flywheel.set_velocity_custom_controller(3700);
	leftDrive.move_velocity(0);
	rightDrive.move_velocity(0);
	intake.move_velocity(200);
	sylib::delay(500);
	intake.move_velocity(0);
	leftDrive.move_velocity(-20);
	rightDrive.move_velocity(-35);
	sylib::delay(500);
	leftDrive.move_velocity(0);
	rightDrive.move_velocity(0);
	while(flywheel.get_velocity_error() > 25){
		sylib::delay(10);
	}
	intake.move_velocity(-200);
	sylib::delay(300);
	intake.move_velocity(0);
	while(flywheel.get_velocity_error() > 25){
		sylib::delay(10);
	}
	intake.move_velocity(-200);
	sylib::delay(2000);
	intake.move_velocity(0);
	turnToAngle(90);
}