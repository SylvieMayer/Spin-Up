#include "main.h"
#include "config.h"
/* 
    auton and auton specific functions will go here
    feel free to make additional files if necessary
*/


const double TRACKING_WIDTH = 10.47;
const double WHEEL_DIAMETER = 3.25;
double theta = 0;
double x_pos = 0;
double y_pos = 0;
double x_target = 0;
double y_target = 0;
double total_left = 0;
double total_right = 0;

void turnToAngle(double angle){
	double error = angle-(int)theta % 360;
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
	printf("turn called: %f\n", error);


	while(!endMovement){
		error = angle-(int)theta % 360;
		power = error * kP;
		power += ((error-prevError)/(sylib::millis()-prevTime)) * kD;
		integral += (error * (sylib::millis() - prevTime));
		power += integral * kI;

		prevError = error;
		prevTime = sylib::millis();

		leftDrive.move_velocity(-power);
		rightDrive.move_velocity(power);

		if(std::abs(error) < 1.5 && waitingForEnd){
			waitingForEnd = false;
			endMovementTime = sylib::millis();
		}
		else{
			waitingForEnd = true;
		}
		if(!waitingForEnd && sylib::millis() - endMovementTime > 500){
			endMovement = true;
		}
		printf("%f | %f\n", error, power);

		sylib::delay(10);
	}
	printf("end\n");
	leftDrive.move_velocity(0);
	rightDrive.move_velocity(0);
}

void odomControlLoop(void * param){
	static int ticks = 0;
	
	static double delta_left = 0;
	static double delta_right = 0;
	static double delta_theta = 0;
    static int previous_left = leftRot.get_position()*WHEEL_DIAMETER*M_PI/86450;
    static int previous_right = rightRot.get_position()*WHEEL_DIAMETER*M_PI/86450;
    static double arc_radius = 0;
    static double delta_local_y= 0;

    static double delta_x = 0;
    static double delta_y = 0;

    int start_left = leftRot.get_position()*WHEEL_DIAMETER*M_PI/86450;
    int start_right = rightRot.get_position()*WHEEL_DIAMETER*M_PI/86450;



	
	while(true){
		int current_left = leftRot.get_position()*WHEEL_DIAMETER*M_PI/86450;
		int current_right = rightRot.get_position()*WHEEL_DIAMETER*M_PI/86450;
		ticks++;

		total_left = -(current_left - start_left);
		total_right = (current_right - start_right);
        

        delta_left = current_left - previous_left;
        delta_right = current_right - previous_right;
        delta_theta = ((total_left - total_right)/TRACKING_WIDTH) - theta;

		previous_left = current_left;
		previous_right = current_right;

		theta = theta + delta_theta;
        
        if(delta_theta != 0){
            arc_radius = (delta_left+delta_right)/(2*delta_theta);
        }
    
        delta_local_y = 2*sin(delta_theta/2) * arc_radius;

        delta_y = delta_local_y * cos(theta);
        delta_x = delta_local_y * sin(theta);

        x_pos = x_pos + delta_x;
        y_pos = y_pos + delta_y;

        if(ticks%5 == 1){
			printf("%d,%f,%f,%f\n",sylib::millis(), theta*180/M_PI, x_pos, y_pos);
			// printf("%f\n", theta);
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