#include "main.h"
#include "config.h"
#include "pros/motors.h"
#include "sylib/system.hpp"
#include <cmath>
/* 
    auton and auton specific functions will go here
    feel free to make additional files if necessary
*/


const double TRACKING_WIDTH = 11.6;
const double WHEEL_DIAMETER = 3.25;
double theta = 0;
double theta_target = 0;
double current_left = 0;
double current_right = 0;
double current_avg = 0;

void moveChassis(double left, double right){
    printf("%d,%f,%f\n", sylib::millis(), left, right);
    leftDrive.move_voltage(left);
    rightDrive.move_voltage(right);
}

void driveDistance(double target_distance, int timeout, int maxSpeed = 150){
	double start_avg = (current_left+current_right)/2.0;

	current_avg = (current_left+current_right)/2.0 - start_avg;

	double error = target_distance - current_avg;

	double prevTime = sylib::millis();
    int startTime = sylib::millis();

    int cutoffTime = startTime + timeout;


    bool endMovement = false;
	int endMovementTime = 0;

    bool withinErrorBounds = false;
    bool prev_withinErrorBounds = false;

	double prevError = error;
	double integral = 0;
	double power = 0;
	const double kP = 15;
	const double kI = 0;
	const double kD = 200;

	const double kP2 = 0.05;

	double theta_error = 0;

    double turnCorrection = 0;
    double left_voltage = 0;
    double right_voltage = 0;

	while(!endMovement && sylib::millis() < cutoffTime){
		theta_error = std::fmod(theta_target - theta, 360);
		current_avg = ((current_left+current_right)/2.0 - start_avg) * WHEEL_DIAMETER * M_PI/86450;
		error = target_distance - current_avg;

		power = error * kP;


		power += ((error-prevError)/(sylib::millis()-prevTime)) * kD;
		integral += (error * (sylib::millis() - prevTime));
        if(std::abs(error) < 15){
            power += integral * kI;
        }
		if(prevError/error < 0){
            integral = 0;
        }

		prevError = error;
		prevTime = sylib::millis();

		if(power > maxSpeed){
			power = maxSpeed;
		}
		else if(power < -maxSpeed){
			power = -maxSpeed;
		}
        
        turnCorrection = theta_error*kP2*power;

        left_voltage = (-power-turnCorrection)*55;
        right_voltage = (-power+turnCorrection)*55;
        if(left_voltage > 100){
            left_voltage += 900;
        }
        else if(left_voltage < 100){
            left_voltage -= 900;
        }

        if(right_voltage > 100){
            right_voltage += 900;
        }
        else if(right_voltage < 100){
            right_voltage -= 900;
        }
        
        moveChassis(left_voltage,right_voltage);

        

        if(std::abs(error) < 0.5){
            withinErrorBounds = true;
        }
        else{
            withinErrorBounds = false;
        }

        if(withinErrorBounds != prev_withinErrorBounds && withinErrorBounds){
            endMovementTime = sylib::millis();
        }

        if(withinErrorBounds && sylib::millis() > endMovementTime + 500){
            endMovement = true;
        }

        prev_withinErrorBounds = withinErrorBounds;

		sylib::delay(10);

	}
	leftDrive.move_velocity(0);
	rightDrive.move_velocity(0);
	
}

void turnToAngle(double angle, int timeout){
	theta_target = angle;
	double error = std::fmod(theta_target - theta, 360);
	double prevError = error;
	double integral = 0;
	double power = 0;
	const double kP = 8;
	const double kI = 0;
	const double kD = 200;
	double prevTime = sylib::millis();
	bool endMovement = false;
	int endMovementTime = 0;

    int startTime = sylib::millis();

    int cutoffTime = startTime + timeout;

    bool withinErrorBounds = false;
    bool prev_withinErrorBounds = false;

    double left_voltage = 0;
    double right_voltage = 0;

	while(!endMovement && sylib::millis() < cutoffTime){
		error = std::fmod(theta_target - theta, 360);
		power = error * kP;
		power += ((error-prevError)/(sylib::millis()-prevTime)) * kD;
        if(std::abs(error) < 5){
            integral += (error * (sylib::millis() - prevTime));
        }
        if(prevError/error < 0){
            integral = 0;
        }
		power += integral * kI;

		prevError = error;
		prevTime = sylib::millis();

		if(power > 125){
			power = 125;
		}
		else if(power < -125){
			power = -125;
		}

        left_voltage = (-power)*45;
        right_voltage = (power)*45;

        if(left_voltage > 100){
            left_voltage += 1500;
        }
        else if(left_voltage < 100){
            left_voltage -= 1500;
        }

        if(right_voltage > 100){
            right_voltage += 1500;
        }
        else if(right_voltage < 100){
            right_voltage -= 1500;
        }
        
        moveChassis(left_voltage,right_voltage);


        if(std::abs(error) < .5){
            withinErrorBounds = true;
        }
        else{
            withinErrorBounds = false;
        }

        if(withinErrorBounds != prev_withinErrorBounds && withinErrorBounds){
            endMovementTime = sylib::millis();
        }

        if(withinErrorBounds && sylib::millis() > endMovementTime + 500){
            endMovement = true;
        }

        prev_withinErrorBounds = withinErrorBounds;
		sylib::delay(10);
	}
	leftDrive.move_velocity(0);
	rightDrive.move_velocity(0);
}

void odomControlLoop(void * param){
	static int ticks = 0;

	int start_left = leftRot.get_position();
    int start_right = rightRot.get_position();
	while(true){
		ticks++;

		current_left = -leftRot.get_position();
		current_right = rightRot.get_position();
		
		theta = (WHEEL_DIAMETER*((current_left-start_left) - (current_right-start_right))/(TRACKING_WIDTH))*(180.0/86450.0);
        if ((ticks) % 12 == 0) {
				master.set_text(0,0,std::to_string((int)flywheel.get_velocity()) + " | " + std::to_string((int)flywheel.get_velocity_target())+ " | " + std::to_string((int)flywheel.get_velocity_error()) + "    ");
		}
		sylib::delay(10);
	}
}

int getFrisbeesInIntake(){
    int sensorDistance = distanceFilter.filter(indexerSensor.get());
    if(sensorDistance > 100){
        return 0;
    }
    else if(sensorDistance > 55){
        return 1;
    }
    else if(sensorDistance > 35){
        return 2;
    }
    else{
        return 3;
    }
}

void shootSingleFrisbee(int speedAfter){
    if(getFrisbeesInIntake() == 0){
        return;
    }
    double startTime = sylib::millis();
    int frisbeesAtStart = getFrisbeesInIntake();
    int lightReading = frisbeeTrackSensor.get_value();
    int startReading = lightReading;
    bool frisbeeInTrack = false;
    intake.move_voltage(-12000);
    trackLighting.set_all(0x444400);
    sylib::delay(250);
    while(!frisbeeInTrack && sylib::millis() < startTime + 750){ // 
        lightReading = frisbeeTrackSensor.get_value();
        if(lightReading <= startReading*0.65){
            frisbeeInTrack = true;
            // flywheel.set_voltage(-12000);
        }  
        sylib::delay(10);

    }
    trackLighting.set_all(0x004444);
    intake.move_velocity(200);
    sylib::delay(150);
    intake.move_velocity(0);
    // flywheel.set_velocity_custom_controller(speedAfter);
}

int getRollerColor(){
    if(rollerSensor.get_proximity() < 200){
        return 0;
    }
	
    double hue = hueFilter.filter(rollerSensor.get_hue());
    if(hue < 260 && hue > 230){
        return 1; // blue
    }
    else if(hue < 30 && hue > 0){
        return 2; // red
    }
    else{
        return 3; // lol it doesnt know
    }
    return 0;
}

void setRollerRed(){
    int rollerStartTime = sylib::millis();
    intake.move_velocity(100);
    while(getRollerColor() != 2 && sylib::millis() - rollerStartTime < 1500){
        sylib::delay(10);
    }
    intake.move_velocity(0);
    sylib::delay(50);
    rollerStartTime = sylib::millis();
    intake.move_velocity(100);
    while(getRollerColor() != 1 && sylib::millis() - rollerStartTime < 1500){
        sylib::delay(10);
    }
    intake.move_velocity(50);
    sylib::delay(250);
    intake.move_velocity(0);
}

void setRollerBlue(){
    int rollerStartTime = sylib::millis();
    intake.move_velocity(100);
    while(getRollerColor() != 1 && sylib::millis() - rollerStartTime < 1500){
        sylib::delay(10);
    }
    intake.move_velocity(0);
    sylib::delay(50);
    rollerStartTime = sylib::millis();
    intake.move_velocity(100);
    while(getRollerColor() != 2 && sylib::millis() - rollerStartTime < 1500){
        sylib::delay(10);
    }
    intake.move_velocity(50);
    sylib::delay(250);
    intake.move_velocity(0);
}

void auton2(){
    flywheel.set_velocity_custom_controller(3450);
    intake.move_voltage(12000);
    driveDistance(40,3000, 125);
    turnToAngle(37.5, 500);
    driveDistance(6, 750);
    sylib::delay(1000);
    int frisbeesAtStart = getFrisbeesInIntake();
    for(int i = 0;i < frisbeesAtStart-1; i++){

        while(std::abs(flywheel.get_velocity_error()) > 60){
            sylib::delay(10);
        }
        flywheel.set_velocity_custom_controller(3450);

        shootSingleFrisbee(3600);
        angler.set_value(false);
        sylib::delay(200);
    }
    intake.move_velocity(-200);
    sylib::delay(1500);
    intake.move_velocity(0);

    turnToAngle(133,1000);
    flywheel.set_velocity_custom_controller(3600);
    if(getFrisbeesInIntake() < 3){
        intake.move_voltage(12000);
    }
    else{
        intake.move_voltage(0);
    }
    flywheel.set_voltage(-12000);
    driveDistance(58, 3000, 150);
    angler.set_value(false);
    turnToAngle(3,800);
    intake.move_velocity(0);
    moveChassis(3550, 3550);
    sylib::delay(500);
    intake.move_velocity(-75);
    sylib::delay(250);
    leftDrive.move_velocity(0);
    rightDrive.move_velocity(0);
    driveDistance(2,500);
    intake.move_velocity(200);
    turnToAngle(10, 500);
    sylib::delay(400);
    intake.move_velocity(0);
    frisbeesAtStart = getFrisbeesInIntake();
    sylib::delay(500);

    for(int i = 0;i < 5; i++){
        sylib::delay(200);
        intake.move_velocity(-200);
        sylib::delay(600);
        intake.move_velocity(200);

    }
    
    // sylib::delay(1000);
    // flywheel.set_velocity_custom_controller(0);

}

void auton1(){
    flywheel.set_velocity_custom_controller(3225);
    moveChassis(2250, 2250);
    sylib::delay(200);
    intake.move_velocity(-75);
    sylib::delay(350);
    intake.move_velocity(100);
    leftDrive.move_velocity(0);
    rightDrive.move_velocity(0);


    driveDistance(3,500);
    intake.move_velocity(0);


    turnToAngle(90,750);
    driveDistance(42,2500);
    turnToAngle(0,750);
	driveDistance(43,2500);
	turnToAngle(-40,500);
    sylib::delay(750);
    while(std::abs(flywheel.get_velocity_error()) > 30){
        printf("%f\n", flywheel.get_velocity());
        sylib::delay(10);
    }
    shootSingleFrisbee(3225);

    sylib::delay(1000);
    while(std::abs(flywheel.get_velocity_error()) > 30){
        printf("%f\n", flywheel.get_velocity());
        sylib::delay(10);
    }
    shootSingleFrisbee(3225);
    flywheel.set_velocity_custom_controller(0);
    turnToAngle(39,500);
    sylib::delay(1000);
    intake.move_voltage(12000);
    driveDistance(80,2500, 100);
    turnToAngle(-90,750);
    intake.move_velocity(0);
    moveChassis(3000, 3000);
    sylib::delay(500);

    intake.move_velocity(-75);
    sylib::delay(350);
    intake.move_velocity(0);
    leftDrive.move_velocity(0);
    rightDrive.move_velocity(0);
    driveDistance(3,500);

}