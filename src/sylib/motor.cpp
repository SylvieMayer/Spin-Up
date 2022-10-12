#include "sylib/motor.hpp"
#include "sylib/env.hpp"
#include <cstdint>
#include <stdint.h>

namespace sylib {
    SylviesPogVelocityEstimator::SylviesPogVelocityEstimator(double motorGearing) : smaFilterVelocity(3), smaFilterAccel(15), smaFilterJerk(3), smaFilterSnap(3), medianFilter(7,2,1), emaFilter(), motorGearing(motorGearing), preFilterAccelSolver(), preFilterJerkSolver(), preFilterSnapSolver(), outputAccelSolver(), outputJerkSolver(), outputSnapSolver(),maxFilterAccel(20){
        previousInternalMotorClock = 0;
        oldMotorTicks = 0;
        dN = 0;
        dT = 0;
        rawVelocity = 0;
        motorVoltageTarget = 0;
        emaFilteredVelocity = 0;
        smaFilteredVelocity = 0;
        smaFilteredAccel = 0;
        medianFilteredVelocity = 0;
        outputVelocity = 0;
        preFilterAcceleration = 0;
        preFilterJerk = 0;
        preFilterSnap = 0;
        outputAcceleration = 0;
        outputJerk = 0;
        outputSnap = 0;
        accelCalculatedkA = 0;
        maxFilteredAccel = 0;
    }
    double SylviesPogVelocityEstimator::getVelocity(double currentMotorTicks, std::uint32_t currentInternalMotorClock){
        dT = 5*std::round((currentInternalMotorClock-previousInternalMotorClock)/5.0);
        if(dT == 0){
            return outputVelocity;
        }
        dN = currentMotorTicks-oldMotorTicks;
        previousInternalMotorClock=currentInternalMotorClock;
        oldMotorTicks = currentMotorTicks;
        rawVelocity = (dN/50)/dT * 60000;
        smaFilteredVelocity = smaFilterVelocity.filter(rawVelocity);
        medianFilteredVelocity = medianFilter.filter(smaFilteredVelocity);

        preFilterAcceleration = preFilterAccelSolver.solveDerivative(medianFilteredVelocity);
        preFilterJerk = preFilterJerkSolver.solveDerivative(preFilterAcceleration);
        preFilterSnap = preFilterSnapSolver.solveDerivative(preFilterJerk);

        maxFilteredAccel = maxFilterAccel.filter(preFilterAcceleration);
        smaFilteredAccel = smaFilterAccel.filter(preFilterAcceleration);
        smaFilteredJerk = smaFilterJerk.filter(preFilterJerk);
        smaFilteredSnap = smaFilterSnap.filter(preFilterSnap);

        accelCalculatedkA = 0.75*(1-(1/((maxFilteredAccel*maxFilteredAccel/50)+1.013)));

        emaFilteredVelocity = emaFilter.filter(smaFilteredVelocity, accelCalculatedkA);
        outputVelocity = emaFilteredVelocity*motorGearing/3600;
        outputAcceleration = outputAccelSolver.solveDerivative(outputVelocity);
        outputJerk = outputJerkSolver.solveDerivative(outputAcceleration);
        outputSnap = outputSnapSolver.solveDerivative(outputJerk);
        return outputVelocity;
    }
    double SylviesPogVelocityEstimator::getVelocityNoCalculations() const{return outputVelocity;}
    double SylviesPogVelocityEstimator::getRawVelocity() const{return rawVelocity;}
    double SylviesPogVelocityEstimator::getEmaFilteredVelocity() const{return emaFilteredVelocity;}
    double SylviesPogVelocityEstimator::getSmaFilteredVelocity() const{return smaFilteredVelocity;}
    double SylviesPogVelocityEstimator::getMedianFilteredVelocity() const{return medianFilteredVelocity;}
    double SylviesPogVelocityEstimator::getAcceleration() const{return outputAcceleration;}
    double SylviesPogVelocityEstimator::getPreFilterAcceleration() const{return preFilterAcceleration;}
    double SylviesPogVelocityEstimator::getJerk() const{return outputJerk;}
    double SylviesPogVelocityEstimator::getSnap() const{return outputSnap;}
    double SylviesPogVelocityEstimator::getCalculatedkA() const{return accelCalculatedkA;}
    double SylviesPogVelocityEstimator::getMaxFilteredAcceleration() const{return maxFilteredAccel;}




    Motor::Motor(const uint8_t smart_port, const double gearing, const bool reverse) : 
        Device(5,0), smart_port(smart_port), gearing(gearing), reverse(reverse), device(vexDeviceGetByIndex(smart_port-1)){
        motorVelocityTracker = sylib::SylviesPogVelocityEstimator(gearing);
        
    }

    void Motor::update(){
        printf("%dms - sylib subtask %d updated\n", sylib::millis(), getSubTaskID());
        mutex_lock _lock(sylib_port_mutexes[smart_port-1]);
        position = vexDeviceMotorPositionRawGet(device, &internalClock);
        velocity = motorVelocityTracker.getVelocity(position, internalClock);
        vexos_velocity = vexDeviceMotorActualVelocityGet(device);
        sma_velocity = motorVelocityTracker.getSmaFilteredVelocity();
        acceleration = motorVelocityTracker.getAcceleration();
        temperature = vexDeviceMotorTemperatureGet(device);
        power = vexDeviceMotorPowerGet(device);
        current_draw = vexDeviceMotorCurrentGet(device);
        direction = vexDeviceMotorDirectionGet(device);
        efficiency = vexDeviceMotorEfficiencyGet(device);
        faults = vexDeviceMotorFaultsGet(device);
        flags = vexDeviceMotorFlagsGet(device);
        torque = vexDeviceMotorTorqueGet(device);
        voltage = vexDeviceMotorVoltageGet(device);
        zero_position_flag = vexDeviceMotorZeroPositionFlagGet(device);
        over_temp = vexDeviceMotorOverTempFlagGet(device);
        over_current = vexDeviceMotorCurrentLimitFlagGet(device);
        brake_mode = vexDeviceMotorBrakeModeGet(device);
        current_limit = vexDeviceMotorCurrentLimitGet(device);
        voltage_limit = vexDeviceMotorVoltageLimitGet(device);
    }

    double Motor::get_actual_velocity(){return velocity;}
    double Motor::get_vexos_velocity(){return vexos_velocity*gearing/200;}
    double Motor::get_sma_velocity(){return sma_velocity;}
    double Motor::get_acceleration(){return acceleration;}

    std::int32_t Motor::move_voltage(std::int16_t voltage){
        mutex_lock _lock(sylib_port_mutexes[smart_port-1]);
        vexDeviceMotorVoltageSet(device, voltage);
        return 1;
    }
}