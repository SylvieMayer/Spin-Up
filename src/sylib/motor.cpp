#include "sylib/motor.hpp"

namespace sylib {
    SylviesPogVelocityEstimator::SylviesPogVelocityEstimator(double motorGearing) : smaFilterVelocity(3), smaFilterAccel(15), smaFilterJerk(3), smaFilterSnap(3), medianFilter(7,2,1), emaFilter(), motorGearing(motorGearing), preFilterAccelSolver(), preFilterJerkSolver(), preFilterSnapSolver(), outputAccelSolver(), outputJerkSolver(), outputSnapSolver(),maxFilterAccel(20){
        internalMotorClock = 0;
        previousInternalMotorClock = 0;
        currentMotorTicks = 0;
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
        vexosRawVelocity = 0;
        maxFilteredAccel = 0;
    }
    double SylviesPogVelocityEstimator::getVelocity(){
        // currentMotorTicks = smartMotor->get_raw_position(&internalMotorClock);
        // vexosRawVelocity = smartMotor->get_actual_velocity();
        dT = 5*std::round((internalMotorClock-previousInternalMotorClock)/5.0);
        if(dT == 0){
            return outputVelocity;
        }
        dN = currentMotorTicks-oldMotorTicks;
        previousInternalMotorClock=internalMotorClock;
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
    double SylviesPogVelocityEstimator::getVexosWrongVelocity() const{return vexosRawVelocity;}
    double SylviesPogVelocityEstimator::getEmaFilteredVelocity() const{return emaFilteredVelocity;}
    double SylviesPogVelocityEstimator::getSmaFilteredVelocity() const{return smaFilteredVelocity;}
    double SylviesPogVelocityEstimator::getMedianFilteredVelocity() const{return medianFilteredVelocity;}
    double SylviesPogVelocityEstimator::getRawPosition() const{return currentMotorTicks;}
    double SylviesPogVelocityEstimator::getAcceleration() const{return outputAcceleration;}
    double SylviesPogVelocityEstimator::getPreFilterAcceleration() const{return preFilterAcceleration;}
    double SylviesPogVelocityEstimator::getJerk() const{return outputJerk;}
    double SylviesPogVelocityEstimator::getSnap() const{return outputSnap;}
    double SylviesPogVelocityEstimator::getCalculatedkA() const{return accelCalculatedkA;}
    double SylviesPogVelocityEstimator::getMaxFilteredAcceleration() const{return maxFilteredAccel;}
}