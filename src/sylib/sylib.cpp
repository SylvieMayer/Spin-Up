#include "sylib/sylib.hpp"
#include "pros/motors.hpp"
#include <cmath>
#include <iostream>

namespace sylib {
    EMAFilter::EMAFilter(){
        ema = 0;
        inputkA = 1;
    }
    double EMAFilter::filter(double rawValue, double kA){
        inputkA = kA;
        ema = rawValue*inputkA + ema * (1-inputkA);
        return ema;
    }
    double EMAFilter::getkA() const{
        return inputkA;
    }
    double EMAFilter::getCurrentEMA() const{
        return ema;
    }


    SMAFilter::SMAFilter(int sampleSize): sampleSize(sampleSize){
        meanValue = 0;
        rawVelocityTotal = 0;
        if(sampleSize < 1){
            throw std::invalid_argument("Sample size must be 1 or greater");
        }
    }
    double SMAFilter::filter(double rawValue){
        rawVelocityTotal += rawValue;
        if(rawVelocityValues.size() >= sampleSize){
            rawVelocityTotal -= rawVelocityValues.front();
            rawVelocityValues.pop();
        }
        rawVelocityValues.push(rawValue);
        meanValue = rawVelocityTotal/rawVelocityValues.size();
        return meanValue;

    }
    int SMAFilter::getQueueLength() const{
        return rawVelocityValues.size();
    }
    int SMAFilter::getQueueMaxLength() const{
        return sampleSize;
    }
    double SMAFilter::getCurrentTotal() const{
        return rawVelocityTotal;
    }
    double SMAFilter::getCurrentValue() const{
        return meanValue;
    }

    MedianFilter::MedianFilter(int sampleSize, int meanSizeEven, int meanSizeOdd) : sampleSize(sampleSize), meanSizeEven(meanSizeEven), meanSizeOdd(meanSizeOdd){
        queueLength = 0;
        medianValue = 0;
        if(sampleSize < 3){
            throw std::invalid_argument("Sample size must be 3 or greater");
        }
        if(meanSizeEven % 2 != 0){
            throw std::invalid_argument("Even median size cannot be odd");
        }
        else if(meanSizeEven>sampleSize){
            throw std::invalid_argument("Even median size cannot be greater than the sample size");
        }
        else if(meanSizeEven < 2){
            throw std::invalid_argument("Even median size cannot be less than 2");
        }
        if(meanSizeOdd % 2 != 1){
            throw std::invalid_argument("Odd median size cannot be even");
        }
        else if(meanSizeOdd>sampleSize){
            throw std::invalid_argument("Odd median size cannot be greater than the sample size");
        }
        else if(meanSizeOdd < 1){
            throw std::invalid_argument("Odd median size cannot be less than 1");
        }
    }
    double MedianFilter::filter(double rawValue){
        queueLength = rawVelocityValues.size();
        if(queueLength >= sampleSize){
            rawVelocityValues.pop_front();
        }
        rawVelocityValues.push_back(rawValue);
        queueLength = rawVelocityValues.size();
        double rawVelocityValuesArray[queueLength];
        for(int i=0;i<queueLength;i++){
            rawVelocityValuesArray[i] = rawVelocityValues[i];
        }
        std::sort(rawVelocityValuesArray, rawVelocityValuesArray+queueLength);
        medianValue = 0;
        if(queueLength%2 == 0){
            for(int i=0;i<meanSizeEven;i++){
                if(i%2==0){
                    int j = std::floor(queueLength/2)+std::ceil(i/2);
                    medianValue += rawVelocityValuesArray[j];
                }
                else{
                    int j = std::ceil(queueLength/2)-std::ceil(i/2);
                    medianValue += rawVelocityValuesArray[j];
                }
            }
            medianValue = medianValue/meanSizeEven;
        }
        else{
            for(int i=0;i<meanSizeOdd;i++){
                if(i%2==0){
                    int j = std::floor(queueLength/2)+std::ceil(i/2);
                    medianValue += rawVelocityValuesArray[j];
                }
                else{
                    int j = std::ceil(queueLength/2)-std::ceil(i/2);
                    medianValue += rawVelocityValuesArray[j];
                }
            }
            medianValue = medianValue/meanSizeOdd;
        }
        return medianValue;
    }
    int MedianFilter::getQueueLength() const{
        return queueLength;
    }
    int MedianFilter::getQueueMaxLength() const{
        return sampleSize;
    }
    int MedianFilter::getEvenCenterSize() const{
        return meanSizeEven;
    }
    int MedianFilter::getOddCenterSize() const{
        return meanSizeOdd;
    }
    double MedianFilter::getCurrentValue() const{
        return medianValue;
    }
    SympleDerivativeSolver::SympleDerivativeSolver(){
        currentInputFunctionValue = 0;
        previousInputFunctionValue = 0;
        deltaInputFunctionValue = 0;
        derivativeFunctionValue = 0;
        previousTime = vexSystemTimeGet();
    }
    double SympleDerivativeSolver::solveDerivative(double input){
        currentInputFunctionValue = input;
        deltaInputFunctionValue = currentInputFunctionValue - previousInputFunctionValue;
        previousInputFunctionValue = currentInputFunctionValue;
        currentTime = vexSystemTimeGet();
        dT = currentTime - previousTime;
        previousTime = currentTime;
        if(dT <= 0){
            return derivativeFunctionValue;
        }
        derivativeFunctionValue = deltaInputFunctionValue/dT;
        return derivativeFunctionValue;
    }
    double SympleDerivativeSolver::getCurrentDerivative(){
        return derivativeFunctionValue;
    }
    double SympleDerivativeSolver::getCurrentInputValue(){
        return currentInputFunctionValue;
    }

    SylviesPogVelocityEstimator::SylviesPogVelocityEstimator(pros::Motor * smartMotor, double motorGearing) : smartMotor(smartMotor), smaFilterVelocity(3), smaFilterAccel(3), smaFilterJerk(3), smaFilterSnap(3), medianFilter(15,2,3), emaFilter(0.5), motorGearing(motorGearing), preFilterAccelSolver(), preFilterJerkSolver(), preFilterSnapSolver(), outputAccelSolver(), outputJerkSolver(), outputSnapSolver(){
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
    }
    double SylviesPogVelocityEstimator::getVelocity(){
        currentMotorTicks = smartMotor->get_raw_position(&internalMotorClock);
        vexosRawVelocity = smartMotor->get_actual_velocity();
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

        smaFilteredAccel = smaFilterAccel.filter(preFilterAcceleration);
        smaFilteredJerk = smaFilterJerk.filter(preFilterJerk);
        smaFilteredSnap = smaFilterSnap.filter(preFilterSnap);

        accelCalculatedkA = 0.75*(1-(1/((smaFilteredAccel*smaFilteredAccel/250)+1)));

        emaFilteredVelocity = emaFilter.filter(medianFilteredVelocity, accelCalculatedkA);
        outputVelocity = emaFilteredVelocity*motorGearing/3600;
        outputAcceleration = outputAccelSolver.solveDerivative(outputVelocity);
        outputJerk = outputJerkSolver.solveDerivative(outputAcceleration);
        outputSnap = outputSnapSolver.solveDerivative(outputJerk);
        return outputVelocity;
    }
    double SylviesPogVelocityEstimator::getVelocityNoCalculations(){
        return outputVelocity;
    }
    double SylviesPogVelocityEstimator::getRawVelocity(){
        return rawVelocity;
    }
    double SylviesPogVelocityEstimator::getVexosWrongVelocity(){
        return vexosRawVelocity;
    }
    double SylviesPogVelocityEstimator::getEmaFilteredVelocity(){
        return emaFilteredVelocity;
    }
    double SylviesPogVelocityEstimator::getSmaFilteredVelocity(){
        return smaFilteredVelocity;
    }
    double SylviesPogVelocityEstimator::getMedianFilteredVelocity(){
        return medianFilteredVelocity;
    }
    double SylviesPogVelocityEstimator::getRawPosition(){
        return currentMotorTicks;
    }
    double SylviesPogVelocityEstimator::getAcceleration(){
        return outputAcceleration;
    }
    double SylviesPogVelocityEstimator::getJerk(){
        return outputJerk;
    }
    double SylviesPogVelocityEstimator::getSnap(){
        return outputSnap;
    }
    double SylviesPogVelocityEstimator::getCalculatedkA(){
        return accelCalculatedkA;
    }
}