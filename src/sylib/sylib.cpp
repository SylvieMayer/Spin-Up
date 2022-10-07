#include "sylib/sylib.hpp"
#include "pros/motors.hpp"
#include <iostream>

namespace sylib {
    EMAFilter::EMAFilter(double kA) : kA(kA){
        ema = 0;
        if(kA < 0 || kA > 1){
            throw std::invalid_argument("kA must be between 0 and 1");
        }
    }
    double EMAFilter::filter(double rawValue){
        ema = rawValue*kA + ema * (1-kA);
        return ema;
    }
    double EMAFilter::getkA() const{
        return kA;
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

    SylviesPogVelocityEstimator::SylviesPogVelocityEstimator(pros::Motor * smartMotor, double motorGearing) : smartMotor(smartMotor), smaFilter(3), medianFilter(15,2,3), emaFilter(0.5), motorGearing(motorGearing){
        internalMotorClock = 0;
        previousInternalMotorClock = 0;
        currentMotorTicks = 0;
        oldMotorTicks = 0;
        dN = 0;
        dT = 0;
        rawVelocity = 0;
        speedTarget = 0;
        motorVoltageTarget = 0;
        emaFilteredVelocity = 0;
        smaFilteredVelocity = 0;
        medianFilteredVelocity = 0;
        outputVelocity = 0;
    }
    double SylviesPogVelocityEstimator::getVelocity(){
        currentMotorTicks = smartMotor->get_raw_position(&internalMotorClock);
        dT = 5*std::round((internalMotorClock-previousInternalMotorClock)/5.0);
        printf("%d|%f\n", internalMotorClock-previousInternalMotorClock, dT);
        if(dT == 0){
            return outputVelocity;
        }
        dN = currentMotorTicks-oldMotorTicks;
        previousInternalMotorClock=internalMotorClock;
        oldMotorTicks = currentMotorTicks;
        rawVelocity = (dN/50)/dT * 60000;
        smaFilteredVelocity = smaFilter.filter(rawVelocity);
        medianFilteredVelocity = medianFilter.filter(smaFilteredVelocity);
        emaFilteredVelocity = emaFilter.filter(medianFilteredVelocity);
        outputVelocity = emaFilteredVelocity*motorGearing/3600;
        return outputVelocity;
    }
    double SylviesPogVelocityEstimator::getVelocityNoCalculations(){
        return outputVelocity;
    }
    double SylviesPogVelocityEstimator::getRawVelocity(){
        return rawVelocity;
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
}