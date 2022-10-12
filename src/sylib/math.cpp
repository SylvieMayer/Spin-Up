#include "sylib/math.hpp"

namespace sylib{
    EMAFilter::EMAFilter(){
        ema = 0;
        inputkA = 1;
    }
    double EMAFilter::filter(double rawValue, double kA){
        inputkA = kA;
        ema = rawValue*inputkA + ema * (1-inputkA);
        return ema;
    }
    double EMAFilter::getkA() const{return inputkA;}
    double EMAFilter::getCurrentEMA() const{return ema;}


    SMAFilter::SMAFilter(int sampleSize): sampleSize(sampleSize){
        meanValue = 0;
        rawVelocityTotal = 0;
        if(sampleSize < 1){
            throw std::invalid_argument("Sample size must be 1 or greater");
        }
    }
    double SMAFilter::filter(double rawValue){
        rawVelocityTotal += rawValue;
        if(rawInputValues.size() >= sampleSize){
            rawVelocityTotal -= rawInputValues.front();
            rawInputValues.pop();
        }
        rawInputValues.push(rawValue);
        meanValue = rawVelocityTotal/rawInputValues.size();
        return meanValue;

    }
    int SMAFilter::getQueueLength() const{return rawInputValues.size();}
    int SMAFilter::getQueueMaxLength() const{return sampleSize;}
    double SMAFilter::getCurrentTotal() const{return rawVelocityTotal;}
    double SMAFilter::getCurrentValue() const{return meanValue;}

    MedianFilter::MedianFilter(int sampleSize, int meanSizeEven, int meanSizeOdd) : sampleSize(sampleSize), meanSizeEven(meanSizeEven), meanSizeOdd(meanSizeOdd){
        queueLength = 0;
        medianValue = 0;
        if(sampleSize < 3)
            throw std::invalid_argument("Sample size must be 3 or greater");
        if(meanSizeEven % 2 != 0)
            throw std::invalid_argument("Even median size cannot be odd");
        else if(meanSizeEven>sampleSize)
            throw std::invalid_argument("Even median size cannot be greater than the sample size");
        else if(meanSizeEven < 2)
            throw std::invalid_argument("Even median size cannot be less than 2");
        if(meanSizeOdd % 2 != 1)
            throw std::invalid_argument("Odd median size cannot be even");
        else if(meanSizeOdd>sampleSize)
            throw std::invalid_argument("Odd median size cannot be greater than the sample size");
        else if(meanSizeOdd < 1)
            throw std::invalid_argument("Odd median size cannot be less than 1");
    }
    double MedianFilter::filter(double rawValue){
        queueLength = rawInputValues.size();
        if(queueLength >= sampleSize){
            rawInputValues.pop_front();
        }
        rawInputValues.push_back(rawValue);
        queueLength = rawInputValues.size();
        double rawInputValuesArray[queueLength];
        for(int i=0;i<queueLength;i++){
            rawInputValuesArray[i] = rawInputValues[i];
        }
        std::sort(rawInputValuesArray, rawInputValuesArray+queueLength);
        medianValue = 0;
        if(queueLength%2 == 0){
            for(int i=0;i<meanSizeEven;i++){
                if(i%2==0){
                    int j = std::floor(queueLength/2)+std::ceil(i/2);
                    medianValue += rawInputValuesArray[j];
                }
                else{
                    int j = std::ceil(queueLength/2)-std::ceil(i/2);
                    medianValue += rawInputValuesArray[j];
                }
            }
            medianValue = medianValue/meanSizeEven;
        }
        else{
            for(int i=0;i<meanSizeOdd;i++){
                if(i%2==0){
                    int j = std::floor(queueLength/2)+std::ceil(i/2);
                    medianValue += rawInputValuesArray[j];
                }
                else{
                    int j = std::ceil(queueLength/2)-std::ceil(i/2);
                    medianValue += rawInputValuesArray[j];
                }
            }
            medianValue = medianValue/meanSizeOdd;
        }
        return medianValue;
    }
    int MedianFilter::getQueueLength() const{return queueLength;}
    int MedianFilter::getQueueMaxLength() const{return sampleSize;}
    int MedianFilter::getEvenCenterSize() const{return meanSizeEven;}
    int MedianFilter::getOddCenterSize() const{return meanSizeOdd;}
    double MedianFilter::getCurrentValue() const{return medianValue;}

    RangeExtremaFilter::RangeExtremaFilter(int sampleSize) : sampleSize(sampleSize){
        queueLength = 0;
        maxValue = 0;
        if(sampleSize < 1){
            throw std::invalid_argument("Sample size must be 1 or greater");
        }
    }
    double RangeExtremaFilter::filter(double rawValue){
        queueLength = rawInputValues.size();
        if(queueLength >= sampleSize){
            rawInputValues.pop_front();
        }
        rawInputValues.push_back(rawValue);
        queueLength = rawInputValues.size();
        double rawInputValuesArray[queueLength];
        for(int i=0;i<queueLength;i++){
            rawInputValuesArray[i] = std::abs(rawInputValues[i]);
        }
        std::sort(rawInputValuesArray, rawInputValuesArray+queueLength, std::greater<double>());
        maxValue = rawInputValuesArray[0];
        return maxValue;
    }
    double RangeExtremaFilter::getCurrentValue() const{return maxValue;}
    int RangeExtremaFilter::getQueueLength() const{return queueLength;}
    int RangeExtremaFilter::getQueueMaxLength() const{return sampleSize;}

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
    double SympleDerivativeSolver::getCurrentDerivative() const{return derivativeFunctionValue;}
    double SympleDerivativeSolver::getCurrentInputValue() const{return currentInputFunctionValue;}

    VoltageEstimation::VoltageEstimation(double kV, double motorGearing) : motorGearing(motorGearing), kV(kV){
        voltageEstimate = 0;
    }
    double VoltageEstimation::estimate(double rpm){
        voltageEstimate = rpm * kV * 3600 / motorGearing;
        return voltageEstimate;
    }
    double VoltageEstimation::getKv() const{return kV;}
    double VoltageEstimation::getMotorGearing() const{return motorGearing;}
    void VoltageEstimation::setkV(double value){kV = value;}

    ProportionalController::ProportionalController(double kP, std::shared_ptr<double> error, double motorGearing): kP(kP), motorGearing(motorGearing), error(error){
        proportional = 0;
    }
    double ProportionalController::output(){
        proportional = *error * kP * motorGearing / 3600;
        return proportional;
    }
    double ProportionalController::getkP() const{return kP;}
    double ProportionalController::getOutput() const{return proportional;}
    void ProportionalController::setkP(double gain){kP = gain;}
    double ProportionalController::operator*(){return output();}

    IntegralController::IntegralController(double kI, std::shared_ptr<double> error, double motorGearing) : kI(kI), motorGearing(motorGearing), error(error){
        integral = 0;
        currentTime = vexSystemTimeGet();
        previousTime = currentTime;
        dT = 0;
    }
    double IntegralController::output(){
        currentTime = vexSystemTimeGet();
        dT = currentTime - previousTime;
        if (dT <= 0) {
            return integral;
        }
        integral += *error * dT * motorGearing / 3600;
        return integral;
    }
    double IntegralController::getOutput() const{return integral;}
    double IntegralController::getkI() const{return kI;}
    double IntegralController::getCurrentTime() const{return currentTime;}
    uint32_t IntegralController::getdT() const{return dT;}
    void IntegralController::setkI(double gain){kI = gain;}
    double IntegralController::operator*(){return output();}

    DerivativeController::DerivativeController(double kD, std::shared_ptr<double> error,  double motorGearing) : kD(kD), motorGearing(motorGearing), error(error){
        derivative = 0;
        currentInput = *error;
        previousInput = currentInput;
        derivative = 0;
        currentTime = vexSystemTimeGet();
        previousTime = currentTime;
        dT = 0;
    }
    double DerivativeController::output(){
        currentTime = vexSystemTimeGet();
        currentInput = *error;
        dT = currentTime - previousTime;
        previousTime = currentTime;
        if(dT <= 0){
            return derivative;
        }
        derivative = (currentInput-previousInput) / dT * motorGearing / 3600;
        previousInput = currentInput;
        return derivative;
    }
    double DerivativeController::getOutput() const{return derivative;}
    double DerivativeController::getCurrentTime() const{return currentTime;}
    double DerivativeController::getkD() const{return kD;}
    uint32_t DerivativeController::getdT() const{return dT;}
    void DerivativeController::setkD(double gain){kD = gain;}
    double DerivativeController::operator*(){return output();}
}