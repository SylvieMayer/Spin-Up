#pragma once
#include <array>
#include <cmath>
#include <cstddef>
#include <iostream>
#include <stdexcept>
#include <string>
#include <queue>
#include <algorithm>
#include "api.h"

extern "C" int32_t vexDeviceGetTimestampByIndex(int32_t index);
extern "C" uint32_t vexSystemTimeGet(void);
extern "C" uint64_t vexSystemHighResTimeGet(void);
extern "C" void vPortExitCritical();
extern "C" void vPortEnterCritical();

namespace sylib {
    class EMAFilter {
        private:
            double inputkA;
            double ema;
        public:
            EMAFilter();
            double filter(double rawValue, double kA);
            double getkA() const;
            double getCurrentEMA() const;
    };

    class SMAFilter {
        private:
            std::queue<double> rawVelocityValues;
            int sampleSize;
            double meanValue;
            double rawVelocityTotal;
        public:
            SMAFilter(int sampleSize);
            double filter(double rawValue);
            int getQueueLength() const;
            int getQueueMaxLength() const;
            double getCurrentTotal() const;
            double getCurrentValue() const;
    };

    class MedianFilter {
        private:
            std::deque<double> rawVelocityValues;
            int sampleSize;
            int queueLength;
            double medianValue;
            int meanSizeEven;
            int meanSizeOdd;
        public:
            MedianFilter(int sampleSize, int meanSizeEven, int meanSizeOdd);
            double filter(double rawValue);
            int getQueueLength() const;
            int getQueueMaxLength() const;
            int getEvenCenterSize() const;
            int getOddCenterSize() const;
            double getCurrentValue() const;
    };

    class SympleDerivativeSolver{
        private:
            double currentInputFunctionValue;
            double previousInputFunctionValue;
            double deltaInputFunctionValue;
            uint32_t currentTime;
            uint32_t previousTime;
            uint32_t dT;
            double derivativeFunctionValue;
        public:
            SympleDerivativeSolver();
            double solveDerivative(double input);
            double getCurrentDerivative();
            double getCurrentInputValue();
    };

    class SylviesPogVelocityEstimator{
        private:
            pros::Motor * smartMotor;
            uint32_t internalMotorClock;
            uint32_t previousInternalMotorClock;
            double currentMotorTicks;
            double oldMotorTicks;
            double dN;
            double dT;
            double rawVelocity;
            double motorVoltageTarget;
            double smaFilteredVelocity;
            double medianFilteredVelocity;
            double emaFilteredVelocity;
            double smaFilteredAccel;
            double smaFilteredJerk;
            double smaFilteredSnap;
            double motorGearing;
            double outputVelocity;
            double preFilterAcceleration;
            double preFilterJerk;
            double preFilterSnap;
            double outputAcceleration;
            double outputJerk;
            double outputSnap;
            double accelCalculatedkA;
            double vexosRawVelocity;
            sylib::SMAFilter smaFilterVelocity;
            sylib::SMAFilter smaFilterAccel;
            sylib::SMAFilter smaFilterJerk;
            sylib::SMAFilter smaFilterSnap;
            sylib::EMAFilter emaFilter;
            sylib::MedianFilter medianFilter;
            sylib::SympleDerivativeSolver preFilterAccelSolver;
            sylib::SympleDerivativeSolver preFilterJerkSolver;
            sylib::SympleDerivativeSolver preFilterSnapSolver;
            sylib::SympleDerivativeSolver outputAccelSolver;
            sylib::SympleDerivativeSolver outputJerkSolver;
            sylib::SympleDerivativeSolver outputSnapSolver;
        public:
            SylviesPogVelocityEstimator(pros::Motor * smartMotor, double motorGearing = 200);
            double getVelocity();
            double getVelocityNoCalculations();
            double getRawVelocity();
            double getVexosWrongVelocity();
            double getRawPosition();
            double getSmaFilteredVelocity();
            double getEmaFilteredVelocity();
            double getMedianFilteredVelocity();
            double getAcceleration();
            double getJerk();
            double getSnap();
            double getCalculatedkA();
    };
}