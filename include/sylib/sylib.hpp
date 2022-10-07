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
            double kA;
            double ema;
        public:
            EMAFilter(double kA);
            double filter(double rawValue);
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
            double speedTarget;
            double motorVoltageTarget;
            double smaFilteredVelocity;
            double medianFilteredVelocity;
            double emaFilteredVelocity;
            double motorGearing;
            double outputVelocity;
            sylib::SMAFilter smaFilter;
            sylib::EMAFilter emaFilter;
            sylib::MedianFilter medianFilter;
        public:
            SylviesPogVelocityEstimator(pros::Motor * smartMotor, double motorGearing = 200);
            double getVelocity();
            double getVelocityNoCalculations();
            double getRawVelocity();
            double getRawPosition();
            double getSmaFilteredVelocity();
            double getEmaFilteredVelocity();
            double getMedianFilteredVelocity();
    };
}