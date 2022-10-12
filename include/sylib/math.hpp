#include "env.hpp"

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
            std::queue<double> rawInputValues;
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
            std::deque<double> rawInputValues;
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

    class RangeExtremaFilter{
        private:
            std::deque<double> rawInputValues;
            double maxValue;
            int queueLength;
            int sampleSize;
        public:
            RangeExtremaFilter(int sampleSize);
            double filter(double rawValue);
            int getQueueLength() const;
            int getQueueMaxLength() const;
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
            double getCurrentDerivative() const;
            double getCurrentInputValue() const;
    };
        class VoltageEstimation{
        private:
            double kV;
            double voltageEstimate;
            double motorGearing;
        public:
            VoltageEstimation(double kV, double motorGearing = 200);
            double estimate(double rpm);
            double getKv() const;
            double getMotorGearing() const;
            void setkV(double value);
    };

    class ProportionalController{
        private:
            std::shared_ptr<double> error;
            double kP;
            double motorGearing;
            double proportional;
        public:
            ProportionalController(double kP, std::shared_ptr<double> error, double motorGearing = 200);
            double update();
            double getkP() const;
            double getOutput() const;
            void setkP(double gain);
            double operator*();
    };

    class IntegralController{
        private:
            std::shared_ptr<double> error;
            double kI;
            double integral;
            double motorGearing;
            uint32_t currentTime;
            uint32_t previousTime;
            uint32_t dT;
        public:
            IntegralController(double kI, std::shared_ptr<double> error, double motorGearing = 200);
            double update();
            double getkI() const;
            double getOutput() const;
            double getCurrentTime() const;
            uint32_t getdT() const;
            void setkI(double gain);
            double operator*();
    };

    class DerivativeController{
        private:
            std::shared_ptr<double> error;
            double kD;
            double currentInput;
            double previousInput;
            double derivative;
            double motorGearing;
            uint32_t currentTime;
            uint32_t previousTime;
            uint32_t dT;
        public:
            DerivativeController(double kD, std::shared_ptr<double> error, double motorGearing = 200);
            double update();
            double getkD() const;
            double getOutput() const;
            double getCurrentTime() const;
            uint32_t getdT() const;
            void setkD(double gain);
            double operator*();
    };
}