#include "env.hpp"

namespace sylib {
    using kv_fn_t=std::function<double(double)>;
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
            kv_fn_t kV;
            double voltageEstimate;
            double motorGearing;
        public:
            VoltageEstimation(kv_fn_t kV, double motorGearing = 200);
            double estimate(double rpm);
            kv_fn_t getKv() const;
            double getMotorGearing() const;
            void setkV(kv_fn_t value);
    };

    class ProportionalController{
        private:
            std::shared_ptr<double> error;
            double kP;
            double motorGearing;
            double proportional;
            bool maxRangeEnabled;
            double kP2;
            double maxRange;
        public:
            ProportionalController(double kP, std::shared_ptr<double> error, double motorGearing = 200, bool maxRangeEnabled = false, double kP2 = 0, double maxRange = 0);
            double update();
            double getkP() const;
            double getOutput() const;
            void setkP(double gain);
            double operator*();
            void setMaxRangeEnabled(bool enabled);
            void setMaxRange(double range);
            void setkP2(double gain);
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
            bool antiWindupEnabled;
            double antiWindupRange;
        public:
            IntegralController(double kI, std::shared_ptr<double> error, double motorGearing = 200, bool antiWindupEnabled = false, double antiWindupRange = 0);
            double update();
            double getkI() const;
            void resetValue();
            double getOutput() const;
            double getCurrentTime() const;
            uint32_t getdT() const;
            void setkI(double gain);
            double operator*();
            void setAntiWindupEnabled(bool enabled);
            void setAntiWindupRange(double range);
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

    class TakeBackHalfController{
        private:
            std::shared_ptr<double> error;
            double kH;
            double output;
            double tbh;
            double previousError;
            uint32_t currentTime;
            uint32_t previousTime;
            uint32_t dT;
        public:
            TakeBackHalfController(double kH, std::shared_ptr<double> error);
            double update();
            double getOutput() const;
            double getkH() const;
            double getTBH() const;
            void setkH(double gain);
            double getCurrentTime() const;
            double operator*() const;
    };
    
    struct SpeedControllerInfo {
        kv_fn_t kV = [](double rpm){return 0;};
        double kP = 0;
        double kI = 0;
        double kD = 0;
        double kH = 0;

        bool antiWindupEnabled = true;
        double antiWindupRange = 50;
        bool pRangeEnabled = false;
        double pRange = 50;
        double kP2 = 0;

        double maxVoltageRange = 0;

        SpeedControllerInfo(kv_fn_t kV = [](double rpm){return 0;}, double kP = 0, double kI = 0, double kD = 0, double kH = 0, bool antiWindupEnabled = true, double antiWindupRange = 0, bool pRangeEnabled = false, double pRange = 0, double kP2 = 0, double maxVoltageRange = 0) :
                        kV(kV), kP(kP), kI(kI), kD(kD), kH(kH), antiWindupEnabled(antiWindupEnabled), antiWindupRange(antiWindupRange), pRangeEnabled(pRangeEnabled), pRange(pRange), kP2(kP2), maxVoltageRange(maxVoltageRange){}
    };
    
}