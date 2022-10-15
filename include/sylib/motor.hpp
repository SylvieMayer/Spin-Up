#pragma once

#include "sylib_apitypes.hpp"
#include "sylib.hpp"
#include "system.hpp"
#include "env.hpp"
#include <cstdint>
#include <stdint.h>


namespace sylib {
    class SylviesPogVelocityEstimator{
        private:
            uint32_t previousInternalMotorClock;
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
            double maxFilteredAccel;
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
            sylib::RangeExtremaFilter maxFilterAccel;
        public:
            SylviesPogVelocityEstimator(double motorGearing = 200);
            double getVelocity(double currentMotorTicks, std::uint32_t currentInternalMotorClock);
            double getVelocityNoCalculations() const;
            double getRawVelocity() const;
            double getRawPosition() const;
            double getSmaFilteredVelocity() const;
            double getEmaFilteredVelocity() const;
            double getMedianFilteredVelocity() const;
            double getAcceleration() const;
            double getPreFilterAcceleration() const;
            double getJerk() const;
            double getSnap() const;
            double getCalculatedkA() const;
            double getMaxFilteredAcceleration() const;
    };
    class Motor : private Device{
        private:
            //SDK things
            const V5_DeviceT device;
            void set_motor_controller();
            // Config settings
            const std::uint8_t smart_port;
            const double gearing;
            bool reversed;
            sylib::SylviesPogVelocityEstimator motorVelocityTracker;
            sylib::VoltageEstimation motorVoltageEstimator;
            sylib::ProportionalController motorPController;
            sylib::IntegralController motorIController;
            sylib::DerivativeController motorDController;
            sylib::TakeBackHalfController motorTBHController;
            sylib::SpeedControllerInfo speedController; 

            // Updating values
            uint32_t internalClock;
            double velocity;
            double velocity_error;
            double raw_velocity;
            double vexos_velocity;
            double sma_velocity;
            double acceleration;
            double position;
            double temperature;
            double power;
            double target_position;
            double target_velocity;
            std::int32_t current_draw;
            std::int32_t direction;
            std::int32_t efficiency;
            std::int32_t faults;
            std::int32_t flags;
            double torque;
            std::int32_t voltage;
            std::int32_t zero_position_flag;
            std::int32_t stopped;
            std::int32_t over_temp;
            std::int32_t over_current;
            V5MotorBrakeMode brake_mode;
            std::int32_t current_limit;
            std::int32_t voltage_limit;

            //Target values
            SylibMotorControlMode sylib_control_mode;
            double position_target;
            double velocity_target;
            std::int32_t voltage_target;


        public:
            Motor(const uint8_t smart_port, const double gearing = 200, const bool reverse = false, const SpeedControllerInfo speedController = SpeedControllerInfo());
            // Background control stuff
            void update() override;

            // Motor control
            void set_position_target_absolute(double new_position, std::int32_t velocity_cap = 200);
            void set_position_target_relative(double new_position, std::int32_t velocity_cap = 200);
            void set_velocity_custom_controller(std::int16_t new_velocity_target);
            void stop();
            void set_voltage(std::int16_t new_voltage_target);
            void set_braking_mode(V5MotorBrakeMode mode);
            void set_amps_limit(std::int32_t limit);
            void set_is_reversed(bool reverse);
            void set_volts_limit(std::int32_t limit);
            void tare_encoder();
            void set_velocity(std::int32_t new_velocity_target);

            void updateSpeedController(sylib::SpeedControllerInfo controller);

            // Motor telemetry
            double get_velocity_error() const;
            std::int32_t get_p_voltage() const;
            std::int32_t get_i_voltage() const;
            std::int32_t get_d_voltage() const;
            double get_position() const;
            double get_position_target() const;
            double get_velocity() const;
            double get_velocity_target() const;
            double get_velocity_motor_reported() const;
            double get_velocity_raw() const;
            double get_velocity_sma_filter_only() const;
            double get_acceleration() const;
            double get_temperature() const;
            double get_torque() const;
            double get_watts() const;
            std::int32_t get_amps() const;
            std::int32_t get_amps_limit() const;
            std::int32_t get_applied_voltage() const;
            std::int32_t get_volts_limit() const;
            std::int32_t get_efficiency() const;
            std::int32_t get_faults() const;
            std::int32_t get_flags() const;
            std::uint32_t get_device_timestamp() const;
            std::int32_t get_position_and_timestamp(std::uint32_t* timestamp);
            std::int32_t get_braking_mode() const;
            std::int32_t get_gearing() const;
            std::int32_t get_smart_port() const;
            bool is_stopped() const;
            bool is_over_current() const;
            bool is_over_temp() const;
            bool is_reversed() const;
    };
}