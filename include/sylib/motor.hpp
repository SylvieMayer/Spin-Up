#pragma once


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
    class Motor : public Device{
        private:
            //SDK things
            const V5_DeviceT device;
            // Config settings
            const std::uint8_t smart_port;
            const double gearing;
            const bool reverse;
            sylib::SylviesPogVelocityEstimator motorVelocityTracker;

            // Updating values
            uint32_t internalClock;
            double velocity;
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
            std::int32_t brake_mode;
            std::int32_t current_limit;
            std::int32_t voltage_limit;


        public:
            Motor(const uint8_t smart_port, const double gearing = 200, const bool reverse = false);
            // Background control stuff
            void update();

            // Motor control
            void set_absolute_position_target(double position, std::int32_t velocity);
            void set_relative_position_target(double position, std::int32_t velocity);
            void set_velocity_target(std::int16_t velocity);
            void stop();
            void set_voltage(std::int16_t voltage);
            void set_braking_mode(std::int32_t mode);
            void set_amps_limit(std::int32_t limit);
            void set_gearing(std::int32_t gearing);
            void set_is_reversed(bool reverse);
            void set_volts_limit(std::int32_t limit);
            void tare_encoder();

            // Motor telemetry
            double get_position();
            double get_position_target();
            double get_velocity();
            double get_velocity_target();
            double get_velocity_motor_reported();
            double get_velocity_raw();
            double get_velocity_sma_filter_only();
            double get_acceleration();
            double get_temperature();
            double get_torque();
            double get_watts();
            std::int32_t get_amps();
            std::int32_t get_amps_limit();
            std::int32_t get_applied_voltage();
            std::int32_t get_volts_limit();
            std::int32_t get_efficiency();
            std::int32_t get_faults();
            std::int32_t get_flags();
            std::uint32_t get_device_timestamp();
            std::int32_t get_position_and_timestamp(std::uint32_t* timestamp);
            std::int32_t get_braking_mode();
            std::int32_t get_gearing();
            std::int32_t get_smart_port();
            bool is_stopped();
            bool is_over_current();
            bool is_over_temp();           
            bool is_reversed();
    };
}