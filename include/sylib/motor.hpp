#pragma once


#include "sylib.hpp"


namespace sylib {
    class SylviesPogVelocityEstimator{
        private:
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
            double getVelocity();
            double getVelocityNoCalculations() const;
            double getRawVelocity() const;
            double getVexosWrongVelocity() const;
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
    class Motor{
        private:
        public:
            // Movement control stuffs
            std::int32_t move(const std::int32_t voltage);
            std::int32_t move_absolute(double position, std::int32_t velocity);
            std::int32_t move_relative(double position, std::int32_t velocity);
            std::int32_t move_velocity(std::int16_t velocity);
            std::int32_t brake();
            std::int32_t move_voltage(std::int16_t voltage);
            std::int32_t modify_profiled_velocity(std::int32_t velocity);

            // Telemetry stuffs
            double get_target_position();
            double get_target_velocity();
            double get_actual_velocity();
            std::int32_t get_current_draw();
            std::int32_t get_direction();
            std::int32_t get_efficiency();
            std::int32_t get_faults();
            std::int32_t get_flags();
            double get_position();
            double get_power();
            std::int32_t get_raw_position(std::uint32_t* timestamp);
            double get_temperature();
            double get_torque();
            std::int32_t get_voltage();
            std::int32_t get_zero_position_flag();
            std::int32_t is_stopped();
            std::int32_t is_over_current();
            std::int32_t is_over_temp();

            // Config things
            std::int32_t get_brake_mode();
            std::int32_t get_current_limit();
            std::int32_t get_encoder_units();
            std::int32_t get_gearing();
            std::int32_t get_port();
            std::int32_t get_voltage_limit();
            std::int32_t is_reversed();
            std::int32_t set_brake_mode(std::int32_t mode);
            std::int32_t set_current_limit(std::int32_t limit);
            std::int32_t set_encoder_units(std::int32_t units);
            std::int32_t set_gearing(std::int32_t gearing);
            std::int32_t set_reversed(bool reverse);
            std::int32_t set_voltage_limit(std::int32_t limit);
            std::int32_t set_zero_position(double position);
            std::int32_t tare_position();

    };
}