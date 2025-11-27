#pragma once

#include <cmath>

#include <algorithm>
#include <atomic>
#include <bit>
#include <cstdint>
#include <numbers>

#include "../utility/endian_promise.hpp"

namespace librmcs::device {

class DmMotor {
public:
    enum class Type : uint8_t { J4310 };
    enum class DmMotorErrorMsg : uint8_t {
        DISABLE = 0,
        ENABLE = 1,
        OVER_VOLTAGE = 8,
        UNDER_VOLTAGE = 0,
        OVER_CURRENT = 11,
        MOS_OVER_TEMPERATURE = 12,
        ROTOR_OVER_TEMPERATURE = 13,
        COMMUNICATION_ERROR = 0xd,
        OVERLOAD = 15
    };

    struct Config {
        explicit Config(Type motor_type) {
            this->encoder_zero_point = 0;
            this->motor_type = motor_type;
            switch (motor_type) {
            case Type::J4310: reduction_ratio = 1.0; break;
            }
            this->reversed = false;
            this->multi_turn_angle_enabled = false;
        }

        Config& set_encoder_zero_point(int value) { return encoder_zero_point = value, *this; }
        Config& set_reduction_ratio(double value) { return reduction_ratio = value, *this; }
        Config& set_reversed() { return reversed = true, *this; }
        Config& enable_multi_turn_angle() { return multi_turn_angle_enabled = true, *this; }

        Type motor_type;
        int encoder_zero_point;
        double reduction_ratio;
        bool reversed;
        bool multi_turn_angle_enabled;
    };

    DmMotor()
        : angle_(0.0)
        , velocity_(0.0)
        , torque_(0.0) {}
    explicit DmMotor(const Config& config)
        : angle_(0.0)
        , velocity_(0.0)
        , torque_(0.0) {
        configure(config);
    }

    DmMotor(const DmMotor&) = delete;
    DmMotor& operator=(const DmMotor&) = delete;

    void configure(const Config& config) {
        encoder_zero_point_ = config.encoder_zero_point % raw_angle_max_;
        if (encoder_zero_point_ < 0)
            encoder_zero_point_ += raw_angle_max_;

        reversed_ = config.reversed;

        last_raw_angle_ = 0;
        multi_turn_angle_enabled_ = config.multi_turn_angle_enabled;
        angle_multi_turn_ = 0;
    }

    void store_status(uint64_t can_data) { can_data_.store(can_data, std::memory_order_relaxed); }

    void update_status() {
        const auto feedback =
            std::bit_cast<DmMotorFeedback>(can_data_.load(std::memory_order::relaxed));

        // uint8_t motor_id = feedback.id_err & 0x0F;
        last_error_msg_ = static_cast<DmMotorErrorMsg>((feedback.id_err >> 4) & 0x0F);

        // Temperature unit: celsius
        temperature_ = static_cast<double>(feedback.temp_mos);
        mos_temperature_ = static_cast<double>(feedback.temp_rotor);

        // Angle unit: rad
        const int raw_angle = (static_cast<uint16_t>(feedback.pos_high) << 8) | feedback.pos_low;
        int calibrated_raw_angle = raw_angle - encoder_zero_point_;
        if (calibrated_raw_angle < 0)
            calibrated_raw_angle += raw_angle_max_;

        if (reversed_)
            calibrated_raw_angle = (raw_angle_max_ - calibrated_raw_angle);

        // TODO: BETTER COEFFICIENT HANDLING FOR DIFFERENT MOTORS
        if (!multi_turn_angle_enabled_) {
            // MOTOR PMAX SET TO 25.1327
            angle_ =
                static_cast<double>(calibrated_raw_angle & 0x1FFF) / 8192.0 * 2 * std::numbers::pi;
        } else {
            auto diff = (calibrated_raw_angle - angle_multi_turn_) % raw_angle_max_;
            if (diff <= -raw_angle_max_ / 2)
                diff += raw_angle_max_;
            else if (diff > raw_angle_max_ / 2)
                diff -= raw_angle_max_;
            angle_multi_turn_ += diff;
            angle_ = static_cast<double>(angle_multi_turn_) / 8192.0 * 2 * std::numbers::pi;
        }
        last_raw_angle_ = raw_angle;

        const uint16_t vel_raw = (static_cast<uint16_t>(feedback.vel_high) << 4)
                               | ((feedback.vel_low_t_high >> 4) & 0x0F);
        const uint16_t torque_raw =
            (static_cast<uint16_t>(feedback.vel_low_t_high & 0x0F) << 8) | feedback.torque_low;

        // Velocity unit: rad/s
        velocity_ = ((static_cast<float>(vel_raw) - 2048.0f) / 4096.0f) * 60.0f;

        // Torque unit: N*m
        torque_ = ((static_cast<float>(torque_raw) - 2048.0f) / 4096.0f) * 20.0f;
    }

    uint64_t generate_command(float control_velocity) const {
        if (std::isnan(control_velocity)) {
            return 0;
        }

        // TODO: BETTER HANDLING OF ERROR STATES
        if (last_error_msg_ == DmMotorErrorMsg::DISABLE) {
            return 0xfcffffffffffffff;
        } else if (last_error_msg_ == DmMotorErrorMsg::COMMUNICATION_ERROR) {
            return 0xfbffffffffffffff;
        }

        control_velocity = std::clamp(control_velocity, -30.0f, 30.0f);
        utility::le_int32_t command =
            std::bit_cast<int32_t>((reversed_ ? -1.0f : 1.0f) * control_velocity);

        return std::bit_cast<uint32_t>(command);
    }

    int calibrate_zero_point() {
        angle_multi_turn_ = 0;
        encoder_zero_point_ = last_raw_angle_;
        return encoder_zero_point_;
    }

    double angle() const { return angle_; }
    double velocity() const { return velocity_; }
    double torque() const { return torque_; }
    double max_torque() const { return max_torque_; } // MAY BE UNUSED
    double temperature() const { return temperature_; }
    double mos_temperature() const { return mos_temperature_; }
    int last_raw_angle() const { return last_raw_angle_; }
    DmMotorErrorMsg last_error_msg() const { return last_error_msg_; }

private:
    struct alignas(uint64_t) DmMotorFeedback {
        uint8_t id_err;                               // D[0]: ID|ERR<<4
        uint8_t pos_high;                             // D[1]: POS[15:8]
        uint8_t pos_low;                              // D[2]: POS[7:0]
        uint8_t vel_high;                             // D[3]: VEL[11:4]
        uint8_t vel_low_t_high;                       // D[4]: VEL[3:0]|T[11:8]
        uint8_t torque_low;                           // D[5]: T[7:0]
        uint8_t temp_mos;                             // D[6]: T_MOS
        uint8_t temp_rotor;                           // D[7]: T_Rotor
    };

    std::atomic<uint64_t> can_data_ = 0;

    static constexpr int raw_angle_max_ = 65536;
    int encoder_zero_point_, last_raw_angle_;

    bool reversed_;
    bool multi_turn_angle_enabled_;
    int64_t angle_multi_turn_;

    double angle_;
    double velocity_;
    double torque_;
    double max_torque_;
    double temperature_;
    double mos_temperature_;
    DmMotorErrorMsg last_error_msg_;
};

} // namespace librmcs::device