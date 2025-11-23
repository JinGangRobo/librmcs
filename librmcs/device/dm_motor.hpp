#pragma once

#include <cmath>
#include <cstring>

#include <algorithm>
#include <atomic>
#include <bit>
#include <numbers>
#include <limits>

#include "../utility/cross_os.hpp"
/*
此处设备驱动只是计算控制数据，不包括CAN ID的计算，并且未通过测试，并且还是ai生成版本。。。。。。
同目录有个csv文件记录电机完整工作的数据格式，作为参考
目前电机canid01，masterid00,速度模式
*/
namespace librmcs::device {

class DmMotor {
public:
    // 支持的达妙电机型号
    enum class Type : uint8_t { DM_J4310 };

    struct Config {
        explicit Config(Type type) {
            motor_type = type;
            encoder_zero_point = 0.0;
            reversed = false;
            
            // 默认线性映射范围 (需与达妙上位机设置保持一致)
            p_max = 12.5; 
            v_max = 30.0;
            t_max = 10.0;
        }

        Config& set_encoder_zero_point(double value) { return encoder_zero_point = value, *this; }
        Config& set_reversed() { return reversed = true, *this; }
        
        // 达妙电机特有：设置 P/V/T 的映射最大值
        Config& set_limits(double p, double v, double t) {
            p_max = p; v_max = v; t_max = t; return *this;
        }

        Type motor_type;
        double encoder_zero_point;
        bool reversed;
        
        double p_max;
        double v_max;
        double t_max;
    };

    DmMotor() = default;

    explicit DmMotor(const Config& config) : DmMotor() {
        configure(config);
    }

    void configure(const Config& config) {
        multi_turn_encoder_count_ = 0;
        last_raw_pos_ = 0;

        // 根据电机类型设置参考参数
        switch (config.motor_type) {
        case Type::DM_J4310:
            max_torque_ = 10.0; 
            break;
        }

        // 应用配置
        p_max_ = config.p_max;
        v_max_ = config.v_max;
        t_max_ = config.t_max;
        encoder_zero_point_ = config.encoder_zero_point;
        reversed_ = config.reversed;

        // 达妙协议映射公式: float = (uint * span / (2^b - 1)) + min
        // span = 2 * max, min = -max
        
        double sign = reversed_ ? -1.0 : 1.0;

        // Position (16-bit): 0~65535 -> -p_max ~ p_max
        status_pos_coefficient_ = sign * (2.0 * p_max_) / 65535.0;
        
        // Velocity (12-bit): 0~4095 -> -v_max ~ v_max
        status_vel_coefficient_ = sign * (2.0 * v_max_) / 4095.0;

        // Torque (12-bit): 0~4095 -> -t_max ~ t_max
        status_torque_coefficient_ = sign * (2.0 * t_max_) / 4095.0;
    }

    void store_status(uint64_t can_data) {
        can_data_.store(can_data, std::memory_order::relaxed);
    }

    void update_status() {
        // 解析反馈帧: ID(8)|ERR(4)|POS(16)|VEL(12)|TORQUE(12)|T_MOS(8)|T_ROTOR(8)
        uint64_t data = can_data_.load(std::memory_order::relaxed);
        const uint8_t* bytes = reinterpret_cast<const uint8_t*>(&data);

        // 提取原始整数值
        uint16_t raw_pos = (static_cast<uint16_t>(bytes[1]) << 8) | bytes[2];
        uint16_t raw_vel = (static_cast<uint16_t>(bytes[3]) << 4) | ((bytes[4] >> 4) & 0x0F);
        uint16_t raw_torque = (static_cast<uint16_t>(bytes[4] & 0x0F) << 8) | bytes[5];
        int8_t t_mos = static_cast<int8_t>(bytes[6]);

        // 1. 位置计算
        // 计算原始值的差分 (利用 uint16 溢出特性处理过零)
        int16_t diff = static_cast<int16_t>(raw_pos - last_raw_pos_);
        multi_turn_encoder_count_ += diff;
        last_raw_pos_ = raw_pos;

        // 转换为物理角度 (rad)
        // 原始映射: 0 -> -p_max, 65535 -> p_max
        // 物理值 = raw * coeff + offset
        // 若 reversed=false: offset = -p_max
        // 若 reversed=true:  offset = p_max (因为 coeff 变负，翻转后 -(-p_max) = p_max)
        double pos_offset = reversed_ ? p_max_ : -p_max_;
        angle_ = (static_cast<double>(multi_turn_encoder_count_) * status_pos_coefficient_) + pos_offset - encoder_zero_point_;

        // 2. Velocity
        double vel_offset = reversed_ ? v_max_ : -v_max_;
        velocity_ = (static_cast<double>(raw_vel) * status_vel_coefficient_) + vel_offset;

        // 3. Torque
        double torque_offset = reversed_ ? t_max_ : -t_max_;
        torque_ = (static_cast<double>(raw_torque) * status_torque_coefficient_) + torque_offset;

        temperature_ = static_cast<double>(t_mos);
    }

    int64_t calibrate_zero_point() {
        // 将当前角度设为零点
        // new_angle = current_angle_raw - new_zero = 0
        // => new_zero = current_angle_raw
        // current_angle_raw = angle_ + encoder_zero_point_
        encoder_zero_point_ += angle_;
        angle_ = 0;
        return static_cast<int64_t>(encoder_zero_point_ * 1000);
    }

    double angle() const { return angle_; }
    double velocity() const { return velocity_; }
    double torque() const { return torque_; }
    double max_torque() const { return max_torque_; }
    double temperature() const { return temperature_; }

    // --- Control Commands ---
    // 注意：以下函数仅生成数据负载 (Payload)。
    // 上层代码需根据模式将数据发送到正确的 CAN ID：
    // MIT 模式 (Torque) -> CAN ID
    // 速度模式 (Velocity) -> CAN ID + 0x200
    // 位置模式 (Angle)    -> CAN ID + 0x100

    constexpr static uint64_t generate_shutdown_command() {
        PACKED_STRUCT({ uint8_t d[8]; }) cmd{.d = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD}};
        return std::bit_cast<uint64_t>(cmd);
    }

    constexpr static uint64_t generate_startup_command() {
        PACKED_STRUCT({ uint8_t d[8]; }) cmd{.d = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC}};
        return std::bit_cast<uint64_t>(cmd);
    }
    
    constexpr static uint64_t generate_clear_error_command() {
        PACKED_STRUCT({ uint8_t d[8]; }) cmd{.d = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFB}};
        return std::bit_cast<uint64_t>(cmd);
    }

    uint64_t generate_disable_command() const {
        return generate_torque_command(0.0);
    }

    uint64_t generate_status_request() const {
        return generate_disable_command();
    }

    /// @brief 生成力矩控制命令 (MIT模式)
    /// @warning 发送至: CAN ID
    uint64_t generate_torque_command(double control_torque) const {
        if (std::isnan(control_torque)) return generate_disable_command();

        double sign = reversed_ ? -1.0 : 1.0;
        float t_des = static_cast<float>(control_torque * sign);

        // MIT 协议: P=0, V=0, Kp=0, Kd=0, T=t_des
        uint16_t p = float_to_uint(0.0f, -p_max_, p_max_, 16);
        uint16_t v = float_to_uint(0.0f, -v_max_, v_max_, 12);
        uint16_t kp = 0;
        uint16_t kd = 0;
        uint16_t t = float_to_uint(t_des, -t_max_, t_max_, 12);

        PACKED_STRUCT({
            uint8_t d0, d1, d2, d3, d4, d5, d6, d7;
        }) cmd;

        cmd.d0 = (p >> 8);
        cmd.d1 = (p & 0xFF);
        cmd.d2 = (v >> 4);
        cmd.d3 = ((v & 0xF) << 4) | (kp >> 8);
        cmd.d4 = (kp & 0xFF);
        cmd.d5 = (kd >> 4);
        cmd.d6 = ((kd & 0xF) << 4) | (t >> 8);
        cmd.d7 = (t & 0xFF);

        return std::bit_cast<uint64_t>(cmd);
    }

    /// @brief 生成速度控制命令 (速度模式)
    /// @warning 发送至: CAN ID + 0x200
    uint64_t generate_velocity_command(double control_velocity, double torque_limit = nan_) const {
        if (std::isnan(control_velocity)) return generate_disable_command();

        double sign = reversed_ ? -1.0 : 1.0;
        float v_des = static_cast<float>(control_velocity * sign);

        PACKED_STRUCT({
            float v;
            uint8_t placeholder[4];
        }) cmd;
        
        cmd.v = v_des;
        (void)torque_limit; // 达妙速度模式不支持直接设置扭矩限制

        return std::bit_cast<uint64_t>(cmd);
    }

    /// @brief 生成位置控制命令 (位置速度模式)
    /// @warning 发送至: CAN ID + 0x100
    uint64_t generate_angle_command(double control_angle, double velocity_limit = nan_) const {
        if (std::isnan(control_angle)) return generate_disable_command();

        double sign = reversed_ ? -1.0 : 1.0;
        // 应用零点偏移
        float p_des = static_cast<float>((control_angle + encoder_zero_point_) * sign);
        float v_limit = std::isnan(velocity_limit) ? 0.0f : static_cast<float>(velocity_limit);

        PACKED_STRUCT({
            float p;
            float v;
        }) cmd;

        cmd.p = p_des;
        cmd.v = v_limit;

        return std::bit_cast<uint64_t>(cmd);
    }

private:
    uint16_t float_to_uint(float x, float x_min, float x_max, int bits) const {
        float span = x_max - x_min;
        float offset = x_min;
        if (x > x_max) x = x_max;
        else if (x < x_min) x = x_min;
        return (uint16_t)((x - offset) * ((float)((1 << bits) - 1)) / span);
    }

    static constexpr double nan_ = std::numeric_limits<double>::quiet_NaN();

    // Config parameters
    double p_max_;
    double v_max_;
    double t_max_;
    double encoder_zero_point_;
    bool reversed_;

    // Pre-calculated coefficients
    double status_pos_coefficient_;
    double status_vel_coefficient_;
    double status_torque_coefficient_;

    // State
    std::atomic<uint64_t> can_data_ = 0;
    int64_t multi_turn_encoder_count_ = 0;
    uint16_t last_raw_pos_ = 0;

    double angle_;
    double velocity_;
    double torque_;
    double temperature_;
    double max_torque_;
};

} // namespace librmcs::device