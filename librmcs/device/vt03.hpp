#pragma once

#include <cstdint>
#include <cstring>

#include <atomic>
#include <bit>

#include "../utility/cross_os.hpp"

namespace librmcs::device {

class Vt03 {
public:
    explicit Vt03() = default;

    void store_status(const std::byte* uart_data, size_t uart_data_length) {
        if (uart_data_length != 2 + 8 + 7 + 4)
            return;

        // Avoid using reinterpret_cast here because it does not account for pointer alignment.
        // Vt03DataPart structures are aligned, and using reinterpret_cast on potentially unaligned
        // uart_data can cause undefined behavior on architectures that enforce strict alignment
        // requirements (e.g., ARM).
        // Directly accessing unaligned memory through a casted pointer can lead to crashes,
        // inefficiencies, or incorrect data reads. Instead, std::memcpy safely copies the data from
        // unaligned memory to properly aligned structures without violating alignment or strict
        // aliasing rules.

        uint16_t part0;
        std::memcpy(&part0, uart_data, 2);
        uart_data += 2;
        data_part0_.store(part0, std::memory_order::relaxed);

        uint64_t part1;
        std::memcpy(&part1, uart_data, 8);
        uart_data += 8;
        data_part1_.store(part1, std::memory_order::relaxed);

        uint64_t part2;
        std::memcpy(&part2, uart_data, 7);
        uart_data += 7;
        data_part2_.store(part2, std::memory_order::relaxed);

        uint32_t part3;
        std::memcpy(&part3, uart_data, 4);
        uart_data += 4;
        data_part3_.store(part3, std::memory_order::relaxed);
    }

    void update_status() {
        auto part0 alignas(uint16_t) =
            std::bit_cast<Vt03DataPart0>(data_part0_.load(std::memory_order::relaxed));

        auto part1 alignas(uint64_t) =
            std::bit_cast<Vt03DataPart1>(data_part1_.load(std::memory_order::relaxed));

        auto channel_to_double = [](int32_t value) {
            value -= 1024;
            if (-660 <= value && value <= 660)
                return value / 660.0;
            else
                return 0.0;
        };
        joystick_right_.y = -channel_to_double(static_cast<uint16_t>(part1.joystick_channel0));
        joystick_right_.x = channel_to_double(static_cast<uint16_t>(part1.joystick_channel1));

        joystick_left_.x = channel_to_double(static_cast<uint16_t>(part1.joystick_channel2));
        joystick_left_.y = -channel_to_double(static_cast<uint16_t>(part1.joystick_channel3));

        mode_switch_ = static_cast<Switch>(part1.mode_switch);
        pause_ = static_cast<bool>(part1.pause);
        fn_1_ = static_cast<bool>(part1.fn_1);
        fn_2_ = static_cast<bool>(part1.fn_2);

        rotary_knob_ = channel_to_double(part1.rotary_knob);
        trigger_ = static_cast<bool>(part1.trigger);

        auto part2 alignas(uint64_t) =
            std::bit_cast<Vt03DataPart2>(data_part2_.load(std::memory_order::relaxed));

        mouse_velocity_.x = -part2.mouse_velocity_y / 32768.0;
        mouse_velocity_.y = -part2.mouse_velocity_x / 32768.0;

        mouse_wheel_ = -part2.mouse_velocity_z / 32768.0;

        mouse_.left = part2.mouse_left;
        mouse_.right = part2.mouse_right;

        auto part3 alignas(uint32_t) =
            std::bit_cast<Vt03DataPart3>(data_part3_.load(std::memory_order::relaxed));

        keyboard_ = part3.keyboard;
    }

    struct Vector {
        constexpr static inline Vector zero() { return {0, 0}; }
        double x, y;
    };

    enum class Switch : uint8_t { UNKNOWN = 0, LEFT = 1, MIDDLE = 2, RIGHT = 3 };

    PACKED_STRUCT(Mouse {
        constexpr static inline Mouse zero() {
            constexpr uint8_t zero = 0;
            return std::bit_cast<Mouse>(zero);
        }

        bool left  : 1;
        bool right : 1;
    });
    static_assert(sizeof(Mouse) == 1);

    PACKED_STRUCT(Keyboard {
        constexpr static inline Keyboard zero() {
            constexpr uint16_t zero = 0;
            return std::bit_cast<Keyboard>(zero);
        }

        bool w     : 1;
        bool s     : 1;
        bool a     : 1;
        bool d     : 1;
        bool shift : 1;
        bool ctrl  : 1;
        bool q     : 1;
        bool e     : 1;
        bool r     : 1;
        bool f     : 1;
        bool g     : 1;
        bool z     : 1;
        bool x     : 1;
        bool c     : 1;
        bool v     : 1;
        bool b     : 1;
    });
    static_assert(sizeof(Keyboard) == 2);

    Vector joystick_right() const { return joystick_right_; }
    Vector joystick_left() const { return joystick_left_; }

    Switch mode_switch() const { return mode_switch_; }

    Vector mouse_velocity() const { return mouse_velocity_; }

    Mouse mouse() const { return mouse_; }
    Keyboard keyboard() const { return keyboard_; }

    double rotary_knob() const { return rotary_knob_; }

    double mouse_wheel() const { return mouse_wheel_; }

private:
    PACKED_STRUCT(Vt03DataPart0 {
        uint8_t sof_1;
        uint8_t sof_2;
    });
    static_assert(sizeof(Vt03DataPart0) == 2);
    std::atomic<uint16_t> data_part0_{std::bit_cast<uint16_t>(Vt03DataPart0{0, 0})};
    static_assert(decltype(data_part0_)::is_always_lock_free);

    PACKED_STRUCT(Vt03DataPart1 {
        uint64_t joystick_channel0 : 11;
        uint64_t joystick_channel1 : 11;
        uint64_t joystick_channel2 : 11;
        uint64_t joystick_channel3 : 11;

        uint64_t mode_switch : 2;
        uint64_t pause       : 1;
        uint64_t fn_1        : 1;
        uint64_t fn_2        : 1;
        uint64_t rotary_knob : 11;
        uint64_t trigger     : 1;

        uint64_t padding : 3;
    });
    static_assert(sizeof(Vt03DataPart1) == 8);
    std::atomic<uint64_t> data_part1_{std::bit_cast<uint64_t>(
        Vt03DataPart1{1024, 1024, 1024, 1024, (uint64_t)Switch::LEFT, 0, 0, 0, 0, 1024, 0})};
    static_assert(decltype(data_part1_)::is_always_lock_free);

    PACKED_STRUCT(Vt03DataPart2 {
        int16_t mouse_velocity_x;
        int16_t mouse_velocity_y;
        int16_t mouse_velocity_z;

        bool mouse_left;
        bool mouse_right;

    });
    static_assert(sizeof(Vt03DataPart2) == 8);
    std::atomic<uint64_t> data_part2_{
        std::bit_cast<uint64_t>(Vt03DataPart2{0, 0, 0, false, false})};
    static_assert(decltype(data_part2_)::is_always_lock_free);

    PACKED_STRUCT(Vt03DataPart3 {
        Keyboard keyboard;
        uint16_t crc;
    });
    static_assert(sizeof(Vt03DataPart3) == 4);
    std::atomic<uint32_t> data_part3_ = {
        std::bit_cast<uint32_t>(Vt03DataPart3{Keyboard::zero(), 0})};
    static_assert(decltype(data_part3_)::is_always_lock_free);

    Vector joystick_right_ = Vector::zero();
    Vector joystick_left_ = Vector::zero();

    Switch mode_switch_ = Switch::UNKNOWN;
    bool pause_ = false;
    bool fn_1_ = false;
    bool fn_2_ = false;
    double rotary_knob_ = 0.0;
    bool trigger_ = false;

    Vector mouse_velocity_ = Vector::zero();

    Mouse mouse_ = Mouse::zero();
    Keyboard keyboard_ = Keyboard::zero();

    double mouse_wheel_ = 0.0;
};

} // namespace librmcs::device