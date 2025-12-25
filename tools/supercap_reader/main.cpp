#include <librmcs/client/cboard.hpp>

class MyRobot : public librmcs::client::CBoard {
public:
    explicit MyRobot(int32_t usb_pid = -1)
        : CBoard(usb_pid) {}

private:
    void can1_receive_callback(
        uint32_t can_id, uint64_t can_data, bool is_extended_can_id, bool is_remote_transmission,
        uint8_t can_data_length) override {
        (void)is_extended_can_id;
        (void)is_remote_transmission;
        (void)can_data_length;
        (void)can_id;

        // LOG_INFO(
        //     "CAN1 received: ID=0x%X, DATA=0x%016lX, LEN=%d", can_id, can_data, can_data_length);

        struct __attribute__((packed, aligned(8))) SupercapStatus {
            uint8_t voltage_B1;
            uint8_t voltage_B2;
            uint8_t reserved;
            uint32_t chassis_pow;
            uint8_t supcap_status;
        } feedback = std::bit_cast<decltype(feedback)>(can_data);

        double voltage = ((feedback.voltage_B1 << 8) | feedback.voltage_B2) / 100.0;
        double chassis_pwr = std::bit_cast<float>(feedback.chassis_pow);
        LOG_INFO("Supercap Voltage=%.2f V, Chassis Power=%.2f W", voltage, chassis_pwr);

    }
};

int main() {
    MyRobot my_robot{};
    my_robot.handle_events();
}