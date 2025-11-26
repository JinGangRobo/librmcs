#include <cstdint>
#include <librmcs/client/cboard.hpp>
#include <librmcs/utility/endian_promise.hpp>
#include <vector>

class MyRobot : public librmcs::client::CBoard {
public:
    explicit MyRobot(int32_t usb_pid = -1)
        : CBoard(usb_pid) {}

private:
    void log_can_device_ids() {
        LOG_INFO("\n--------------------------");
        LOG_INFO("CAN 1 Device IDs:");
        for (const auto& id : can_1_device_ids_) {
            LOG_INFO("  0x%X", id);
        }

        LOG_INFO("CAN 2 Device IDs:");
        for (const auto& id : can_2_device_ids_) {
            LOG_INFO("  0x%X", id);
        }
    }

    void can1_receive_callback(
        uint32_t can_id, uint64_t can_data, bool is_extended_can_id, bool is_remote_transmission,
        uint8_t can_data_length) override {
        (void)can_data;
        (void)is_extended_can_id;
        (void)is_remote_transmission;
        (void)can_data_length;

        if (std::find(can_1_device_ids_.begin(), can_1_device_ids_.end(), can_id)
            == can_1_device_ids_.end()) {
            can_1_device_ids_.push_back(can_id);
        }

        log_can_device_ids();
    }

    void can2_receive_callback(
        uint32_t can_id, uint64_t can_data, bool is_extended_can_id, bool is_remote_transmission,
        uint8_t can_data_length) override {
        (void)can_data;
        (void)is_extended_can_id;
        (void)is_remote_transmission;
        (void)can_data_length;

        if (std::find(can_2_device_ids_.begin(), can_2_device_ids_.end(), can_id)
            == can_2_device_ids_.end()) {
            can_2_device_ids_.push_back(can_id);
        }

        log_can_device_ids();
    }

    std::vector<uint32_t> can_1_device_ids_;
    std::vector<uint32_t> can_2_device_ids_;
};

int main() {
    MyRobot my_robot{};
    my_robot.handle_events();
}