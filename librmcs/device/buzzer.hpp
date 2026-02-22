#pragma once

#include <bit>
#include <cstdint>

#include "../utility/cross_os.hpp"

namespace librmcs::device {
class Buzzer {
public:
    enum class Tone : uint8_t { OFF, LOW, MEDIUM, HIGH };
    enum class Mode : uint8_t { CONTINUOUS, ONCE };

    struct BuzzerScore {
        Tone first_tone;
        Tone second_tone;
        Tone third_tone;
        uint8_t padding;
    };

    void set_score(const BuzzerScore& score, Mode mode) {
        if (mode == Mode::CONTINUOUS) {
            current_score_continuous_ = score;
        } else {
            current_score_once_ = score;
            current_progress_++;
        }
    }

    void remove_score(Mode mode) {
        set_score({Tone::OFF, Tone::OFF, Tone::OFF, 0}, mode);
        current_progress_ = 0;
    }

    void update() {
        if (std::bit_cast<uint32_t>(current_score_continuous_)) {
            current_progress_++;
        }
        if (current_progress_ > 254) {
            current_progress_ = 0;
        }
    }

    uint8_t generate_command() {
        BuzzerScoreMsg msg;
        BuzzerScore score_to_use;

        if (std::bit_cast<uint32_t>(current_score_once_)) {
            if (!reseted_) {
                reseted_ = true;
                return 0;
            } else
                score_to_use = current_score_once_;
        } else {
            score_to_use = current_score_continuous_;
        }

        msg.first_tone = static_cast<uint8_t>(score_to_use.first_tone);
        msg.second_tone = static_cast<uint8_t>(score_to_use.second_tone);
        msg.third_tone = static_cast<uint8_t>(score_to_use.third_tone);
        msg.score_id = static_cast<uint8_t>(current_progress_ & 0b11);

        return std::bit_cast<uint8_t>(msg);
    }

private:
    BuzzerScore current_score_continuous_;
    BuzzerScore current_score_once_;
    uint8_t current_progress_;

    bool reseted_ = false;

    PACKED_STRUCT(BuzzerScoreMsg {
        uint8_t first_tone  : 2;
        uint8_t second_tone : 2;
        uint8_t third_tone  : 2;
        uint8_t score_id    : 2;
    });
    static_assert(sizeof(BuzzerScoreMsg) == 1);
};
} // namespace librmcs::device