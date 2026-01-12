#pragma once

#include <cstdint>
#include <vector>
#include <initializer_list>

class SerialFrame {
public:
    static constexpr uint8_t FRAME_HEAD = 0xAA;
    static constexpr uint8_t FRAME_TAIL = 0x0D;

public:
    SerialFrame(uint8_t cmd)
        : command_(cmd) {}

    SerialFrame(uint8_t cmd, std::initializer_list<uint8_t> data)
        : command_(cmd), data_(data) {}

public:
    void set_command(uint8_t cmd) {
        command_ = cmd;
    }

    void set_data(const std::vector<uint8_t>& data) {
        data_ = data;
    }

    void clear_data() {
        data_.clear();
    }

public:
    /// Length = CMD + DATA
    uint8_t length() const {
        return static_cast<uint8_t>(1 + data_.size());
    }

    /// 生成完整帧（可直接发串口）
    std::vector<uint8_t> to_bytes() const {
        std::vector<uint8_t> frame;
        frame.reserve(data_.size() + 5);

        frame.push_back(FRAME_HEAD);
        frame.push_back(length());
        frame.push_back(command_);

        for (auto b : data_) {
            frame.push_back(b);
        }

        frame.push_back(calc_checksum());
        frame.push_back(FRAME_TAIL);

        return frame;
    }

private:
    uint8_t calc_checksum() const {
        uint8_t cs = length() ^ command_;
        for (auto b : data_) {
            cs ^= b;
        }
        return cs;
    }

private:
    uint8_t              command_{0};
    std::vector<uint8_t> data_;
};
