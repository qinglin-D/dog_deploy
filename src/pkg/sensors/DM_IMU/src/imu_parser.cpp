#include "imu_parser.h"
#include "crc16.h"

#include <cstdio>
#include <cstring>

IMUParser::IMUParser() {
    reset();
}

void IMUParser::setCallback(IMUDataCallback cb) {
    callback_ = std::move(cb);
}

// 重置状态机，丢帧后重新同步帧头
void IMUParser::reset() {
    state_ = SYNC1;
    buf_pos_ = 0;
    data_len_ = 0;
    frame_type_ = 0;
    dev_id_ = 0;
    current_data_ = IMUData{};
}

// 根据帧类型返回数据体字节数
// 加速度/角速度/欧拉角各 3 个 float = 12 字节，四元数 4 个 float = 16 字节
int IMUParser::expectedDataLen(uint8_t type) const {
    switch (type) {
        case 0x01: return 12; // 加速度 (AccX, AccY, AccZ)
        case 0x02: return 12; // 角速度 (GyroX, GyroY, GyroZ)
        case 0x03: return 12; // 欧拉角 (Roll, Pitch, Yaw)
        case 0x04: return 16; // 四元数 (W, X, Y, Z)
        default:   return 0;  // 未知类型，丢弃
    }
}

// 喂入原始字节流，内部状态机逐字节解析
// USB 帧格式: 55 AA ID TYPE [DATA] CRC16 0A
void IMUParser::feed(const uint8_t *data, int len) {
    for (int i = 0; i < len; ++i) {
        uint8_t byte = data[i];

        switch (state_) {
        case SYNC1:
            // 等待帧头第一个字节 0x55
            if (byte == 0x55) {
                state_ = SYNC2;
            }
            break;

        case SYNC2:
            // 等待帧头第二个字节 0xAA
            if (byte == 0xAA) {
                state_ = ID;
            } else if (byte != 0x55) {
                // 不是 0xAA 也不是新的 0x55，重新同步
                state_ = SYNC1;
            }
            // 如果是 0x55，保持 SYNC2 状态（处理 55 55 AA 这种情况）
            break;

        case ID:
            dev_id_ = byte;
            state_ = TYPE;
            break;

        case TYPE:
            frame_type_ = byte;
            data_len_ = expectedDataLen(byte);
            if (data_len_ == 0) {
                // 未知帧类型，丢弃并重新同步
                state_ = SYNC1;
            } else {
                buf_pos_ = 0;
                state_ = DATA;
            }
            break;

        case DATA:
            buf_[buf_pos_++] = byte;
            if (buf_pos_ >= data_len_) {
                state_ = CRC1;
            }
            break;

        case CRC1:
            buf_[buf_pos_++] = byte; // CRC 低字节
            state_ = CRC2;
            break;

        case CRC2:
            buf_[buf_pos_++] = byte; // CRC 高字节
            state_ = END_MARKER;
            break;

        case END_MARKER:
            // 帧尾必须是 0x0A
            if (byte == 0x0A) {
                processFrame();
            }
            // 无论帧尾是否正确，都回到 SYNC1 准备下一帧
            state_ = SYNC1;
            break;
        }
    }
}

// 打印字节数组的十六进制表示（调试用）
static void hexdump(const uint8_t *data, int len) {
    for (int i = 0; i < len; ++i) {
        fprintf(stderr, "%02X ", data[i]);
    }
}

// CRC 校验通过后，根据帧类型解析数据体并更新 IMUData
void IMUParser::processFrame() {
    // CRC 为小端序：低字节在前，高字节在后
    uint16_t received_crc = buf_[data_len_] | (buf_[data_len_ + 1] << 8);

    // 组装完整帧用于调试输出: 55 AA ID TYPE DATA CRC 0A
    int full_len = 2 + 2 + data_len_ + 2 + 1;
    uint8_t full_frame[32];
    full_frame[0] = 0x55;
    full_frame[1] = 0xAA;
    full_frame[2] = dev_id_;
    full_frame[3] = frame_type_;
    memcpy(full_frame + 4, buf_, data_len_);
    full_frame[4 + data_len_] = buf_[data_len_];     // CRC 低字节
    full_frame[5 + data_len_] = buf_[data_len_ + 1]; // CRC 高字节

    // 尝试不同的 CRC 计算范围和两种算法，以匹配设备的实际 CRC 实现
    struct CRCTest {
        const char *desc;
        const uint8_t *ptr;
        int len;
    };

    // 范围 1：ID + TYPE + DATA（最常规的范围）
    uint8_t scope1[20];
    scope1[0] = dev_id_;
    scope1[1] = frame_type_;
    memcpy(scope1 + 2, buf_, data_len_);

    // 范围 2：TYPE + DATA（不含 ID）
    uint8_t scope2[20];
    scope2[0] = frame_type_;
    memcpy(scope2 + 1, buf_, data_len_);

    // 范围 3：仅 DATA
    // 直接使用 buf_，不额外复制

    // 范围 4：包含帧头 55 AA + ID + TYPE + DATA
    uint8_t scope4[24];
    scope4[0] = 0x55;
    scope4[1] = 0xAA;
    scope4[2] = dev_id_;
    scope4[3] = frame_type_;
    memcpy(scope4 + 4, buf_, data_len_);

    CRCTest tests[] = {
        {"ID+TYPE+DATA",      scope1, 2 + data_len_},
        {"TYPE+DATA",         scope2, 1 + data_len_},
        {"DATA only",         buf_,   data_len_},
        {"55AA+ID+TYPE+DATA", scope4, 4 + data_len_},
    };

    // 尝试所有 CRC 范围和两种算法
    bool matched = false;
    for (auto &t : tests) {
        if (crc16_compute(t.ptr, t.len) == received_crc ||
            crc16_v1(t.ptr, t.len) == received_crc) {
            matched = true;
            break;
        }
    }
    // 也尝试 CRC 字节序交换（大端序）的情况
    if (!matched) {
        uint16_t swapped = (received_crc << 8) | (received_crc >> 8);
        for (auto &t : tests) {
            if (crc16_compute(t.ptr, t.len) == swapped ||
                crc16_v1(t.ptr, t.len) == swapped) {
                matched = true;
                break;
            }
        }
    }

    // 所有尝试都失败时，打印调试信息
    if (!matched) {
        fprintf(stderr, "\n--- CRC mismatch (type=0x%02X, data_len=%d) ---\n", frame_type_, data_len_);
        fprintf(stderr, "Full frame: ");
        hexdump(full_frame, full_len);
        fprintf(stderr, "0A\n");
        fprintf(stderr, "Received CRC: 0x%04X  (swapped: 0x%04X)\n",
                received_crc, (uint16_t)((received_crc << 8) | (received_crc >> 8)));
        for (auto &t : tests) {
            uint16_t c8 = crc16_compute(t.ptr, t.len);
            uint16_t c1 = crc16_v1(t.ptr, t.len);
            fprintf(stderr, "  %-20s (len=%2d): <<8=0x%04X  <<1=0x%04X\n",
                    t.desc, t.len, c8, c1);
        }
        return;
    }

    // CRC 校验通过，按帧类型解析数据
    current_data_.device_id = dev_id_;

    switch (frame_type_) {
    case 0x01: // 加速度 (g)，三个 float 小端序
        memcpy(&current_data_.accel.x, buf_ + 0, 4);
        memcpy(&current_data_.accel.y, buf_ + 4, 4);
        memcpy(&current_data_.accel.z, buf_ + 8, 4);
        current_data_.updated |= 1;
        break;

    case 0x02: // 角速度 (°/s)，三个 float 小端序
        memcpy(&current_data_.gyro.x, buf_ + 0, 4);
        memcpy(&current_data_.gyro.y, buf_ + 4, 4);
        memcpy(&current_data_.gyro.z, buf_ + 8, 4);
        current_data_.updated |= 2;
        break;

    case 0x03: // 欧拉角 (°): Roll, Pitch, Yaw
        memcpy(&current_data_.euler.x, buf_ + 0, 4);  // Roll (横滚)
        memcpy(&current_data_.euler.y, buf_ + 4, 4);  // Pitch (俯仰)
        memcpy(&current_data_.euler.z, buf_ + 8, 4);  // Yaw (航向)
        current_data_.updated |= 4;
        break;

    case 0x04: // 四元数: W, X, Y, Z
        memcpy(&current_data_.quat.w, buf_ + 0,  4);
        memcpy(&current_data_.quat.x, buf_ + 4,  4);
        memcpy(&current_data_.quat.y, buf_ + 8,  4);
        memcpy(&current_data_.quat.z, buf_ + 12, 4);
        current_data_.updated |= 8;
        break;
    }

    // 每收到一帧就触发回调，updated 位标记表示本帧数据类型
    // if (callback_) {
    //     callback_(current_data_);
    // }
    current_data_.updated = 0;
}
