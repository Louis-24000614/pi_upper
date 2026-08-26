#include "proto/frame.h"

#include "proto/crc8.h"

namespace uart {

size_t EncodeFrame(uint8_t msg_type, const uint8_t* payload, size_t payload_len, uint8_t* dst,
                   size_t cap) {
  if (payload_len > kMaxPayloadSize || (payload == nullptr && payload_len != 0)) {
    return 0;
  }
  const size_t total = payload_len + kFrameOverhead;
  if (cap < total) {
    return 0;
  }

  dst[0] = kSync1;
  dst[1] = kSync2;
  dst[2] = msg_type;
  dst[3] = static_cast<uint8_t>(payload_len);
  for (size_t i = 0; i < payload_len; ++i) {
    dst[4 + i] = payload[i];
  }
  // CRC 覆盖 TYPE + LENGTH + PAYLOAD；同步字不参与计算。
  dst[total - 1] = Crc8(dst + 2, payload_len + 2);
  return total;
}

void Reassembler::ResetFrame() {
  state_ = State::kWaitSync1;
  payload_index_ = 0;
  running_crc_ = 0;
}

void Reassembler::Reset() {
  ResetFrame();
  last_byte_us_ = 0;
}

void Reassembler::Feed(const uint8_t* data, size_t len, uint64_t now_us,
                       const FrameHandler& handler) {
  // 帧收到一半突然断流：残留的半帧不能和后来的字节拼成一帧，否则会解出一个
  // 长度和内容都错位的"合法"帧。超时后丢弃重新找同步字。
  if (last_byte_us_ != 0 && state_ != State::kWaitSync1 &&
      now_us - last_byte_us_ > kInterByteTimeoutUs) {
    ++stats_.timeouts;
    ResetFrame();
  }
  if (len != 0) {
    last_byte_us_ = now_us;
  }

  for (size_t i = 0; i < len; ++i) {
    const uint8_t byte = data[i];
    switch (state_) {
      case State::kWaitSync1:
        if (byte == kSync1) {
          state_ = State::kWaitSync2;
        }
        break;

      case State::kWaitSync2:
        if (byte == kSync2) {
          state_ = State::kReadType;
        } else if (byte != kSync1) {
          // 连续的 0x55 要留在本状态：55 55 AA 里的第二个 55 才是真正的同步字首字节。
          state_ = State::kWaitSync1;
        }
        break;

      case State::kReadType:
        msg_type_ = byte;
        running_crc_ = Crc8Update(0, byte);
        state_ = State::kReadLength;
        break;

      case State::kReadLength:
        payload_len_ = byte;
        running_crc_ = Crc8Update(running_crc_, byte);
        payload_index_ = 0;
        if (byte > kMaxPayloadSize) {
          ++stats_.overflows;
          ResetFrame();
          break;
        }
        state_ = byte == 0 ? State::kReadCrc : State::kReadPayload;
        break;

      case State::kReadPayload:
        payload_[payload_index_++] = byte;
        running_crc_ = Crc8Update(running_crc_, byte);
        if (payload_index_ >= payload_len_) {
          state_ = State::kReadCrc;
        }
        break;

      case State::kReadCrc:
        if (byte == running_crc_) {
          ++stats_.frames;
          ResetFrame();
          if (handler) {
            handler(msg_type_, payload_, payload_len_);
          }
          break;
        }
        ++stats_.crc_errors;
        ResetFrame();
        // 坏帧的最后一个字节可能正是下一帧的同步字首字节，直接进入等待第二个
        // 同步字，省掉一次不必要的重新搜索。与固件的行为保持一致。
        if (byte == kSync1) {
          state_ = State::kWaitSync2;
        }
        break;
    }
  }
}

}  // namespace uart
