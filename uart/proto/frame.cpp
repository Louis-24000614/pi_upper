#include "proto/frame.h"

#include "proto/detail/bytes.h"
#include "proto/crc32c.h"

namespace uart {

size_t EncodeFrame(const Header& header, const uint8_t* payload, size_t payload_len, uint8_t* dst,
                   size_t cap) {
  if (payload_len > kMaxPayloadSize || (payload == nullptr && payload_len != 0)) {
    return 0;
  }

  uint8_t raw[kMaxDecodedSize] = {};
  PutU16(raw + 0, kMagic);
  raw[2] = header.version;
  raw[3] = header.msg_type;
  raw[4] = header.flags;
  raw[5] = 0;  // reserved，协议要求必须为 0
  PutU16(raw + 6, header.sequence);
  PutU16(raw + 8, static_cast<uint16_t>(payload_len));
  PutU64(raw + 10, header.timestamp_us);
  for (size_t i = 0; i < payload_len; ++i) {
    raw[kHeaderSize + i] = payload[i];
  }

  // CRC 覆盖未编码的帧头与 payload，不含 CRC 字段自身和结尾分隔符。
  const size_t crc_offset = kHeaderSize + payload_len;
  PutU32(raw + crc_offset, Crc32c(raw, crc_offset));
  const size_t raw_len = crc_offset + kCrcSize;

  if (cap < CobsEncodeBound(raw_len) + 1) {
    return 0;
  }
  const size_t encoded = CobsEncode(raw, raw_len, dst, cap - 1);
  if (encoded == 0) {
    return 0;
  }
  dst[encoded] = 0;  // 帧边界
  return encoded + 1;
}

FrameError DecodeRawFrame(const uint8_t* raw, size_t len, Header* out_header,
                          const uint8_t** out_payload) {
  if (len < kHeaderSize + kCrcSize || len > kMaxDecodedSize) {
    return FrameError::kLength;
  }
  if (GetU16(raw) != kMagic) {
    return FrameError::kMagic;
  }
  if (raw[5] != 0) {
    return FrameError::kReserved;
  }

  const uint16_t payload_length = GetU16(raw + 8);
  if (len != kHeaderSize + payload_length + kCrcSize) {
    return FrameError::kLength;
  }

  const size_t crc_offset = kHeaderSize + payload_length;
  if (GetU32(raw + crc_offset) != Crc32c(raw, crc_offset)) {
    return FrameError::kCrc;
  }

  out_header->version = raw[2];
  out_header->msg_type = raw[3];
  out_header->flags = raw[4];
  out_header->sequence = GetU16(raw + 6);
  out_header->payload_length = payload_length;
  out_header->timestamp_us = GetU64(raw + 10);
  *out_payload = raw + kHeaderSize;
  return FrameError::kOk;
}

void Reassembler::Reset() {
  encoded_len_ = 0;
  discarding_ = false;
}

void Reassembler::FinishFrame(const FrameHandler& handler) {
  const size_t decoded_len = CobsDecode(encoded_, encoded_len_, decoded_, sizeof(decoded_));
  encoded_len_ = 0;
  if (decoded_len == 0) {
    ++stats_.format_errors;
    return;
  }

  Header header;
  const uint8_t* payload = nullptr;
  const FrameError error = DecodeRawFrame(decoded_, decoded_len, &header, &payload);
  switch (error) {
    case FrameError::kOk:
      ++stats_.frames;
      if (handler) {
        handler(header, payload, header.payload_length);
      }
      return;
    case FrameError::kCrc:
      // 头部不可信，不能据此回 ACK，静默丢弃并计数。
      ++stats_.crc_errors;
      return;
    default:
      ++stats_.format_errors;
      return;
  }
}

void Reassembler::Feed(const uint8_t* data, size_t len, const FrameHandler& handler) {
  for (size_t i = 0; i < len; ++i) {
    const uint8_t byte = data[i];
    if (byte != 0) {
      if (encoded_len_ == sizeof(encoded_)) {
        // 超长说明已经错位，丢弃到下一个边界重新同步。
        if (!discarding_) {
          ++stats_.overflows;
          discarding_ = true;
        }
      } else if (!discarding_) {
        encoded_[encoded_len_++] = byte;
      }
      continue;
    }

    if (discarding_ || encoded_len_ == 0) {
      // 空帧（连续分隔符）不计错误，这是重新同步时的正常现象。
      encoded_len_ = 0;
      discarding_ = false;
      continue;
    }
    FinishFrame(handler);
  }
}

}  // namespace uart
