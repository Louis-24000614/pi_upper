#ifndef UART_LINK_SPEECH_QUEUE_H_
#define UART_LINK_SPEECH_QUEUE_H_

#include <array>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <string>

namespace uart {

struct SpeechItem {
  std::string event_id;
  uint8_t audio_id = 0;
};

/// ACK releases a request, while measured track duration releases the speaker.
/// Without a complete duration table only the existing UID path is available.
class SpeechQueue {
 public:
  static constexpr size_t kCapacity = 64;
  bool SetDuration(unsigned id, uint32_t ms) {
    if (id < 1 || id > 32 || ms == 0) return false;
    durations_[id - 1] = ms;
    return true;
  }
  bool durations_ready() const {
    for (uint32_t ms : durations_) if (ms == 0) return false;
    return true;
  }
  bool Enqueue(const SpeechItem& item, std::string* reason) {
    auto reject = [&](const char* why) { if (reason) *reason = why; return false; };
    if (item.audio_id < 1 || item.audio_id > 32 || item.event_id.empty() ||
        item.event_id.size() > 128) return reject("bad_request");
    for (char ch : item.event_id) {
      if (!((ch >= 'a' && ch <= 'z') || (ch >= 'A' && ch <= 'Z') ||
            (ch >= '0' && ch <= '9') || ch == '-' || ch == '_' || ch == '.' || ch == ':'))
        return reject("bad_event_id");
    }
    if (item.audio_id > 12 && !durations_ready()) return reject("duration_unconfigured");
    if (queue_.size() + (inflight_ ? 1U : 0U) >= kCapacity) return reject("queue_full");
    if (inflight_ && current_.event_id == item.event_id) return reject("duplicate_event");
    for (const auto& pending : queue_)
      if (pending.event_id == item.event_id) return reject("duplicate_event");
    queue_.push_back(item);
    return true;
  }
  const SpeechItem* Next(uint64_t now_ms) const {
    if (inflight_ || queue_.empty() || now_ms < occupied_until_ms_) return nullptr;
    return &queue_.front();
  }
  void MarkSent(uint64_t now_ms) {
    current_ = queue_.front();
    queue_.pop_front();
    inflight_ = true;
    occupied_until_ms_ = durations_ready() ? now_ms + durations_[current_.audio_id - 1] : now_ms;
  }
  bool Finish(SpeechItem* out) {
    if (!inflight_) return false;
    if (out) *out = current_;
    inflight_ = false;
    return true;
  }
  bool PopQueued(SpeechItem* out) {
    if (queue_.empty()) return false;
    if (out) *out = queue_.front();
    queue_.pop_front();
    return true;
  }
  bool inflight() const { return inflight_; }
  size_t size() const { return queue_.size(); }

 private:
  std::array<uint32_t, 32> durations_{};
  std::deque<SpeechItem> queue_;
  SpeechItem current_;
  bool inflight_ = false;
  uint64_t occupied_until_ms_ = 0;
};

}  // namespace uart
#endif
