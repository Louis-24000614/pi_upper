#include "link/speech_queue.h"
#include "check.h"

int main() {
  std::string reason;
  uart::SpeechItem sent;
  for (unsigned configured : std::array<unsigned, 3>{0, 1, 32}) {
    uart::SpeechQueue queue;
    CHECK(!queue.Enqueue({"invalid", 0}, &reason));
    CHECK(!queue.Enqueue({"invalid", 33}, &reason));
    CHECK(!queue.SetDuration(0, 100));
    CHECK(!queue.SetDuration(1, 0));
    for (unsigned id = 1; id <= configured; ++id) CHECK(queue.SetDuration(id, 100000 + id));
    CHECK(queue.durations_ready() == (configured == 32));
    for (unsigned id = 1; id <= 32; ++id)
      CHECK(queue.Enqueue({"audio-" + std::to_string(id), static_cast<uint8_t>(id)}, &reason));
    for (unsigned id = 1; id <= 32; ++id) {
      CHECK(queue.Next(100)->audio_id == id);
      queue.MarkSent(100);
      CHECK(queue.Next(100) == nullptr);
      CHECK(!queue.Enqueue({"audio-" + std::to_string(id), static_cast<uint8_t>(id)}, &reason));
      CHECK(reason == "duplicate_event");
      CHECK(queue.Finish(&sent));
      CHECK(sent.event_id == "audio-" + std::to_string(id));
      // The next item is available at the same time once the ACK arrives.
    }
    CHECK(queue.Next(100) == nullptr);
  }
  uart::SpeechQueue full;
  for (unsigned i = 0; i < uart::SpeechQueue::kCapacity; ++i)
    CHECK(full.Enqueue({"uid-" + std::to_string(i), static_cast<uint8_t>(i % 32 + 1)}, &reason));
  CHECK(!full.Enqueue({"overflow", 1}, &reason));
  CHECK(reason == "queue_full");
  for (unsigned i = 0; i < uart::SpeechQueue::kCapacity; ++i) {
    CHECK(full.PopQueued(&sent));
    CHECK(sent.event_id == "uid-" + std::to_string(i));
  }
  CHECK(!full.PopQueued(&sent));
  return uart::test::Finish("speech_queue");
}
