#include "link/speech_queue.h"
#include "check.h"

int main() {
  uart::SpeechQueue queue;
  std::string reason;
  CHECK(queue.Enqueue({"uid-1", 1}, &reason));
  CHECK(!queue.Enqueue({"inspect-1", 13}, &reason));
  CHECK(reason == "duration_unconfigured");
  CHECK(!queue.Enqueue({"invalid", 33}, &reason));
  CHECK(!queue.SetDuration(0, 100));
  CHECK(!queue.SetDuration(1, 0));
  for (unsigned id = 1; id <= 32; ++id) CHECK(queue.SetDuration(id, 1000 + id));
  CHECK(queue.durations_ready());
  CHECK(queue.Enqueue({"inspect-1", 13}, &reason));
  CHECK(queue.Next(100)->event_id == "uid-1");
  queue.MarkSent(100);
  CHECK(queue.Next(101) == nullptr);
  uart::SpeechItem sent;
  CHECK(queue.Finish(&sent));
  CHECK(sent.event_id == "uid-1");
  // ACK does not end the measured playback window.
  CHECK(queue.Next(1100) == nullptr);
  CHECK(queue.Next(1101)->event_id == "inspect-1");
  queue.MarkSent(1101);
  CHECK(!queue.Enqueue({"inspect-1", 13}, &reason));
  CHECK(reason == "duplicate_event");
  CHECK(queue.Finish(&sent));
  CHECK(queue.Next(2113) == nullptr);
  uart::SpeechQueue full;
  for (unsigned i = 0; i < uart::SpeechQueue::kCapacity; ++i)
    CHECK(full.Enqueue({"uid-" + std::to_string(i), static_cast<uint8_t>(i % 12 + 1)}, &reason));
  CHECK(!full.Enqueue({"overflow", 1}, &reason));
  CHECK(reason == "queue_full");
  for (unsigned i = 0; i < uart::SpeechQueue::kCapacity; ++i) {
    CHECK(full.PopQueued(&sent));
    CHECK(sent.event_id == "uid-" + std::to_string(i));
  }
  CHECK(!full.PopQueued(&sent));
  return uart::test::Finish("speech_queue");
}
