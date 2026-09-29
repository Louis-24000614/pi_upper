#include "mission_coordinator.h"

namespace mission {
namespace {

constexpr uint8_t kHomePoint = 0U;

}  // namespace

Coordinator::Coordinator(Services services) : services_(services) { Reset(); }

void Coordinator::Reset() {
  state_ = State::kIdle;
  started_ = false;
  return_home_started_ = false;
  route_index_ = 0U;
  last_rfid_generation_ = 0U;
  have_last_rfid_generation_ = false;
  pending_card_ = false;
  pending_card_number_ = 0U;
  visited_.fill(false);
  output_ = Output{};

  // 固定赛场地图的默认巡逻顺序；这不是 RFID 顺序推断。
  for (uint8_t i = 0U; i < kPatrolPointCount; ++i) {
    patrol_order_[i] = static_cast<uint8_t>(i + 1U);
  }

  if (services_.navigation != nullptr) services_.navigation->Stop();
  SetState(State::kIdle);
  SetMotion(MotionIntent::kStop);
}

void Coordinator::Start() {
  started_ = true;
  return_home_started_ = false;
  route_index_ = 0U;
  visited_.fill(false);
  output_.visited = visited_;
  output_.visited_count = 0U;
  output_.last_card_number = 0U;
  SetTargetPoint(patrol_order_[route_index_]);
  SetState(State::kWaitingForLink);
  SetMotion(MotionIntent::kStop);
}

void Coordinator::Stop() {
  started_ = false;
  SetState(State::kIdle);
  SetMotion(MotionIntent::kStop);
  if (services_.navigation != nullptr) services_.navigation->Stop();
}

bool Coordinator::ValidPoint(uint8_t point_id) {
  return point_id >= 1U && point_id <= kPatrolPointCount;
}

uint8_t Coordinator::CountVisited(
    const std::array<bool, kPatrolPointCount + 1U>& visited) {
  uint8_t count = 0U;
  for (uint8_t point = 1U; point <= kPatrolPointCount; ++point) {
    if (visited[point]) ++count;
  }
  return count;
}

void Coordinator::SetState(State state) {
  if (state_ == state && output_.state == state) return;
  state_ = state;
  output_.state = state;
  output_.changed = true;
}

void Coordinator::SetMotion(MotionIntent motion) {
  if (output_.motion == motion) return;
  output_.motion = motion;
  output_.changed = true;
}

void Coordinator::SetTargetPoint(uint8_t point_id) {
  if (output_.target_point == point_id) return;
  output_.target_point = point_id;
  output_.changed = true;
  if (services_.navigation != nullptr) {
    services_.navigation->SetTargetPoint(point_id);
  }
}

void Coordinator::EnterReturnHome() {
  return_home_started_ = true;
  SetTargetPoint(kHomePoint);
  SetState(State::kReturnHome);
  SetMotion(MotionIntent::kReturnHome);
  if (services_.navigation != nullptr) services_.navigation->ReturnHome();
}

void Coordinator::CompleteMission() {
  SetState(State::kCompleted);
  SetMotion(MotionIntent::kStop);
  if (services_.navigation != nullptr) services_.navigation->Stop();
}

void Coordinator::HandleRfid(const TaskObservation& observation) {
  const RfidObservation& rfid = observation.rfid;
  if (!rfid.present) return;

  /*
   * generation 只用于识别“新卡到达”。卡号本身不映射物理点位；
   * 物理点必须由导航层依据固定地图和当前位置确认。
   */
  if (!have_last_rfid_generation_ || rfid.generation != last_rfid_generation_) {
    last_rfid_generation_ = rfid.generation;
    have_last_rfid_generation_ = true;
    pending_card_ = true;
    pending_card_number_ = rfid.card_number;
    output_.last_card_number = rfid.card_number;
  }

  if (!pending_card_ || !observation.patrol_point_detected ||
      !ValidPoint(observation.patrol_point_id) ||
      observation.patrol_point_id != output_.target_point ||
      state_ == State::kCompleted || state_ == State::kFault ||
      visited_[observation.patrol_point_id]) {
    return;
  }

  const uint8_t point_id = observation.patrol_point_id;
  pending_card_ = false;

  visited_[point_id] = true;
  output_.visited = visited_;
  output_.visited_count = CountVisited(visited_);
  output_.speak_requested = true;
  output_.speech_id = point_id;
  if (services_.speech != nullptr) {
    (void)services_.speech->Speak(point_id);
  }

  SetState(State::kAtPatrolPoint);
  SetMotion(MotionIntent::kStop);
  if (output_.visited_count >= kPatrolPointCount) {
    EnterReturnHome();
    return;
  }

  ++route_index_;
  SetTargetPoint(patrol_order_[route_index_]);
}

void Coordinator::HandleJunction(const JunctionObservation& junction) {
  if (state_ != State::kFollowRoad && state_ != State::kApproachJunction) return;
  if (!junction.detected) return;
  SetState(State::kApproachJunction);
  if (!junction.direction_valid || junction.direction == TurnDirection::kUnknown) {
    SetMotion(MotionIntent::kStop);
    if (services_.navigation != nullptr) services_.navigation->Stop();
    return;
  }

  if (junction.direction == TurnDirection::kStraight) {
    SetState(State::kSearchRoad);
    SetMotion(MotionIntent::kSearchRoad);
    if (services_.navigation != nullptr) services_.navigation->SearchRoad();
    return;
  }

  SetState(State::kTurning);
  SetMotion(junction.direction == TurnDirection::kLeft ? MotionIntent::kTurnLeft
                                                       : MotionIntent::kTurnRight);
  if (services_.navigation != nullptr) services_.navigation->RequestTurn(junction.direction);
}

void Coordinator::HandleTurnResult(const TaskObservation& observation) {
  if (state_ != State::kTurning) return;
  if (!observation.turn_finished) return;
  if (!observation.turn_succeeded) {
    SetState(State::kFault);
    SetMotion(MotionIntent::kStop);
    if (services_.navigation != nullptr) services_.navigation->Stop();
    return;
  }
  SetState(State::kSearchRoad);
  SetMotion(MotionIntent::kSearchRoad);
  if (services_.navigation != nullptr) services_.navigation->SearchRoad();
}

void Coordinator::HandleObstacle(const TaskObservation& observation) {
  if (!observation.obstacle_hard_blocked) return;
  if (state_ == State::kCompleted || state_ == State::kFault) return;
  SetState(State::kAvoidObstacle);
  SetMotion(MotionIntent::kStop);
  if (services_.navigation != nullptr) services_.navigation->Stop();
  if (services_.obstacle != nullptr && services_.obstacle->ReplanFinished()) {
    SetState(State::kFollowRoad);
    SetMotion(MotionIntent::kFollowRoad);
    if (services_.navigation != nullptr) services_.navigation->FollowRoad();
  }
}

void Coordinator::HandleTunnel(const TaskObservation& observation) {
  if (!observation.tunnel_active || state_ == State::kCompleted ||
      state_ == State::kFault || state_ == State::kAvoidObstacle) {
    return;
  }
  SetState(State::kTunnel);
  SetMotion(MotionIntent::kFollowRoad);
  if (services_.tunnel != nullptr) (void)services_.tunnel->Update();
}

void Coordinator::Tick(const TaskObservation& observation) {
  output_.changed = false;
  output_.speak_requested = false;
  output_.speech_id = 0U;

  if (!started_) {
    SetState(State::kIdle);
    SetMotion(MotionIntent::kStop);
    return;
  }
  if (!observation.link_ready) {
    SetState(State::kWaitingForLink);
    SetMotion(MotionIntent::kStop);
    if (services_.navigation != nullptr) services_.navigation->Stop();
    return;
  }
  if (observation.search_timed_out) {
    SetState(State::kFault);
    SetMotion(MotionIntent::kStop);
    if (services_.navigation != nullptr) services_.navigation->Stop();
    return;
  }

  HandleRfid(observation);
  HandleObstacle(observation);
  if (state_ == State::kAvoidObstacle || state_ == State::kFault ||
      state_ == State::kCompleted) {
    return;
  }
  HandleTunnel(observation);
  if (state_ == State::kTunnel) return;

  if (return_home_started_) {
    if (observation.road_visible) {
      SetMotion(MotionIntent::kReturnHome);
      if (services_.navigation != nullptr) services_.navigation->ReturnHome();
    } else {
      SetMotion(MotionIntent::kStop);
    }
    return;
  }

  HandleTurnResult(observation);
  if (state_ == State::kTurning || state_ == State::kSearchRoad ||
      state_ == State::kFault) {
    if (state_ == State::kSearchRoad && observation.road_visible) {
      SetState(State::kFollowRoad);
      SetMotion(MotionIntent::kFollowRoad);
      if (services_.navigation != nullptr) services_.navigation->FollowRoad();
    }
    return;
  }

  if (state_ == State::kWaitingForLink || state_ == State::kAtPatrolPoint ||
      state_ == State::kIdle) {
    SetState(State::kFollowRoad);
  }
  if (state_ == State::kFollowRoad) {
    if (!observation.road_visible) {
      SetMotion(MotionIntent::kStop);
      if (services_.navigation != nullptr) services_.navigation->Stop();
      return;
    }
    SetMotion(MotionIntent::kFollowRoad);
    if (services_.navigation != nullptr) services_.navigation->FollowRoad();
    HandleJunction(observation.junction);
  }
}

}  // namespace mission
