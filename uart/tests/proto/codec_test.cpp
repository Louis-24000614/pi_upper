/// @file
/// 消息层单元测试。字段偏移写错时编解码往返仍然自洽（自己错、自己对得上），
/// 所以这里除了往返之外，还要对照协议文档硬编码若干字节位置和 payload 长度。

#include "proto/codec.h"

#include <cmath>
#include <cstring>
#include <limits>

#include "check.h"

namespace {

using namespace uart;  // NOLINT(build/namespaces) 测试内为了少写前缀

/// payload 长度必须与协议第 4 节的消息总表一致。写成编译期断言，改错了编不过。
static_assert(kSizeTimeSyncReq == 8, "TIME_SYNC_REQ payload 应为 8 字节");
static_assert(kSizeArmRequest == 4, "ARM_REQUEST payload 应为 4 字节");
static_assert(kSizeCmdVel == 12, "CMD_VEL payload 应为 12 字节");
static_assert(kSizeAck == 8, "ACK payload 应为 8 字节");
static_assert(kSizeHelloInfo == 16, "HELLO_INFO payload 应为 16 字节");
static_assert(kSizeTimeSyncResp == 24, "TIME_SYNC_RESP payload 应为 24 字节");
static_assert(kSizeOdomState == 52, "ODOM_STATE payload 应为 52 字节");
static_assert(kSizeImuState == 44, "IMU_STATE payload 应为 44 字节");
static_assert(kSizeImuDebug == 24, "IMU_DEBUG payload 应为 24 字节");
static_assert(kSizeSystemStatus == 36, "SYSTEM_STATUS payload 应为 36 字节");
static_assert(kSizeFaultEvent == 12, "FAULT_EVENT payload 应为 12 字节");

/// 消息 ID 对照协议总表。
void TestMsgTypeValues() {
  CHECK(static_cast<uint8_t>(MsgType::kHelloReq) == 0x01);
  CHECK(static_cast<uint8_t>(MsgType::kTimeSyncReq) == 0x02);
  CHECK(static_cast<uint8_t>(MsgType::kArmRequest) == 0x10);
  CHECK(static_cast<uint8_t>(MsgType::kDisarm) == 0x11);
  CHECK(static_cast<uint8_t>(MsgType::kCmdVel) == 0x12);
  CHECK(static_cast<uint8_t>(MsgType::kResetOdom) == 0x13);
  CHECK(static_cast<uint8_t>(MsgType::kClearFaultRequest) == 0x14);
  CHECK(static_cast<uint8_t>(MsgType::kAck) == 0x80);
  CHECK(static_cast<uint8_t>(MsgType::kHelloInfo) == 0x81);
  CHECK(static_cast<uint8_t>(MsgType::kTimeSyncResp) == 0x82);
  CHECK(static_cast<uint8_t>(MsgType::kOdomState) == 0x90);
  CHECK(static_cast<uint8_t>(MsgType::kImuState) == 0x91);
  CHECK(static_cast<uint8_t>(MsgType::kImuDebug) == 0x92);
  CHECK(static_cast<uint8_t>(MsgType::kSystemStatus) == 0x93);
  CHECK(static_cast<uint8_t>(MsgType::kFaultEvent) == 0x94);
}

/// CMD_VEL 的字节布局：token 在偏移 0，两个 float32 在 4 和 8，均小端。
/// token 值取自协议 10.2 的黄金向量，与 frame 层的测试互相印证。
void TestCmdVelLayout() {
  CmdVel msg;
  msg.arm_token = 0x12345678;
  msg.linear_x_mps = 1.0f;
  msg.angular_z_radps = -2.0f;

  uint8_t buf[kSizeCmdVel] = {};
  CHECK(EncodeCmdVel(msg, buf, sizeof(buf)) == kSizeCmdVel);
  // token 小端
  CHECK(buf[0] == 0x78 && buf[1] == 0x56 && buf[2] == 0x34 && buf[3] == 0x12);
  // 1.0f 的 IEEE-754 位模式为 0x3F800000，小端写入为 00 00 80 3F
  CHECK(buf[4] == 0x00 && buf[5] == 0x00 && buf[6] == 0x80 && buf[7] == 0x3F);
  // -2.0f 为 0xC0000000
  CHECK(buf[8] == 0x00 && buf[9] == 0x00 && buf[10] == 0x00 && buf[11] == 0xC0);

  CmdVel back;
  CHECK(DecodeCmdVel(buf, sizeof(buf), &back));
  CHECK(back.arm_token == msg.arm_token);
  CHECK(back.linear_x_mps == msg.linear_x_mps);
  CHECK(back.angular_z_radps == msg.angular_z_radps);
}

/// 零速 CMD_VEL 的 payload 应与协议 10.2 向量解码出的 payload 一致。
void TestZeroCmdVelPayload() {
  CmdVel msg;
  msg.arm_token = 0x12345678;
  uint8_t buf[kSizeCmdVel] = {};
  CHECK(EncodeCmdVel(msg, buf, sizeof(buf)) == kSizeCmdVel);
  const uint8_t want[kSizeCmdVel] = {0x78, 0x56, 0x34, 0x12, 0, 0, 0, 0, 0, 0, 0, 0};
  CHECK(std::memcmp(buf, want, sizeof(want)) == 0);
}

/// NaN/Inf 在两个方向都必须被拒绝，绝不能进入控制环。
void TestNonFiniteRejected() {
  uint8_t buf[kSizeCmdVel] = {};

  CmdVel nan_linear;
  nan_linear.linear_x_mps = std::numeric_limits<float>::quiet_NaN();
  CHECK(EncodeCmdVel(nan_linear, buf, sizeof(buf)) == 0);

  CmdVel inf_angular;
  inf_angular.angular_z_radps = std::numeric_limits<float>::infinity();
  CHECK(EncodeCmdVel(inf_angular, buf, sizeof(buf)) == 0);

  // 收到含 NaN 的 CMD_VEL 也要拒绝（上位机同时充当假下位机时会走这条路）。
  const uint8_t nan_payload[kSizeCmdVel] = {0, 0, 0, 0, 0x00, 0x00, 0xC0, 0x7F, 0, 0, 0, 0};
  CmdVel out;
  CHECK(!DecodeCmdVel(nan_payload, sizeof(nan_payload), &out));

  // 遥测里的非有限值同样拒绝整帧，避免污染里程计。
  uint8_t odom[kSizeOdomState] = {};
  OdomState valid;
  valid.status_flags = kOdomValid;
  CHECK(EncodeOdomState(valid, odom, sizeof(odom)) == kSizeOdomState);
  OdomState decoded;
  CHECK(DecodeOdomState(odom, sizeof(odom), &decoded));
  odom[24] = 0x00;
  odom[25] = 0x00;
  odom[26] = 0x80;
  odom[27] = 0x7F;  // x_m = +Inf
  CHECK(!DecodeOdomState(odom, sizeof(odom), &decoded));
}

/// 长度必须严格相等：多一个或少一个字节都要拒绝，不能尽力解析。
void TestStrictLength() {
  uint8_t buf[kSizeSystemStatus + 1] = {};
  SystemStatus status;
  status.remote_state = RemoteState::kArmed;
  CHECK(EncodeSystemStatus(status, buf, sizeof(buf)) == kSizeSystemStatus);

  SystemStatus out;
  CHECK(DecodeSystemStatus(buf, kSizeSystemStatus, &out));
  CHECK(!DecodeSystemStatus(buf, kSizeSystemStatus - 1, &out));
  CHECK(!DecodeSystemStatus(buf, kSizeSystemStatus + 1, &out));

  // 缓冲不足时编码函数返回 0，不做部分写入。
  CHECK(EncodeSystemStatus(status, buf, kSizeSystemStatus - 1) == 0);
}

void TestHelloInfoRoundTrip() {
  HelloInfo msg;
  msg.protocol_version = 1;
  msg.fw_major = 2;
  msg.fw_minor = 3;
  msg.fw_patch = 4;
  msg.capabilities = kCapMotor | kCapImu;
  msg.boot_id = 0xDEADBEEF;
  msg.config_valid = 1;
  msg.remote_state = RemoteState::kDisarmed;
  msg.peer_version_echo = 1;

  uint8_t buf[kSizeHelloInfo] = {};
  CHECK(EncodeHelloInfo(msg, buf, sizeof(buf)) == kSizeHelloInfo);
  // capabilities 在偏移 4，boot_id 在偏移 8，config_valid 在 12。
  CHECK(buf[4] == 0x05);
  CHECK(buf[8] == 0xEF && buf[11] == 0xDE);
  CHECK(buf[12] == 1);
  CHECK(buf[15] == 0);  // reserved

  HelloInfo back;
  CHECK(DecodeHelloInfo(buf, sizeof(buf), &back));
  CHECK(back.boot_id == msg.boot_id);
  CHECK(back.capabilities == msg.capabilities);
  CHECK(back.config_valid == 1);
  CHECK(back.remote_state == RemoteState::kDisarmed);
  CHECK(back.fw_major == 2 && back.fw_minor == 3 && back.fw_patch == 4);
}

void TestAckRoundTrip() {
  Ack msg;
  msg.request_sequence = 0x1234;
  msg.request_type = static_cast<uint8_t>(MsgType::kArmRequest);
  msg.result = AckResult::kDeniedConfig;
  msg.arm_token = 0xA1B2C3D4;

  uint8_t buf[kSizeAck] = {};
  CHECK(EncodeAck(msg, buf, sizeof(buf)) == kSizeAck);
  CHECK(buf[0] == 0x34 && buf[1] == 0x12);
  CHECK(buf[2] == 0x10);
  CHECK(buf[3] == 3);

  Ack back;
  CHECK(DecodeAck(buf, sizeof(buf), &back));
  CHECK(back.request_sequence == msg.request_sequence);
  CHECK(back.request_type == msg.request_type);
  CHECK(back.result == AckResult::kDeniedConfig);
  CHECK(back.arm_token == msg.arm_token);
}

void TestTimeSyncRoundTrip() {
  TimeSyncReq req;
  req.t1_host_ns = 0x0102030405060708ull;
  uint8_t req_buf[kSizeTimeSyncReq] = {};
  CHECK(EncodeTimeSyncReq(req, req_buf, sizeof(req_buf)) == kSizeTimeSyncReq);
  CHECK(req_buf[0] == 0x08 && req_buf[7] == 0x01);
  TimeSyncReq req_back;
  CHECK(DecodeTimeSyncReq(req_buf, sizeof(req_buf), &req_back));
  CHECK(req_back.t1_host_ns == req.t1_host_ns);

  TimeSyncResp resp;
  resp.t1_host_ns = req.t1_host_ns;
  resp.t2_mcu_rx_us = 1000;
  resp.t3_mcu_tx_us = 1200;
  uint8_t resp_buf[kSizeTimeSyncResp] = {};
  CHECK(EncodeTimeSyncResp(resp, resp_buf, sizeof(resp_buf)) == kSizeTimeSyncResp);
  TimeSyncResp resp_back;
  CHECK(DecodeTimeSyncResp(resp_buf, sizeof(resp_buf), &resp_back));
  CHECK(resp_back.t1_host_ns == resp.t1_host_ns);
  CHECK(resp_back.t2_mcu_rx_us == 1000);
  CHECK(resp_back.t3_mcu_tx_us == 1200);
}

/// 编码器计数是有符号的，倒车时会变负，必须能正确往返。
void TestOdomRoundTripWithNegativeTicks() {
  OdomState msg;
  msg.left_ticks = -123456789LL;
  msg.right_ticks = 987654321LL;
  msg.left_speed_mps = -0.5f;
  msg.right_speed_mps = 0.5f;
  msg.x_m = 1.25f;
  msg.y_m = -2.5f;
  msg.yaw_rad = 3.125f;
  msg.linear_mps = 0.25f;
  msg.angular_radps = -0.125f;
  msg.path_length_m = 12.5f;
  msg.status_flags = kOdomValid | kOdomImuFused;

  uint8_t buf[kSizeOdomState] = {};
  CHECK(EncodeOdomState(msg, buf, sizeof(buf)) == kSizeOdomState);
  // status_flags 在偏移 48。
  CHECK(buf[48] == 0x03);

  OdomState back;
  CHECK(DecodeOdomState(buf, sizeof(buf), &back));
  CHECK(back.left_ticks == msg.left_ticks);
  CHECK(back.right_ticks == msg.right_ticks);
  CHECK(back.left_speed_mps == msg.left_speed_mps);
  CHECK(back.x_m == msg.x_m);
  CHECK(back.y_m == msg.y_m);
  CHECK(back.yaw_rad == msg.yaw_rad);
  CHECK(back.linear_mps == msg.linear_mps);
  CHECK(back.angular_radps == msg.angular_radps);
  CHECK(back.path_length_m == msg.path_length_m);
  CHECK(back.status_flags == msg.status_flags);
}

void TestImuRoundTrip() {
  ImuState msg;
  msg.accel_x = 0.5f;
  msg.accel_y = -0.25f;
  msg.accel_z = 9.75f;
  msg.gyro_x = 0.125f;
  msg.gyro_y = -0.0625f;
  msg.gyro_z = 1.5f;
  msg.temperature = 36.5f;
  msg.roll = 0.25f;
  msg.pitch = -0.5f;
  msg.yaw = 2.0f;
  msg.status_flags = kImuValid | kImuCalibrated;

  uint8_t buf[kSizeImuState] = {};
  CHECK(EncodeImuState(msg, buf, sizeof(buf)) == kSizeImuState);
  CHECK(buf[40] == 0x05);  // status_flags 在偏移 40

  ImuState back;
  CHECK(DecodeImuState(buf, sizeof(buf), &back));
  CHECK(back.accel_x == msg.accel_x && back.accel_z == msg.accel_z);
  CHECK(back.gyro_z == msg.gyro_z);
  CHECK(back.temperature == msg.temperature);
  CHECK(back.roll == msg.roll && back.pitch == msg.pitch && back.yaw == msg.yaw);
  CHECK(back.status_flags == msg.status_flags);

  ImuDebug dbg;
  dbg.velocity_x_mps = 0.5f;
  dbg.velocity_y_mps = -0.5f;
  dbg.position_x_m = 2.0f;
  dbg.position_y_m = -2.0f;
  dbg.yaw_rad = 1.0f;
  dbg.status_flags = kImuValid | kImuNotForNavigation;
  uint8_t dbg_buf[kSizeImuDebug] = {};
  CHECK(EncodeImuDebug(dbg, dbg_buf, sizeof(dbg_buf)) == kSizeImuDebug);
  CHECK(dbg_buf[20] == 0x09);
  ImuDebug dbg_back;
  CHECK(DecodeImuDebug(dbg_buf, sizeof(dbg_buf), &dbg_back));
  CHECK(dbg_back.position_x_m == dbg.position_x_m);
  CHECK((dbg_back.status_flags & kImuNotForNavigation) != 0);
}

void TestSystemStatusRoundTrip() {
  SystemStatus msg;
  msg.remote_state = RemoteState::kArmed;
  msg.config_valid = 1;
  msg.command_saturated = 1;
  msg.fault_code = 0x11223344;
  msg.uptime_ms = 60000;
  msg.command_age_ms = 20;
  msg.crc_errors = 1;
  msg.format_errors = 2;
  msg.rx_overflows = 3;
  msg.tx_drops = 4;
  msg.arm_token = 0xCAFEBABE;

  uint8_t buf[kSizeSystemStatus] = {};
  CHECK(EncodeSystemStatus(msg, buf, sizeof(buf)) == kSizeSystemStatus);
  CHECK(buf[0] == 3);  // REMOTE_ARMED
  CHECK(buf[3] == 0);  // reserved
  // arm_token 在偏移 32，是上位机取 token 的唯一来源，偏移错了整条控制链路都起不来。
  CHECK(buf[32] == 0xBE && buf[35] == 0xCA);

  SystemStatus back;
  CHECK(DecodeSystemStatus(buf, sizeof(buf), &back));
  CHECK(back.remote_state == RemoteState::kArmed);
  CHECK(back.arm_token == 0xCAFEBABE);
  CHECK(back.command_age_ms == 20);
  CHECK(back.rx_overflows == 3);
  CHECK(back.tx_drops == 4);
}

/// 无有效命令时 command_age_ms 是 0xFFFFFFFF，不能被当成 42 亿毫秒。
void TestCommandAgeSentinel() {
  SystemStatus msg;
  msg.command_age_ms = kCommandAgeNone;
  uint8_t buf[kSizeSystemStatus] = {};
  CHECK(EncodeSystemStatus(msg, buf, sizeof(buf)) == kSizeSystemStatus);
  SystemStatus back;
  CHECK(DecodeSystemStatus(buf, sizeof(buf), &back));
  CHECK(back.command_age_ms == kCommandAgeNone);
}

void TestFaultEventRoundTrip() {
  FaultEvent msg;
  msg.fault_code = 0x00000004;
  msg.timestamp_ms = 123456;
  msg.detail = 0x99;
  uint8_t buf[kSizeFaultEvent] = {};
  CHECK(EncodeFaultEvent(msg, buf, sizeof(buf)) == kSizeFaultEvent);
  FaultEvent back;
  CHECK(DecodeFaultEvent(buf, sizeof(buf), &back));
  CHECK(back.fault_code == msg.fault_code);
  CHECK(back.timestamp_ms == msg.timestamp_ms);
  CHECK(back.detail == msg.detail);
}

void TestArmRequestRoundTrip() {
  ArmRequest msg;
  msg.boot_id = 0x01020304;
  uint8_t buf[kSizeArmRequest] = {};
  CHECK(EncodeArmRequest(msg, buf, sizeof(buf)) == kSizeArmRequest);
  CHECK(buf[0] == 0x04 && buf[3] == 0x01);
  ArmRequest back;
  CHECK(DecodeArmRequest(buf, sizeof(buf), &back));
  CHECK(back.boot_id == msg.boot_id);
}

/// 未知枚举值必须映射到保守取值：未知状态不能被当成 ARMED，未知结果码不能被当成 OK。
void TestUnknownEnumsAreConservative() {
  uint8_t status[kSizeSystemStatus] = {};
  status[0] = 99;
  SystemStatus out;
  CHECK(DecodeSystemStatus(status, sizeof(status), &out));
  CHECK(out.remote_state == RemoteState::kDisconnected);

  uint8_t ack[kSizeAck] = {};
  ack[3] = 200;
  Ack ack_out;
  CHECK(DecodeAck(ack, sizeof(ack), &ack_out));
  CHECK(ack_out.result == AckResult::kUnsupported);
}

}  // namespace

int main() {
  TestMsgTypeValues();
  TestCmdVelLayout();
  TestZeroCmdVelPayload();
  TestNonFiniteRejected();
  TestStrictLength();
  TestHelloInfoRoundTrip();
  TestAckRoundTrip();
  TestTimeSyncRoundTrip();
  TestOdomRoundTripWithNegativeTicks();
  TestImuRoundTrip();
  TestSystemStatusRoundTrip();
  TestCommandAgeSentinel();
  TestFaultEventRoundTrip();
  TestArmRequestRoundTrip();
  TestUnknownEnumsAreConservative();
  return uart::test::Finish("codec");
}
