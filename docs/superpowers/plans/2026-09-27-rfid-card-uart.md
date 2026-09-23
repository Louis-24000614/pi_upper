# RFID_CARD UART Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [x]`) syntax for tracking.

**Goal:** 把下位机 UART5 已经解析出的读卡结果，经现有 USART2 协议发给上位机。

**Architecture:** 协议标识保持 1。新增一条 MCU→主机遥测 `0x95 RFID_CARD`，3 字节，无 ACK。下位机在 `CommTxTask` 里按 100 ms 调用 `RfidTask_GetLastCard()` 组帧，世代号只在发送侧累加。上位机 `uart` 模块只解码并保存最近一帧，不播报、不计分。

**Tech Stack:** RC 固件 C（STM32H743，现有 `ros_link` / `CommTxTask`）；pi_upper `uart/` C++（`proto` + `link/sess`，CTest）。

## Global Constraints

- 帧格式不变：`55 AA | TYPE | LENGTH | PAYLOAD | CRC8`，CRC-8/ATM，多字节小端。
- 协议标识保持 `1`，不增加 capability 位，不改 `HELLO_INFO`。
- 不改 `RfidTask_GetLastCard(uint8_t *card_number)` 的签名。卡号是扇区 0、Block 1 的第 0 字节，有效值 `0x01`–`0x0C`。
- 不接喇叭，不改 `Speaker_Speak`，不接任务计分、GUI、`Tools/ros_protocol.py`。
- 不把 UID、ATQA、SAK 放进这帧。
- 不加新文件。改动落在现有协议定义、组帧和会话接收上。
- 未得到用户明确要求时不提交 git。

---

### Task 1: 下位机定义并发送 `0x95`

**Files:**
- Modify: `/home/orangepi/RC/Components/Inc/uart_protocol.h`
- Modify: `/home/orangepi/RC/App/Inc/ros_link.h`
- Modify: `/home/orangepi/RC/App/Src/ros_link.c`
- Modify: `/home/orangepi/RC/App/Src/app_tasks.c`（`CommTxTask`，约 388–432 行）
- Modify: `/home/orangepi/RC/UART_MESSAGES.md`
- Modify: `/home/orangepi/RC/UART_PROTOCOL.md`（第 7 节 MCU 发送消息末尾）

**Interfaces:**
- Consumes: `uint8_t RfidTask_GetLastCard(uint8_t *card_number)`，返回 1 表示有卡。
- Produces: `bool RosLink_BuildRfidCard(uint64_t now_us, uint8_t *encoded, size_t capacity, size_t *length)`。线上 TYPE=`0x95`，PAYLOAD 长度 3：`[present, card_number, generation]`。

- [x] **Step 1: 在消息枚举里加 ID**

`UartMessageType` 在 `UART_MSG_SYSTEM_STATUS` 后增加：

```c
UART_MSG_RFID_CARD = 0x95       /* MCU -> 主机：读卡结果，3 字节。 */
```

- [x] **Step 2: 在 `ros_link.h` / `ros_link.c` 里维护世代号并组帧**

`ros_link.h` 在 `RosLink_BuildStatus` 声明后面加上 `RosLink_BuildRfidCard`，参数与 `RosLink_BuildOdometry` 相同：`now_us`、`encoded`、`capacity`、`length`。

`ros_link.c` 文件顶部增加静态状态，`RosLink_Init` 里清零。世代号规则：

- `present==1` 且 `card_number` 在 1–12，并且当前没有握着有效卡，或卡号与上次不同：`generation` 加 1（`uint8_t` 自然回绕），记下该卡号。
- 同一张有效卡继续在场：不加。
- `present==1` 且卡号为 0：不加，也不要清除「正在握着的有效卡」。
- `present==0`：不加，清除「正在握着的有效卡」。

```c
static uint8_t s_rfid_generation;
static uint8_t s_rfid_last_number;
static uint8_t s_rfid_holding;

static void observe_rfid(uint8_t present, uint8_t card_number) {
  uint8_t valid = (present != 0U) && (card_number >= 1U) && (card_number <= 12U);
  if (valid != 0U) {
    if ((s_rfid_holding == 0U) || (card_number != s_rfid_last_number)) {
      ++s_rfid_generation;
      s_rfid_last_number = card_number;
      s_rfid_holding = 1U;
    }
  } else if (present == 0U) {
    s_rfid_holding = 0U;
  }
}

bool RosLink_BuildRfidCard(uint64_t now_us, uint8_t *encoded, size_t capacity,
                           size_t *length) {
  uint8_t card_number = 0U;
  uint8_t present = RfidTask_GetLastCard(&card_number);
  uint8_t payload[3];
  (void)now_us;
  if (present == 0U) card_number = 0U;
  observe_rfid(present, card_number);
  payload[0] = present ? 1U : 0U;
  payload[1] = card_number;
  payload[2] = s_rfid_generation;
  return encode_message(UART_MSG_RFID_CARD, payload, sizeof(payload), now_us,
                        encoded, capacity, length);
}
```

`ros_link.c` 增加 `#include "rfid_task.h"`。`RosLink_Init` 在现有清零处加上 `s_rfid_generation = s_rfid_last_number = s_rfid_holding = 0U`。

- [x] **Step 3: 插进 `CommTxTask` 的现有优先级链**

在系统状态分支之后、里程计分支之前。周期 100 ms，和 `SYSTEM_STATUS` 相同。状态到期时仍先发状态，下一轮循环再发读卡。

```c
uint64_t last_status = 0U, last_rfid = 0U, last_odom = 0U, last_imu = 0U, last_debug = 0U;
```

```c
} else if (current - last_status >= 100000ULL) {
  built = RosLink_BuildStatus(...);
  last_status = current;
} else if (current - last_rfid >= 100000ULL) {
  built = RosLink_BuildRfidCard(current, encoded, sizeof(encoded), &length);
  last_rfid = current;
} else if (current - last_odom >= 20000ULL) {
```

读卡帧不进 ACK 队列。未 ARM 也发送。发送失败时沿用现有的 `retry_pending`，重试同一缓冲，不会把世代号再加一次。

- [x] **Step 4: 协议文档各加一小节**

`UART_MESSAGES.md` 总览表加一行：`0x95 RFID_CARD`，MCU→主机，3 字节，10 Hz。正文说明三个字段、世代号规则，以及卡号来自扇区 0 Block 1 第 0 字节（`0x01`–`0x0C`）。发送优先级写成：ACK / HELLO_INFO > SYSTEM_STATUS > RFID_CARD > ODOM_STATE > IMU_STATE > IMU_DEBUG。

`UART_PROTOCOL.md` 第 7 节末尾用同样的 3 字节表记一笔，不改帧格式那一节。

- [x] **Step 5: 确认固件侧没有新的主机测试要跑**

世代号依赖 `RfidTask_GetLastCard`，现有 `Tests/protocol_vectors.c` 不链接 `ros_link`。这一步不新增 RC 主机测试。检查改动文件能通过阅读对上：ID 为 `0x95`，组帧长度为 3，`CommTxTask` 只有这一处新分支。

---

### Task 2: 上位机接收 `0x95` 并保存最近一帧

**Files:**
- Modify: `/home/orangepi/pi_upper/uart/proto/msg.h`
- Modify: `/home/orangepi/pi_upper/uart/proto/codec.h`
- Modify: `/home/orangepi/pi_upper/uart/proto/codec.cpp`
- Modify: `/home/orangepi/pi_upper/uart/link/sess.h`（`Telemetry`）
- Modify: `/home/orangepi/pi_upper/uart/link/sess.cpp`（`OnFrame`）
- Modify: `/home/orangepi/pi_upper/uart/tests/proto/codec_test.cpp`
- Modify: `/home/orangepi/pi_upper/uart/tests/mock/mcu.h`
- Modify: `/home/orangepi/pi_upper/uart/tests/link/sess_test.cpp`
- Modify: `/home/orangepi/pi_upper/docs/api/uart.md`（线协议那一节的收包列表）
- Modify: `/home/orangepi/pi_upper/docs/reference/comm/uart.md`（消息清单里补一条）

**Interfaces:**
- Consumes: Task 1 的线上字节：TYPE `0x95`，长度必须为 3。
- Produces:
  - `enum class MsgType { kRfidCard = 0x95 }`
  - `struct RfidCard { uint8_t present; uint8_t card_number; uint8_t generation; }`
  - `constexpr size_t kSizeRfidCard = 3`
  - `size_t EncodeRfidCard(const RfidCard&, uint8_t*, size_t)`
  - `bool DecodeRfidCard(const uint8_t*, size_t, RfidCard*)`
  - `Telemetry::rfid`、`rfid_us`、`has_rfid`

- [x] **Step 1: 先写会失败的编解码测试**

在 `codec_test.cpp` 的 `static_assert` 和 `TestMsgTypeValues` 里加上 `kSizeRfidCard == 3`、`MsgType::kRfidCard == 0x95`。再加：

```cpp
void TestRfidCardLayout() {
  RfidCard msg;
  msg.present = 1;
  msg.card_number = 12;
  msg.generation = 2;
  uint8_t buf[kSizeRfidCard] = {};
  CHECK(EncodeRfidCard(msg, buf, sizeof(buf)) == kSizeRfidCard);
  CHECK(buf[0] == 1 && buf[1] == 12 && buf[2] == 2);
  RfidCard out;
  CHECK(DecodeRfidCard(buf, kSizeRfidCard, &out));
  CHECK(out.present == 1 && out.card_number == 12 && out.generation == 2);
  CHECK(!DecodeRfidCard(buf, kSizeRfidCard - 1, &out));
  CHECK(!DecodeRfidCard(buf, kSizeRfidCard + 1, &out));
  CHECK(EncodeRfidCard(msg, buf, 2) == 0);
}
```

在 `main` 里调用 `TestRfidCardLayout()`。

- [x] **Step 2: 跑测试，确认现在编不过**

在 `/home/orangepi/pi_upper` 执行：

```bash
cmake -B build -DCMAKE_BUILD_TYPE=Debug
cmake --build build -j --target uart_codec
ctest --test-dir build -R uart_codec --output-on-failure
```

Expected: 编译失败，提示 `kSizeRfidCard` 或 `EncodeRfidCard` 不存在。

- [x] **Step 3: 补上消息定义和编解码**

`msg.h` 的 `MsgType` 在 `kMotionResult` 后加 `kRfidCard = 0x95`。文件末尾加：

```cpp
/// RFID_CARD (0x95)。UART5 读卡结果的最近一帧。
struct RfidCard {
  uint8_t present = 0;
  uint8_t card_number = 0;
  uint8_t generation = 0;
};
```

`codec.h` 加 `kSizeRfidCard = 3`、`EncodeRfidCard`、`DecodeRfidCard`。`codec.cpp` 按字节读写，长度不等于 3 时解码返回 false，缓冲不足时编码返回 0。不做数值范围修正：卡号是否在 1–12 由以后的任务层判断。

- [x] **Step 4: 再跑 `uart_codec`，期望通过**

- [x] **Step 5: 会话层保存最近一帧**

`Telemetry` 按里程计的写法增加 `RfidCard rfid`、`uint64_t rfid_us`、`bool has_rfid`。`OnFrame` 增加：

```cpp
case MsgType::kRfidCard:
  if (DecodeRfidCard(payload, len, &telemetry_.rfid)) {
    telemetry_.rfid_us = now_us;
    telemetry_.has_rfid = true;
  }
  break;
```

长度不对的帧丢掉，不改动上一帧。`DropSession` 不额外清这帧，与里程计相同。

`mcu.h` 增加 `SendRfidCard`，照 `SendOdom` 的写法。`sess_test.cpp` 增加 `TestRfidCardCaptured`：连上后送 `present=1, card_number=3, generation=1`，检查 `telemetry().has_rfid` 和三个字段；再送一帧长度不为 3 的 `0x95`，确认字段仍是上一帧。在 `main` 里调用它。

- [x] **Step 6: 跑会话测试**

```bash
cmake --build build -j --target uart_codec uart_sess
ctest --test-dir build -R 'uart_codec|uart_sess' --output-on-failure
```

Expected: 两个测试进程返回 0。

- [x] **Step 7: 上位机文档只加收包说明**

`docs/api/uart.md` 的收包列表加上 `RFID_CARD`。`docs/reference/comm/uart.md` 的消息说明加上 `0x95`、3 字节、保存在 `Telemetry::rfid`。写明本模块不播报、不计分。

---

## 不做的事

- 喇叭和 `ARRIVE_1`–`ARRIVE_12`。
- 上位机任务层的「到达 X 号」和已访记录。三次比赛的清空也留到那一层。
- `HELLO_INFO.capabilities` 的新位。
- RC 的 Python 键盘调试工具。
- 把 UART5 的原始读卡器帧转发到 USART2。
