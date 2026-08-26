# Uart

## Contents

- [Overview](#overview)
- [Architecture](#architecture)
- [API](#api)
- [Design](#design)
- [Testing](#testing)

## Overview

`uart/` 负责上位机与 STM32H743 下位机之间的全部串口通信：帧的编解码、会话与 ARM 状态机、50 Hz 速度下发、遥测解析与诊断统计。

模块边界很硬：本模块只做协议与链路，不做任何业务决策。速度指令从哪来（自主导航还是手动遥控）由 `state` 仲裁，这里拿到的是已经定好的目标速度；遥测解析出来的里程计和姿态原样抛给上层，不做滤波、不做坐标变换、不做规划。

依赖只有标准库和 POSIX termios。Qt 仅用于对外的信号槽接口，协议核心不链接 Qt，保证能在无 Qt 环境下单测。

线协议见下位机仓库的 `UART_PROTOCOL.md`（唯一权威），上位机侧的实现约定见 `docs/api/uart.md`。

## Architecture

目录按职责分两层，边界在文件树上直接可见：

```text
uart/
├── uart.h / .cpp        对外唯一入口，唯一依赖 Qt 的一层（待实现）
├── proto/               纯协议：无 IO、无状态、无时间、无 Qt
│   ├── crc32c.h/.cpp
│   ├── cobs.h/.cpp
│   ├── frame.h/.cpp
│   ├── msg.h
│   ├── codec.h/.cpp
│   └── detail/bytes.h   私有实现细节，禁止跨模块 include
├── link/                有状态、碰操作系统的部分
│   ├── port.h/.cpp
│   ├── clock.h/.cpp
│   └── sess.h/.cpp
└── tests/               镜像上面的结构，另有 mock/ 放假传输与假下位机
```

`proto/` 只依赖标准库，可以整个搬到别的项目、也能编进单片机。`link/` 依赖 POSIX 与系统时钟。构建上对应两个目标 `uart_proto` 与 `uart_link`，后者依赖前者；反向依赖会直接链接失败，所以分层不只是约定，是构建系统兜住的。proto 层的测试只链 `uart_proto`，纯协议代码里一旦悄悄用了串口或时钟就会编不过。

include 路径统一取模块根，源码里写成 `#include "proto/frame.h"`、`#include "link/sess.h"`，从引用处就能看出跨了哪一层。

依赖方向：

```mermaid
flowchart TB
  facade["uart<br/>线程 + Qt 信号槽（待实现）"]
  sess["link/sess<br/>会话状态机：建链 · ARM · token · 看门狗 · 重连"]
  clock["link/clock<br/>单调时钟接口"]
  port["link/port<br/>Transport 接口 · termios 串口"]
  codec["proto/codec + proto/msg<br/>消息序列化，逐字节小端"]
  frame["proto/frame<br/>帧头 · 长度校验 · CRC32C · 收帧状态机"]
  cobs["proto/cobs"]
  crc["proto/crc32c"]
  facade --> sess
  sess --> codec
  sess --> port
  sess --> clock
  codec --> frame
  frame --> cobs
  frame --> crc
```

线程模型：一条通信线程同时承担接收解帧与 50 Hz 周期发送。接收侧从 `Transport` 读到字节就喂给分帧状态机，解出完整帧后解码并通过信号抛给上层，不在通信线程里做业务处理。发送侧按 `steady_clock` 维护 50 Hz 节拍，每拍取最新的目标速度发一帧 `CMD_VEL`。

上层写入目标速度、读取最新遥测都走加锁的最新值邮箱，不排队——速度指令是覆盖式的，历史值没有意义。

## API

公开接口的完整说明以各头文件的 Doxygen 注释为准，这里只列已实现的入口。

`proto/crc32c.h` — `Crc32c()` 一次性计算；`Crc32cUpdate()` / `Crc32cFinish()` 分段计算，用于帧头与 payload 不连续存放时避免额外拷贝。

`proto/cobs.h` — `CobsEncodeBound()` 给出最坏情况缓冲大小，`CobsEncode()` / `CobsDecode()` 做编解码，返回写入字节数，0 表示失败。输出不含结尾分隔符，分隔符由帧层追加。

`proto/frame.h` — 协议常量（`kMagic`、`kHeaderSize`、`kMaxPayloadSize`、`kMaxEncodedSize` 等）、`Header` 结构体、`FrameError` 错误分类、`EncodeFrame()` 装配整帧、`DecodeRawFrame()` 校验已解码的原始帧、`Reassembler` 按 `0x00` 边界收帧并维护 `Stats` 统计（帧数、CRC 错误、格式错误、溢出）。

`proto/msg.h` — 消息 ID（`MsgType`）、结果码（`AckResult`）、远程状态（`RemoteState`）、能力位与状态位常量，以及 11 个有 payload 的消息的进程内结构体。只有数据定义，没有逻辑。

`proto/codec.h` — 各消息 payload 长度常量与字段级编解码。编码返回写入字节数，0 表示缓冲不足或字段非法；解码返回 bool，要求长度严格相等。MCU→主机方向也提供编码函数，供测试与仿真里的"假下位机"构造遥测帧。

`link/port.h` — `Transport` 抽象接口（`Read`/`Write`/`WaitReadable`，-1 表示链路损坏应重连），以及 termios 实现 `SerialPort`（raw 模式、8N1、无流控、打开时清空残留缓冲）。

`link/clock.h` — `Clock` 单调时间接口与 `SteadyClock` 实现。抽出接口是为了让超时与节拍能用假时钟测。

`link/sess.h` — `Session` 会话状态机，模块的主要对外面。`Start()` 进入建链，`Poll()` 由通信线程反复调用，`SetVelocity()` 写入覆盖式目标速度，`RequestArm()` / `RequestDisarm()` / `RequestResetOdom()` / `RequestClearFault()` 是管理命令，`Shutdown()` 做零速加 DISARM 收尾。状态查询有 `link_state()`、`remote_state()`、`arm_token()`、`config_valid()`，数据查询有 `telemetry()`、`time_sync()`、`diagnostics()`。时序参数集中在 `SessionConfig`。

`proto/detail/bytes.h` — 模块内部的小端序读写辅助，不对外暴露。

尚未实现：`uart.h/cpp` 的通信线程与 Qt 信号槽外壳。`Session` 本身已经是完整可用的，只是需要一个线程来驱动 `Poll()` 并把遥测转成信号。

单位一律沿用协议约定：距离 m、速度 m/s、角度 rad、角速度 rad/s。坐标系为 REP-103 的 `base_link`——X 向前、Y 向左、Z 向上，正角速度表示左转。

## Design

**分层动机**：COBS 和 CRC32C 是纯函数，帧层只关心字节布局，消息层只关心字段语义，会话层才涉及状态与时间。这样黄金测试向量可以直接打在帧层，安全逻辑的错误可以在会话层用假时钟复现，不需要真串口。

**不 memcpy 结构体**：协议明确禁止把编译器结构体直接上线。所有字段逐字节读写，避免对齐、填充和 ABI 差异导致两端布局不一致。

**50 Hz 独立线程而非 Qt 定时器**：Linux 非实时，Qt 事件循环被 UI 或推理拖慢时定时器抖动可能超过下位机 250 ms 的命令看门狗，直接触发安全停车。用独立线程加单调时钟补偿周期更可控。

**指令有效期**：上层给的速度带本地有效期，超期后本模块主动改发零速而不是保持旧值。低频"最后速度保持"在这个协议下是错误用法。

**token 生命周期**：`boot_id` 变化（下位机复位）或链路重连都会让旧 token 失效，此时必须清空控制状态重新建链。本模块不缓存跨会话的 token。

**已知限制**：香橙派侧的 UART 针脚编号与设备节点尚未确定，硬件联调前只能跑 fake 传输层。下位机当前 `config_valid=0`，ARM 会返回 `ACK_DENIED_CONFIG`，运动控制要等对方绑定电机、编码器引脚和底盘参数后才能验证。

## Testing

方法为**带 mock 的单元测试**加**回放式的协议向量比对**，全部不需要硬件。测试不引入 GoogleTest，用 `tests/check.h` 里的极简断言，靠 CTest 判定进程返回值。

运行方式（在仓库根执行）：

```bash
cmake -B build -DCMAKE_BUILD_TYPE=Debug
cmake --build build -j
ctest --test-dir build --output-on-failure
```

判定标准是三个测试进程全部返回 0，任何断言失败都会打印文件行号与实际/期望的十六进制字节。

**已实现并通过**（`uart_crc32c`、`uart_cobs`、`uart_frame`、`uart_codec`、`uart_port`、`uart_sess` 六项全绿）：

CRC-32C 用协议给定的 `123456789 -> 0xE3069283` 自检，同时验证分段计算与一次性计算等价、单比特翻转会改变结果。

COBS 覆盖空输入、全零、混合数据的编解码往返，`0xFF` 满块前后（253/254/255 字节）的边界，以及对内部含 `0x00`、码字跨界、缓冲不足这三类非法输入的拒绝。

帧层用 `UART_PROTOCOL.md` 第 10 节的三条黄金向量做字节级比对：`HELLO_REQ` 与零速 `CMD_VEL` 的编码结果必须与文档完全一致，第三条故意保留旧 CRC 的坏帧必须计入 `crc_errors` 且不触发任何回调。另外覆盖逐字节喂入与整块喂入等价、0 到 `kMaxPayloadSize` 全部长度的往返、超长 payload 拒绝、魔数/reserved/长度错误的分类、空帧不计错误、超长垃圾数据计一次溢出并在下一个分隔符处重新同步。

消息层的 payload 长度用 `static_assert` 锁死，改错编不过。字段偏移不能只靠编解码往返验证——偏移写错时往返仍然自洽，所以对照协议文档硬编码了关键字节位置：`CMD_VEL` 的 token 与两个 float 的 IEEE-754 位模式、`HELLO_INFO` 的 `capabilities`/`boot_id`/`config_valid`、`SYSTEM_STATUS` 偏移 32 的 `arm_token`、`ODOM_STATE` 偏移 48 与 `IMU_STATE` 偏移 40 的 `status_flags`。此外覆盖负编码器计数的往返、长度多一字节或少一字节均拒绝、NaN/Inf 在下发与接收两个方向都被拒绝、`command_age_ms` 的 `0xFFFFFFFF` 哨兵值不被当成数值、未知枚举值保守映射（未知状态不得变成 ARMED，未知结果码不得变成 OK）。

传输层的 termios 路径用**伪终端**测：真串口设备当前不存在，但 PTY 走同一套 termios 配置，raw 模式、二进制透传和 poll 行为都能覆盖，只有波特率和电平这类物理特性测不到。关键一条是让 `0x00`、`0x0A`、`0x0D` 和 XON/XOFF 的 `0x11`/`0x13` 原样往返——这些字节一旦被终端层改写或吞掉，表现就是随机丢帧，极难定位。此外覆盖无数据时 `WaitReadable` 超时、设备不存在与波特率不支持的失败路径、未打开时读写返回错误。内存 fake 另测了分片读取和读写失败注入。

会话层用假传输、假时钟和一个"假下位机"做一问一答式的交互测试，20 个用例，每条对应协议里一条明确要求：HELLO 重发直到收到 `HELLO_INFO`；`config_valid=0` 与未建链时都不得发出 `ARM_REQUEST`；`ARM_REQUEST` 必须携带本次 `boot_id`；未持有 token 时不发速度帧；拿到 token 后 100 ms 内恰好发 5 帧（50 Hz）且 token 与速度正确；指令超过本地有效期后改发零速而不是保持旧值、也不是停发；非有限速度在写入时就被拒；`boot_id` 变化清 token 并停止下发；下位机报告非 ARMED 时同步清 token；静默超过 1 s 掉线重连；传输报错关闭链路并清会话；`Shutdown()` 发 3 帧零速且 `DISARM` 排在最后；ARMED 下拒绝 `RESET_ODOM`；遥测与故障事件正确解析并带接收时刻；时间同步能估出 500 ms 的人为偏移，往返时延过大的样本被丢弃；协议主版本不匹配时不进入已连接；协议 10.3 的坏 CRC 帧不影响会话状态且计入诊断；逐字节分片读取下行为不变。

**待实现**：`uart.h/cpp` 线程外壳的测试（线程启停与信号转发的冒烟测试）。

**测试逼出来的一个实现 bug**：`Session::Poll` 原先用"短读即读空"作为读取循环的结束条件。非阻塞 fd 上短读只表示"此刻可用这么多"，不代表后续没有数据，因此分片较小时会丢掉同一帧的后续字节。真串口上短读通常确实等于读空，所以这个错误在硬件联调里只会表现为偶发丢帧，极难定位；是 fake 的逐字节分片模式把它逼出来的。现在改为读到返回 0 才结束，并对单次 Poll 的字节数设上限，避免对端刷数据时饿死 50 Hz 发送节拍。

**与固件源码的对照结果**：字段偏移不只对照了协议文档，还逐条比对了下位机 `App/Src/ros_link.c` 里各消息的构造代码（`enqueue_ack`、`enqueue_hello`、`RosLink_BuildOdometry` / `BuildImu` / `BuildStatus` / `BuildFault`），全部一致。

COBS 有一处文档测不出的差异：恰好 254 个非零字节时，"补末尾空块"（256 字节）和"省掉空块"（255 字节）两种写法都能自洽往返，但字节流不同，必须与固件一致才能互通。已查 `Components/Src/uart_protocol.c` 的 `UartProtocol_CobsEncode`，确认为前者，本实现与测试都按前者。黄金向量全是短帧，测不到这个分支，改动编码器时要留意。

**Related**：[串口通信契约](../../api/uart.md) · [总体架构](../../architecture/overview.md)
