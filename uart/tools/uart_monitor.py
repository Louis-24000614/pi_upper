#!/usr/bin/env python3
"""持续监视香橙派与 STM32 之间的二进制 UART 链路。

本工具只周期发送 HELLO_REQ，不发送 ARM、速度或运动动作，适合静态排查
USART2 的双向通信、丢帧和 RFID 上报。运行时不要同时启动 uart_vel。
"""

from __future__ import annotations

import argparse
import collections
import errno
import os
import select
import struct
import sys
import termios
import time
import tty
from dataclasses import dataclass


SYNC = b"\x55\xaa"
MAX_PAYLOAD = 128
HELLO_REQ = 0x01
ACK = 0x80
HELLO_INFO = 0x81
ODOM_STATE = 0x90
IMU_STATE = 0x91
IMU_DEBUG = 0x92
SYSTEM_STATUS = 0x93
MOTION_RESULT = 0x94
RFID_CARD = 0x95

TYPE_NAMES = {
    ACK: "ACK",
    HELLO_INFO: "HELLO_INFO",
    ODOM_STATE: "ODOM",
    IMU_STATE: "IMU",
    IMU_DEBUG: "IMU_DEBUG",
    SYSTEM_STATUS: "STATUS",
    MOTION_RESULT: "MOTION_RESULT",
    RFID_CARD: "RFID",
}

REMOTE_STATES = {0: "DISABLED", 1: "READY", 2: "ARMED", 3: "FAULT"}
ACK_RESULTS = {
    0: "OK",
    1: "DENIED_STATE",
    2: "DENIED_CONFIG",
    3: "BAD_PAYLOAD",
    4: "UNSUPPORTED",
    5: "BUSY",
    6: "VERSION_MISMATCH",
}


def crc8(data: bytes) -> int:
    """CRC-8/ATM：poly=0x07，init=0，无反射、无最终异或。"""
    value = 0
    for byte in data:
        value ^= byte
        for _ in range(8):
            value = ((value << 1) ^ 0x07) & 0xFF if value & 0x80 else (value << 1) & 0xFF
    return value


def encode_frame(message_type: int, payload: bytes) -> bytes:
    body = bytes((message_type, len(payload))) + payload
    return SYNC + body + bytes((crc8(body),))


@dataclass
class ParserStats:
    valid_frames: int = 0
    crc_errors: int = 0
    length_errors: int = 0
    discarded_bytes: int = 0


class FrameParser:
    def __init__(self) -> None:
        self.buffer = bytearray()
        self.stats = ParserStats()

    def feed(self, data: bytes) -> list[tuple[int, bytes]]:
        self.buffer.extend(data)
        frames: list[tuple[int, bytes]] = []
        while True:
            sync_at = self.buffer.find(SYNC)
            if sync_at < 0:
                # 保留结尾的单个 0x55，它可能是下一帧同步头的开头。
                keep = 1 if self.buffer.endswith(SYNC[:1]) else 0
                discarded = len(self.buffer) - keep
                if discarded > 0:
                    self.stats.discarded_bytes += discarded
                    del self.buffer[:discarded]
                return frames
            if sync_at > 0:
                self.stats.discarded_bytes += sync_at
                del self.buffer[:sync_at]
            if len(self.buffer) < 4:
                return frames
            payload_len = self.buffer[3]
            if payload_len > MAX_PAYLOAD:
                self.stats.length_errors += 1
                del self.buffer[0]
                continue
            frame_len = payload_len + 5
            if len(self.buffer) < frame_len:
                return frames
            candidate = bytes(self.buffer[:frame_len])
            if crc8(candidate[2:-1]) != candidate[-1]:
                self.stats.crc_errors += 1
                del self.buffer[0]
                continue
            del self.buffer[:frame_len]
            self.stats.valid_frames += 1
            frames.append((candidate[2], candidate[4:-1]))


def baud_constant(baud: int) -> int:
    name = f"B{baud}"
    if not hasattr(termios, name):
        raise ValueError(f"本机 termios 不支持波特率 {baud}")
    return int(getattr(termios, name))


def open_serial(device: str, baud: int) -> int:
    fd = os.open(device, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    try:
        tty.setraw(fd, termios.TCSANOW)
        attrs = termios.tcgetattr(fd)
        attrs[0] &= ~(termios.IXON | termios.IXOFF | termios.IXANY)
        attrs[2] &= ~(termios.PARENB | termios.CSTOPB | termios.CSIZE)
        if hasattr(termios, "CRTSCTS"):
            attrs[2] &= ~termios.CRTSCTS
        attrs[2] |= termios.CS8 | termios.CLOCAL | termios.CREAD
        attrs[4] = baud_constant(baud)
        attrs[5] = baud_constant(baud)
        attrs[6][termios.VMIN] = 0
        attrs[6][termios.VTIME] = 0
        termios.tcsetattr(fd, termios.TCSANOW, attrs)
        termios.tcflush(fd, termios.TCIOFLUSH)
        return fd
    except Exception:
        os.close(fd)
        raise


def timestamp(started: float) -> str:
    return f"{time.monotonic() - started:9.3f}s"


class Monitor:
    def __init__(self, fd: int, hello_interval: float, link_timeout: float) -> None:
        self.fd = fd
        self.hello_interval = hello_interval
        self.link_timeout = link_timeout
        self.parser = FrameParser()
        self.started = time.monotonic()
        self.last_summary = self.started
        self.last_hello_tx = 0.0
        self.last_hello_rx = 0.0
        self.last_valid_rx = 0.0
        self.last_valid_gap_ms = 0.0
        self.rx_up = False
        self.bytes_total = 0
        self.bytes_at_summary = 0
        self.frames_at_summary = 0
        self.crc_at_summary = 0
        self.discarded_at_summary = 0
        self.type_counts: collections.Counter[int] = collections.Counter()
        self.types_at_summary: collections.Counter[int] = collections.Counter()
        self.last_rfid: tuple[int, int, int] | None = None
        self.last_status: tuple[int, int, int, int] | None = None
        self.hello_tx_count = 0
        self.hello_rx_count = 0

    def log(self, message: str) -> None:
        print(f"[{timestamp(self.started)}] {message}", flush=True)

    def send_hello(self, now: float) -> None:
        frame = encode_frame(HELLO_REQ, b"\x01")
        try:
            written = os.write(self.fd, frame)
        except BlockingIOError:
            self.log("HELLO_TX_FAIL 串口发送缓冲忙")
            return
        except OSError as exc:
            self.log(f"HELLO_TX_FAIL errno={exc.errno} {exc.strerror}")
            return
        if written != len(frame):
            self.log(f"HELLO_TX_FAIL 短写 {written}/{len(frame)}")
            return
        self.last_hello_tx = now
        self.hello_tx_count += 1

    def handle_frame(self, message_type: int, payload: bytes, now: float) -> None:
        if self.last_valid_rx > 0:
            self.last_valid_gap_ms = max(
                self.last_valid_gap_ms, (now - self.last_valid_rx) * 1000.0
            )
        self.last_valid_rx = now
        self.type_counts[message_type] += 1
        if not self.rx_up:
            self.rx_up = True
            self.log("RX_UP 收到有效CRC帧")

        if message_type == HELLO_INFO and len(payload) == 7:
            protocol, capabilities, boot_id, remote_state = struct.unpack("<BBIB", payload)
            self.last_hello_rx = now
            self.hello_rx_count += 1
            rtt_ms = (now - self.last_hello_tx) * 1000.0 if self.last_hello_tx else -1.0
            state = REMOTE_STATES.get(remote_state, f"UNKNOWN({remote_state})")
            self.log(
                f"HELLO_RX rtt={rtt_ms:.1f}ms protocol={protocol} "
                f"caps=0x{capabilities:02x} boot_id=0x{boot_id:08x} state={state}"
            )
            return

        if message_type == RFID_CARD and len(payload) == 3:
            present, card_number, generation = payload
            current = (present, card_number, generation)
            if current != self.last_rfid:
                if present and 1 <= card_number <= 12:
                    meaning = "VALID"
                elif present:
                    meaning = "INVALID_READ"
                else:
                    meaning = "REMOVED"
                self.log(
                    f"RFID {meaning} present={present} card={card_number} generation={generation}"
                )
                self.last_rfid = current
            return

        if message_type == SYSTEM_STATUS and len(payload) == 7:
            remote_state, saturated, fault_code, comm_errors = struct.unpack("<BBBI", payload)
            current = (remote_state, saturated, fault_code, comm_errors)
            if current != self.last_status:
                state = REMOTE_STATES.get(remote_state, f"UNKNOWN({remote_state})")
                self.log(
                    f"STATUS state={state} saturated={saturated} "
                    f"fault={fault_code} mcu_comm_errors={comm_errors}"
                )
                self.last_status = current
            return

        if message_type == ACK and len(payload) == 2:
            request_type, result = payload
            self.log(
                f"ACK request=0x{request_type:02x} "
                f"result={ACK_RESULTS.get(result, f'UNKNOWN({result})')}"
            )
            return

        if message_type == MOTION_RESULT and len(payload) == 11:
            action, quarters, result, final_yaw, target_yaw = struct.unpack("<BBBff", payload)
            self.log(
                f"MOTION_RESULT action={action} quarters={quarters} result={result} "
                f"yaw={final_yaw:.3f}/{target_yaw:.3f}"
            )

    def print_summary(self, now: float) -> None:
        elapsed = max(now - self.last_summary, 1e-6)
        bytes_delta = self.bytes_total - self.bytes_at_summary
        frames_delta = self.parser.stats.valid_frames - self.frames_at_summary
        crc_delta = self.parser.stats.crc_errors - self.crc_at_summary
        discarded_delta = self.parser.stats.discarded_bytes - self.discarded_at_summary
        type_delta = self.type_counts - self.types_at_summary
        types = ",".join(
            f"{TYPE_NAMES.get(key, f'0x{key:02x}')}:{value}"
            for key, value in sorted(type_delta.items())
        ) or "-"
        age_ms = (now - self.last_valid_rx) * 1000.0 if self.last_valid_rx else -1.0
        hello_age = now - self.last_hello_rx if self.last_hello_rx else float("inf")
        hello_state = "OK" if hello_age <= max(2.5 * self.hello_interval, 1.5) else "NO_RESPONSE"
        self.log(
            f"SUMMARY rx={bytes_delta / elapsed:.0f}B/s frames={frames_delta} "
            f"crc_err={crc_delta} junk={discarded_delta} max_gap={self.last_valid_gap_ms:.1f}ms "
            f"last_rx={age_ms:.1f}ms hello={hello_state} "
            f"hello_tx/rx={self.hello_tx_count}/{self.hello_rx_count} types={types}"
        )
        self.last_summary = now
        self.bytes_at_summary = self.bytes_total
        self.frames_at_summary = self.parser.stats.valid_frames
        self.crc_at_summary = self.parser.stats.crc_errors
        self.discarded_at_summary = self.parser.stats.discarded_bytes
        self.types_at_summary = self.type_counts.copy()
        self.last_valid_gap_ms = 0.0

    def run(self) -> None:
        self.log(
            "START 只发送HELLO_REQ；不会ARM、不会发送速度或动作。"
        )
        while True:
            now = time.monotonic()
            if self.last_hello_tx == 0.0 or now - self.last_hello_tx >= self.hello_interval:
                self.send_hello(now)

            readable, _, _ = select.select([self.fd], [], [], 0.05)
            if readable:
                try:
                    data = os.read(self.fd, 4096)
                except OSError as exc:
                    if exc.errno not in (errno.EAGAIN, errno.EWOULDBLOCK, errno.EINTR):
                        raise
                    data = b""
                if data:
                    self.bytes_total += len(data)
                    received_at = time.monotonic()
                    for message_type, payload in self.parser.feed(data):
                        self.handle_frame(message_type, payload, received_at)

            now = time.monotonic()
            if self.rx_up and now - self.last_valid_rx > self.link_timeout:
                self.rx_up = False
                self.log(f"RX_DOWN 超过{self.link_timeout:.1f}s没有有效CRC帧")
            if now - self.last_summary >= 1.0:
                self.print_summary(now)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="持续监视 STM32 USART2 二进制协议和 RFID 上报")
    parser.add_argument("--device", default="/dev/ttyS6", help="串口设备，默认 /dev/ttyS6")
    parser.add_argument("--baud", type=int, default=921600, help="波特率，默认 921600")
    parser.add_argument(
        "--hello-interval", type=float, default=1.0, help="HELLO_REQ发送周期（秒），默认1.0"
    )
    parser.add_argument(
        "--link-timeout", type=float, default=1.0, help="有效帧断流判定时间（秒），默认1.0"
    )
    args = parser.parse_args()
    if args.hello_interval <= 0 or args.link_timeout <= 0:
        parser.error("周期和超时必须大于0")
    return args


def main() -> int:
    args = parse_args()
    try:
        fd = open_serial(args.device, args.baud)
    except (OSError, ValueError) as exc:
        print(f"打开串口失败: {exc}", file=sys.stderr)
        return 1
    try:
        Monitor(fd, args.hello_interval, args.link_timeout).run()
    except KeyboardInterrupt:
        print("\nSTOP", flush=True)
    except OSError as exc:
        print(f"\n串口错误: errno={exc.errno} {exc.strerror}", file=sys.stderr)
        return 1
    finally:
        os.close(fd)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
