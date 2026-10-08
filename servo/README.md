# servo/

C++ 分层库 + `servo_cli`：当前项目使用 **PWM14_M0** 驱动 180° 舵机，不能沿用旧的 Pin 7 / PWM14_M2 接线说明。芯片编号与系统启用情况必须按实际设备树核对，见 [`docs/reference/hw/servo.md`](../docs/reference/hw/servo.md)。

```text
math/   角度→脉宽
pwm/    sysfs
servo.h 门面
app/    配置与命令行
```

```bash
cmake -B build
cmake --build build -j
ctest --test-dir build -R servo_ --output-on-failure
sudo ./build/servo/servo_cli --angle 90
```

最后一条是硬件动作，只在获得舵机操作授权并确认端点安全后执行。

`--angle` 是绝对角度，原来的单次模式仍默认保持 3 秒后关闭 PWM。新增
`--stdin --config config/servo.yaml` 常驻模式接收 `angle <request_id> <0..180>`、
`close`；成功写入回复 `SERVO_APPLIED <request_id>`，失败回复 `SERVO_FAIL <request_id> ...`。
写入回执不等于机械到位，涵洞执行器另等待 3 秒。常驻期间保持 PWM；close、EOF、
Linux INT/TERM/HUP 会关闭输出，不自动归位。仅常驻模式新增信号清理。

涵洞每任务一次翻转、局域网 ROI 与持久化参数见
[`涵洞两侧识别`](../docs/reference/perception/culvert-inspection.md)。
