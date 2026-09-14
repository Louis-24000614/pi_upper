# servo/

C++ 分层库 + `servo_cli`：香橙派 40 针 **Pin 7 / PWM14_M2** 驱动 180° 舵机。接线与 overlay 见 [`docs/reference/hw/servo.md`](../docs/reference/hw/servo.md)。

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
