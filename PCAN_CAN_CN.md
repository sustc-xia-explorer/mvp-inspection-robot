# PEAK PCAN-USB 与 Innodisk 同机时的接口选择

2026-10-24 用户已确认底盘 CAN-H/L 接在外置 PEAK PCAN-USB 上，并提供了 sysfs 接口映射。当前底盘 ROS 驱动应绑定 **`can2`**，Innodisk 不承担本次底盘控制链路。

| 接口 | 用户输出中的设备/驱动 | 本次状态 | 用途 |
| --- | --- | --- | --- |
| `can0` | `device -> ../../../ttyACM0` | UP | Innodisk 第一通道，未接本次底盘 |
| `can1` | `device -> ../../../ttyACM0` | UP | Innodisk 第二通道，未接本次底盘 |
| `can2` | `device -> ../../../3-10.3:1.0`，`driver -> .../peak_usb` | DOWN | 外置 PEAK，实际接底盘 |

此前 `can0` 的 `Operation not supported` 是在 Innodisk 接口上执行了原生 CAN 位时序配置；绑定 `can0` 后无底盘反馈，也与底盘接在另一块适配器的事实一致。下面给出本次 `can2` 的具体步骤，配置与实车收发仍由工控机现场验证。重启、重插或重新创建接口后需重新检查名称，不能把 `can2` 当作永久硬件身份。

本项目 `yhs_can_control` 通过 SocketCAN 绑定一个网络接口。选择实际接入底盘的适配器时，修改驱动参数 `if_name`，NeuPAN 算法无需因此重写。

## 1. 先读取硬件与接口对应关系

保持底盘 ROS 驱动停止、速度桥接禁用，执行只读检查：

```bash
lsusb
lsusb -t
ip -br link
pgrep -a emucd_64
ls -l /sys/class/net/can*/device /sys/class/net/can*/device/driver
```

判断依据：

- PEAK 的内核 SocketCAN USB 驱动通常显示 `peak_usb`。`/sys/class/net/canX/device/driver` 指向 `peak_usb` 时，可以将该 `canX` 对应到 PEAK。
- Innodisk 的网络设备关联 tty，设备路径可包含 `ttyACM0`；结合 `emucd_64` 的 tty 参数、命名参数与 USB 描述符核对。仅有 `cdc_acm` 不足以确认所有 tty-backed CAN 的具体型号。
- 部分 tty 设备没有直接的 `device/driver` 链接，`ls` 报某一项不存在不等于 CAN 接口失效；保留其 `device` 路径及其他检查结果。
- 接口编号不代表物理插座，Innodisk 工具还可显式指定名字。启动顺序或重插后需重新确认。
- 必须同时确认底盘 CAN-H/L 实际接在 PEAK 盒子、Innodisk 的哪个端口，还是两者都接在同一总线上。USB 已识别不等于底盘线已经接到该适配器。

本次已经由设备路径与 `peak_usb` 驱动链接确认 PEAK 是 `can2`。这些只读命令用于日后重启或重插后的复核。

PEAK 官方说明，其内核驱动将 CAN 通道作为 SocketCAN 网络接口，可通过 `ip link` 配置；另外安装的 PCAN 字符设备驱动是另一种接口模式。见 [PEAK Linux 驱动说明](https://www.peak-system.com/fileadmin/media/linux/index.php)、[驱动实现说明](https://www.peak-system.com/fileadmin/media/linux/can-implementation.php)。

## 2. 配置已经确认的 PEAK 接口

本机 `can2` 已由 `peak_usb` 识别，无需为本次接口重新安装 CAN 驱动。保持底盘 ROS 驱动停止、桥接禁用，执行：

```bash
sudo ip link set can2 down
sudo ip link set can2 type can bitrate 500000
sudo ip link set can2 up
ip -details -statistics link show can2
timeout 10s candump can2
```

详情应出现 `bitrate 500000`。观察是否有底盘反馈，单纯配置成功或接口 UP 不等于收发验收通过。

下面保留带驱动检查的通用入口，适用于重启、重插后接口名发生变化的情况；本次输入 `can2`，无需与上面的配置流程重复执行：

```bash
read -rp '输入已确认属于 PEAK 的 CAN 接口名: ' FW_PCAN_IFACE
FW_PCAN_DRIVER="$(basename "$(readlink -f "/sys/class/net/$FW_PCAN_IFACE/device/driver")")"
if [ "$FW_PCAN_DRIVER" = peak_usb ]; then
  sudo ip link set dev "$FW_PCAN_IFACE" down
  if sudo ip link set dev "$FW_PCAN_IFACE" type can bitrate 500000; then
    sudo ip link set dev "$FW_PCAN_IFACE" up
    ip -details -statistics link show dev "$FW_PCAN_IFACE"
    timeout 10s candump "$FW_PCAN_IFACE"
  fi
else
  echo '未确认所选接口由 peak_usb 驱动，停止配置；请先核对映射。'
fi
```

检查 `bitrate 500000`、CAN 状态和 RX 统计。若已确认由 `peak_usb` 驱动仍配置失败，保留具体错误和接口详情，不直接切换到 Innodisk 配置方式。

Innodisk 由 `emucd_64 -s7` 设置两通道 500 kbit/s，不通过上面的原生 CAN 位时序命令配置。本次其 `can0/can1` 无需用于底盘测试；PEAK 的配置不依赖该工具。Innodisk 的独立使用步骤见 [Innodisk 接入说明](INNODISK_CAN_CN.md)。

## 3. 将底盘驱动指向正确接口

确认底盘反馈出现在所选接口后，找到实际加载的驱动包：

```bash
source ~/neupan_noetic_build/activate.bash
rospack find yhs_can_control
```

修改该路径下 `config/yhs_can_control_param.yaml` 的 `yhs_can_control.if_name`。本次设置为：

```yaml
yhs_can_control:
  if_name: can2
```

只改原文件中的 `if_name`，保留其余参数；不将整个 YAML 替换成上面两行。launch 会重新加载 YAML，启动前单独执行 `rosparam set` 可能被覆盖。参数在节点启动时读取，修改后重启即可，无需重新编译底盘包或 NeuPAN。

按主硬件说明完成启动准备后运行：

```bash
roslaunch yhs_can_control yhs_can_control.launch
```

launch 参数摘要应显示 `/yhs_can_control/if_name: can2`。在另一个已激活环境的终端验证：

```bash
rosparam get /yhs_can_control/if_name
rostopic hz /ctrl_fb
```

Ctrl+C 结束频率统计后，分别执行 `rostopic echo -n 1 /motor_fb`、`rostopic echo -n 1 /io_fb`。若只有底盘驱动在运行，`ctrl_cmd timeout; sending zero velocity` 仍可出现；它与是否收到 CAN 反馈是两项独立检查。若 `can2` 仍没有原始反馈，保留 `ip -details -statistics link show can2` 和 `candump` 输出继续查供电、接线及终端电阻，暂不使能速度桥接。

该驱动识别的反馈 ID 包括 `18C4D1EF`、`18C4DAEF`、`18C4DBEF`。TX 计数增长或看到 `18C4D1D0` 等本机控制帧，不等于底盘反馈已接通。实车节点会发送电机使能请求，启动顺序见 [硬件说明第 3.7 节](FW_MINI_HARDWARE_CN.md#37-启动底盘驱动接入路径与目标)。

## 4. USB 能识别 PEAK，但没有其 SocketCAN 接口

先结合 `lsusb -t` 和系统日志核对驱动：

```bash
lsmod
sudo dmesg | tail -n 80
```

若硬件已识别、没有使用其他 PCAN 驱动，可执行 `sudo modprobe peak_usb` 后再检查接口。若已使用厂家 `pcan` 字符设备驱动，先检查其工作模式和 `/proc/pcan`；本 ROS 底盘包需要 SocketCAN 网络接口，不能将 `/dev/pcan*` 填入 `if_name`。PEAK 官方说明，手动安装厂家驱动可能屏蔽内核自带驱动，因此不在未知配置下直接卸载或安装另一套驱动。

若型号实际上是较新的 PCAN-USB FD，内核兼容性还取决于硬件版本；提供完整 USB VID/PID 和识别日志后再选择驱动，不仅凭“PCAN”名称推断 Linux 5.15 一定支持。
