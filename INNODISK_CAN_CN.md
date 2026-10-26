# Innodisk Dual CAN 接入 FW-mini（Ubuntu 20.04 / ROS Noetic）

本说明解决 `yhs_can_control` 启动时的 `>>bind dev_handler error!`，适用于本次确认的 Innodisk USB Dual CAN。现有 `FW-mini-ros` 底盘驱动继续使用，不需要重写，也不需要因此重新编译 NeuPAN。

**双适配器最终映射已确认：`can0/can1` 是 Innodisk 的两个 ttyACM0 通道，`can2` 由 `peak_usb` 驱动，且底盘 CAN-H/L 实际接在外置 PEAK 上。** 当前底盘测试按 [PEAK `can2` 接入步骤](PCAN_CAN_CN.md) 配置波特率并修改 ROS 驱动 `if_name: can2`，无需调整 Innodisk 的波特率。本文保留为 Innodisk 独立使用时的安装说明，不作为本次 PEAK 底盘连接的前置步骤。

## 1. 已确认的设备与缺失环节

2026-10-24 工控机初次排查输出（不是后续双适配器环境的完整当前状态）：

| 项目 | 实际值 |
| --- | --- |
| 运行内核 | `5.15.0-139-generic` |
| USB VID:PID | `04d8:0205` |
| USB 产品描述 | `innodisk USB Dual CAN` |
| USB 内核驱动 | `cdc_acm` |
| 当前设备节点 | `/dev/ttyACM0` |
| 稳定设备链接 | `/dev/serial/by-id/usb-Microchip_Technology_Inc._innodisk_USB_Dual_CAN-if00` |
| CAN 网络接口 | `can0/can1/can2` 均不存在 |

USB 产品描述确认了 Innodisk Dual CAN 设备系列；具体板卡型号仍以硬件标签为准。两个 CH340 串口不能据此当作 CAN 设备。

所需链路为：

```text
Innodisk USB Dual CAN
  → cdc_acm 提供 /dev/ttyACM0                         已有
  → emuc2socketcan 内核模块 + emucd_64 生成 can0/can1  当前待接通
  → FW-mini-ros / yhs_can_control                     已有 ROS 底盘包
  → 底盘 CAN 协议
```

`FW-mini-ros/yhs_can_control/src/yhs_can_control.cpp` 使用 `socket(PF_CAN, SOCK_RAW, CAN_RAW)`，从参数 `if_name` 查询网络接口索引并绑定。因此不能把 `if_name` 改成 `/dev/ttyACM0` 来替代底层适配器驱动。

## 2. 原厂驱动包与本地编译核对

来源：[Innodisk EMUC-B202 产品页](https://www.innodisk.com/en/products/networking/can-bus/emuc-b202) 的 [Driver 压缩包](https://www.innodisk.com/upload/file/EMUC-B202.zip)。本次下载的大包包含：

```text
Linux/ExUC_SocketCAN_driver_v3.10_utility_v3.6_20260901.zip
```

已将这个约 1.6 MB 的内层 ZIP 单独提取到开发电脑：

```text
/home/xiapilang/work/neupan_innodisk_can_20261024/
  ExUC_SocketCAN_driver_v3.10_utility_v3.6_20260901.zip
  ExUC_SocketCAN_driver_v3.10_utility_v3.6_20260901.zip.sha256
```

将这两个文件复制到工控机的 `~/work/`。内层 ZIP 的 SHA256 为：

```text
527e4b88a18f4a06079324c23a976166eeb911dc32ef89581d7d89a2366f8ec3
```

该包 `Release_Note.txt` 包含 Linux 5.14 及以后 TTY API 的适配。本地已用隔离的 **5.15.0-139-generic 内核头文件**完成模块和 64 位工具编译，输出 `emuc2socketcan.ko`、`emucd_64`；模块 `vermagic` 为 `5.15.0-139-generic SMP mod_unload modversions`。

这是编译验证，尚未在工控机加载或验证 CAN 收发。本地使用的头文件来自 Ubuntu 22.04 的 `5.15.0-139.149` 包，与 Ubuntu 20.04 HWE 包的构建环境可能不同，**请在工控机上编译，不复制本地生成的 `.ko` 使用**。核对记录见 [驱动准备记录](docs/results/innodisk_can_preparation_20261024.json)。

## 3. 在工控机编译

以下命令在工控机执行；保持 `yhs_can_control` 停止，NeuPAN 桥接未使能。先确认稳定链接仍指向本次设备：

```bash
uname -r
readlink -f /dev/serial/by-id/usb-Microchip_Technology_Inc._innodisk_USB_Dual_CAN-if00
ip -br link
```

本次预期分别为 `5.15.0-139-generic`、`/dev/ttyACM0`，且尚无 CAN 接口。如果重新插拔后 tty 编号改变，后续命令使用实际的 `/dev/ttyACM编号`。

```bash
sudo apt update
sudo apt install -y build-essential libelf-dev unzip can-utils iproute2 "linux-headers-$(uname -r)"

cd ~/work
sha256sum -c ExUC_SocketCAN_driver_v3.10_utility_v3.6_20260901.zip.sha256
mkdir -p ~/work/innodisk_socketcan_20260901
unzip -n ExUC_SocketCAN_driver_v3.10_utility_v3.6_20260901.zip \
  -d ~/work/innodisk_socketcan_20260901
cd ~/work/innodisk_socketcan_20260901
make -j2
modinfo -F vermagic ./emuc2socketcan.ko
ls -l ./emuc2socketcan.ko ./emucd_64
```

上述 ZIP 直接包含 `driver/`、`utility/`、`Makefile`，没有额外顶层目录。`make` 按系统位数选择工具，64 位工控机生成 `emucd_64`；这条构建路径不要求安装 32 位 multilib。模块 `vermagic` 首项应与 `uname -r` 一致。提示缺少 `vmlinux` 而跳过 BTF、最终仍成功生成 `.ko`，不等于编译失败。

若 apt 找不到当前运行内核的头文件，先取得对应发行版、完整版本的头文件，再继续；不要通过改成别的内核版本目录来强行构建。

## 4. 创建并启用 SocketCAN 接口

本节配置的是 CAN 适配器，不启动 ROS 底盘控制节点，也不发送应用层运动命令。它会激活 CAN 通道；首次接通时保持车辆在遥控模式、摇杆回中，并先停用其他运动命令发布者。

以下是**首次创建接口**的步骤。若已有 `emucd_64` 或 CAN 接口正在工作，先确认当前使用者，不要重复操作同一个 tty。若 PEAK 已占用 `can0` 或 `can1`，本节 Innodisk 命名改为未占用的 `emuccan0 emuccan1`，并同步替换后续所有 Innodisk 接口名；不抢占或重命名已存在的 PEAK 接口：

```bash
pgrep -a emucd_64
ip -br link
```

确认尚未运行后：

```bash
cd ~/work/innodisk_socketcan_20260901
sudo modprobe can
sudo modprobe can_raw
sudo modprobe can_dev
sudo insmod ./emuc2socketcan.ko
sudo ./emucd_64 -s7 -e0 /dev/ttyACM0 can0 can1

sudo ip link set can0 txqueuelen 1000
sudo ip link set can1 txqueuelen 1000
sudo tc qdisc replace dev can0 root handle 1: pfifo
sudo tc qdisc replace dev can1 root handle 1: pfifo
sudo ip link set can0 up
sudo ip link set can1 up

ip -br link
ip -details -statistics link show can0
ip -details -statistics link show can1
candump can0 can1
```

关键区别：

- FW-mini 要求 **500 kbit/s**；这个厂家工具的 **`-s7` 表示两通道均为 500 kbit/s**，`-s6` 是 250 kbit/s。不要照抄厂家 HOWTO 中用于其他设备的 `-s9`（1 Mbit/s）。
- `-e0` 与厂家 `start.sh` 默认一致，关闭可选错误报告，不关闭正常 CAN 数据；需要排查总线错误时再参考厂家说明调整。
- `can0 can1` 显式指定接口名；不指定时厂家默认可能生成 `emuccan0/emuccan1`。
- 原厂说明要求**两个 CAN 接口都设为 UP** 后设备才自动激活，即使底盘只接其中一个通道，也执行两条 UP 命令。
- 波特率由 `emucd_64 -s7` 设置；这套 Innodisk 接入不执行 PCAN 的 `ip link set can0 type can bitrate 500000`，也不使用 `peak_usb` 或 `slcand`。

`emucd_64` 默认在后台运行，ROS 使用 CAN 期间应保持该进程和模块工作。接口可能显示 `state UNKNOWN`，不能单凭这个字段判定失败；检查是否出现 `link/can`、接收计数是否增长以及是否收到持续的底盘反馈。该适配器的详情不一定像原生 CAN 控制器那样显示 `bitrate` 字段。

`candump` 接收 CAN 帧，不发送应用层控制命令。检查输出中哪个接口有底盘反馈，Ctrl+C 退出接收。若只有 `can1` 有对应反馈，应把实际启动包的 `config/yhs_can_control_param.yaml` 中 `if_name: can0` 改为 `if_name: can1`，后续检查也改用 `can1`。接口编号不能替代物理通道接线确认。

### 4.1 接口已存在但 ip link 设置波特率失败

用户后续输出中 `can0` 为 `UP,LOWER_UP`，但 `RX packets=0`、`TX packets=55920`；执行 PCAN 的位时序配置命令得到 `Operation not supported`。后续 sysfs 已确认 `can0/can1 -> ttyACM0`，真正连接底盘的是 `can2 -> peak_usb`。本次底盘问题按 [PEAK 接口步骤](PCAN_CAN_CN.md) 处理。以下只适用于今后确实需要重配 Innodisk 的情况，不是当前底盘反馈为空时的修复步骤。

先在底盘 ROS 驱动停止的状态下检查：

```bash
pgrep -a emucd_64
ip -br link
sudo journalctl -b -t emucd -n 50 --no-pager
```

确认当前 tty 对应的工具进程是否带有 `-s7`，日志是否报告两通道设置为 `500 KBPS`，再确认两个接口都 UP。已经以 `-s7` 成功启动的进程，不需要因 `ip link` 的报错重复配置。`can0` 的 TX 统计是驱动发送计数，不等于底盘已响应；RX 为 0 时还需检查 `can1`、物理通道、供电和接线。

若当前参数不是 `-s7`，或需要直接观察配置过程，先停止底盘 ROS 驱动、保持桥接禁用，再用 `pgrep -a emucd_64` 找到**占用本次 `/dev/ttyACM0` 的进程**，向该 PID 发送 `SIGINT` 正常结束它。可使用 `sudo kill -INT PID`，其中 `PID` 必须替换为刚才核对的数值；不要终止其他适配器实例，也不要用 `kill -9` 跳过 TTY 清理。等待该进程退出、它创建的两个接口消失后，再执行：

```bash
cd ~/work/innodisk_socketcan_20260901
sudo ./emucd_64 -F -s7 -e0 /dev/ttyACM0 can0 can1
```

原有 `emuc2socketcan` 模块保持加载即可。`-F` 使工具在前台运行，正常情况下会打印 `set can speed to 500 KBPS on both channel`、接口创建信息及 `SocketCAN utility ON`。如果设置或接口创建失败，保留完整错误输出；不要在旧进程未退出时重复启动。这个日志是工具调用配置接口的结果，实际收发仍需验证。

保持这个前台终端运行，在**另一个终端**执行：

```bash
sudo ip link set can0 txqueuelen 1000
sudo ip link set can1 txqueuelen 1000
sudo tc qdisc replace dev can0 root handle 1: pfifo
sudo tc qdisc replace dev can1 root handle 1: pfifo
sudo ip link set can0 up
sudo ip link set can1 up
ip -details -statistics link show can0
ip -details -statistics link show can1
timeout 10s candump can0 can1
```

预期的底盘反馈 ID 包括 `18C4D1EF`（控制反馈）、`18C4DAEF`（IO 反馈）、`18C4DBEF`（电机反馈）。如果之后启动 ROS 驱动，看到 `18C4D1D0` 等本机控制帧不等于已经收到底盘反馈。若原始 CAN 已有上述反馈但 ROS 话题无数据，再检查实际驱动绑定接口、帧格式及 XOR 校验。此时不通过修改 NeuPAN 参数或发送非零速度试探通信。

## 5. 继续使用现有 FW-mini ROS 驱动

确认底盘持续反馈后，回到 [硬件接入说明第 3.7 节](FW_MINI_HARDWARE_CN.md#37-启动底盘驱动接入路径与目标)。现有驱动启动后会按当前配置发送驱动/转向电机使能请求，先按该节完成架空、控制权及零指令检查，再使能速度桥接。

```bash
source ~/neupan_noetic_build/activate.bash
rospack find yhs_can_control
roslaunch yhs_can_control yhs_can_control.launch
```

`rospack find` 用来确认实际加载的是哪一份驱动包；launch 会重新加载该包的 YAML，因此启动前单独 `rosparam set` 可能被覆盖。收到正常反馈、且不再出现 bind 错误，才说明“CAN 接口 → ROS 驱动”这一环接通。雷达、FAST-LIO、TF、`/scan` 和 NeuPAN 按原文档分别验证。

## 6. 故障定位与重启

| 现象 | 下一步 |
| --- | --- |
| `insmod: File exists` | 用 `lsmod` 确认模块已加载；不要为重试而卸载正在使用的模块 |
| `Invalid module format` | 看 `sudo dmesg`、比较 `uname -r` 与 `modinfo -F vermagic`，在当前内核下重新编译 |
| `Key was rejected by service` | 检查 Secure Boot 模块签名要求，按设备维护流程签名/登记模块；此流程不自动关闭 Secure Boot |
| 工具退出、没生成接口 | 查看 `sudo journalctl -b -t emucd`、`sudo dmesg`，核对 tty 路径、占用进程和模块加载情况 |
| `unknown comport` | 将稳定链接用 `readlink -f` 解析后传实际 `/dev/ttyACM编号`；不要直接把任意 by-id 路径当作工具支持的端口名 |
| `Operation not supported` 出现在设置 bitrate 时 | 删除误用的 PCAN 配置命令；Innodisk 波特率在 `emucd_64 -s7` 中设置 |
| 有 CAN 接口、两个通道均无底盘数据 | 确认两个接口都 UP，再查底盘供电、物理通道、CAN-H/L、终端电阻和 500 kbit/s 配置 |
| ROS 仍报 bind 错误 | 用主文档第 3.7 节的只绑定检查获取 errno，检查实际 YAML 的 `if_name` |

本次采用 `insmod` 临时加载，没有创建开机服务。重启或 USB 重插后需检查模块、tty 和接口，再执行需要的创建步骤；重启进入不同内核时重新编译。不要同时运行厂家 `start.sh` 和本节命令：其脚本会终止已有 `emucd_64`、卸载模块，并使用另一套默认接口名。

结束测试时先禁用速度桥接、停止底盘 ROS 驱动并确认车辆停止，再停止本次 `emucd_64` 实例和卸载模块；不要在车辆运行中终止 CAN 通道。
