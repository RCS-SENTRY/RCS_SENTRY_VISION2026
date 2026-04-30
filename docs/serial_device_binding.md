# 串口设备固定指南

交换机如果通过 NUC USB 5V 供电，Linux 可能把交换机内部 USB 芯片枚举成 `/dev/ttyUSB*` 或 `/dev/ttyACM*`，导致下位机串口编号变化。长期运行不要依赖固定的 `/dev/ttyUSB0`。

推荐启动参数使用：

```bash
serial_device:=/dev/rm_serial
```

本机当前检测结果：

```text
/dev/ttyUSB0: CH340, ID_PATH=pci-0000:00:14.0-usb-0:8:1.0, 460800 下有下位机回包，建议绑定为 /dev/rm_serial
/dev/ttyUSB1: CH340, USB path 3-9, 当前未读到下位机回包，可能是交换机/其它 USB 串口设备
```

仓库内已提供当前机器可用的规则模板：

```bash
cd /home/rm/Desktop/SENTRY_FULL/XMU_RCS_SENTRY
sudo cp docs/99-rm-serial.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger --action=add --subsystem-match=tty --sysname-match=ttyUSB0 --settle
ls -l /dev/rm_serial
```

如果仍未出现，先用下面命令确认规则是否命中。输出中应包含 `rm_serial`：

```bash
sudo udevadm test /sys/class/tty/ttyUSB0 2>&1 | grep rm_serial
```

仍不出现时，可以临时创建软链接验证 bringup：

```bash
sudo ln -sf ttyUSB0 /dev/rm_serial
ls -l /dev/rm_serial
```

## 查看设备

```bash
ls -l /dev/serial/by-id/
dmesg -w
lsusb
udevadm info -a -n /dev/ttyUSB0
```

如果 `/dev/serial/by-id/...` 能稳定指向下位机串口，也可以直接把 `serial_device` 设置成对应 by-id 路径。

## udev 规则模板

创建规则文件：

```bash
sudo nano /etc/udev/rules.d/99-rm-serial.rules
```

写入模板，替换 `idVendor` 和 `idProduct`：

```udev
SUBSYSTEM=="tty", ATTRS{idVendor}=="xxxx", ATTRS{idProduct}=="yyyy", SYMLINK+="rm_serial", MODE="0666"
```

重新加载规则：

```bash
sudo udevadm control --reload-rules
sudo udevadm trigger --action=add --subsystem-match=tty --sysname-match=ttyUSB0 --settle
ls -l /dev/rm_serial
```

最稳的硬件方案是使用 power-only USB 线，或用独立 5V DC/DC 给交换机供电，避免交换机参与 USB 数据枚举。
