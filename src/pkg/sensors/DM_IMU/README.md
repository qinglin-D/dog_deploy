# DM-IMU-L1 USB 数据读取 & 可视化

## 可视化上位机（推荐）

Python GUI 程序，提供 **3D 姿态显示 + 实时数据曲线**：

```bash
# 直接读取串口
python3 imu_viewer.py /dev/ttyACM0

# 指定波特率
python3 imu_viewer.py /dev/ttyACM0 -b 115200

# 从 C++ 程序管道输入
./build/imu_reader /dev/ttyACM0 -c | python3 imu_viewer.py

# 跳过 CRC 校验（CRC 算法不确定时使用）
python3 imu_viewer.py /dev/ttyACM0 --no-crc
```

### 界面说明

```
┌──────────────────────────────────────────┐
│  左侧: 3D 姿态立方体                      │
│    - 鼠标拖拽旋转视角                      │
│    - 滚轮缩放                             │
│    - 红/绿/蓝 轴 = X/Y/Z                  │
│  右侧: 实时曲线 (欧拉角 / 加速度 / 角速度)  │
│  最右: 当前数值面板                        │
└──────────────────────────────────────────┘
```

依赖：`pip install PyQt5 PyOpenGL pyserial`（均已安装）

---

## C++ 命令行程序

```bash
mkdir build && cd build
cmake .. && make -j$(nproc)
```

### 使用方法

```bash
./imu_reader <device> [options]
```

| 参数 | 说明 |
|------|------|
| `<device>` | 串口设备路径，如 `/dev/ttyACM0` |
| `-b <baud>` | 波特率（默认 921600） |
| `-c` | CSV 输出模式 |
| `-f <file>` | 写入 CSV 日志文件 |
| `--no-config` | 跳过 IMU 自动配置 |
| `-h` | 显示帮助 |

### 示例

```bash
./imu_reader /dev/ttyACM0              # 交互式显示
./imu_reader /dev/ttyACM0 -c           # CSV 模式
./imu_reader /dev/ttyACM0 -f data.csv  # 保存到文件
```

### 串口权限

```bash
sudo usermod -a -G dialout $USER
# 重新登录生效
```

## 协议说明

USB 数据帧格式（每帧 `55 AA` 开头，`0A` 结尾，float 小端序）：

| 类型 | 帧结构 | 长度 |
|------|--------|------|
| 加速度 `01` | `55 AA ID 01 AccX AccY AccZ CRC16 0A` | 19B |
| 角速度 `02` | `55 AA ID 02 GyroX GyroY GyroZ CRC16 0A` | 19B |
| 欧拉角 `03` | `55 AA ID 03 Roll Pitch Yaw CRC16 0A` | 19B |
| 四元数 `04` | `55 AA ID 04 W X Y Z CRC16 0A` | 23B |

## 文件结构

```
├── imu_viewer.py              # 可视化上位机 (Python)
├── CMakeLists.txt
├── build/imu_reader           # 命令行程序 (C++)
└── src/
    ├── crc16.h / crc16.cpp
    ├── serial_port.h / .cpp
    ├── imu_parser.h / .cpp
    └── main.cpp
```
