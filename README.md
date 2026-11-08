# MVS Camera + YOLO Real-time Inference System

<div align="center">

**工业级机器视觉检测系统** — 海康MVS相机 × OpenCV YOLO实时推理

[![C++](https://img.shields.io/badge/language-C%2B%2B17-blue)]() [![OpenCV](https://img.shields.io/badge/OpenCV-4.x-green)]() [![ONNX](https://img.shields.io/badge/Model-ONNX-orange)]()

</div>

---

## 📋 目录

- [项目简介](#项目简介)
- [核心功能](#核心功能)
- [快速开始](#快速开始)
- [编译指南](#编译指南)
- [使用说明](#使用说明)
- [交互控制](#交互控制)
- [架构设计](#架构设计)
- [常见问题](#常见问题)
- [许可证](#许可证)

---

## 项目简介

本项目是一个**工业相机 + YOLO目标检测**的实时推理系统，基于海康威视MVS SDK和OpenCV DNN模块构建。系统支持USB3.0/GigE工业相机的实时图像采集、YOLO模型推理、可视化展示及参数调节，适用于生产线质检、零件识别等工业场景。

### 技术栈

| 组件 | 版本 | 用途 |
|------|------|------|
| C++17 | - | 核心语言 |
| OpenCV 4.x | dnn/imgproc/highgui | 图像处理与推理 |
| ONNX Runtime (via cv::dnn) | - | 模型推理引擎 |
| Hikrobot MVS SDK | latest | 相机驱动与控制 |
| CMake | 3.16+ | 构建系统 |

---

## 核心功能

### ✅ 已实现功能

1. **多协议相机支持** — USB3.0 / GigE 工业相机自动枚举与连接
2. **YOLO实时推理** — 支持检测(Detection)、关键点(Keypoint)等多种模型格式
3. **交互式调参** — GUI滑动条实时调节置信度、IoU阈值、曝光时间、增益
4. **像素格式兼容** — 自动处理 BayerRG8/BGR8/Mono8/RGB8 格式转换
5. **图片测试模式** — 无需相机即可验证模型推理效果
6. **结果保存** — 支持原始帧与检测结果分别保存
7. **性能监控** — 实时显示FPS、推理耗时、帧计数

### 🔧 技术特性

- **Letterbox预处理** — 保持宽高比的智能缩放，填充灰色边框
- **自定义NMS** — 浮点坐标非极大值抑制算法
- **错误恢复机制** — 超时自动重试、异常捕获保护
- **内存安全** — RAII资源管理（`Cam`类析构函数自动释放）

---

## 快速开始

### 前置要求

```bash
# 1. 安装OpenCV 4.x
sudo apt install libopencv-dev

# 2. 安装海康MVS SDK
# 从海康机器人官网下载并安装MVS SDK到 /opt/MVS

# 3. 确认相机权限
sudo usermod -aG plugdev $USER
# 重新登录使权限生效

# 4. 检查相机是否被识别
lsusb | grep -i hik
```

### 一键编译

```bash
cd /home/mvs_yolo
mkdir -p build && cd build
cmake ..
make -j$(nproc)
```

### 首次运行

```bash
# 列出可用相机
./mvs_yolo --list

# 使用测试图片验证模型
./mvs_yolo --model best.onnx --image test.jpg

# 启动实时推理（默认类别: B1,B2,B3,R1,R2,R3）
./mvs_yolo --model best.onnx
```

---

## 编译指南

### CMake配置

```cmake
cmake_minimum_required(VERSION 3.16)
project(mvs_yolo LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# OpenCV路径（根据实际安装位置调整）
set(OpenCV_DIR "/usr/lib/x86_64-linux-gnu/cmake/opencv4")

find_package(OpenCV 4 REQUIRED COMPONENTS core imgproc imgcodecs highgui dnn)

# MVS SDK路径
set(MVS_ROOT "/opt/MVS" CACHE PATH "MVS SDK root")
find_path(MVS_INCLUDE_DIR NAMES MvCameraControl.h HINTS "${MVS_ROOT}/include")
find_library(MVS_LIBRARY NAMES MvCameraControl
        HINTS "${MVS_ROOT}/lib/64" "${MVS_ROOT}/lib/aarch64")
```

### 手动编译命令

```bash
g++ -O2 -std=c++17 mvs_yolo.cpp -o mvs_yolo \
  $(pkg-config --cflags opencv4) \
  -I/opt/MVS/include \
  -L/opt/MVS/lib/64 -lMvCameraControl \
  $(pkg-config --libs opencv4) \
  -Wl,-rpath,/opt/MVS/lib/64
```

### 输出文件

| 文件 | 说明 |
|------|------|
| `best.onnx` | YOLO预训练模型（10MB） |
| `test.jpg` | 测试图片 |
| `result.jpg` | 推理结果输出 |
| `cap_*_raw.jpg` | 原始帧截图 |
| `cap_*_det.jpg` | 检测结果截图 |

---

## 使用说明

### 命令行参数

```
用法: ./mvs_yolo [选项]

--list              列出所有可用相机
--model <onnx>      YOLO模型路径（必需）
--image <jpg>       使用指定图片测试推理
--classes a,b,c     自定义类别名称（逗号分隔）
--serial <sn>       指定相机序列号
--width <px>        显示窗口宽度（默认1280）
--help, -h          显示帮助信息
```

### 使用示例

#### 1. 枚举相机设备

```bash
./mvs_yolo --list
```

**输出示例：**
```
=== 相机列表 ===
  [0] MV-CS060-10GM SN=Hikrobot_01234567
  [1] MV-CA014-20GM SN=Hikrobot_07654321
```

#### 2. 图片推理测试

```bash
./mvs_yolo --model best.onnx --image test.jpg
```

**输出示例：**
```
图片 1920x1080
检出 3 个
  B1 0.92 kp=0
  R1 0.87 kp=0
  B2 0.76 kp=0
已存 result.jpg
```

#### 3. 实时相机推理

```bash
# 使用默认设置
./mvs_yolo --model best.onnx

# 指定相机序列号
./mvs_yolo --model best.onnx --serial "Hikrobot_01234567"

# 自定义类别名
./mvs_yolo --model best.onnx --classes defect1,defect2,normal

# 调整窗口大小
./mvs_yolo --model best.onnx --width 1920
```

#### 4. 组合参数

```bash
./mvs_yolo --model model.onnx --image sample.jpg --classes A,B,C,D --width 1600
```

---

## 交互控制

实时推理模式下，可通过键盘和GUI控件进行操作：

### 键盘快捷键

| 按键 | 功能 |
|------|------|
| `ESC` / `q` | 退出程序 |
| `p` | 暂停/继续画面 |
| `s` | 保存当前帧（原始+检测） |
| `b` | 切换Letterbox模式 |

### GUI滑动条

窗口顶部提供四个可调滑块：

1. **Confidence** (0-100) — 检测置信度阈值，默认25
2. **IoU** (0-100) — NMS重叠度阈值，默认45
3. **Exposure** (0-1000) — 相机曝光时间，实时映射到设备范围
4. **Gain** (0-100) — 相机增益，实时映射到设备范围

> 💡 **提示**：曝光和增益滑块会自动读取相机的实际范围并进行线性映射，确保调节平滑自然。

---

## 架构设计

### 系统流程图

```
┌─────────────┐    ┌──────────────┐    ┌─────────────┐
│  MVS Camera │───▶│  Image Grab   │───▶│  Format     │
│  (USB3/Gige)│    │  & Buffer     │    │  Conversion │
└─────────────┘    └──────────────┘    └──────┬──────┘
                                              │
                                              ▼
┌─────────────┐    ┌──────────────┐    ┌─────────────┐
│  Display &  │◀───│  Draw Result │◀───│  YOLO       │
│  UI Control │    │  Visualization│    │  Inference  │
└─────────────┘    └──────────────┘    └─────────────┘
         │                                       │
         ▼                                       ▼
  ┌─────────────┐                       ┌─────────────┐
  │  Slider     │                       │  ONNX Model │
  │  Callbacks  │                       │  (cv::dnn)  │
  └─────────────┘                       └─────────────┘
```

### 核心类说明

#### `Cam` — 相机管理类

封装海康MVS SDK的所有操作，提供RAII资源管理。

```cpp
class Cam {
    static List();           // 枚举设备
    void Open(sn);           // 打开指定相机
    void Config(exp, gain);  // 配置参数
    bool Grab(bgr, timeout); // 获取一帧
    Range(emin, emax, ...);  // 查询参数范围
};
```

**支持的像素格式：**
- `Mono8` → 灰度图
- `BGR8_Packed` → 直接使用
- `RGB8_Packed` → RGB→BGR转换
- `BayerRG8/GB8/GR8/BG8` → OpenCV去马赛克

#### `Detector` — YOLO推理类

基于OpenCV DNN的YOLO模型推理引擎。

```cpp
class Detector {
    void Load(path);                    // 加载ONNX模型
    vector<Det> Run(image, options);    // 执行推理
    static Draw(img, detections, opt);  // 绘制结果
};
```

**推理流程：**
1. Letterbox预处理（保持宽高比）
2. blobFromImage转换（归一化+通道交换）
3. 前向传播获取原始输出
4. 后处理：解码框坐标 → 置信度筛选 → NMS
5. 返回结构化检测结果

#### `Det` — 检测结果结构

```cpp
struct Det {
    cv::Rect2f box;           // 检测框（浮点精度）
    int id;                   // 类别ID
    float conf;               // 置信度
    std::string name;         // 类别名称
    std::vector<cv::Point2f> kp;  // 关键点坐标
};
```

### 关键算法

#### Letterbox预处理

```cpp
// 计算缩放比例
float s = min(size/w, size/h);
int nw = round(w * s), nh = round(h * s);

// 缩放并填充灰色边框(114,114,114)
resize(src, resized, Size(nw, nh));
copyMakeBorder(resized, padded, dh/2, dh-dh/2, dw/2, dw-dw/2, 
               BORDER_CONSTANT, Scalar(114,114,114));
```

#### 自定义NMS实现

```cpp
// 按置信度降序排序
sort(indices, greater<>());

// 贪心选择
for (i : indices) {
    if (dead[i]) continue;
    keep.push_back(i);
    for (j : remaining) {
        if (iou(box[i], box[j]) > threshold) dead[j] = true;
    }
}
```

---

## 常见问题

### ❓ Q1: 提示"找不到相机"怎么办？

**排查步骤：**

```bash
# 1. 检查USB连接
lsusb | grep -i hik

# 2. 检查用户组权限
groups | grep plugdev

# 3. 关闭其他MVS客户端
ps aux | grep -i MVS

# 4. 重新插拔相机USB线
```

### ❓ Q2: 推理速度慢怎么办？

**优化建议：**

1. **降低输入尺寸**：修改 `Opt::size` 从640改为320或416
2. **使用GPU加速**：修改后端为CUDA
   ```cpp
   net_.setPreferableBackend(cv::dnn::DNN_BACKEND_CUDA);
   net_.setPreferableTarget(cv::dnn::DNN_TARGET_CUDA);
   ```
3. **减少类别数**：类别越多，推理越慢
4. **提高置信度阈值**：减少候选框数量

### ❓ Q3: 检测结果不准确？

**调参指南：**

| 问题 | 调整方向 |
|------|----------|
| 漏检 | 降低Confidence阈值 |
| 重复框 | 降低IoU阈值 |
| 误检 | 提高Confidence阈值 |
| 图像过暗 | 增加Exposure/Gain |
| 图像过曝 | 降低Exposure/Gain |

### ❓ Q4: 编译报错"找不到MVS SDK"？

```bash
# 确认SDK安装路径
ls /opt/MVS/include/MvCameraControl.h
ls /opt/MVS/lib/64/libMvCameraControl.so

# 如果路径不同，指定自定义路径
cmake -DMVS_ROOT=/your/mvs/path ..
```

### ❓ Q5: 不支持的像素格式？

某些相机默认输出特殊格式，需要在MVS客户端中修改：

1. 打开MVS客户端软件
2. 选择相机 → 流处理器(Stream Processor)
3. 将像素格式改为 `BayerRG8` 或 `BGR8`
4. 应用设置

### ❓ Q6: 模型输出维度不匹配？

错误信息会提示：
```
模型输出 C=XX，但 4+N+K=YY。请检查 --classes 个数和关键点设置
```

**解决方案：**
- 确认 `--classes` 数量与模型训练时的类别数一致
- 如果模型不含关键点，设置 `--classes A,B,C`（不带关键点参数）

---

## 性能指标

### 典型配置

| 项目 | 数值 |
|------|------|
| 输入分辨率 | 1920×1080 |
| 模型尺寸 | 640×640 |
| CPU推理耗时 | ~30-50ms/frame |
| 端到端FPS | ~20-30 FPS |
| 内存占用 | ~200MB |

### 优化潜力

- GPU加速可提升3-5倍推理速度
- 减小输入尺寸可显著提升FPS
- TRT/TensorRT部署可达到实时性能

---

## 文件结构

```
mvs_yolo/
├── mvs_yolo.cpp          # 主程序源代码（~1000行）
├── CMakeLists.txt        # CMake构建配置
├── best.onnx             # YOLO预训练模型
├── test.jpg              # 测试图片
├── README.md             # 本文档
├── cmake-build-debug/    # CMake构建目录
└── .idea/                # CLion项目配置
```

---

## 扩展开发

### 添加新模型支持

1. 准备ONNX格式的YOLO模型
2. 确认模型输出维度：`C = 4 + num_classes + num_keypoints*2`
3. 启动时指定类别名：`--classes class1,class2,...`

### 集成到其他项目

本代码采用单文件设计，可直接复制使用。关键依赖：

```cpp
#include <MvCameraControl.h>  // 海康SDK
#include <opencv2/opencv.hpp> // OpenCV
```

### 贡献指南

欢迎提交Issue和Pull Request！重点关注：

- ✅ 更多相机型号适配
- ✅ GPU后端支持
- ✅ 模型量化优化
- ✅ ROS/ROS2接口

---

## 许可证

本项目为个人/企业内部使用工具，相机SDK遵循海康威视许可协议。

---

## 联系方式

- **GitHub**: [lxkgg2/mvs_camera_inference](https://github.com/lxkgg2/mvs_camera_inference)
- **环境**: Ubuntu 20.04/22.04 + OpenCV 4.x + MVS SDK

---

<div align="center">

**Made with ❤️ for Industrial Machine Vision**

</div>
