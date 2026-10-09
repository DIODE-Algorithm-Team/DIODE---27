# ChArUco 标定板使用指南

## 概述

本文档说明如何将原有的棋盘格标定改为 **ChArUco 标定板**，以及具体的使用方法。

## 修改内容

### 1. 文件修改列表

| 文件 | 改动类型 | 说明 |
|------|---------|------|
| `calibration/calibrate_camera.cpp` | 已修改 | 添加 ChArUco 检测功能 |
| `configs/calibration.yaml` | 已修改 | 添加 ChArUco 配置参数 |

### 2. calibrate_camera.cpp 改动详情

#### 2.1 新增头文件

```cpp
#include <opencv2/aruco.hpp>  // OpenCV ArUco 模块
```

#### 2.2 新增 load_charuco() 函数

核心功能：
- 检测 ArUco marker：`cv::aruco::detectMarkers()`
- 插值 ChArUco 角点：`cv::aruco::interpolateCornersCharuco()`
- 构建 3D-2D 对应点

#### 2.3 修改 main() 函数

根据配置文件中的 `use_charuco` 字段自动切换：
- `true` → 调用 `load_charuco()` 使用 ChArUco
- `false` → 调用 `load()` 使用棋盘格

### 3. calibration.yaml 新增参数

```yaml
# ChArUco 标定板参数
use_charuco: true                    # 启用 ChArUco
charuco_dict: 7                      # 字典 ID
charuco_squares_x: 14                # 横向格数
charuco_squares_y: 9                 # 纵向格数
charuco_square_length_mm: 20         # 方格边长 (mm)
charuco_marker_length_mm: 15         # ArUco marker 边长 (mm)
```

## ArUco 字典对照表

| 字典 ID | 名称 | Marker 大小 | Marker 数量 |
|---------|------|------------|------------|
| 0 | DICT_4X4_50 | 4×4 | 50 |
| 1 | DICT_4X4_100 | 4×4 | 100 |
| 2 | DICT_4X4_250 | 4×4 | 250 |
| 3 | DICT_4X4_1000 | 4×4 | 1000 |
| 4 | DICT_5X5_50 | 5×5 | 50 |
| 5 | DICT_5X5_100 | 5×5 | 100 |
| 6 | DICT_5X5_250 | 5×5 | 250 |
| **7** | **DICT_7X7_1000** | **7×7** | **1000** |
| 8 | DICT_6X6_250 | 6×6 | 250 |
| 9 | DICT_6X6_1000 | 6×6 | 1000 |

## 使用步骤

### 第一步：确认板子参数

在 [configs/calibration.yaml](file:///d:/RM/sp_vision_25-main/configs/calibration.yaml) 中确认以下参数：

```yaml
use_charuco: true
charuco_dict: 7                      # 你的板子是 7×7
charuco_squares_x: 14               # 数一下板子横向上的方格数
charuco_squares_y: 9                # 数一下板子纵向上的方格数
charuco_square_length_mm: 20        # 用尺子量一个方格的边长
charuco_marker_length_mm: 15        # 用尺子量一个 ArUco marker 的边长
```

**注意**：尺寸参数必须精确测量，单位为毫米 (mm)。

### 第二步：采集标定图像

使用采集程序（capture）拍摄标定板：

#### 2.1 启动采集

```bash
./capture --config configs/calibration.yaml assets/calib_images/
```

#### 2.2 采集要点

| 要求 | 说明 |
|------|------|
| **图像数量** | 15-30 张 |
| **覆盖范围** | 画面中心、四角、四边 |
| **距离范围** | 0.5m ~ 3m（覆盖实际使用距离） |
| **角度变化** | ±30° 俯仰和偏航 |

#### 2.3 操作说明

- 按 `s` 键保存当前帧
- 按 `q` 键退出
- 程序会实时显示检测结果

### 第三步：运行标定

```bash
# 编译
cd build
make calibrate_camera

# 运行标定
./calibrate_camera --config configs/calibration.yaml assets/calib_images/
```

### 第四步：查看输出

标定完成后会输出 YAML 格式的内参：

```yaml
# 重投影误差: 0.2345px  ← 检查这个值
camera_matrix: [1851.7, 0, 721.1, 0, 1851.8, 571.7, 0, 0, 1]
distort_coeffs: [-0.093, 0.189, -0.0004, -0.004, 0]
```

### 第五步：验证结果

#### 5.1 重投影误差标准

| 误差范围 | 状态 | 处理方式 |
|---------|------|---------|
| < 0.3px | 优秀 | 直接使用 |
| 0.3 ~ 0.5px | 合格 | 可用 |
| 0.5 ~ 1.0px | 一般 | 建议重新采集 |
| > 1.0px | 不合格 | 必须重新采集 |

#### 5.2 验证方法

将标定板放在已知距离，用得到的内参进行反投影测试：
- 识别角点
- 用内参计算位姿
- 将 3D 点投影回图像
- 比较投影点与实际检测点的像素差

### 第六步：应用到其他配置

将标定结果复制到实际使用的配置文件中（如 `configs/standard4.yaml`）：

```yaml
camera_matrix: [1851.7, 0, 721.1, 0, 1851.8, 571.7, 0, 0, 1]
distort_coeffs: [-0.093, 0.189, -0.0004, -0.004, 0]
```

## 常见问题

### Q1: 程序提示"未检测到 marker"

**原因**：当前字典与板子不匹配

**解决**：
1. 确认板子上 marker 的点阵大小（数一下每行/每列的方块数）
2. 对照上面的字典表，修改 `charuco_dict`
3. 例如 7×7 的 marker → `charuco_dict: 7`

### Q2: 程序提示"角点检测太少"

**原因**：
- 标定板离镜头太远
- 标定板只出现在画面边缘
- 光照不足或反光

**解决**：
- 调整相机与标定板的距离（建议 0.5m ~ 2m）
- 确保标定板完整出现在画面中
- 改善光照条件

### Q3: 重投影误差太大

**原因**：
- 采集的图像质量差
- 标定板尺寸参数不准确
- 标定板覆盖的姿态不够丰富

**解决**：
- 重新采集，确保每张图都清晰
- 仔细测量方格和 marker 的边长
- 增加不同距离和角度的样本

### Q4: 标定板参数如何确定？

**方法**：
1. 数板子横向上的方格数 → `charuco_squares_x`
2. 数板子纵向上的方格数 → `charuco_squares_y`
3. 用尺子量方格边长 → `charuco_square_length_mm`
4. 用尺子量 marker 边长 → `charuco_marker_length_mm`

**示例**：你的板子是 9 行 × 14 列，方格 20mm，marker 15mm：
```yaml
charuco_squares_x: 14
charuco_squares_y: 9
charuco_square_length_mm: 20
charuco_marker_length_mm: 15
```

## 与原方案的对比

| 方面 | 棋盘格 | ChArUco |
|------|--------|---------|
| 检测鲁棒性 | 差（需要完整可见） | 好（部分遮挡可检测） |
| 角点数量 | 固定 | 可根据板大小调整 |
| 标定精度 | 高 | 高 |
| 使用门槛 | 低 | 中（需配置参数） |

## 注意事项

1. **OpenCV 版本**：需要 OpenCV 4.x 及以上版本
2. **opencv_contrib**：ArUco 模块需要 opencv_contrib 支持
3. **光照条件**：标定时应避免强反光和阴影
4. **标定距离**：应覆盖实际使用的距离范围
5. **保存数据**：标定完成后，建议保存标定图像用于验证

## 参考链接

- [OpenCV ArUco 文档](https://docs.opencv.org/4.x/d5/dae/tutorial_aruco_detection.html)
- [ChArUco 标定原理](https://docs.opencv.org/4.x/da/d16/classcv_1_1aruco_1_1CharucoBoard.html)
