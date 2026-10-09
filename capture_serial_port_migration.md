# capture 程序串口改造指南

## 背景

当前 `calibration/capture.cpp` 只支持 CAN 总线（`io::CBoard`）读取 IMU 四元数。但有些机器人用的是串口通信（`io::Gimbal`）。

本文档记录了将 `capture` 改造为同时支持 CAN 和串口的步骤，上手时直接照着改即可。

---

## 改动文件

只改一个文件：`calibration/capture.cpp`

---

## 第 1 步：添加头文件

在文件顶部的 `#include` 区域，`io/cboard.hpp` 下面加一行：

```cpp
#include "io/camera.hpp"
#include "io/cboard.hpp"
#include "io/gimbal/gimbal.hpp"   // ← 新增
#include "tools/img_tools.hpp"
```

---

## 第 2 步：修改 `capture_loop` 函数签名

原来的 `capture_loop` 硬编码了 `io::CBoard`。我们需要让它既能接收 CBoard 也能接收 Gimbal。

把第 27 行的函数签名：

```cpp
void capture_loop(
  const std::string & config_path, const std::string & can, const std::string & output_folder)
```

**替换为**（删掉不用的 `can` 参数，改用模板接受两种通信对象）：

```cpp
template <typename T>
void capture_loop(
  T & board, const std::string & config_path, const std::string & output_folder)
```

---

## 第 3 步：去掉函数内部的 CBoard 构造

把第 30 行：

```cpp
io::CBoard cboard(config_path);
```

**删掉**（通信对象现在从外部传入）。

---

## 第 4 步：统一四元数读取方式

CBoard 和 Gimbal 读取四元数的方法名不同：
- `CBoard::imu_at(timestamp)` 
- `Gimbal::q(timestamp)`

为了让模板能同时适配两者，在第 38 行位置加一个辅助函数。在 `write_q` 函数**后面**、`capture_loop` 函数**前面**插入：

```cpp
// 统一接口：CBoard 用 imu_at()，Gimbal 用 q()
template <typename T>
Eigen::Quaterniond read_imu(T & board, std::chrono::steady_clock::time_point t)
{
  if constexpr (std::is_same_v<T, io::CBoard>)
    return board.imu_at(t);
  else
    return board.q(t);
}
```

然后把第 38 行：

```cpp
Eigen::Quaterniond q = cboard.imu_at(timestamp);
```

**替换为**：

```cpp
Eigen::Quaterniond q = read_imu(board, timestamp);
```

---

## 第 5 步：修改 `main` 函数

把第 72–93 行的 `main` 函数**整体替换**为：

```cpp
int main(int argc, char * argv[])
{
  // 读取命令行参数
  cv::CommandLineParser cli(argc, argv, keys);
  if (cli.has("help")) {
    cli.printMessage();
    return 0;
  }
  auto config_path = cli.get<std::string>(0);
  auto output_folder = cli.get<std::string>("output-folder");

  // 新建输出文件夹
  std::filesystem::create_directory(output_folder);

  // 读取配置文件，判断用 CAN 还是串口
  auto yaml = YAML::LoadFile(config_path);

  tools::logger()->info("默认标定板尺寸为10列7行");

  if (yaml["com_port"]) {
    // 配置文件里有 com_port → 用串口通信
    tools::logger()->info("使用串口通信");
    io::Gimbal gimbal(config_path);
    capture_loop(gimbal, config_path, output_folder);
  } else {
    // 否则使用 CAN 通信
    tools::logger()->info("使用CAN通信");
    io::CBoard cboard(config_path);
    capture_loop(cboard, config_path, output_folder);
  }

  tools::logger()->warn("注意四元数输出顺序为wxyz");

  return 0;
}
```

> 注意：`main` 函数顶部需要额外 `#include <yaml-cpp/yaml.h>`（如果原来没有的话）。

---

## 改完之后的完整文件（对照表）

| 行号（原） | 改动 |
|-----------|------|
| 8 行后 | 新增 `#include "io/gimbal/gimbal.hpp"` |
| 17 行后 | 新增 `read_imu` 辅助模板函数 |
| 27–28 行 | 函数签名改为模板 + 去掉 `can` 参数 |
| 30 行 | **删除** `io::CBoard cboard(config_path);` |
| 38 行 | `cboard.imu_at(...)` → `read_imu(board, ...)` |
| 72–93 行 | `main` 函数整体替换（加 YAML 判断分支） |

---

## 配置文件要求

改动后 `configs/calibration.yaml` 需要包含对应的通信配置：

**串口模式**（新增 `com_port`）：
```yaml
com_port: "/dev/gimbal"
```

**CAN 模式**（保持原样）：
```yaml
can_interface: "can0"
quaternion_canid: 0x100
bullet_speed_canid: 0x101
send_canid: 0xff
```

两者都有时，`com_port` 优先。

---

## 编译验证

```bash
cmake -B build && make -C build/ capture
```

C++17 的 `if constexpr` 会在编译期自动去除不适用的分支，所以串口编译出来的二进制里不会残留 CAN 相关代码，反之亦然。
