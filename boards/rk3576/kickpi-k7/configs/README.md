# boards/configs — BSP 配置

本目录存放 **BSP 级别的配置**，即那些仅靠 kickpi-k7 板子本身就能运行的功能 defconfig。

## 什么是 BSP 配置？

BSP（Board Support Package）描述的是**"这块板子有什么硬件能力"**。这里的配置应该只包含：

- **最小系统**：点亮 NSH shell 所需的最小配置
- **板载外设**：板子自带的硬件（如 eMMC、SD 卡槽、USB 口、GPIO）
- **基础调试**：串口控制台、基本文件系统

这些配置绑定 RK3576/KICKPI-K7，可作为本板其他场景（如 `configs/` 下的聚合配置）的基础。

## 添加新配置

如果要添加新的 BSP 配置，请确保：

1. 只启用**板载硬件**相关的配置，不引入外部硬件依赖
2. 从 `nsh/defconfig` 出发，按需逐步添加功能
3. 配置名应清晰表达功能（如 `adb`、`sdmmc`），避免模糊命名

## 使用方法（以nsh为例）

```bash
# 使用make
./build.sh vendor/rockchip/boards/rk3576/kickpi-k7/configs/nsh

# 或者使用cmake
./build.sh vendor/rockchip/boards/rk3576/kickpi-k7/configs/nsh --cmake
```
