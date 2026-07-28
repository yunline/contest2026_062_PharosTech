# nsh — 最小 NSH 启动配置

点亮 kickpi-k7 到 NSH 命令行的**最小 defconfig**，仅包含启动内核和串口控制台所需的配置项。

## 包含内容

- **内核基础**：ARM64 架构、RK3576 芯片初始化、GIC v2 中断控制器、自旋锁、调度器
- **串口控制台**：UART0（1500000 波特率），通过串口进入 NSH shell
- **NSH shell**：内置应用框架、readline、命令历史
- **基本文件系统**：procfs、romfs、tmpfs（NSH 启动依赖）
- **内存配置**：4G DDR 布局（RK3576_DDR_4G）
- **调试基础**：调试符号、断言、栈着色、RAM 日志
