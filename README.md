# STM32H723VGTx-BootLoader

STM32 自定义 BootLoader，用于靶车控制板在线固件升级（OTA）。

本项目基于 **STM32H723VGT6** 实现自定义 BootLoader，通过 UART 与 ESP32 通信，实现应用程序（App）在线更新。

---

## 功能特性

- 自定义 BootLoader 启动流程
- App 与 BootLoader 分区运行
- 支持 App 主动请求进入 OTA 模式
- 使用 RTC Backup Register 作为 OTA 升级标志
- 支持 BootLoader 跳转至 App
- 支持 App 软件复位返回 BootLoader
- 支持擦除 App Flash 区域
- 支持 BootLoader 写入 App Flash
- 支持 UART 与 ESP32 通信
- 为后续 OTA 分包升级 / CRC 校验预留接口

---

## 系统架构

```text
手机
  ↓
ESP32
  ↓ UART
STM32 BootLoader
  ↓
STM32 App
```

## TODO

- [ ] 和ESP32的通讯协议，接受并校验串口发来的二进制文件
- [ ] 将二进制文件写入FLASH,并且正常跳转到app,退出OTA升级