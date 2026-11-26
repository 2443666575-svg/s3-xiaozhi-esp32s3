# S3 小智 · ESP32-S3 主控固件

ESP32-S3（N16R8）小智语音主控：USB UVC 摄像头（GPIO19/IO20）、视觉识别、
环境监测（AHT20 / BMP280 / SHT40）、板间串口联动、定时任务与 GLM 兜底。

## 定稿版本

`IO19_IO20摄像头定稿版_20261125/` 为 2026-11-25 实机验证版本，连续拍摄 5~6 张正常。
涉及 **IO19(D-) / IO20(D+) 接 USB 摄像头** 的项目直接复用该目录。

## 重要：脱敏说明

本仓库为公开发布版本，以下内容已替换为占位符：

- 智谱 API Key → `YOUR_GLM_API_KEY_HERE`（在 `glm_secrets.h` 或 `main.cpp` 中填写自己的）
- WiFi 名称 / 密码 → `YOUR_WIFI_SSID` / `YOUR_WIFI_PASSWORD`
- 本机路径 → `%USERPROFILE%`

**历史归档、备份 zip 与 `.workbuddy` 目录未上传**（可能含旧密钥与本机信息）。

## 烧录要点

1. 工具 → USB CDC On Boot = **Disabled**（USB Host 独占 GPIO19/20）
2. PSRAM = **OPI PSRAM**，Flash Size = **16MB**
3. 日志走 UART0（TX=43 / RX=44，115200）
4. 烧录需按住 BOOT 再点 RST，或拔掉摄像头上电烧

## 关键依赖

LVGL 分配器需为 PSRAM 优先版本，见 `IO19_IO20摄像头定稿版_20261125/lv_mem_core_clib.c_PSRAM修复版`。
否则 LVGL 小对象会挤占内部 RAM，导致语音 TLS 分配失败。

## 目录

- `Ai智能/ai_vox-1.3.2/` — 完整 SDK 与工程源码
- `IO19_IO20摄像头定稿版_20261125/` — 定稿代码 + LVGL 补丁 + 使用说明
- `AHT20_BH1750_接口对照.md` — 传感器接口与踩坑记录
- `S3小智独立版使用说明_20261119.docx` — 无终端独立版说明
