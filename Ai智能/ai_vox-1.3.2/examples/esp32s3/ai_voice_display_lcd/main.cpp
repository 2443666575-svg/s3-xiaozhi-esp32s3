/* ============================================================================
 * ★★★★★ 【定稿版本 · 2026-11-25】USB 摄像头主控固件（IO19 / IO20）
 * ----------------------------------------------------------------------------
 * 用途标记：以后任何项目只要涉及【ESP32-S3 + GPIO19/IO20 接 USB 摄像头】
 *           （USB Host / UVC / MJPEG / VISION 识别），直接复用本文件与配套配置，
 *           不要从头写。这是经实机连续拍摄 5~6 张验证通过的稳定版本。
 *
 * 本版本解决的问题（按时间顺序）：
 *   1) 首次拍照卡死 → 超时收紧、去掉 esp_restart、阶段打点 CamMark()
 *   2) 内存门槛误杀 → 门槛只做极端兜底（1024 字节），DMA largest 替代 internal largest
 *   3) 二次拍照识别失败 → 顺序改为【抓帧 → 上传识别 → 语音重连 → 解码上屏】
 *   4) esp-aes 分配失败 / websocket 掉线 → LVGL 分配器改为 PSRAM 优先
 *      （改动在 libraries/lvgl/src/stdlib/clib/lv_mem_core_clib.c，不是本文件）
 *
 * ★★★ 复用时必须同时满足的 5 个条件（缺一不可，踩过坑）：
 *   ① Tools -> USB CDC On Boot = Disabled
 *      S3 只有一个 USB PHY，IO19(D-) / IO20(D+) 给 USB Host 就不能给 USB-CDC 串口。
 *      烧录需按住 BOOT 再点 RST，或拔掉摄像头上电烧。日志走 UART0(TX=43/RX=44)。
 *   ② PSRAM = OPI PSRAM（N16R8 为 8MB），Flash = 16MB
 *   ③ lv_conf.h 中 CONFIG_LV_DRAW_LAYER_SIMPLE_BUF_SIZE = 24576
 *      ★ 绝不能改成 4096：本机 sdkconfig 的 SPIRAM_MALLOC_ALWAYSINTERNAL=4096，
 *        块尺寸 ≤4096 会走【内部 RAM】，24480 字节才走 PSRAM。改小会反向榨干内存。
 *   ④ libraries/lvgl/src/stdlib/clib/lv_mem_core_clib.c 必须是 PSRAM 优先分配器版本
 *      （本目录同名的 .bak_20261125 是原始版本，可参考差异）
 *      否则 LVGL 的 <4KB 小对象全挤内部 RAM，语音 TLS 20KB 拿不到 → esp-aes 报错掉线。
 *   ⑤ 内部 RAM 预算：开机 89KB → 启动后 33KB → UVC 常驻约 15KB → 语音 TLS 需 20KB+
 *      这是结构性紧张，不要再往内部 RAM 加大缓冲；新缓冲一律 heap_caps_malloc(SPIRAM)。
 *
 * 关键实现位置（以下行号对应 2026-11-25 定稿，后续改动后请按函数名搜索）：
 *   4394  CamMark()              阶段打点 + 25s 总预算
 *   4440  camera_frame_cb()      帧回调 + 128KB memcpy 互锁
 *   4532  InitCamera()           UVC 懒启动 + PSRAM/DMA 内存门槛
 *   4752  CamShowPhotoOnScreen() JPEG 解码 → 240x320 RGB565 上屏
 *   5018  CamUploadOnce()        multipart/form-data 上传到 vision 地址
 *   5195  CamUploadAndRespond()  总调度：上传 → 重连 → 上屏 → 回执
 *   5326  CamTakePhoto()         抓帧（不在此解码，顺序见上）
 *   7856  MCP 分发 self.camera.capture
 * ========================================================================== */
#include <Arduino.h>

/* ★★★ USB 摄像头总开关 (2026-10-31)
 *   1 = 启用 UVC 拍照(USB Host 独占 GPIO19/20), 0 = 彻底不编摄像头代码。
 *   关掉后固件行为与本功能加入前完全一致(USB CDC 也要改回 Enabled)。 */
#define ENABLE_USB_CAMERA 1

/* ★★★ 编译期硬保护：USB CDC On Boot 必须与摄像头开关匹配。
 *
 *   ■ 启用摄像头(ENABLE_USB_CAMERA=1)时必须是 Disabled —— 这不是偏好, 是物理约束:
 *     ESP32-S3 只有【一个】USB PHY, GPIO19/20 要么给 USB-Serial-JTAG(串口),
 *     要么给 USB-OTG(Host)。要用 Host 抓摄像头, 就必须把 PHY 让出来。
 *     代价: 日志改走 UART0(TX=43/RX=44), 要接 USB-TTL 适配器才看得到;
 *           而且固件跑起来后 19/20 被 Host 占着, **烧录要先按住 BOOT 再点 RST**
 *           进下载模式(或先把摄像头拔掉再上电烧)。
 *     ★ 板间链路 g_link 是 UART1(GPIO39/40), 与 UART0 无关, 不受此设置影响。
 *
 *   ■ 关掉摄像头(ENABLE_USB_CAMERA=0)时必须是 Enabled —— 2026-10-20 的老坑:
 *     那时 Serial 会退回 UART0, 调试输出容易和串口链路搅在一起, 直接编不过最省事。
 */
#if ENABLE_USB_CAMERA
#if !defined(ARDUINO_USB_CDC_ON_BOOT) || (ARDUINO_USB_CDC_ON_BOOT != 0)
#error "启用摄像头时 Tools -> USB CDC On Boot 必须设为 Disabled: USB Host 要独占 GPIO19/20 的 USB PHY。日志将改走 UART0(43/44)"
#endif
#else
#if !defined(ARDUINO_USB_CDC_ON_BOOT) || (ARDUINO_USB_CDC_ON_BOOT == 0)
#error "Tools -> USB CDC On Boot 必须设为 Enabled, 否则 Serial 会占用 UART0 和 GPIO40/39 板间链路抢同一个串口"
#endif
#endif

#include <WiFi.h>
#include <HTTPClient.h>        // 照片上传到服务器下发的 vision 地址 (multipart/form-data)
#include <WiFiClientSecure.h>  // 视觉接口是 https, 必须走 TLS
#include <cJSON.h>             // 解析 capabilities.vision 与识别结果
#include <esp_system.h>        // esp_restart (语音链路重连失败时的自愈兜底) / esp_reset_reason
#include <esp_attr.h>          // RTC_NOINIT_ATTR (连续启动计数, 不掉电不清)
#include <Wire.h>              // I2C 总线(环境传感器 AHT20/BMP280)
#include <Adafruit_AHTX0.h>    // AHT20 温湿度传感器
#include <Adafruit_BMP280.h>   // BMP280 气压传感器
#include <cstdint>
#include <cctype>   // toupper (作物标准里 KEY 大小写归一)
#include <cstring>  // strncmp (self.led.pick 里解析中文数字用)
#include <time.h>  // configTime / time / localtime (屏保时钟)
#include <variant>  // std::holds_alternative (FilteredObserver 过滤 TextReceivedEvent)
#include <driver/spi_common.h>
#include <esp_heap_caps.h>
#include <freertos/FreeRTOS.h>  // xTaskGetHandle / vTaskPrioritySet / vTaskDelay (音频任务提权)
#include <freertos/task.h>
#include <esp_lcd_panel_io.h>
#include <esp_lcd_panel_ops.h>
#include <esp_lcd_panel_vendor.h>
#include <esp_log.h>  // esp_log_level_set/get: 传感器探测期临时压掉 i2c.master 的噪声日志
#if ENABLE_USB_CAMERA
#include <usb_stream.h>  // USB UVC Host 驱动 (摄像头, USB1.1 Full-Speed)
#include "tjpgd.h"       // TJpgDec JPEG 解码器 (usb_stream 库自带)
#endif

#include "ai_vox_engine.h"
#include "audio_device/audio_input_device_i2s_std.h"
#include "audio_device/audio_input_device_pdm.h"
#include "audio_device/audio_output_device_i2s_std.h"
#include "components/espressif/button/button_gpio.h"
#include "components/espressif/button/iot_button.h"
#include "components/espressif/esp_audio_codec/esp_audio_simple_dec.h"
#include "components/espressif/esp_audio_codec/esp_mp3_dec.h"
#include "components/wifi_configurator/wifi_configurator.h"
#include "display.h"
#include "led_strip.h"
#include "network_config_mode_mp3.h"
#include "network_connected_mp3.h"
#include "notification_0_mp3.h"

#ifndef ARDUINO_ESP32S3_DEV
#error "This example only supports ESP32S3-Dev board."
#endif

#define AUDIO_INPUT_DEVICE_TYPE_PDM (0)
#define AUDIO_INPUT_DEVICE_TYPE_I2S_STD (1)
#define AUDIO_INPUT_DEVICE_TYPE AUDIO_INPUT_DEVICE_TYPE_PDM  // 鹿小班二合一模块的麦克风为 LMD2718 PDM 麦克风, 须用 PDM 模式
// #define AUDIO_INPUT_DEVICE_TYPE AUDIO_INPUT_DEVICE_TYPE_I2S_STD

/**
 *  开机自动连接WiFi, 无需配网: 取消下面两个宏的注释并填写实际的WiFi名称和密码
 *  若需要改为SmartConfig配网, 注释掉这两个宏即可
 */
#define WIFI_SSID "YOUR_WIFI_SSID"
#define WIFI_PASSWORD "YOUR_WIFI_PASSWORD"

namespace {
/**
 *  SC_TYPE_ESPTOUCH            protocol: ESPTouch
 *  SC_TYPE_AIRKISS,            protocol: AirKiss
 *  SC_TYPE_ESPTOUCH_AIRKISS,   protocol: ESPTouch and AirKiss
 *  SC_TYPE_ESPTOUCH_V2,        protocol: ESPTouch v2
 */
constexpr smartconfig_type_t kSmartConfigType = SC_TYPE_ESPTOUCH_AIRKISS;  // ESPTouch and AirKiss

// Microphone pin configurations
#if AUDIO_INPUT_DEVICE_TYPE == AUDIO_INPUT_DEVICE_TYPE_I2S_STD
constexpr gpio_num_t kMicPinSck = GPIO_NUM_6;  // SCK (BCK, BCLK): Serial-Data Clock for I2S Interface
constexpr gpio_num_t kMicPinWs = GPIO_NUM_5;   // WS (WR, WCLK): Serial Data-Word Select for I2S Interface
constexpr gpio_num_t kMicPinSd = GPIO_NUM_4;   // SD (DIN, DOUT, DI, DO, DATA): Serial-Data Output for I2S Interface
#elif AUDIO_INPUT_DEVICE_TYPE == AUDIO_INPUT_DEVICE_TYPE_PDM
constexpr gpio_num_t kMicPinSck = GPIO_NUM_15;  // PDM CLK: 麦克风时钟
constexpr gpio_num_t kMicPinSd = GPIO_NUM_7;    // PDM DATA: 麦克风数据
#endif

constexpr gpio_num_t kSpeakerPinSck = GPIO_NUM_9;  // SCK (BCK, BCLK): Serial-Data Clock for I2S Interface
constexpr gpio_num_t kSpeakerPinWs = GPIO_NUM_10;  // WS (WR, WCLK): Serial Data-Word Select for I2S Interface
constexpr gpio_num_t kSpeakerPinSd = GPIO_NUM_11;  // SD (DIN, DOUT, DI, DO, DATA): Serial-Data Output for I2S Interface

constexpr gpio_num_t kButtonBoot = GPIO_NUM_0;

constexpr gpio_num_t kDisplayBacklightPin = GPIO_NUM_46;
constexpr gpio_num_t kDisplayMosiPin = GPIO_NUM_17;
constexpr gpio_num_t kDisplayClkPin = GPIO_NUM_16;
constexpr gpio_num_t kDisplayDcPin = GPIO_NUM_8;
constexpr gpio_num_t kDisplayRstPin = GPIO_NUM_18;
// 新屏为八针 ST7789 (GND/3V3/SCL/SDA/DC/RES/CS/BLK), 比七针版多一个 CS 引脚
// CS 由 ESP 侧管理: esp_lcd 在每次 SPI 传输时自动拉低/拉高 CS, 分配 GPIO21
constexpr gpio_num_t kDisplayCsPin = GPIO_NUM_21;  // 新屏 CS

// ---------------- 板间串口通讯 (UART1, TX=IO39 / RX=IO40) ----------------
// 用途: 与另一块 ESP32-S3 / ESP32 做【点对点串口通讯】, 双向收发。
//       2026-10-07 起一直为串口预留这两个脚, 2026-10-14 现场复核确认未被占用。
//
// ★ 接线三要素(接错就是"永远收不到、且两边都不报错"这种闷声坑):
//   1. 交叉: 本机 TX(39) -> 对方 RX; 本机 RX(40) -> 对方 TX   (同名相接是错的!)
//   2. 共地: 本机 GND 必须与对方 GND 相连 (不共地 = 没有参考电平 = 乱码或完全无数据)
//   3. 电平: 本机是 3.3V TTL。若对方是 5V 逻辑(如 Arduino UNO), 必须分压或加电平转换。
//
// ★ 关于 39/40 的特殊性: 在 ESP32-S3 上它们兼作【外部 JTAG】(MTCK/MTDO)。
//   本工程默认用芯片内置的 USB-Serial/JTAG 调试通道, 所以这两脚可自由用作 UART。
//   (官方文档: 若 USB Serial/JTAG 控制器用于调试, GPIO39~GPIO42 可用于其他功能)
//   唯一代价: 若将来要接【外部 JTAG 调试器】, 需把串口挪到别的脚。
constexpr gpio_num_t kLinkRxPin = GPIO_NUM_40;  // 本机接收 <- 接对方 TX
constexpr gpio_num_t kLinkTxPin = GPIO_NUM_39;  // 本机发送 -> 接对方 RX
constexpr uint32_t kLinkBaud = 115200;          // 波特率, 两端必须一致
constexpr uint32_t kLinkFrameIdleMs = 200;      // 对端不带换行时的"帧结束"判定: 空闲这么久即视为一帧

// ---- 终端灯握手指令 (本机 = 主控, 对端 = 终端 B 板) ----
// 协议约定(两端必须一致):
//   主控 → 终端: "LED_ON"  要求终端点亮白色灯
//   主控 → 终端: "LED_OFF" 要求终端熄灯
//   终端 → 主控: "LED_ON_OK"  表示"我确实已经点亮了白色灯"
//   终端 → 主控: "LED_OFF_OK" 表示"我确实已经熄灯了"
// ★ 为什么回执不是简单 ACK: 主控要拿回执【内容】判断"是对哪条指令的确认",
//   如果只回一个 ACK, 主控分不清这是"开灯的确认"还是"上一条遗留的回执"。
constexpr const char* kCmdTerminalLedOn = "LED_ON";
constexpr const char* kCmdTerminalLedOff = "LED_OFF";
constexpr const char* kAckTerminalLedOn = "LED_ON_OK";
constexpr const char* kAckTerminalLedOff = "LED_OFF_OK";
constexpr uint32_t kTerminalAckTimeoutMs = 1500;  // 等回执超时: 超过即判定"终端没有响应"
constexpr int kHandshakeLedIndex = 0;             // 握手成功后点亮第几颗 WS2812 (0 = 第 1 颗)

// ---- 终端 OLED 显示数字 (2026-10-17 新增) ----
// 协议:
//   主控 → 终端: "OLED:123"   要求终端在 OLED 上居中大字显示 123
//   终端 → 主控: "OLED_OK:123" 回执, 带显示的数值便于主控核对
// ★ 数值范围限制: 终端用"位数自适应字号"显示, 6 位以内最舒服;
//   超过 6 位字号会缩到很小, 所以这里直接限制 0~999999, 超范围报错而不发指令。
constexpr const char* kCmdTerminalOledPrefix = "OLED:";
constexpr const char* kAckTerminalOledPrefix = "OLED_OK:";
constexpr int32_t kTerminalOledMin = 0;
constexpr int32_t kTerminalOledMax = 999999;

// ---- 终端温湿度查询 (2026-10-17 新增) ----
// ★★ 为什么用"终端主动上报 + 主控缓存"而不是"主控问一次等一次回":
//   1. MCP 工具回调里绝不能 delay 等串口回执(会卡住音频/语音事件);
//   2. 如果改成"问了等回", 工具就必须异步返回, AI 拿不到值就没法一次把话说完;
//   3. 终端本来就要每 2 秒上报一次(见终端固件), 主控只要一直缓存最新值,
//      用户问的时候【立刻】就能给出确切的数字 —— 零延迟、零超时风险。
//   协议(终端 → 主控, 每 2 秒一次): "ENV,T=29.6,H=52.3"
//   另支持主控主动问: 发 "ENV?" -> 终端立刻回一条 ENV,...
constexpr const char* kAckTerminalEnvPrefix = "ENV,";
constexpr uint32_t kTerminalEnvStaleMs = 10000;  // 超过这么久没收到上报, 判为"数据过期"

// ---- 终端全量数据上报 (2026-10-20 新增) ----
// 协议(终端 → 主控, 每 2 秒一次):
//   "ALL,T=24.7,H=74.3,PR=1012.1,CO2=412,LX=320,ST=27.1,SH=62.8,N=72,P=45,K=88,PH=7.0"
//   T=空气温度 ℃ / H=空气湿度 % / PR=气压 hPa / CO2=空气CO2 ppm / LX=光照 lx /
//   ST=土壤温度 ℃ / SH=土壤湿度 % / N=氮 / P=磷 / K=钾 / PH=土壤PH
//   ★ P 表示"磷"而不是气压 —— 气压另用 PR, 避免两个 P 撞车。
// ★ 无效值约定 -999: 传感器没接/读失败时终端发 -999, 主控见到 < -100 就当无效(显示 --)。
constexpr const char* kAckTerminalAllPrefix = "ALL,";

// ★ 必须用 UART1, 【绝不能】用 UART0: UART0(43/44) 被 USB 日志/下载占用,
//   抢它会导致"烧录后持续复位、串口日志总在同一行截断"。
HardwareSerial g_link_serial(1);

// ---------------- 音频任务提权(残余卡顿根治, 2026-10-16) ----------------
// ★ 前置声明放在【匿名 namespace 之外】—— 见下方 `}  // namespace` 之后的声明。
//   踩过的坑(2026-10-16 编译报错 undefined reference to
//   `(anonymous namespace)::EnsureAudioTaskPriority()`):
//     声明写在 namespace 内、定义写在 namespace 外 → 两者是【不同符号】,
//     链接器找匿名空间版本却找不到定义 → undefined reference。
//   教训: 声明与定义必须处于【同一个命名空间】。本工程 setup()/loop() 在 namespace 外,
//   所以给它们调用的函数, 声明也必须放 namespace 外。

// ---------------- WS2812 RGB 灯链 (GPIO38, RMT 驱动) ----------------
// IO38 级联 8 颗 WS2812: IO38 -> R8 -> RGB1 -> RGB2 -> ... -> RGB8
// ★ 2026-10-14 更新: 灯链已从 6 颗扩到 8 颗(用户 2026-10-14 16:18 实测 6 颗开关正常后扩测)。
// ★ 2026-10-14 更正: 现已确认【没有板载灯】, 原"DevKitC 板载灯与 RGB1 并联"的注释作废。
// ★★ 灯故障已解决(2026-10-14): 真因是 InitWs2812() 里手动 pinMode()+digitalWrite()
//    产生的 IO MUX 毛刺被 WS2812 当成数据, 已删除。教训: WS2812 这类自定时器件
//    的数据脚【绝不要手动翻转】, 时序全交给 led_strip 驱动。
constexpr gpio_num_t kWs2812Pin = GPIO_NUM_38;
constexpr uint32_t kWs2812Count = 8;  // 级联灯珠数量 (RGB1~RGB8)

// ★ 调试开关: 开机时逐颗点亮自检。默认【关闭】。
//   需要确认灯珠数量、级联顺序是否正确时, 改成 true 编译一次。
//   逐颗点亮每颗 300ms(8 颗约 2.4 秒); 验证完记得改回 false。
//   判读: 串口打到第 N 颗 vs 眼睛看到第几颗 —— 数目一致说明级联完整。
constexpr bool kLedSelfTestOnBoot = false;

// 灯效模式
enum class LedMode {
  kOff,      // 熄灭
  kWhite,    // 常亮白灯
  kBreath,   // 呼吸灯(白)
  kRainbow,  // 彩色彩虹流水(整条带子铺满彩虹并滚动)
  kChase,    // 跑马灯(单颗亮点在灯链上循环跑动)
  kCustom,   // 指定灯亮(只点亮用户点名的若干颗, 颜色统一)
};

// 白灯亮度(0~255): 全白 255,255,255 太刺眼且发热, 用柔和白
constexpr uint8_t kWhiteLevel = 200;
// 呼吸灯参数: 单次呼吸(渐亮+渐暗)周期, 以及最低/最高亮度
constexpr uint32_t kBreathPeriodMs = 2400;  // 一次完整呼吸 2.4 秒
constexpr uint8_t kBreathMinLevel = 8;      // 最暗(接近灭但不断流)
constexpr uint8_t kBreathMaxLevel = 200;    // 最亮

// 跑马灯参数
constexpr uint32_t kRainbowStepMs = 80;   // 每步间隔(ms): 越小跑得越快, 80ms 约 12.5 步/秒
constexpr uint8_t kRainbowLevel = 180;    // 灯珠亮度上限(0~255): 彩色不要开到 255, 以免刺眼/发热
constexpr uint8_t kRainbowHueStep = 32;   // 相邻灯珠色相步进(0~255 一圈): 32 = 每颗差 45°, 8 颗正好铺满 360°

// 彩灯跑马灯参数(3 颗亮点一起跑)
constexpr uint32_t kChaseStepMs = 400;    // 每步间隔(ms): 越大跑得越慢。
                                          // ★ 2026-10-15 用户反馈"太快了", 从 150 调慢到 400。
                                          //   想更慢就继续调大(如 600/800), 想快就调小。
constexpr uint32_t kChaseDotCount = 3;    // 亮点个数: 3 颗彩灯一起跑
constexpr uint8_t kChaseDotLevel = 200;   // 亮点亮度(0~255)

// ★★ 2026-10-16 卡顿优化: LED 灯效统一刷新间隔。
//   原为每处各自写死 20ms(50Hz)。改为 33ms(约 30Hz) —— 动效观感无差别,
//   但 led_strip_refresh() 是 RMT 同步阻塞调用, 降频可显著减少它对
//   I2S 音频 DMA 喂数窗口的挤占(真因分析见 loop() 末尾与 setup() 末尾的注释)。
constexpr uint32_t kLedRefreshIntervalMs = 33;

// 指定灯参数
constexpr uint8_t kCustomLevel = 200;     // "打开第N个灯"时的峰值亮度
constexpr bool kCustomBreath = true;      // 指定灯是否做呼吸效果(淡入淡出); false = 常亮
constexpr uint8_t kCustomSat = 255;       // 指定灯的饱和度(0~255): 255 = 饱和色; 0 = 纯白
constexpr uint8_t kCustomHue = 85;        // 默认色相(0~255): 仅当没说颜色时使用; 85 ≈ 绿

// 颜色名的色相表(0~255 色相环): 供 self.led.pick 的 color 参数使用
//   红 0 / 黄 43 / 绿 85 / 青 128 / 蓝 170 / 紫 213 / 白 = 特殊(饱和度0)
constexpr uint8_t kHueRed = 0;
constexpr uint8_t kHueYellow = 43;
constexpr uint8_t kHueGreen = 85;
constexpr uint8_t kHueCyan = 128;
constexpr uint8_t kHueBlue = 170;
constexpr uint8_t kHuePurple = 213;

constexpr auto kDisplaySpiMode = 3;  // ST7789 串口协议用 SPI mode 3 (CPOL=1, CPHA=1); 若新屏点亮异常(白屏/花屏)可改试 mode 0
constexpr uint32_t kDisplayWidth = 240;
constexpr uint32_t kDisplayHeight = 320;  // 已确认新屏为 240x320 竖屏; max_transfer_sz 按宽x高自动计算
constexpr bool kDisplayMirrorX = false;
constexpr bool kDisplayMirrorY = false;
constexpr bool kDisplayInvertColor = false;  // ★ 实测确认(2026-10-13 照片): true 时所有颜色显示成补色(白→黑、绿气泡→紫、蓝字→橙黄), 本屏必须 false
constexpr bool kDisplaySwapXY = false;
constexpr auto kDisplayRgbElementOrder = LCD_RGB_ELEMENT_ORDER_RGB;

// ★★ 2026-10-16 卡顿优化(第 3 刀): 自定义 Observer, 把"用不到的 TextReceivedEvent"
//   在【入队前】就丢掉, 而不是推进队列再被主循环搬走。
//
//   背景(SDK 源码 src/core/ai_vox_engine_impl.cpp 第 284~295 行):
//     WebSocket 【每一个文本帧】都会 PushEvent(TextReceivedEvent{整帧 JSON})。
//     一次对话服务端下发几十~上百帧(唤醒确认/STT 中间结果/LLM 文本流/TTS 元数据/
//     MCP 调用/情感标记/结束帧...), 每帧几百 B~几 KB。
//   上一轮(2026-10-15)已把"每帧 printf 出去"关掉, 那是最大头;
//   但【入队 + 主循环每圈把整个 deque 搬走】这两步仍然存在:
//     · PushEvent 要构造 std::string(拷贝整帧字节) → 堆分配
//     · PopEvents 每圈 return std::move(event_queue_) → 搬运包含这些 string 的 deque
//     两者的代价与"帧数 × 帧长"成正比, 正是残余卡顿的可疑来源之一。
//
//   本工程从【不消费】TextReceivedEvent(仅调试时才打印), 所以在入队处直接丢弃它
//   是最彻底的省法 —— 此后 SDK 侧只多一次 variant 类型判断, 不产生任何堆分配,
//   主循环也不再搬运这些字符串。
//
//   ★ 为什么可以安全重写: Observer::PushEvent 是 virtual 的, 派生类覆盖只影响
//     本工程自己的 g_observer 实例, 不碰 SDK 源码、不影响其它示例。
//   ★ 需要抓协议原文时, 把 kKeepTextReceivedEvent 改成 true 重新编译即可(行为回到原样)。
//
// ★★ 官方图像识别入口 (2026-10-31, 对齐 xiaozhi-esp32 mcp_server.cc ParseCapabilities):
//   服务器在 MCP initialize 的返回里把视觉接口下发给设备:
//     "capabilities": { "vision": { "url": "https://.../vision/explain", "token": "..." } }
//   设备拍照后把 JPEG 以 multipart/form-data POST 到该 url, 响应即识别结果。
//   这套方案不需要 API Key、不需要自己买大模型, 是小智官方固件同款路径。
//   ★ 唯一难点: 这条消息【只出现一次】(连接握手时), 而本工程的 Observer 把
//     TextReceivedEvent 直接丢弃了 —— 所以在丢弃之前先做一次子串预筛
//     ('vision' 出现的帧极少, 99% 的帧只多一次 memmem, 不影响卡顿优化)。
#if ENABLE_USB_CAMERA
static std::string s_explain_url;    // 服务器下发的视觉识别 URL (空 = 未配置)
static std::string s_explain_token;  // 对应的 Bearer token (可为空)

// 视觉接口地址的来源优先级: ① 服务器下发(正常路径, 无需填任何东西)
//                            ② 下面这个兜底常量(只有服务器确实不下发时才手填)
// ★ 正常别填: 填了就等于绕开官方下发, token 也要自己想办法, 容易踩坑。
static constexpr const char* kCamExplainUrlFallback = "";
static const char* CamExplainUrl() {
  return s_explain_url.empty() ? kCamExplainUrlFallback : s_explain_url.c_str();
}

// 在【单个】JSON 对象里找 vision: 既认 obj.vision, 也认 obj.capabilities.vision
static cJSON* CamVisionInObj(cJSON* obj) {
  if (!cJSON_IsObject(obj)) return nullptr;
  cJSON* v = cJSON_GetObjectItem(obj, "vision");
  if (cJSON_IsObject(v)) return v;
  cJSON* caps = cJSON_GetObjectItem(obj, "capabilities");
  if (cJSON_IsObject(caps)) {
    v = cJSON_GetObjectItem(caps, "vision");
    if (cJSON_IsObject(v)) return v;
  }
  return nullptr;
}

// 从一帧 JSON 原文里抠出 capabilities.vision.url / token。
// ★★ 关键(2026-10-31 实机踩坑): 小智协议的 MCP 帧是【包了一层】的 ——
//     {"type":"mcp","payload":{"jsonrpc":"2.0","id":1,"method":"initialize",
//                              "params":{"capabilities":{"vision":{...}}}}}
//    SDK 的 OnJsonData 解析的是 "payload" 里面那个对象(见 engine_impl OnMcpJsonObj),
//    而 Observer 拿到的是【整帧原文】, 所以要自己先钻进 payload 一层。
//    只查根上的 params/result 永远查不到 —— 症状就是"每次都被判成服务器没下发"。
static cJSON* CamFindVisionObj(cJSON* root) {
  if (root == nullptr) return nullptr;
  cJSON* hit = CamVisionInObj(root);        // 裸 JSON-RPC 帧(有些实现不包)
  if (hit != nullptr) return hit;

  const char* outer_keys[] = {"params", "result"};
  for (const char* k : outer_keys) {        // 裸帧: params.capabilities.vision
    hit = CamVisionInObj(cJSON_GetObjectItem(root, k));
    if (hit != nullptr) return hit;
  }

  cJSON* payload = cJSON_GetObjectItem(root, "payload");  // ★ 小智协议: type=mcp 的包装层
  if (cJSON_IsObject(payload)) {
    hit = CamVisionInObj(payload);
    if (hit != nullptr) return hit;
    for (const char* k : outer_keys) {      // payload.params.capabilities.vision
      hit = CamVisionInObj(cJSON_GetObjectItem(payload, k));
      if (hit != nullptr) return hit;
    }
  }
  return nullptr;
}

// 打印一帧的片段用于诊断(只在命中关键词时调, 不会刷屏)。
// 截断到 460 字节: 够看清 method / capabilities 结构, 又不会把串口灌满。
static void CamLogFrameExcerpt(const char* why, const std::string& text) {
  printf("[cam] frame(%s) %uB: %.460s\n", why, (unsigned)text.size(), text.c_str());
}

static void CamParseVisionConfig(const std::string& text) {
  const bool is_initialize = text.find("\"initialize\"") != std::string::npos;
  cJSON* root = cJSON_Parse(text.c_str());
  if (root == nullptr) {
    printf("[cam] vision parse: cJSON_Parse failed (%uB)\n", (unsigned)text.size());
    if (is_initialize) CamLogFrameExcerpt("init-parse-fail", text);
    return;
  }
  cJSON* vision = CamFindVisionObj(root);
  if (cJSON_IsObject(vision)) {
    cJSON* url = cJSON_GetObjectItem(vision, "url");
    if (cJSON_IsString(url) && url->valuestring[0] != '\0') {
      s_explain_url = url->valuestring;
      cJSON* token = cJSON_GetObjectItem(vision, "token");
      s_explain_token = cJSON_IsString(token) ? token->valuestring : "";
      printf("[cam] vision url set: %s (token %s)\n", s_explain_url.c_str(),
             s_explain_token.empty() ? "absent" : "present");
    } else {
      printf("[cam] vision object found but url missing/empty\n");
      CamLogFrameExcerpt("vision-no-url", text);
    }
  } else if (text.find("vision") != std::string::npos) {
    // 帧里提到 vision 但结构不在预期位置 —— 打出来照着实测改解析
    printf("[cam] vision mentioned but not found where expected\n");
    CamLogFrameExcerpt("vision-unknown-shape", text);
  } else if (is_initialize) {
    // ★ 决定性证据: initialize 帧里连 vision 字样都没有 = 服务器确实不下发
    printf("[cam] WARN: server initialize did NOT provide capabilities.vision -> "
           "image recognition unavailable (device not activated, or server without vision)\n");
    CamLogFrameExcerpt("init-no-vision", text);
  }
  cJSON_Delete(root);
}
#endif

class FilteredObserver : public ai_vox::Observer {
 public:
  void PushEvent(ai_vox::Event&& event) override {
    if (std::holds_alternative<ai_vox::TextReceivedEvent>(event)) {
#if ENABLE_USB_CAMERA
      // ★ 官方视觉方案: 丢弃之前先预筛解析 capabilities.vision (其余帧零成本)。
      //   预筛词用 "capabilities" 而不是 "vision": initialize 帧即便【没有】vision,
      //   也要走到解析里把"服务器没下发"这件事明确打印出来, 否则排查全靠猜。
      const auto& content = std::get<ai_vox::TextReceivedEvent>(event).content;
      if (content.find("vision") != std::string::npos ||
          content.find("capabilities") != std::string::npos) {
        CamParseVisionConfig(content);
      }
#endif
      return;  // 直接丢弃: 本工程不用它, 省掉入队与搬运开销
    }
    ai_vox::Observer::PushEvent(std::move(event));
  }
};

std::unique_ptr<Display> g_display;
auto g_observer = std::make_shared<FilteredObserver>();
auto g_audio_output_device = std::make_shared<ai_vox::AudioOutputDeviceI2sStd>(kSpeakerPinSck, kSpeakerPinWs, kSpeakerPinSd);
button_handle_t g_button_boot_handle = nullptr;

// ---------------- 语音"待命"的延迟断链 (2026-10-31) ----------------
// 需求: 说"待命" → 屏幕立刻进屏保 + 主程序进待命状态(断开语音会话) → 按 BOOT 键唤醒。
// ★ 为什么要延迟: 断开连接会把音频引擎一起 reset, AI 那句"好的, 已进入待命"会当场被掐掉。
//   所以屏保【立刻】显示(用户马上看到反馈), 断链【延后】2.5 秒(等那句话播完)。
//   由 loop 轮询执行 —— 不能用 vTaskDelay 直接在 MCP 回调里睡, 那会卡住整条 loop
//   (音频任务虽能抢占, 但 LVGL 渲染与事件处理都会停)。
static bool g_standby_pending = false;
static uint32_t g_standby_at_ms = 0;
// ★ 区分"待命"的两条进入路径(2026-11-02 用户纠正):
//   · false = 语音"待命"(self.display.standby) → 屏保【已经】在 MCP 回调里上好了, 断链时不用再上;
//   · true  = 聆听/说话时按 BOOT 键 → 只断链进 kStandby(顶部显示"待命"), 【不进屏保】。
//   loop 里到点后据此决定要不要兜底再上一次屏保。
static bool g_standby_from_boot = false;
constexpr uint32_t kStandbySuspendDelayMs = 2500;

// ---------------- WS2812 灯控 ----------------

led_strip_handle_t g_led_strip = nullptr;
LedMode g_led_mode = LedMode::kOff;

// ---------------- 指定灯亮 (self.led.pick) ----------------
// 用一颗 bit 表示一颗灯是否被点名: bit0=RGB1 ... bit7=RGB8, 共 8 颗。
// 用位图而不是 int 数组, 是因为"打开 3 6 8"这种需求天然就是"选中集合", 位图最直观,
// 判断"某颗是否亮"只需 (mask >> i) & 1, 且一颗灯的开关就是一次按位或/与。
uint8_t g_led_pick_mask = 0x00;  // 0 = 全部不亮
// 被点名【每颗灯各自】的颜色(HSV 的色相/饱和度)。
// ★★ 2026-10-15 修正: 原来是两个标量(g_led_pick_hue/sat), 全链共用一个颜色 ——
//   "让第一变红 第二变绿"这种"一句多色"需求表达不了(后说的色覆盖全部, 且若拆成两次
//   工具调用, 第二次只带一个灯号会把 mask 覆盖掉 → 前一颗被熄灭, 正是用户报的现象)。
//   改成按灯索引的数组: 每颗灯独立记自己的色相/饱和度, mask 只负责"这颗亮不亮"。
uint8_t g_led_pick_hue[kWs2812Count] = {0};
uint8_t g_led_pick_sat[kWs2812Count] = {0};

// 把 8 颗灯的默认颜色都设为绿色(与 kCustomHue/kCustomSat 一致)。
// ★ 为什么不直接给数组写初值: 数组初值只能写常量列表, 而颜色应当从
//   kCustomHue/kCustomSat 推导, 用函数初始化可以保持"只有一处定义默认色"。
//   由 InitWs2812() 在灯链就绪后调用。
static void InitLedPickColors() {
  for (uint32_t i = 0; i < kWs2812Count; ++i) {
    g_led_pick_hue[i] = kCustomHue;
    g_led_pick_sat[i] = kCustomSat;
  }
}

// 点亮全部 8 颗为指定亮度(0~255)的白色: WS2812 无独立白光通道, R=G=B 即为白
void LedShowWhite(uint8_t level) {
  if (g_led_strip == nullptr) {
    return;
  }
  for (uint32_t i = 0; i < kWs2812Count; ++i) {
    ESP_ERROR_CHECK(led_strip_set_pixel(g_led_strip, i, level, level, level));
  }
  ESP_ERROR_CHECK(led_strip_refresh(g_led_strip));
}

// 熄灭: 发一帧全 0 把灯链置灭。
// ★★ 2026-10-14 二次修正: 上一版改成了"连发 4 帧 + 帧间 delayMicroseconds(600)"，
//   现【回退为单帧】。理由:
//     · led_strip_clear() 发出的帧【本身就带合法的 reset code】
//       (驱动内部 reset_code = 280us 低电平, 见 led_strip_rmt_encoder.c),
//       所以单帧已经足够让灯珠正确锁存并复位, 不需要外部再补延时。
//     · 帧间那 600us 是"人肉造"的 reset, 与驱动自己的 reset 叠加属于冗余;
//       虽然无害, 但增加了我方代码出错的面(上一版的 pinMode 毛刺就是这么来的)。
//     · 原则: 【时序交给驱动, 我方不做任何手动电平操作】。
void LedShowOff() {
  if (g_led_strip == nullptr) {
    return;
  }
  ESP_ERROR_CHECK(led_strip_clear(g_led_strip));
}

// HSV -> RGB: h 为 0~255 的色相环(hue wheel), s/v 为 0~255 的饱和度/亮度
// 说明: 用色相环而不是固定调色板, 8 颗灯珠可以拿到"连续渐变"的颜色, 像真正的彩虹流水
static void HsvToRgb(uint8_t h, uint8_t s, uint8_t v, uint8_t& r, uint8_t& g, uint8_t& b) {
  if (s == 0) {  // 饱和度为 0 → 灰/白
    r = g = b = v;
    return;
  }
  const uint8_t region = h / 43;               // 255/6 ≈ 42.5, 分成 6 个扇区
  const uint8_t remainder = (h - region * 43) * 6;  // 扇区内插值 0~255
  const uint8_t p = static_cast<uint8_t>((v * (255 - s)) >> 8);
  const uint8_t q = static_cast<uint8_t>((v * (255 - ((s * remainder) >> 8))) >> 8);
  const uint8_t t = static_cast<uint8_t>((v * (255 - ((s * (255 - remainder)) >> 8))) >> 8);
  switch (region) {
    case 0: r = v; g = t; b = p; break;
    case 1: r = q; g = v; b = p; break;
    case 2: r = p; g = v; b = t; break;
    case 3: r = p; g = q; b = v; break;
    case 4: r = t; g = p; b = v; break;
    default: r = v; g = p; b = q; break;
  }
}

// 跑马灯的一帧: 每颗灯珠取色相环上的一个点, 整条带子随时间滚动
static void LedShowRainbowFrame(uint32_t tick) {
  if (g_led_strip == nullptr) {
    return;
  }
  // 每走一步(step), 色相整体前移一点 → 形成"流动"观感
  const uint32_t step = tick / kRainbowStepMs;
  const uint8_t base_hue = static_cast<uint8_t>((step * 8) & 0xFF);  // 每步整体转 8/256 圈
  for (uint32_t i = 0; i < kWs2812Count; ++i) {
    const uint8_t hue = static_cast<uint8_t>(base_hue + i * kRainbowHueStep);
    uint8_t r = 0, g = 0, b = 0;
    HsvToRgb(hue, 255, kRainbowLevel, r, g, b);
    ESP_ERROR_CHECK(led_strip_set_pixel(g_led_strip, i, r, g, b));
  }
  ESP_ERROR_CHECK(led_strip_refresh(g_led_strip));
}

// 把 g_led_pick_mask 画到灯链上: 被点名的颗【各按自己的颜色】亮起, 其余全灭。
// ★ 亮度由参数传入(而不是写死 kCustomLevel) —— 这样才能被呼吸效果每帧改成不同值。
// ★ 颜色是【逐颗独立】的: 第 i 颗用 g_led_pick_hue[i] / g_led_pick_sat[i],
//   所以"第1颗红 + 第2颗绿"可以同时成立(这是 2026-10-15 修正的核心)。
void LedShowPickMaskAt(uint8_t level) {
  if (g_led_strip == nullptr) {
    return;
  }
  for (uint32_t i = 0; i < kWs2812Count; ++i) {
    if ((g_led_pick_mask >> i) & 0x01) {
      uint8_t r = 0, g = 0, b = 0;
      HsvToRgb(g_led_pick_hue[i], g_led_pick_sat[i], level, r, g, b);
      ESP_ERROR_CHECK(led_strip_set_pixel(g_led_strip, i, r, g, b));
    } else {
      ESP_ERROR_CHECK(led_strip_set_pixel(g_led_strip, i, 0, 0, 0));
    }
  }
  ESP_ERROR_CHECK(led_strip_refresh(g_led_strip));
}

// 用当前颜色 + 峰值亮度画一帧。常亮场景与呼吸场景的"最亮那一帧"都用它。
void LedShowPickMask() {
  LedShowPickMaskAt(kCustomLevel);
}

// 颜色名 -> 色相/饱和度。命中返回 true; 未知颜色返回 false(调用方保持原色)。
// 接受中文名和英文名两种写法, 方便 AI 用任一形式传参。
static bool LedColorFromName(const char* name, uint8_t& hue, uint8_t& sat) {
  if (name == nullptr) {
    return false;
  }
  const std::string s(name);
  if (s == "red"    || s == "红" || s == "红色" || s == "红瓦" || s == "红多") { hue = kHueRed;    sat = 255; return true; }
  if (s == "green"  || s == "绿" || s == "绿色" || s == "绿灯")                 { hue = kHueGreen;  sat = 255; return true; }
  if (s == "blue"   || s == "蓝" || s == "蓝色" || s == "蓝灯")                 { hue = kHueBlue;   sat = 255; return true; }
  if (s == "yellow" || s == "黄" || s == "黄色" || s == "黄灯")                 { hue = kHueYellow; sat = 255; return true; }
  if (s == "cyan"   || s == "青" || s == "青色" || s == "青蓝")                 { hue = kHueCyan;   sat = 255; return true; }
  if (s == "purple" || s == "紫" || s == "紫色")                                { hue = kHuePurple; sat = 255; return true; }
  if (s == "white"  || s == "白" || s == "白色" || s == "白灯")                 { hue = 0;          sat = 0;   return true; }
  return false;
}

// 把【全部已点名】的灯统一设为某颜色(对应只有 color、没有逐灯 pairs 的场景)。
static void LedPickSetAllColor(const char* name) {
  uint8_t hue = 0, sat = 0;
  if (!LedColorFromName(name, hue, sat)) {
    printf("[LED] pick: unknown color '%s', keep default\n", name != nullptr ? name : "(null)");
    return;
  }
  for (uint32_t i = 0; i < kWs2812Count; ++i) {
    g_led_pick_hue[i] = hue;
    g_led_pick_sat[i] = sat;
  }
}

// 把【单颗】灯设为某颜色(对应逐灯 pairs 场景)。index 为 0 基。
static void LedPickSetOneColor(int index, const char* name) {
  if (index < 0 || index >= static_cast<int>(kWs2812Count)) {
    return;
  }
  uint8_t hue = 0, sat = 0;
  if (!LedColorFromName(name, hue, sat)) {
    printf("[LED] pick: unknown color '%s' for led %d, keep default\n", name != nullptr ? name : "(null)", index + 1);
    return;
  }
  g_led_pick_hue[index] = hue;
  g_led_pick_sat[index] = sat;
}

// 指定灯的呼吸刷新: 由 loop 以约 20ms 周期调用, 仅在 kCustom 模式且开启了呼吸时动作。
// ★ 与白光呼吸(LedBreathUpdate)同一套三角波思路, 只是颜色换成 g_led_pick_* 指定的色。
//   三角波: 亮度在 kBreathMinLevel ~ kCustomLevel 之间来回推, 保证"不断流"(最暗不完全灭)。
void LedPickBreathUpdate() {
  if (g_led_mode != LedMode::kCustom || !kCustomBreath) {
    return;
  }
  static uint32_t s_last_tick = 0;
  // ★★ 2026-10-16 卡顿优化: 刷新间隔 20ms -> kLedRefreshIntervalMs(33ms)。
  //   理由: 本函数是 loop 里【最频繁的同步阻塞源】—— 每 20ms 无条件调一次
  //   led_strip_refresh()(RMT 阻塞), 与音频任务的 DMA 喂数窗口抢 CPU。
  //   30Hz 对"呼吸"这种人眼感知为缓变的动效已经足够顺滑(呼吸周期 2.4s,
  //   一帧才变化约 3% 亮度), 降到 30Hz 观感无差异, 但 RMT 占用减少 40%。
  const uint32_t now = millis();
  if (s_last_tick != 0 && now - s_last_tick < kLedRefreshIntervalMs) {
    return;
  }
  s_last_tick = now;

  const uint32_t phase = now % kBreathPeriodMs;
  const uint32_t half = kBreathPeriodMs / 2;
  const uint32_t ramp = phase < half ? phase : kBreathPeriodMs - phase;  // 三角波 0~half
  const uint32_t span = (kCustomLevel > kBreathMinLevel) ? (kCustomLevel - kBreathMinLevel) : 0;
  const uint8_t level = static_cast<uint8_t>(kBreathMinLevel + ramp * span / half);

  // ★ 亮度没变就不刷 RMT(呼吸曲线在三角波顶点/底点附近连续多帧同值)。
  //   不加这个去重, 30Hz 全都在重画同一帧, 纯属浪费 RMT 带宽。
  static uint8_t s_last_level = 0xFF;
  if (level == s_last_level) {
    return;
  }
  s_last_level = level;
  LedShowPickMaskAt(level);
}

// 彩灯跑马灯的一帧: kChaseDotCount 颗亮点等间隔排布在灯链上, 整体随时间滚动。
// ★ 每颗亮点【颜色不同】(取色相环上的等分点), 所以是"彩灯跑"而不是"白点跑"。
//
// 实现思路(避免"除法取整导致亮点数不对"的坑):
//   8 颗灯要放 3 个亮点, 8/3 除不尽。若用 (i+pos)%kLane 判据, kLane=2 会得出 4 颗亮 —— 错的。
//   正确做法: 【乘法定位】—— 亮点应该落在 pos + d * 8 / 3 处(d = 0,1,2)。
//   即把灯链按 8 等分, 亮点取第 0、8/3、16/3 三个位置(向下取整 = 0,2,5)。
//   这样亮点数【恒等于 kChaseDotCount】, 且随 pos 递增整体向前滚动。
static void LedShowChaseFrame(uint32_t pos) {
  if (g_led_strip == nullptr) {
    return;
  }
  // 先算这一帧 3 个亮点各自落在哪一颗上
  uint32_t head_pos[kChaseDotCount];
  for (uint32_t d = 0; d < kChaseDotCount; ++d) {
    head_pos[d] = (pos + d * kWs2812Count / kChaseDotCount) % kWs2812Count;
  }
  for (uint32_t i = 0; i < kWs2812Count; ++i) {
    int hit = -1;
    for (uint32_t d = 0; d < kChaseDotCount; ++d) {
      if (head_pos[d] == i) {
        hit = static_cast<int>(d);
        break;
      }
    }
    if (hit >= 0) {
      // 每个亮点取色相环上的等分色, 让 3 颗颜色各不相同
      const uint8_t hue = static_cast<uint8_t>(hit * (256 / kChaseDotCount));
      // 让颜色也随时间缓慢流动, 观感更"活"
      const uint8_t rolling_hue = static_cast<uint8_t>((hue + pos * 4) & 0xFF);
      uint8_t r = 0, g = 0, b = 0;
      HsvToRgb(rolling_hue, 255, kChaseDotLevel, r, g, b);
      ESP_ERROR_CHECK(led_strip_set_pixel(g_led_strip, i, r, g, b));
    } else {
      ESP_ERROR_CHECK(led_strip_set_pixel(g_led_strip, i, 0, 0, 0));
    }
  }
  ESP_ERROR_CHECK(led_strip_refresh(g_led_strip));
}

const char* LedModeName(LedMode mode) {
  switch (mode) {
    case LedMode::kOff: return "off";
    case LedMode::kWhite: return "white";
    case LedMode::kBreath: return "breath";
    case LedMode::kRainbow: return "rainbow";
    case LedMode::kChase: return "chase";
    case LedMode::kCustom: return "custom";
  }
  return "unknown";
}

void LedSetMode(LedMode mode) {
  g_led_mode = mode;
  if (mode == LedMode::kOff) {
    LedShowOff();
  } else if (mode == LedMode::kWhite) {
    LedShowWhite(kWhiteLevel);
  } else if (mode == LedMode::kBreath) {
    LedShowWhite(kBreathMinLevel);  // 呼吸由 loop 逐步刷新, 先给个起始值避免闪一下满亮
  } else if (mode == LedMode::kRainbow) {
    LedShowRainbowFrame(millis());  // 跑马灯由 loop 逐步刷新, 先画第一帧避免出现黑一下
  } else if (mode == LedMode::kChase) {
    LedShowChaseFrame(0);  // 单点跑马灯: 从第 1 颗起点开始, 由 loop 逐步推进
  } else if (mode == LedMode::kCustom) {
    // 指定灯亮: 呼吸模式下给个最低亮度起点(避免先闪一下满亮), 否则直接画常亮帧
    if (kCustomBreath) {
      LedShowPickMaskAt(kBreathMinLevel);
    } else {
      LedShowPickMask();
    }
  }
  printf("[LED] mode -> %s\n", LedModeName(mode));
}

// ★ 2026-10-14 上电自检: 逐颗点亮, 用于定位"某几颗灯不受控"的硬件故障。
//   用途见 kWs2812SelfTest 的注释。每颗亮 300ms, 同时打印序号,
//   方便对照"串口说第 N 颗亮"与"实际看到第几颗亮"是否一致。
//   判读举例(以 8 颗灯链为例):
//   - 串口打印到第 8 颗, 但眼睛只看到 4 颗亮 → RGB5~RGB8 有问题(断路/虚焊)
//   - 串口打印到第 2 颗就停了 → 后面的灯珠把数据"吃掉"或数据线断在更前面
void LedSelfTest() {
  if (g_led_strip == nullptr) {
    return;
  }
  printf("[LED] self-test start (expect %u leds light up one by one)\n", (unsigned)kWs2812Count);
  for (uint32_t i = 0; i < kWs2812Count; ++i) {
    ESP_ERROR_CHECK(led_strip_clear(g_led_strip));
    // 点亮第 i 颗为白色(适度亮度, 避免刺眼)
    ESP_ERROR_CHECK(led_strip_set_pixel(g_led_strip, i, 60, 60, 60));
    ESP_ERROR_CHECK(led_strip_refresh(g_led_strip));
    printf("[LED] self-test: led %u (0-based) should be ON\n", (unsigned)i);
    delay(300);
  }
  ESP_ERROR_CHECK(led_strip_clear(g_led_strip));
  delay(200);
  printf("[LED] self-test done (all cleared)\n");
}

void InitWs2812() {
  // ★★ 2026-10-14 重大修正: 【删除了原来在这里的 pinMode()+digitalWrite() 预拉低】
  //
  //   上一版为了"清掉上电伪数据"加了三行:
  //       pinMode(kWs2812Pin, OUTPUT);
  //       digitalWrite(kWs2812Pin, LOW);
  //       delay(2);
  //   结果反而【制造】了"通电后末尾两颗灯常亮"的故障 —— 元凶就是这里。
  //
  //   为什么加"拉低"反而坏事:
  //     · ESP32-S3 的 pinMode() 把引脚切成 GPIO 输出模式时, IO MUX 重新映射
  //       会产生一个几十 ns ~ 数 µs 的瞬态毛刺(高->低跳变)。
  //     · WS2812 的判据是: 高 0.4us=bit0 / 高 0.8us=bit1 / 低 >50us=复位。
  //       这个毛刺【之前没有 >50us 的复位低电平】, 所以被灯珠当成"数据帧"开始接收!
  //     · 紧接着 delay(2) 的 2ms 低电平虽然产生了复位, 但毛刺已经被前几颗灯珠
  //       吃掉若干 bit -> 后面灯珠数据错位 -> 表现为"末尾两颗亮"。
  //     · 也就是说: 这段"加固"代码本身就是一个 WS2812 的非法时序发生器。
  //
  //   正确做法: 什么都别手动做, 交给 led_strip 驱动。
  //     驱动在 led_strip_new_rmt_device() 里会正确配置 RMT 通道并接管该引脚,
  //     它的 RMT 编码器本身就是合法的 WS2812 时序, 不会产生非法毛刺。
  //     初始化完成后调 LedShowOff() 发合法帧即可把灯链置灭。
  //
  //   ★ 通用教训: 对 WS2812 这类"自定时协议"器件, 【任何手动 GPIO 操作都可能
  //     被器件当成数据】。绝不要为了提高稳定性去手动翻转它的数据脚 —— 手动
  //     操作越"精心", 制造非法时序的风险越大。

  led_strip_config_t strip_config = {
      .strip_gpio_num = static_cast<int>(kWs2812Pin),
      .max_leds = kWs2812Count,
      .led_model = LED_MODEL_WS2812,
      .color_component_format = LED_STRIP_COLOR_COMPONENT_FMT_GRB,  // 板载 WS2812 数据顺序为 GRB
      .flags = {.invert_out = false},
  };

  led_strip_rmt_config_t rmt_config = {
      .clk_src = RMT_CLK_SRC_DEFAULT,
      .resolution_hz = 10 * 1000 * 1000,  // 10MHz, WS2812 时序所需
      .mem_block_symbols = 0,
      .flags = {.with_dma = 0},
  };

  ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_config, &rmt_config, &g_led_strip));
  LedShowOff();
  InitLedPickColors();  // 指定灯的默认颜色(绿) —— 数组初值是 0(白), 必须在这里刷成默认色
  printf("[LED] ws2812 init done: pin=%d, leds=%u\n", static_cast<int>(kWs2812Pin), static_cast<unsigned>(kWs2812Count));

  // ★ 调试开关打开时跑逐颗自检(见 kWs2812SelfTest 注释)。验证完记得关掉。
  if (kLedSelfTestOnBoot) {
    LedSelfTest();
  }
}

// 呼吸灯刷新: 由 loop 以约 20ms 周期调用, 用三角波把亮度在 min~max 之间来回推
void LedBreathUpdate() {
  if (g_led_mode != LedMode::kBreath) {
    return;
  }
  static uint32_t s_last_tick = 0;
  const uint32_t now = millis();
  if (s_last_tick != 0 && now - s_last_tick < kLedRefreshIntervalMs) {
    return;  // 约 30Hz 刷新足够顺滑, 也不必让 RMT 太忙(挤占音频 DMA)
  }
  s_last_tick = now;

  const uint32_t phase = now % kBreathPeriodMs;                 // 0 ~ period
  const uint32_t half = kBreathPeriodMs / 2;                    // 半周期
  const uint32_t ramp = phase < half ? phase : kBreathPeriodMs - phase;  // 三角波 0 ~ half
  const uint32_t span = kBreathMaxLevel - kBreathMinLevel;
  const uint8_t level = static_cast<uint8_t>(kBreathMinLevel + ramp * span / half);
  LedShowWhite(level);
}

// 跑马灯刷新: 由 loop 以约 20ms 周期调用, 每 kRainbowStepMs 推进一步
// ★ 与呼吸灯同理, 必须走 loop 非阻塞轮询; 这里靠 s_last_step 记忆"上次画的是第几步",
//   只有步号变化才真正推屏, 避免每 20ms 都重画同一帧白白占用 RMT 与 CPU
void LedRainbowUpdate() {
  if (g_led_mode != LedMode::kRainbow) {
    return;
  }
  static uint32_t s_last_step = UINT32_MAX;
  const uint32_t now = millis();
  static uint32_t s_last_tick = 0;
  if (s_last_tick != 0 && now - s_last_tick < kLedRefreshIntervalMs) {
    return;  // 轮询间隔降到 33ms; 真正推屏由步号变化控制
  }
  s_last_tick = now;

  const uint32_t step = now / kRainbowStepMs;
  if (step == s_last_step) {
    return;  // 还没到下一步, 无需重画
  }
  s_last_step = step;
  LedShowRainbowFrame(now);
}

// 单点跑马灯刷新: 由 loop 以约 20ms 周期调用, 每 kChaseStepMs 往前跑一颗。
// ★ 同样走非阻塞轮询, 靠步号变化决定是否真推屏(与 LedRainbowUpdate 同套路)。
void LedChaseUpdate() {
  if (g_led_mode != LedMode::kChase) {
    return;
  }
  static uint32_t s_last_step = UINT32_MAX;
  const uint32_t now = millis();
  static uint32_t s_last_tick = 0;
  if (s_last_tick != 0 && now - s_last_tick < kLedRefreshIntervalMs) {
    return;
  }
  s_last_tick = now;

  const uint32_t step = now / kChaseStepMs;
  if (step == s_last_step) {
    return;
  }
  s_last_step = step;
  LedShowChaseFrame(step);
}

// ================== 监测报警灯效 (2026-10-23 新增) ==================
//  终端数据报警(任一监测通道超上限/低于下限)期间, 主控灯带被监测系统接管为
//  【红色呼吸】; 报警全部解除后恢复用户之前选的灯效。
//
//  ★★ 为什么用"接管标志"而不是新增一个 LedMode:
//    g_led_mode 是用户通过 self.led.set 选的【意愿】, 报警只是【临时覆盖】。
//    退出报警时一句 LedSetMode(g_led_mode) 就能原样恢复 —— 包括 custom(指定灯)模式
//    的逐颗颜色位图, 不用保存/还原任何额外状态。
//  ★ 覆盖期间用户再说"灯带变彩虹"会收下意愿(g_led_mode 被改)但显示仍是红呼吸,
//    报警解除后变成彩虹 —— 报警优先级高于灯效指令, 这是刻意的。
static bool g_led_alarm = false;

// 画一帧红色呼吸(33ms 节流, 与白光呼吸同一套三角波, 只是颜色固定红)
static void LedAlarmBreathFrame(uint32_t now) {
  if (g_led_strip == nullptr) {
    return;
  }
  static uint32_t s_last_tick = 0;
  if (s_last_tick != 0 && now - s_last_tick < kLedRefreshIntervalMs) {
    return;  // 30Hz 刷新, 与其它灯效同一节奏, 不多占 RMT
  }
  s_last_tick = now;

  const uint32_t phase = now % kBreathPeriodMs;
  const uint32_t half  = kBreathPeriodMs / 2;
  const uint32_t ramp  = phase < half ? phase : kBreathPeriodMs - phase;
  const uint32_t span  = kBreathMaxLevel - kBreathMinLevel;
  const uint8_t level  = static_cast<uint8_t>(kBreathMinLevel + ramp * span / half);
  for (uint32_t i = 0; i < kWs2812Count; ++i) {
    ESP_ERROR_CHECK(led_strip_set_pixel(g_led_strip, i, level, 0, 0));  // 红 (GRB 顺序由驱动管)
  }
  ESP_ERROR_CHECK(led_strip_refresh(g_led_strip));
}

// 报警开始: 进入红色呼吸
static void LedAlarmEnter() {
  if (g_led_alarm) {
    return;
  }
  g_led_alarm = true;
  printf("[LED] 报警灯效 ON: 红色呼吸\n");
  LedAlarmBreathFrame(millis());  // 首帧立即出光, 不等下一轮 loop
}

// 报警解除: 恢复用户原本的灯效模式
static void LedAlarmExit() {
  if (!g_led_alarm) {
    return;
  }
  g_led_alarm = false;
  printf("[LED] 报警灯效 OFF: 恢复 %s\n", LedModeName(g_led_mode));
  LedSetMode(g_led_mode);  // 按原模式重画首帧(off 会清灯, custom 会重画点名位图)
}

// loop 里调用: 报警期间替代所有常规灯效刷新
static void LedAlarmUpdate() {
  if (!g_led_alarm) {
    return;
  }
  LedAlarmBreathFrame(millis());
}

void InitDisplay() {
  printf("init display\n");
  pinMode(kDisplayBacklightPin, OUTPUT);
  digitalWrite(kDisplayBacklightPin, HIGH);

  spi_bus_config_t buscfg{
      .mosi_io_num = kDisplayMosiPin,
      .miso_io_num = GPIO_NUM_NC,
      .sclk_io_num = kDisplayClkPin,
      .quadwp_io_num = GPIO_NUM_NC,
      .quadhd_io_num = GPIO_NUM_NC,
      .data4_io_num = GPIO_NUM_NC,
      .data5_io_num = GPIO_NUM_NC,
      .data6_io_num = GPIO_NUM_NC,
      .data7_io_num = GPIO_NUM_NC,
      .data_io_default_level = false,
      .max_transfer_sz = kDisplayWidth * kDisplayHeight * sizeof(uint16_t),
      .flags = 0,
      .isr_cpu_id = ESP_INTR_CPU_AFFINITY_AUTO,
      .intr_flags = 0,
  };
  ESP_ERROR_CHECK(spi_bus_initialize(SPI3_HOST, &buscfg, SPI_DMA_CH_AUTO));

  esp_lcd_panel_io_handle_t panel_io = nullptr;
  esp_lcd_panel_handle_t panel = nullptr;

  esp_lcd_panel_io_spi_config_t io_config = {};
  io_config.cs_gpio_num = kDisplayCsPin;
  io_config.dc_gpio_num = kDisplayDcPin;
  io_config.spi_mode = kDisplaySpiMode;
  io_config.pclk_hz = 20 * 1000 * 1000;  // 此类排线屏 40MHz 常不稳定, 先用 20MHz 点亮, 稳定后可尝试 40MHz
  io_config.trans_queue_depth = 10;
  io_config.lcd_cmd_bits = 8;
  io_config.lcd_param_bits = 8;
  ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi(SPI3_HOST, &io_config, &panel_io));

  esp_lcd_panel_dev_config_t panel_config = {};
  panel_config.reset_gpio_num = kDisplayRstPin;
  panel_config.rgb_ele_order = kDisplayRgbElementOrder;
  panel_config.bits_per_pixel = 16;
  ESP_ERROR_CHECK(esp_lcd_new_panel_st7789(panel_io, &panel_config, &panel));

  esp_lcd_panel_reset(panel);
  delay(120);  // 给 ST7789 上电稳定时间, 部分克隆芯片要求 reset 后至少等 120ms

  esp_lcd_panel_init(panel);
  esp_lcd_panel_invert_color(panel, kDisplayInvertColor);
  esp_lcd_panel_swap_xy(panel, kDisplaySwapXY);
  esp_lcd_panel_mirror(panel, kDisplayMirrorX, kDisplayMirrorY);

  g_display = std::make_unique<Display>(panel_io, panel, kDisplayWidth, kDisplayHeight, 0, 0, kDisplayMirrorX, kDisplayMirrorY, kDisplaySwapXY);
  g_display->Start();
}

// ★ 2026-10-15 卡顿优化: 堆内存周期打印【已默认关闭】。
//   原来 build_opt.h 里有 -DPRINT_HEAP_INFO_INTERVAL=5000, 每 5 秒打印 3 行内存信息
//   (约 300 字节)。串口输出是同步阻塞的, 115200bps 下这 300 字节要占几十毫秒,
//   且是每 5 秒一次的周期性抖动, 会打断音频与灯效时序 —— 肉眼可见的"周期性卡顿"。
//   这是纯调试功能, 日常运行不需要, 已从 build_opt.h 移除该宏。
//   需要查内存泄漏时, 在 build_opt.h 加一行【-DPRINT_HEAP_INFO_INTERVAL=5000】即可。
//
// ★★ 重要提醒(踩过): build_opt.h 是 GCC 的 response file(@file)格式,
//   【里面不能写注释】。任何带空格的说明文字都会被编译器当成多个参数,
//   报错 "cannot specify '-o' with '-c','-S' or '-E' with multiple files"。
//   注释只能写在 .cpp/.h 里, build_opt.h 必须【每行仅一个有效编译选项】。
#ifdef PRINT_HEAP_INFO_INTERVAL
void PrintMemInfo() {
  if (heap_caps_get_total_size(MALLOC_CAP_SPIRAM) > 0) {
    const auto total_size = heap_caps_get_total_size(MALLOC_CAP_SPIRAM);
    const auto free_size = heap_caps_get_free_size(MALLOC_CAP_SPIRAM);
    const auto min_free_size = heap_caps_get_minimum_free_size(MALLOC_CAP_SPIRAM);
    printf("SPIRAM total size: %zu B (%zu KB), free size: %zu B (%zu KB), minimum free size: %zu B (%zu KB)\n",
           total_size,
           total_size >> 10,
           free_size,
           free_size >> 10,
           min_free_size,
           min_free_size >> 10);
  }

  if (heap_caps_get_total_size(MALLOC_CAP_INTERNAL) > 0) {
    const auto total_size = heap_caps_get_total_size(MALLOC_CAP_INTERNAL);
    const auto free_size = heap_caps_get_free_size(MALLOC_CAP_INTERNAL);
    const auto min_free_size = heap_caps_get_minimum_free_size(MALLOC_CAP_INTERNAL);
    printf("IRAM total size: %zu B (%zu KB), free size: %zu B (%zu KB), minimum free size: %zu B (%zu KB)\n",
           total_size,
           total_size >> 10,
           free_size,
           free_size >> 10,
           min_free_size,
           min_free_size >> 10);
  }

  if (heap_caps_get_total_size(MALLOC_CAP_DEFAULT) > 0) {
    const auto total_size = heap_caps_get_total_size(MALLOC_CAP_DEFAULT);
    const auto free_size = heap_caps_get_free_size(MALLOC_CAP_DEFAULT);
    const auto min_free_size = heap_caps_get_minimum_free_size(MALLOC_CAP_DEFAULT);
    printf("DRAM total size: %zu B (%zu KB), free size: %zu B (%zu KB), minimum free size: %zu B (%zu KB)\n",
           total_size,
           total_size >> 10,
           free_size,
           free_size >> 10,
           min_free_size,
           min_free_size >> 10);
  }
}
#endif

void PlayMp3(const uint8_t* data, size_t size) {
  auto ret = esp_mp3_dec_register();
  if (ret != ESP_AUDIO_ERR_OK) {
    printf("Failed to register mp3 decoder: %d\n", ret);
    abort();
  }

  esp_audio_simple_dec_handle_t decoder = nullptr;
  esp_audio_simple_dec_cfg_t audio_dec_cfg{
      .dec_type = ESP_AUDIO_SIMPLE_DEC_TYPE_MP3,
      .dec_cfg = nullptr,
      .cfg_size = 0,
  };
  ret = esp_audio_simple_dec_open(&audio_dec_cfg, &decoder);
  if (ret != ESP_AUDIO_ERR_OK) {
    printf("Failed to open mp3 decoder: %d\n", ret);
    abort();
  }
  g_audio_output_device->OpenOutput(16000);

  esp_audio_simple_dec_raw_t raw = {
      .buffer = const_cast<uint8_t*>(data),
      .len = size,
      .eos = true,
      .consumed = 0,
      .frame_recover = ESP_AUDIO_SIMPLE_DEC_RECOVERY_NONE,
  };

  uint8_t* frame_data = (uint8_t*)malloc(4096);
  esp_audio_simple_dec_out_t out_frame = {
      .buffer = frame_data,
      .len = 4096,
      .needed_size = 0,
      .decoded_size = 0,
  };

  while (raw.len > 0) {
    const auto ret = esp_audio_simple_dec_process(decoder, &raw, &out_frame);
    if (ret == ESP_AUDIO_ERR_BUFF_NOT_ENOUGH) {
      // Handle output buffer not enough case
      out_frame.buffer = reinterpret_cast<uint8_t*>(realloc(out_frame.buffer, out_frame.needed_size));
      if (out_frame.buffer == nullptr) {
        break;
      }
      out_frame.len = out_frame.needed_size;
      continue;
    }

    if (ret != ESP_AUDIO_ERR_OK) {
      break;
    }

    g_audio_output_device->Write(reinterpret_cast<int16_t*>(out_frame.buffer), out_frame.decoded_size >> 1);
    raw.len -= raw.consumed;
    raw.buffer += raw.consumed;
  }

  free(frame_data);

  g_audio_output_device->CloseOutput();
  esp_audio_simple_dec_close(decoder);
  esp_audio_dec_unregister(ESP_AUDIO_TYPE_MP3);
}

void ConfigureWifi() {
  printf("configure wifi\n");
  auto wifi_configurator = std::make_unique<WifiConfigurator>(WiFi, kSmartConfigType);

  ESP_ERROR_CHECK(iot_button_register_cb(
      g_button_boot_handle,
      BUTTON_PRESS_DOWN,
      nullptr,
      [](void*, void* data) {
        printf("boot button pressed\n");
        static_cast<WifiConfigurator*>(data)->StartSmartConfig();
      },
      wifi_configurator.get()));

  g_display->ShowStatus("网络配置中");
  PlayMp3(kNotification0mp3, sizeof(kNotification0mp3));

#if defined(WIFI_SSID) && defined(WIFI_PASSWORD)
  printf("wifi config start with wifi: %s, %s\n", WIFI_SSID, WIFI_PASSWORD);
  wifi_configurator->Start(WIFI_SSID, WIFI_PASSWORD);
#else
  printf("wifi config start\n");
  wifi_configurator->Start();
#endif

  while (true) {
    const auto state = wifi_configurator->WaitStateChanged();
    if (state == WifiConfigurator::State::kConnecting) {
      printf("wifi connecting\n");
      g_display->ShowStatus("网络连接中");
    } else if (state == WifiConfigurator::State::kSmartConfiguring) {
      printf("wifi smart configuring\n");
      g_display->ShowStatus("配网模式");
      PlayMp3(kNetworkConfigModeMp3, sizeof(kNetworkConfigModeMp3));
    } else if (state == WifiConfigurator::State::kFinished) {
      break;
    }
  }

  iot_button_unregister_cb(g_button_boot_handle, BUTTON_PRESS_DOWN, nullptr);

  printf("wifi connected\n");
  printf("- mac address: %s\n", WiFi.macAddress().c_str());
  printf("- bssid:       %s\n", WiFi.BSSIDstr().c_str());
  printf("- ssid:        %s\n", WiFi.SSID().c_str());
  printf("- ip:          %s\n", WiFi.localIP().toString().c_str());
  printf("- gateway:     %s\n", WiFi.gatewayIP().toString().c_str());
  printf("- subnet mask: %s\n", WiFi.subnetMask().toString().c_str());

  g_display->ShowStatus("网络已连接");
  PlayMp3(kNetworkConnectedMp3, sizeof(kNetworkConnectedMp3));

  // ---------- NTP 对时 (屏保时钟用) ----------
  // 时区 UTC+8 (中国标准时间), 无夏令时
  // 注意: 这里只是"发起"同步, 函数立即返回, 后台 SNTP 任务会去取时间
  //       取到之前 time() 返回很小的值(1970 附近), 屏保会显示"对时中..."且数码管熄灭
  // 用 2 个国内可达的 NTP 服务器, 避免单点失败
  constexpr long kTimeZoneOffsetSec = 8 * 3600;  // UTC+8
  configTime(kTimeZoneOffsetSec, 0, "ntp.aliyun.com", "cn.pool.ntp.org");
  printf("[TIME] ntp sync started (UTC+8)\n");

  // ---------- 出网自检 (2026-11-08) ----------
  // ★ 有 IP ≠ 能上外网。故障表现是 "got ip 成功 → 20 秒后 fetch_config 报
  //   ESP_ERR_HTTP_CONNECT → 加载协议失败"，光看那行分不清三种情况:
  //     a) DNS 挂了(路由器 DNS 转发不通)
  //     b) 路由器 WAN 没外网(DHCP 照给 IP, 但出不去)
  //     c) 时间还是 1970 → TLS 证书 notBefore 校验失败 → 握手失败
  //   这里各测一次, 一眼定位。总耗时最多 ~5 秒, 只在启动时跑一次。
  {
    IPAddress srv;
    const int dns_ret = WiFi.hostByName("api.tenclass.net", srv);
    const bool dns_ok = (dns_ret == 1) && (srv != IPAddress(0, 0, 0, 0));
    printf("[NET] dns api.tenclass.net -> %s (ret=%d)\n",
           dns_ok ? srv.toString().c_str() : "FAIL", dns_ret);

    bool tcp_ok = false;
    if (dns_ok) {
      WiFiClient probe;
      probe.setTimeout(5);  // 秒
      tcp_ok = probe.connect(srv, 443);
      probe.stop();
    }
    printf("[NET] tcp api.tenclass.net:443 -> %s\n", tcp_ok ? "OK" : "FAIL");

    // 时间是否已同步: configTime 是异步的, 所以要等一会儿再判;
    // 小于 2020 年(1577836800) = NTP 没回来 → TLS 证书 notBefore 校验必失败
    time_t now_epoch = time(nullptr);
    for (int i = 0; i < 30 && now_epoch < 1577836800L; ++i) {  // 最多等 3 秒
      delay(100);
      now_epoch = time(nullptr);
    }
    printf("[NET] clock = %ld (%s)\n", (long)now_epoch,
           now_epoch < 1577836800L ? "未对时! TLS 证书校验必失败" : "已对时");
  }
}

// ---------------- 环境传感器: AHT20(温湿度) + BMP280(气压) ----------------
// 接线(用户 2026-10-14 接线图): AHT20 模块 4 针 = 3V3 / IO41 / GND / IO42
//   即 I2C: SDA=IO41, SCL=IO42。BMP280 挂同一条总线(地址 0x76 或 0x77)。
// AHT20 地址固定 0x38; BMP280 地址由 SDO 引脚电平决定: GND=0x76, VCC=0x77。
// 传感器不存在时不阻塞不报错, 只是 MCP 查询时告知"未连接"。
static Adafruit_AHTX0 g_aht;
static Adafruit_BMP280 g_bmp;
static bool g_aht_ok = false;
static bool g_bmp_ok = false;
// 最近一次读数(loop 单线程读、单线程用, 无需加锁)
static float g_env_temp_c = 0.0f;    // °C
static float g_env_hum = 0.0f;       // %RH
static float g_env_press_hpa = 0.0f; // hPa
static bool g_env_valid = false;
static uint32_t g_env_last_read_ms = 0;
static uint32_t g_env_last_aht_ms = 0;   // AHT20 上次成功读数时刻(状态机自己用)
// 3 秒读一次: AHT20 测一次约需 80ms, 不能太密; 3s 对环境变化足够快
static constexpr uint32_t kEnvReadIntervalMs = 3000;

// ---- 土壤温湿度 SHT40 (I2C 0x44, 插土里, 2026-11-06 新增) ----
// ★ 全局变量必须定义在 InitSensors()/SensorPoll() 之前(它们要引用)。
static bool g_sht_ok = false;          // 探测期是否找到 0x44
static bool g_sht_dead = false;        // 连续失败后判掉线, 不再反复尝试
static float g_soil_temp_c = 0.0f;     // 土壤温度 °C
static float g_soil_hum = 0.0f;        // 土壤湿度 %RH(SHT 测的是土中空气湿度, 只作趋势参考)
static bool  g_soil_valid = false;     // 是否拿到过至少一次有效读数
static constexpr uint8_t kShtI2cAddr = 0x44;   // SHT40 默认地址
static constexpr uint32_t kShtMeasureWaitMs = 10;  // SHT40 高精度测量约 8.2ms, 留余量
enum class ShtPhase { kIdle, kWaiting };
static ShtPhase g_sht_phase = ShtPhase::kIdle;
static uint32_t g_sht_trigger_ms = 0;
static uint32_t g_sht_last_ms = 0;
static uint8_t  g_sht_fail_count = 0;
// SHT40 状态机前置声明(定义在下方, SensorPoll 里每轮调用)
static bool Sht30Poll(uint32_t now);

// ---------------- AHT20 非阻塞读取状态机 ----------------
// ★★ 2026-10-15 修复一个【整机卡死】级别的隐患。
//   原先 SensorPoll() 调 Adafruit 库的 g_aht.getEvent()。该库内部是这么等的(实测源码):
//       while (getStatus() & AHTX0_STATUS_BUSY) { delay(10); }   // ← 无超时!
//   而 getStatus() 在 I2C 读失败时返回 0xFF:
//       uint8_t Adafruit_AHTX0::getStatus(void){ if(!i2c_dev->read(&ret,1)) return 0xFF; ... }
//   0xFF & 0x80 恒为真 → 【这个循环永远不会退出】。
//   后果: 只要 I2C 被干扰一次 / 传感器虚接 / 总线挂死, loop() 就永久停在这一行 ——
//         表现就是用户说的"卡顿", 严重时会彻底死机(只能断电重启)。
//   修法: 自己实现 AHT20 协议, 拆成"发命令"和"收结果"两步的状态机。
//         每轮 loop 只做一次毫秒级 I2C 操作, 且任何一步失败都立即放弃本次读取,
//         绝不等、绝不循环。
static constexpr uint8_t kAhtI2cAddr = 0x38;        // AHT20 固定地址
static constexpr uint32_t kAhtMeasureWaitMs = 80;    // 测量耗时(数据手册 ~75ms, 留余量)
static constexpr uint32_t kAhtAbortMs = 400;         // 超过这么久没出结果就放弃本次(防死等)

enum class AhtPhase { kIdle, kWaiting };
static AhtPhase g_aht_phase = AhtPhase::kIdle;
static uint32_t g_aht_trigger_ms = 0;
static uint8_t g_aht_fail_count = 0;
static bool g_aht_dead = false;   // 连续失败后判为掉线, 不再反复尝试(避免持续拖累 loop)

// 步骤 1: 触发一次测量(只发 3 字节命令, 立即返回, 不等结果)
static bool Aht20StartMeasure() {
  Wire.beginTransmission(kAhtI2cAddr);
  Wire.write(0xAC);  // 触发测量命令
  Wire.write(0x33);
  Wire.write(0x00);
  return Wire.endTransmission() == 0;
}

// 步骤 2: 取结果。一次 I2C 读 6 字节, 首字节是状态(Bit7=busy)。
//   仍在忙 → 返回 false(下轮再来); 成功 → 填充温湿度并返回 true。
static bool Aht20ReadResult(float& temp, float& hum) {
  if (Wire.requestFrom(kAhtI2cAddr, static_cast<uint8_t>(6)) != 6) {
    return false;  // I2C 失败: 立即返回, 绝不重试/等待
  }
  uint8_t d[6];
  for (int i = 0; i < 6; ++i) {
    d[i] = Wire.read();
  }
  if (d[0] & 0x80) {
    return false;  // 还在忙
  }
  // 20bit 湿度 + 20bit 温度, 与 AHT20 数据手册一致
  const uint32_t raw_h = (static_cast<uint32_t>(d[1]) << 12) |
                         (static_cast<uint32_t>(d[2]) << 4) |
                         (static_cast<uint32_t>(d[3]) >> 4);
  const uint32_t raw_t = ((static_cast<uint32_t>(d[3]) & 0x0F) << 16) |
                         (static_cast<uint32_t>(d[4]) << 8) |
                         static_cast<uint32_t>(d[5]);
  hum = (static_cast<float>(raw_h) * 100.0f) / 1048576.0f;  // 2^20
  temp = (static_cast<float>(raw_t) * 200.0f) / 1048576.0f - 50.0f;
  return true;
}

void InitSensors() {
  constexpr int kI2cSdaPin = 41;
  constexpr int kI2cSclPin = 42;
  // ★★ 2026-10-31 降噪 + 加固(起因: 关掉 USB CDC 后日志改走 UART0, 才看见这段红字):
  //     E (1303) i2c.master: I2C transaction unexpected nack detected
  //     E (1303) i2c.master: s_i2c_synchronous_transaction(924): I2C transaction failed
  //     E (1304) i2c.master: i2c_master_multi_buffer_transmit(1186): I2C transaction failed
  //     [Sensor] AHT20: OK (0x38)
  //   真相: AHT20 上电后要 >=40ms 才肯应答, 而我们是"一上电立刻读状态寄存器" →
  //   第一次 I2C 事务被 NACK(上面三条就是这一个事务的三层日志)。Adafruit 库内部随后
  //   做软复位 + 重读, 所以最终打印 OK —— 功能一直是好的, 只是每次都刷三行"错误",
  //   真正出故障时反而看不出来。
  //   两手处理:
  //     ① 先给上电时间再碰总线(预热);
  //     ② 探测阶段把 i2c.master 这个 tag 临时压到 NONE, 探完立刻恢复 ERROR ——
  //        "地址没人应答"本来就是探测的正常结果(尤其 BMP280 没插时必然 NACK),
  //        不该按故障级别刷屏; 运行期真出问题照旧报。
  Wire.begin(kI2cSdaPin, kI2cSclPin);
  Wire.setClock(100000);  // 100kHz 标准模式, AHT20 稳妥
  delay(120);             // ① 上电预热: 让 3V3 与传感器内部都稳下来(手册要 >=40ms)

  const esp_log_level_t i2c_log_lv = esp_log_level_get("i2c.master");
  esp_log_level_set("i2c.master", ESP_LOG_NONE);  // ② 探测期静音(下面 finally 式恢复)

  // AHT20: 默认地址 0x38。第一次偶发不应答时重试, 别一次判死。
  for (int attempt = 1; attempt <= 3; ++attempt) {
    g_aht_ok = g_aht.begin(&Wire);
    if (g_aht_ok) break;
    delay(60);
  }
  printf("[Sensor] AHT20: %s\n", g_aht_ok ? "OK (0x38)" : "NOT FOUND (check wiring/power)");

  // BMP280: 先试 0x76(SDO=GND) 再试 0x77(SDO=VCC), 两轮; 都没有就算了。
  // ★ begin(addr, chipid) 没有 Wire 参数 —— Wire 是构造函数绑定的(默认 &Wire),
  //   不能写成 g_bmp.begin(0x76, &Wire), 会编译错。
  uint8_t bmp_addr = 0;
  for (int attempt = 1; attempt <= 2 && bmp_addr == 0; ++attempt) {
    if (g_bmp.begin(BMP280_ADDRESS_ALT)) {  // ALT = 0x76
      bmp_addr = BMP280_ADDRESS_ALT;
    } else if (g_bmp.begin(BMP280_ADDRESS)) {  // 默认 = 0x77
      bmp_addr = BMP280_ADDRESS;
    } else if (attempt < 2) {
      delay(60);
    }
  }
  g_bmp_ok = (bmp_addr != 0);
  if (g_bmp_ok) {
    // 正常连续测量模式: 温度 x2 / 气压 x16 过采样 + 重滤波, 数据稳
    g_bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,
                      Adafruit_BMP280::SAMPLING_X2,
                      Adafruit_BMP280::SAMPLING_X16,
                      Adafruit_BMP280::FILTER_X16,
                      Adafruit_BMP280::STANDBY_MS_63);
    printf("[Sensor] BMP280: OK (0x%02X)\n", bmp_addr);
  } else {
    printf("[Sensor] BMP280: NOT FOUND (0x76/0x77 both missed)\n");
  }

  // SHT40(土壤温湿度): 地址固定 0x44。探测 = 发 SHT40 软复位命令 0x94(1 字节),
  // ★ 不能空 beginTransmission: 空发送时总线上没设备 endTransmission 也可能返回 0(假阳性),
  //   导致"开机打印 OK、运行却一直 NACK"的诡异现象。发真实命令, 设备必须应答才算在。
  for (int attempt = 1; attempt <= 3; ++attempt) {
    Wire.beginTransmission(kShtI2cAddr);
    Wire.write(0x94);  // SHT40 软复位命令
    const uint8_t rc = Wire.endTransmission();
    printf("[Sensor] SHT40 probe attempt %d: endTransmission=%u\n", attempt, (unsigned)rc);
    if (rc == 0) {
      g_sht_ok = true;
      break;
    }
    delay(60);
  }
  printf("[Sensor] SHT40: %s (0x44, soil)\n", g_sht_ok ? "OK" : "NOT FOUND (check wiring/power)");

  // 恢复日志级别: 运行期 I2C 真出问题(NACK/超时)必须还能看见
  esp_log_level_set("i2c.master", i2c_log_lv);
}

// AHT20 状态机推进: 【每轮 loop 都调用】, 单次开销是毫秒级 I2C 事务。
// 自己控制节奏: 距上次成功读数 >= kEnvReadIntervalMs 才发起新一轮测量。
// 返回 true 表示本轮拿到了新的温湿度。
static bool Aht20Poll(uint32_t now) {
  if (!g_aht_ok || g_aht_dead) {
    return false;
  }
  if (g_aht_phase == AhtPhase::kIdle) {
    // 到点才发起新的一轮测量
    if (g_env_last_aht_ms != 0 && now - g_env_last_aht_ms < kEnvReadIntervalMs) {
      return false;
    }
    if (Aht20StartMeasure()) {
      g_aht_phase = AhtPhase::kWaiting;
      g_aht_trigger_ms = now;
    } else {
      g_aht_fail_count++;
      if (g_aht_fail_count >= 3) {
        printf("[Sensor] AHT20: trigger failed x%u, stop retrying\n", (unsigned)g_aht_fail_count);
        g_aht_dead = true;
      }
    }
    return false;
  }

  // kWaiting: 测量已发起, 到 80ms 后取结果
  const uint32_t waited = now - g_aht_trigger_ms;
  if (waited < kAhtMeasureWaitMs) {
    return false;  // 还没到, 下轮再看(不阻塞)
  }
  float t = 0.0f, h = 0.0f;
  if (Aht20ReadResult(t, h)) {
    g_env_temp_c = t;
    g_env_hum = h;
    g_aht_fail_count = 0;
    g_env_last_aht_ms = now;
    g_aht_phase = AhtPhase::kIdle;
    return true;
  }
  if (waited >= kAhtAbortMs) {
    // ★ 超时放弃本次: 不等、不重试循环, 直接回 Idle
    g_aht_fail_count++;
    g_aht_phase = AhtPhase::kIdle;
    g_env_last_aht_ms = now;  // 重置节奏, 避免下轮立刻又发
    printf("[Sensor] AHT20: read timeout (fail x%u)\n", (unsigned)g_aht_fail_count);
    if (g_aht_fail_count >= 3) {
      printf("[Sensor] AHT20: unreachable, stop retrying\n");
      g_aht_dead = true;
    }
  }
  return false;
}

// 由 loop() 周期调用: 更新传感器缓存。
// ★ 2026-10-15 重构: AHT20 与 BMP280 的节流【分开管理】, 不能再共用一个 3 秒门。
//   原因: AHT20 状态机分"发命令"和"80ms 后取结果"两步, 若被 3 秒节流挡在门外,
//   两步之间要空等 3 秒 —— 数据更新周期就从 3 秒退化成 6 秒。
//   状态机必须【每轮 loop 都推进】, 它内部自己判断"到没到发起下一轮的时机"。
void SensorPoll() {
  const uint32_t now = millis();

  // ---- AHT20: 状态机每轮推进(内部自管节奏, 单次开销毫秒级) ----
  const bool aht_fresh = Aht20Poll(now);

  // ---- SHT40(土壤温湿度): 状态机每轮推进, 与 AHT20 同套路 ----
  Sht30Poll(now);

  // ---- BMP280: 3 秒读一次(气压慢变量; readPressure 是纯 I2C 读, 无忙等, 很快) ----
  bool bmp_fresh = false;
  if (g_bmp_ok && (g_env_last_read_ms == 0 || now - g_env_last_read_ms >= kEnvReadIntervalMs)) {
    g_env_last_read_ms = now;
    // readPressure() 返回 Pa, 除以 100 转 hPa(百帕, 与天气预报同单位)
    g_env_press_hpa = g_bmp.readPressure() / 100.0f;
    bmp_fresh = true;
  }

  // 只要任一传感器拿过有效读数就标记为可用(g_env_valid 不再每轮清零:
  // 历史读数依然有效, 不该因为"这一轮恰好没新数据"就报"传感器未连接")
  if (aht_fresh || bmp_fresh) {
    g_env_valid = true;
    printf("[Env] T=%.1fC H=%.1f%% P=%.1fhPa\n", g_env_temp_c, g_env_hum, g_env_press_hpa);
  }

  // ★ 土壤数据固定节奏打印(串口监视器每 3 秒稳定刷一行, 不依赖"刚拿到新读数"):
  //   · 在线且有效 → 打印读数
  //   · 在线但还没读数 → 提示"采集中"
  //   · 没探测到/判掉线 → 提示未连接, 方便一眼看出接线/供电问题
  {
    static uint32_t s_soil_log_ms = 0;
    if (now - s_soil_log_ms >= kEnvReadIntervalMs) {
      s_soil_log_ms = now;
      if (g_sht_dead) {
        printf("[Soil] SHT40 掉线(连续失败), 检查接线/供电\n");
      } else if (!g_sht_ok) {
        printf("[Soil] SHT40 未连接(0x44 无应答), 检查 VCC/GND/SDA/SCL\n");
      } else if (!g_soil_valid) {
        printf("[Soil] SHT40 采集中...\n");
      } else {
        printf("[Soil] 土壤温度 %.1f°C  土壤湿度 %.0f%%\n", g_soil_temp_c, g_soil_hum);
      }
    }
  }
}

// ---------------- 土壤温湿度传感器: SHT40 (I2C 0x44) ----------------
// ★ 2026-11-06 用户新增: SHT40 不锈钢防水探头【插进土壤】当"土壤温湿度"用。
//   挂在 AHT20/BMP280 同一条 I2C 总线(SDA=IO41, SCL=IO42), 地址固定 0x44, 无需新引脚。
// ★ 与 AHT20 同套路: 非阻塞状态机, 每轮 loop 推进一步, 任何 I2C 失败立即放弃不等待。
//   SHT40 单次测量: 发 1 字节命令 0xFD(高精度), 延时约 8.2ms 后读 6 字节(温+CRC+湿+CRC)。
//   (全局变量与常量定义已前置到 InitSensors 之前, 这里只放函数实现)

// SHT40 单次测量: 发 0xFD(高精度单次测量), 立即返回不等结果
static bool Sht30StartMeasure() {
  Wire.beginTransmission(kShtI2cAddr);
  Wire.write(0xFD);
  const uint8_t rc = Wire.endTransmission();
  if (rc != 0) {
    printf("[SHT] trigger endTransmission=%u (2=NACK 设备无应答, 3=其他错误)\n", (unsigned)rc);
  }
  return rc == 0;
}

// SHT40 读结果: 一次读 6 字节(温度高/低/CRC + 湿度高/低/CRC)。
// 简化处理: 不校验 CRC(短距 I2C 误码极低), 直接套 SHT40 数据手册公式。
// ★ SHT40 公式与 SHT30 不同: T = -45 + 175*raw/65535, RH = -6 + 125*raw/65535
static bool Sht30ReadResult(float& temp, float& hum) {
  const int got = Wire.requestFrom(kShtI2cAddr, static_cast<uint8_t>(6));
  if (got != 6) {
    printf("[SHT] read got %d bytes (expected 6)\n", got);
    return false;
  }
  uint8_t d[6];
  for (int i = 0; i < 6; ++i) {
    d[i] = Wire.read();
  }
  printf("[SHT] raw: %02X %02X %02X %02X %02X %02X\n",
         d[0], d[1], d[2], d[3], d[4], d[5]);
  const uint16_t raw_t = (static_cast<uint16_t>(d[0]) << 8) | d[1];
  const uint16_t raw_h = (static_cast<uint16_t>(d[3]) << 8) | d[4];
  temp = -45.0f + 175.0f * static_cast<float>(raw_t) / 65535.0f;
  hum = -6.0f + 125.0f * static_cast<float>(raw_h) / 65535.0f;
  return true;
}

// SHT40 状态机推进(与 Aht20Poll 同套路, 由 SensorPoll 每轮调用)
static bool Sht30Poll(uint32_t now) {
  if (!g_sht_ok || g_sht_dead) {
    return false;
  }
  if (g_sht_phase == ShtPhase::kIdle) {
    if (g_sht_last_ms != 0 && now - g_sht_last_ms < kEnvReadIntervalMs) {
      return false;
    }
    if (Sht30StartMeasure()) {
      g_sht_phase = ShtPhase::kWaiting;
      g_sht_trigger_ms = now;
    } else {
      g_sht_fail_count++;
      if (g_sht_fail_count >= 3) {
        printf("[Sensor] SHT40: trigger failed x%u, stop retrying\n", (unsigned)g_sht_fail_count);
        g_sht_dead = true;
      }
    }
    return false;
  }

  // kWaiting: 测量已发起, 到 15ms 后取结果
  if (now - g_sht_trigger_ms < kShtMeasureWaitMs) {
    return false;
  }
  float t = 0.0f, h = 0.0f;
  if (Sht30ReadResult(t, h)) {
    g_soil_temp_c = t;
    g_soil_hum = h;
    g_soil_valid = true;
    g_sht_fail_count = 0;
    g_sht_last_ms = now;
    g_sht_phase = ShtPhase::kIdle;
    return true;
  }
  // 读失败: 放弃本轮, 回 Idle, 下轮再试(不循环等)
  g_sht_fail_count++;
  g_sht_phase = ShtPhase::kIdle;
  g_sht_last_ms = now;
  printf("[Sensor] SHT40: read failed (fail x%u)\n", (unsigned)g_sht_fail_count);
  if (g_sht_fail_count >= 3) {
    printf("[Sensor] SHT40: unreachable, stop retrying\n");
    g_sht_dead = true;
  }
  return false;
}

// ================== 板间串口通讯实现 (UART1, 39/40) ==================
// 协议: 【行协议】—— 以 '\n'(或 '\r') 作为一帧结束符, 帧内是任意 UTF-8 文本。
//   为什么用行协议而不是"帧头 + 长度 + CRC"的二进制帧:
//     · 串口助手 / 示波器 / 另一块板的终端都能直接肉眼看清内容, 调试成本最低;
//     · 两端都是自己写的固件, 走的是短距 TTL 直连, 误码率极低, 不需要重装甲;
//     · 以后真要传二进制结构体, 再升级成二进制帧也不迟(改这一个文件即可)。
//   兼容性: 对方若【不带换行】也能收到 —— 靠 kLinkFrameIdleMs 空闲超时兜底。

// ---- 接收状态(组帧缓冲 + "最近一行"缓存) ----
char g_link_rx_buf[256] = {0};         // 收满一行前的临时缓冲
size_t g_link_rx_len = 0;              // 缓冲内已有字节数
uint32_t g_link_rx_first_byte_ms = 0;  // 本帧第一个字节到达时刻(用于空闲超时判定)
char g_link_last_line[256] = {0};      // 最近收到的完整一行(供 MCP 工具读取 / 给 AI 看)
bool g_link_has_line = false;          // 是否收到过任何一行
uint32_t g_link_last_rx_ms = 0;        // 最近一次收帧时刻(诊断用)

void InitSerialLink() {
  // ★ 参数顺序: begin(baud, config, rxPin, txPin) —— 注意 rx 在前, tx 在后!
  g_link_serial.begin(kLinkBaud, SERIAL_8N1, kLinkRxPin, kLinkTxPin);
  g_link_serial.setTimeout(20);
  printf("[Link] uart1 ready: TX=%d RX=%d baud=%u (3.3V TTL; 交叉接线 + 必须共地)\n",
         static_cast<int>(kLinkTxPin), static_cast<int>(kLinkRxPin), (unsigned)kLinkBaud);
}

// 通过串口发一行(自动补 '\n')。供 MCP 工具 self.serial.send 调用。
void LinkSendLine(const char* text) {
  if (text == nullptr) {
    return;
  }
  g_link_serial.print(text);
  g_link_serial.print('\n');
  g_link_serial.flush();  // 等发送移位寄存器排空再返回, 保证"说发出去了"就是真发出去了
  printf("[Link] TX >> %s\n", text);
}

// ================== 终端温湿度缓存 (终端主动上报) ==================
// 终端每 2 秒发一行 "ENV,T=29.6,H=52.3"; 这里解析后缓存, 供 MCP 工具秒回。
//
// ★ 为什么单独解析而不是直接返回整行原文:
//   要判断"数据是否过期"(终端断电/线松了, 主控不能一直拿旧值骗用户),
//   必须把数值和时间戳一起存下来。返回原文没法做过期判定。
//
// ★ 为什么解析要容错: 串口可能收到半截行、也可能字段顺序变、也可能带空格,
//   一律用"找 T= / H= 这两个标记再 strtof"的宽松解析, 不做 sscanf 严格匹配。
struct TerminalEnv {
  bool valid;           // 是否收到过至少一次有效上报
  float temperature;    // 摄氏度
  float humidity;       // %RH
  uint32_t rx_ms;       // 最近一次有效上报的时刻
};
TerminalEnv g_terminal_env = {false, 0.0f, 0.0f, 0};

// 从 "ENV,T=29.6,H=52.3" 里取数值。成功返回 true 并写入 out_t/out_h。
static bool ParseTerminalEnv(const char* line, float* out_t, float* out_h) {
  if (line == nullptr || out_t == nullptr || out_h == nullptr) {
    return false;
  }
  if (strncmp(line, kAckTerminalEnvPrefix, strlen(kAckTerminalEnvPrefix)) != 0) {
    return false;
  }

  const char* p_t = strstr(line, "T=");
  const char* p_h = strstr(line, "H=");
  if (p_t == nullptr || p_h == nullptr) {
    return false;  // 可能是 "ENV,ERR" 这种错误上报
  }

  const float t = strtof(p_t + 2, nullptr);
  const float h = strtof(p_h + 2, nullptr);

  // isnan + 量程双检: DHT11 偶发坏值会给出明显离谱的数
  if (isnan(t) || isnan(h)) return false;
  if (t < -40.0f || t > 80.0f) return false;
  if (h < 0.0f || h > 100.0f) return false;

  *out_t = t;
  *out_h = h;
  return true;
}

// 在 loop 里每轮调用: 检查"最近一行"是不是 ENV 上报, 是则更新缓存。
// ★ 用"已处理时刻"去重, 与握手模块同一套路数(不用 0 当哨兵)。
static void TerminalEnvPoll() {
  if (!g_link_has_line) {
    return;
  }
  static uint32_t s_env_handled_ms = 0;
  static bool s_env_handled_valid = false;
  if (s_env_handled_valid && g_link_last_rx_ms == s_env_handled_ms) {
    return;  // 这一帧已经处理过
  }
  s_env_handled_ms = g_link_last_rx_ms;
  s_env_handled_valid = true;

  float t = 0.0f;
  float h = 0.0f;
  if (ParseTerminalEnv(g_link_last_line, &t, &h)) {
    g_terminal_env.valid = true;
    g_terminal_env.temperature = t;
    g_terminal_env.humidity = h;
    g_terminal_env.rx_ms = millis();
    printf("[TerminalEnv] 缓存更新: T=%.1fC H=%.1f%%\n", t, h);
  }
}

// ================== 终端全量数据缓存 (2026-10-20 新增) ==================
// 终端每 2 秒发一行 "ALL,T=24.7,H=74.3,PR=1012.1,CO2=412,LX=320,ST=27.1,SH=62.8,N=72,P=45,K=88,PH=7.0"
// 这里解析成一个 Display::TerminalData, 既喂给屏幕数据页, 也供 MCP 工具直接读。
//
// ★★ 为什么不用 sscanf("%*[^T]T=%f,..."):
//   1) 字段顺序一旦变(终端固件改了)整行就废了;
//   2) 某个字段多一个空格/少一位小数就整体解析失败;
//   3) 用"找 KEY= 再 strtof"的宽松解析, 任何一个字段坏掉只损失那一项, 其余照常显示。
Display::TerminalData g_terminal_all = {};
bool g_terminal_all_valid = false;  // 是否收到过至少一帧有效 ALL 数据

// 在行内查找 "KEY=" 并返回 '=' 之后的指针。找不到返回 nullptr。
// ★★ 为什么必须校验"左边一个字符是逗号或行首":
//   协议里有 T 也有 ST、有 P 也有 PR/PH。直接 strstr(line,"T=") 会在 "ST=27.1" 里
//   命中 T=; 直接 strstr(line,"P=") 虽然不会误中 PR=, 但会误中 "PH=" 之外的...总之
//   加这一道左边界校验后, "T=" 只认 ",T=", "P=" 只认 ",P=", 彻底不串项。
static const char* LinkFindValue(const char* line, const char* key) {
  if (line == nullptr || key == nullptr) {
    return nullptr;
  }
  const size_t klen = strlen(key);
  const char* p = line;
  while ((p = strstr(p, key)) != nullptr) {
    const bool left_ok = (p == line) || (p[-1] == ',' || p[-1] == ' ');
    if (left_ok && p[klen] == '=') {
      return p + klen + 1;
    }
    p += klen;
  }
  return nullptr;
}

// 取一个字段。成功返回 true 并写 *out。
// ★ 无效值(对端约定 -999)在这里就判成"没取到": 一律 < -100 视为无效。
//   量程下限最大的是温度 -40℃, 所以 -100 这个门限绝不会误杀真实数据。
static bool LinkPickFloat(const char* line, const char* key, float* out) {
  const char* v = LinkFindValue(line, key);
  if (v == nullptr || out == nullptr) {
    return false;
  }
  char* end = nullptr;
  const float f = strtof(v, &end);
  if (end == v) {
    return false;  // 一个数字都没解析出来(比如 "PH=" 后面直接是逗号)
  }
  if (isnan(f) || f < -100.0f) {
    return false;  // -999 / 垃圾值
  }
  *out = f;
  return true;
}

// 取一项并标记有效性。ok 数组与 TerminalData 的字段顺序一一对应。
static void LinkTakeField(const char* line, const char* key, int idx, float* slot, bool* ok) {
  float v = 0.0f;
  if (LinkPickFloat(line, key, &v)) {
    *slot = v;
    ok[idx] = true;
  }
}

// 解析一行 ALL 数据。成功(至少一项有效)返回 true 并填满 out。
static bool ParseTerminalAll(const char* line, Display::TerminalData* out) {
  if (line == nullptr || out == nullptr) {
    return false;
  }
  if (strncmp(line, kAckTerminalAllPrefix, strlen(kAckTerminalAllPrefix)) != 0) {
    return false;
  }
  memset(out, 0, sizeof(*out));
  LinkTakeField(line, "T",   0,  &out->t,   out->ok);
  LinkTakeField(line, "H",   1,  &out->h,   out->ok);
  LinkTakeField(line, "PR",  2,  &out->pr,  out->ok);
  LinkTakeField(line, "CO2", 3,  &out->co2, out->ok);
  LinkTakeField(line, "LX",  4,  &out->lx,  out->ok);
  LinkTakeField(line, "ST",  5,  &out->st,  out->ok);
  LinkTakeField(line, "SH",  6,  &out->sh,  out->ok);
  LinkTakeField(line, "N",   7,  &out->n,   out->ok);
  LinkTakeField(line, "P",   8,  &out->p,   out->ok);
  LinkTakeField(line, "K",   9,  &out->k,   out->ok);
  LinkTakeField(line, "PH",  10, &out->ph,  out->ok);
  out->rx_ms = millis();

  bool any = false;
  for (int i = 0; i < Display::kTermItemCount; ++i) {
    any = any || out->ok[i];
  }
  return any;
}

// 在 loop 里每轮调用: 若"最近一行"是 ALL 上报, 解析并喂给屏幕数据页。
// ★ 用"已处理时刻"去重(与握手/ENV 模块同一套路数), 保证同一帧不会被处理两遍。
static void TerminalAllPoll() {
  if (!g_link_has_line) {
    return;
  }
  static uint32_t s_all_handled_ms = 0;
  static bool s_all_handled_valid = false;
  if (s_all_handled_valid && g_link_last_rx_ms == s_all_handled_ms) {
    return;  // 这一帧已经处理过
  }
  s_all_handled_ms = g_link_last_rx_ms;
  s_all_handled_valid = true;

  Display::TerminalData d = {};
  if (!ParseTerminalAll(g_link_last_line, &d)) {
    return;
  }
  g_terminal_all = d;
  g_terminal_all_valid = true;

  // 喂给屏幕。★ 注意 SetTerminalData 内部【不会】自动弹页 ——
  //   弹页只由语音指令触发, 否则每 2 秒一帧都会把用户从对话界面踢走。
  if (g_display) {
    g_display->SetTerminalData(g_terminal_all);
  }

  // ★ 顺手把温湿度同步进老的 ENV 缓存: 终端新固件只发 ALL 不发 ENV 了,
  //   不同步的话原有的 self.terminal.env 工具会永远回答"还没收到数据"。
  //   这样老工具零改动继续可用。
  if (d.ok[0] && d.ok[1]) {
    g_terminal_env.valid = true;
    g_terminal_env.temperature = d.t;
    g_terminal_env.humidity = d.h;
    g_terminal_env.rx_ms = millis();
  }

  int invalid_cnt = 0;
  for (int i = 0; i < Display::kTermItemCount; ++i) {
    if (!d.ok[i]) {
      ++invalid_cnt;
    }
  }
  printf("[TermAll] 收到: T=%.1f H=%.1f PR=%.1f CO2=%.0f LX=%.0f "
         "ST=%.1f SH=%.1f NPK=%.0f/%.0f/%.0f PH=%.1f (无效项 %d 个)\n",
         d.ok[0] ? d.t : 0.0f, d.ok[1] ? d.h : 0.0f, d.ok[2] ? d.pr : 0.0f,
         d.ok[3] ? d.co2 : 0.0f, d.ok[4] ? d.lx : 0.0f, d.ok[5] ? d.st : 0.0f,
         d.ok[6] ? d.sh : 0.0f, d.ok[7] ? d.n : 0.0f, d.ok[8] ? d.p : 0.0f,
         d.ok[9] ? d.k : 0.0f, d.ok[10] ? d.ph : 0.0f, invalid_cnt);
}

// ================== 作物生长标准下发 (2026-10-20 新增) ==================
// 用户对主控说「查一下土豆生长的温度湿度标准，下发给终端」→
//   ① AI 用云端模型算出 11 项适宜区间;
//   ② 本机整理成一行 CFG 通过 39/40 发给 B 板;
//   ③ B 板存进 Flash(NVS)，之后每 2 秒的读数就按这套标准染色(绿=正常 红=超限)。
//
// ★★ 为什么参数只用一个字符串 ranges:
//   MCP 工具只支持 int64 / string / bool 三种类型(见 ai_vox_types.h)。
//   11 项 × 上下限 = 22 个数，逐个当参数会让 schema 膨胀一百多行，AI 还容易漏填。
//   用一个固定格式的字符串，工具描述里写清格式 + 一个例子，AI 反而更稳。
//
// ★★ 为什么还要在本机解析一遍再重发，而不是直接透传 AI 给的字符串:
//   1) AI 的格式可能带小数、空格、大小写不一，透传出去 B 板解析不了；
//   2) 本机能给出精确的错误提示让 AI 自己重发，不用用户来猜；
//   3) 组装出来的行是规范化的，B 板那边的解析逻辑可以写得很死板(更不容易出错)。
constexpr int kCropItemCount = 11;

// ★ 顺序必须与 B 板 kCropKey[] / g_metric[] 完全一致
constexpr const char* kCropKeys[kCropItemCount] = {"T", "H", "PR", "CO2", "LX", "ST", "SH", "N", "P", "K", "PH"};
constexpr const char* kCropLabels[kCropItemCount] = {"空气温度", "空气湿度", "气压",   "空气CO2", "光照强度", "土壤温度",
                                                     "土壤湿度", "土壤氮",   "土壤磷", "土壤钾",   "土壤PH"};
// 每项保留几位小数（与 B 板显示精度一致，避免下发 23.47 但屏上只显示 23.5）
constexpr int kCropDec[kCropItemCount] = {1, 1, 1, 0, 0, 1, 1, 0, 0, 0, 1};

// ★★ 终端字库能画出来的全部汉字。必须与 B 板 font_data.h 保持一致（同一份 chars_dash.txt 生成）。
//   作物名里只要有一个字不在这串里，B 板就会画成红框 —— 所以在下发【之前】就拦下来，
//   让 AI 当场换个名字，而不是等用户看到满屏红框再来问。
constexpr const char* kCropNameCharset =
    "空气温度湿压光照强土壤氮磷钾酸值电以上下左右室内外正常偏高低报警传感器数据实时监测"
    "未接等待秒更新共项平均超限全部标准已设定作物无"
    "土豆马铃薯番茄西红柿黄瓜辣椒茄子生菜白菜菠菜芹菜韭菜大葱大蒜生姜萝卜胡萝卜"
    "西瓜甜瓜玉米小麦水稻大豆花生棉花向日葵葡萄苹果梨桃香蕉山药红薯"
    "西兰花豆角南瓜冬瓜苦瓜蘑菇油菜甘蓝洋葱豌豆绿豆高粱谷子芝麻烟草茶叶"
    "花卉多肉绿萝月季";

struct CropRange {
  bool  has;
  float lo;
  float hi;
};

// 一个 UTF-8 字符占几字节
static int CropUtf8Len(const char* p) {
  const unsigned char b0 = static_cast<unsigned char>(*p);
  if ((b0 & 0xE0) == 0xC0) return 2;
  if ((b0 & 0xF0) == 0xE0) return 3;
  if ((b0 & 0xF8) == 0xF0) return 4;
  return 1;
}

// 作物名的每个字是否都在终端字库里。false 时把第一个缺的字写进 bad。
// ★ 用 strstr 在字符集里找整个 UTF-8 字符是安全的: 多字节字符的首字节区间
//   (0xC0~0xF7) 与续字节区间 (0x80~0xBF) 不重叠，不会匹配到半个字。
static bool CropNameDrawable(const char* s, char* bad, size_t bad_n) {
  if (bad != nullptr && bad_n > 0) {
    bad[0] = '\0';
  }
  if (s == nullptr) {
    return false;
  }
  for (const char* p = s; *p != '\0';) {
    const int len = CropUtf8Len(p);
    char one[5] = {0};
    for (int i = 0; i < len && p[i] != '\0'; ++i) {
      one[i] = p[i];
    }
    if (strstr(kCropNameCharset, one) == nullptr) {
      if (bad != nullptr && bad_n > 0) {
        snprintf(bad, bad_n, "%s", one);
      }
      return false;
    }
    p += len;
  }
  return true;
}

// 解析一项 "10:25" 或 "10~25"。
// ★ 上下限写反了自动交换：意图很明显就是"10 到 25 之间"，没必要报错让 AI 重发。
static bool CropParseOne(const char* v, float* lo, float* hi) {
  if (v == nullptr || lo == nullptr || hi == nullptr) {
    return false;
  }
  char* end = nullptr;
  const float a = strtof(v, &end);
  if (end == v) {
    return false;  // 一个数字都没解析出来
  }
  if (*end != ':' && *end != '~') {
    return false;  // 上下限之间必须是冒号或波浪号
  }
  const char* q = end + 1;
  const float b = strtof(q, &end);
  if (end == q) {
    return false;
  }
  // ★ 用"量级上限"而不是 isinf: strtof 溢出会返回 INF，量级判断一样能拦住，
  //   而且不依赖 isinf 宏的具体实现（新旧 newlib 对 C99 宏的处理不一致）。
  if (isnan(a) || isnan(b) || fabsf(a) > 1e6f || fabsf(b) > 1e6f) {
    return false;
  }
  *lo = (a <= b) ? a : b;
  *hi = (a <= b) ? b : a;
  return true;
}

// 解析 "T=10:25;H=60:80;PH=5.5:6.5"。分号、逗号、空格都当分隔符，键名大小写不敏感。
// 返回成功解析出的项数；不认识的键直接忽略（这样 AI 多写几项也不会整体失败）。
static int ParseCropRanges(const char* s, CropRange* out) {
  if (s == nullptr || out == nullptr) {
    return 0;
  }
  for (int i = 0; i < kCropItemCount; ++i) {
    out[i].has = false;
    out[i].lo = 0.0f;
    out[i].hi = 0.0f;
  }

  int parsed = 0;
  const char* p = s;
  while (*p != '\0') {
    while (*p == ';' || *p == ',' || *p == ' ' || *p == '\t') {
      ++p;
    }
    if (*p == '\0') {
      break;
    }
    // 切出一段 "KEY=VALUE"
    const char* seg_end = p;
    while (*seg_end != '\0' && *seg_end != ';' && *seg_end != ',' && *seg_end != ' ') {
      ++seg_end;
    }
    // 找等号
    const char* eq = p;
    while (eq < seg_end && *eq != '=') {
      ++eq;
    }
    if (eq < seg_end) {
      const int klen = static_cast<int>(eq - p);
      char key[8] = {0};
      if (klen > 0 && klen < static_cast<int>(sizeof(key))) {
        for (int i = 0; i < klen; ++i) {
          key[i] = static_cast<char>(toupper(static_cast<unsigned char>(p[i])));
        }
        float lo = 0.0f;
        float hi = 0.0f;
        if (CropParseOne(eq + 1, &lo, &hi)) {
          for (int i = 0; i < kCropItemCount; ++i) {
            if (strcmp(key, kCropKeys[i]) == 0) {
              out[i].has = true;
              out[i].lo = lo;
              out[i].hi = hi;
              ++parsed;
              break;
            }
          }
        }
      }
    }
    p = seg_end;
  }
  return parsed;
}

// 组装发给 B 板的 CFG 行。返回长度，-1 表示缓冲区不够。
static int BuildCropCfg(const char* crop, const CropRange* r, char* out, size_t cap) {
  int len = snprintf(out, cap, "CFG,NAME=%s", crop);
  if (len < 0 || static_cast<size_t>(len) >= cap) {
    return -1;
  }
  for (int i = 0; i < kCropItemCount; ++i) {
    if (!r[i].has) {
      continue;
    }
    const int w = snprintf(out + len, cap - static_cast<size_t>(len), ",%s=%.*f:%.*f", kCropKeys[i],
                           kCropDec[i], r[i].lo, kCropDec[i], r[i].hi);
    if (w < 0 || static_cast<size_t>(w) >= cap - static_cast<size_t>(len)) {
      return -1;
    }
    len += w;
  }
  return len;
}

// ---------------- 作物标准下发的回执跟踪 (2026-10-20 新增) ----------------
// ★★ 为什么必须做回执并显示在屏幕上：
//   下发是单向的 —— A 板发出去就返回了，用户完全不知道 B 板收到没有。
//   现象就是"我说了同步到终端，但屏幕上什么变化都没有"（用户 2026-10-20 实测反馈）。
//   有了回执 + 状态栏显示，三种情况一眼能分开：
//     · 显示"终端已记住 XX"  → 成功
//     · 显示"终端不接受该标准" → B 板收到了但解析失败（协议不匹配）
//     · 显示"终端没有回应"    → 串口不通 / B 板没上电（3 秒超时）
static constexpr uint32_t kCropAckTimeoutMs = 3000;
static bool     g_crop_ack_pending = false;
static uint32_t g_crop_ack_sent_ms = 0;
static char     g_crop_ack_name[24] = {0};

// 把下发结果同时写到【状态栏】和【对话区的系统消息】里。
// ★★ 为什么不能只写状态栏：状态栏会被对话状态机立刻覆盖 ——
//   AI 一开口就是 ShowStatus("说话中")、结束后 ShowStatus("待命")，
//   而回执是在这两者之间的几百毫秒内到的，只写状态栏等于闪一下就没了，
//   用户根本看不到（2026-10-20 实测就是这个感受）。
//   系统消息会留在对话列表里，用户可以回头确认到底成功没有。
static void CropNotify(const char* msg) {
  if (g_display == nullptr) {
    return;
  }
  g_display->ShowStatus(msg);
  g_display->SetChatMessage(Display::Role::kSystem, msg);
}

// loop 里每轮调用：等 B 板的 CFG_OK / CFG_ERR，或超时。
// ★ 非阻塞：只比对字符串和时刻，没在等回执时开销是一次 bool 判断。
static void CropAckPoll() {
  if (!g_crop_ack_pending) {
    return;
  }
  // ---- 超时判定 ----
  if (millis() - g_crop_ack_sent_ms > kCropAckTimeoutMs) {
    g_crop_ack_pending = false;
    CropNotify("终端没有回应, 请检查 39/40 接线和终端供电");
    printf("[Crop] ACK 超时(%ums), 终端可能没上电或串口不通\n",
           static_cast<unsigned>(kCropAckTimeoutMs));
    return;
  }
  if (!g_link_has_line) {
    return;
  }
  // ---- 同一帧只处理一次（与 TerminalAllPoll 同一套路数）----
  static uint32_t s_ack_handled_ms = 0;
  static bool s_ack_handled_valid = false;
  if (s_ack_handled_valid && g_link_last_rx_ms == s_ack_handled_ms) {
    return;
  }
  s_ack_handled_ms = g_link_last_rx_ms;
  s_ack_handled_valid = true;

  if (strncmp(g_link_last_line, "CFG_OK", 6) == 0) {
    g_crop_ack_pending = false;
    char st[64] = {0};
    snprintf(st, sizeof(st), "终端已记住%s的标准, 屏幕会按标准染色", g_crop_ack_name);
    CropNotify(st);
    printf("[Crop] 终端回执 %s\n", g_link_last_line);
  } else if (strncmp(g_link_last_line, "CFG_ERR", 7) == 0) {
    g_crop_ack_pending = false;
    CropNotify("终端不接受该标准, 请重试");
    printf("[Crop] 终端解析失败 %s\n", g_link_last_line);
  }
}

// ================== 自动监测与联动执行器 (2026-10-23 新增) ==================
//  用户说"全自动检测"或"打开光照监测" → AI 经 self.monitor.start 给出通道与阈值
//  → 主控用终端每 2 秒上报的 ALL 数据实时比对 → 超限自动发 ACT 指令联动执行器
//  → 屏幕常驻监测面板 + 语音播报动作。说"停止监测"退出。
//
//  ★★ 架构铁律: 大模型只出【知识】(阈值, 一次性), 绝不参与【实时决策】。
//    监测闭环完全在本机 loop 里跑(非阻塞), 若让监测去等大模型, 语音链路必卡死。
//
//  ★★ 防抖三件套(缺一不可, 否则答辩时执行器会抽风):
//    1) 回差 margin: 超上限后必须回落 margin 才算恢复, 阈值附近抖动不会反复开关;
//    2) 最小保持 kMonStateHoldMs: 单通道状态 8 秒内不许翻第二次;
//    3) 执行器期望状态缓存: 只在期望值变化时发 ACT, 不刷串口。
//  通道与执行器的联动是【多对一合并】的: 风扇 = 温度超限 | 湿度超限 | CO2超限;
//  暖光灯 = 温度偏低 | 光照偏低; 加湿器 = 湿度偏低 | 土壤湿度偏低。先合并再下发,
//  避免"温度让关风扇、CO2 让开风扇"互相打架。

// 当前会话状态。由 loop 事件流里的 StateChangedEvent 分支持续更新。
// 用途: ① 监测播报的"状态门卫"(AI 说话中不插话) ② BOOT 键按状态决定是否调 Advance。
// ★ 初值 kIdle: 引擎 Start() 之前就是这个态, 各使用方对它都有安全分支。
static volatile ai_vox::ChatState g_chat_state = ai_vox::ChatState::kIdle;

// ================== 智谱 GLM 自主控硬件 (ReAct 兜底, 借鉴 mimiclaw) ==================
// 用户需求(2026-11-08): 官方小智没理解模糊指令(如"帮我把房间弄凉快")时,
// 本地把用户原话喂给智谱 GLM-4-Flash(免费), 让 GLM 自主决定调哪个硬件工具并执行。
// ★ 智谱接口兼容 OpenAI 协议, 与 mimiclaw 的 DeepSeek 接入同构。
// ★★ API Key 安全: 不硬编码, 放同目录 glm_secrets.h(已 gitignore 同理, 需手动建)。
#if __has_include("glm_secrets.h")
#include "glm_secrets.h"
#endif
#ifndef GLM_API_KEY
#define GLM_API_KEY ""  // 留空 = 智谱兜底通道关闭
#endif
static constexpr const char* kGlmUrl = "https://open.bigmodel.cn/api/paas/v4/chat/completions";
static constexpr const char* kGlmModel = "glm-4-flash";

// 记录"用户上一句原话"和"本轮官方是否已调硬件工具", 用于判定要不要智谱兜底
static char     g_last_user_text[256] = {0};
static bool     g_official_handled_hw = false;   // 官方本轮是否调过硬件 MCP 工具
static uint32_t g_glm_recheck_ms = 0;            // 兜底判定时刻(AI 回复完后的时刻)

// 智谱 ReAct 工具清单(只暴露硬件, 避免 token 暴涨与模型混乱)。
// 每个工具: name / description / 参数 JSON Schema。执行分发见 GlmExecTool()。
static const char* const kGlmToolsJson =
  "["
  "{\"type\":\"function\",\"function\":{\"name\":\"servo_set\","
  "\"description\":\"转动某一路舵机到指定角度(0~180)。当用户想转动/调整舵机位置时用。\","
  "\"parameters\":{\"type\":\"object\",\"properties\":{\"servo\":{\"type\":\"integer\",\"description\":\"舵机编号1~3\"},"
  "\"angle\":{\"type\":\"integer\",\"description\":\"角度0~180\"}},\"required\":[\"servo\",\"angle\"]}}},"
  "{\"type\":\"function\",\"function\":{\"name\":\"motor1_set\","
  "\"description\":\"控制1号直流电机(可调速)。direction: forward正转/reverse反转/stop停; speed:0~255可选。\","
  "\"parameters\":{\"type\":\"object\",\"properties\":{\"direction\":{\"type\":\"string\"},\"speed\":{\"type\":\"integer\"}},"
  "\"required\":[\"direction\"]}}},"
  "{\"type\":\"function\",\"function\":{\"name\":\"motor2_set\","
  "\"description\":\"控制2号直流电机(只能开关,不能调速)。direction: forward正转/reverse反转/stop停/brake刹车。\","
  "\"parameters\":{\"type\":\"object\",\"properties\":{\"direction\":{\"type\":\"string\"}},\"required\":[\"direction\"]}}},"
  "{\"type\":\"function\",\"function\":{\"name\":\"led_set\","
  "\"description\":\"控制8颗RGB灯。color: red/green/blue/yellow/cyan/purple/white; 或 off 关灯。led:1~8指定单颗,0=全部。\","
  "\"parameters\":{\"type\":\"object\",\"properties\":{\"color\":{\"type\":\"string\"},\"led\":{\"type\":\"integer\"}},"
  "\"required\":[\"color\"]}}},"
  "{\"type\":\"function\",\"function\":{\"name\":\"fan_set\","
  "\"description\":\"控制终端(另一块板B)的风扇档位。当用户想降温、通风、开风扇时用。level:0停/1低速/2中速/3高速。\","
  "\"parameters\":{\"type\":\"object\",\"properties\":{\"level\":{\"type\":\"integer\",\"description\":\"0~3\"}},"
  "\"required\":[\"level\"]}}}"
  "]";

enum MonChanId {
  kMonLx = 0,   // 光照 BH1750
  kMonT,        // 空气温度 AHT20
  kMonH,        // 空气湿度 AHT20
  kMonCo2,      // CO2 JW01
  kMonSh,       // 土壤湿度 485
  kMonNpk,      // 土壤养分(N/P/K 取最小值) 485
  kMonCount
};

struct MonChan {
  bool    enabled;
  bool    has_lo;
  bool    has_hi;
  float   lo;
  float   hi;
  uint8_t state;        // 0 正常 1 超上限 2 低于下限 3 数据无效
  float   value;
  bool    value_ok;
  uint32_t state_ms;    // 进入当前状态的时刻(最小保持用)
};

static MonChan g_mon[kMonCount] = {};
static bool    g_mon_active = false;
static char    g_mon_title[48] = {0};        // 面板标题 "自动监测-草莓"

// 执行器期望状态缓存(只在变化时发 ACT)
static bool     g_mon_fan_on    = false;
static bool     g_mon_light_on  = false;
static bool     g_mon_lamp_on   = false;   // 照明灯(白光 LAMP): 光照不足补光用
static bool     g_mon_humid_on  = false;
static bool     g_mon_shade_out = false;      // 遮阳伞是否处于"展开"状态
static bool     g_npk_pulse_pending = false;  // 施肥舵机脉冲: 转150°后 1.5s 回90°
static uint32_t g_npk_pulse_ms = 0;

static uint32_t g_mon_last_tick_ms  = 0;
static uint32_t g_mon_last_speak_ms = 0;
static char     g_mon_speak_pending[192] = {0};  // 被节流暂扣的播报, 到点补发(只留最新)

static constexpr uint32_t kMonPeriodMs    = 2000;   // 监测周期(与终端上报同频)
static constexpr uint32_t kMonDataTtlMs   = 10000;  // ALL 帧超 10 秒没来 = 数据失效
static constexpr uint32_t kMonStateHoldMs = 8000;   // 单通道状态最小保持
static constexpr uint32_t kMonSpeakGapMs  = 4000;   // 两条播报的最小间隔
static constexpr uint32_t kMonNpkPulseMs  = 1500;   // 施肥舵机 150° 保持时长

struct MonMeta {
  const char* name;        // 面板显示名
  const char* unit;
  int         term_idx;    // g_terminal_all 字段下标, -1 = NPK 合成值
  const char* hi_action;   // 超上限联动文案(面板第二行)
  const char* lo_action;   // 低于下限联动文案
};
static constexpr MonMeta kMonMeta[kMonCount] = {
    {"光照",     "lx",    4, "遮阳伞展开", "照明灯补光"},
    {"空气温度", "C",     0, "风扇降温",   "暖光灯升温"},
    {"空气湿度", "%",     1, "风扇除湿",   "加湿器加湿"},
    {"CO2",     "ppm",    3, "风扇通风",   ""},
    {"土壤湿度", "%",     6, "",          "加湿器补水"},
    {"土壤养分", "mg/kg", -1, "",          "施肥舵机投肥"},
};

// ---- 通道名解析: 英文/中文/常见口语都收, 中文标点当分隔符 ----
struct MonChanWord {
  const char* word;
  uint8_t     chan;
};
static constexpr MonChanWord kMonWords[] = {
    {"all", 0x3F}, {"全部", 0x3F}, {"全自动", 0x3F},
    {"lux", kMonLx}, {"光照", kMonLx}, {"lx", kMonLx}, {"light", kMonLx},
    {"temp", kMonT}, {"温度", kMonT}, {"t", kMonT},
    {"hum", kMonH}, {"humidity", kMonH}, {"空气湿度", kMonH}, {"湿度", kMonH}, {"h", kMonH},
    {"co2", kMonCo2}, {"二氧化碳", kMonCo2},
    {"soil", kMonSh}, {"sh", kMonSh}, {"土壤", kMonSh}, {"土壤湿度", kMonSh},
    {"npk", kMonNpk}, {"养分", kMonNpk}, {"施肥", kMonNpk}, {"土壤养分", kMonNpk},
};

static uint8_t MonParseChannels(const char* s) {
  if (s == nullptr || s[0] == '\0') {
    return 0;
  }
  // 预处理: 把中文逗号/分号/顿号替换成 ASCII ','(否则"光照、温度"会粘成一个 token)
  char buf[160];
  size_t o = 0;
  for (const char* p = s; *p != '\0' && o + 1 < sizeof(buf);) {
    const unsigned char b0 = static_cast<unsigned char>(p[0]);
    const unsigned char b1 = static_cast<unsigned char>(p[1]);
    const unsigned char b2 = static_cast<unsigned char>(p[2]);
    if (b0 == 0xEF && b1 == 0xBC && (b2 == 0x8C || b2 == 0x9B)) {  // ，；
      buf[o++] = ',';
      p += 3;
      continue;
    }
    if (b0 == 0xE3 && b1 == 0x80 && b2 == 0x81) {  // 、
      buf[o++] = ',';
      p += 3;
      continue;
    }
    buf[o++] = *p++;
  }
  buf[o] = '\0';

  uint8_t mask = 0;
  char* save = nullptr;
  for (char* tok = strtok_r(buf, ",; \t/|", &save); tok != nullptr;
       tok = strtok_r(nullptr, ",; \t/|", &save)) {
    for (char* q = tok; *q != '\0'; ++q) {
      if (*q >= 'A' && *q <= 'Z') {
        *q = static_cast<char>(*q - 'A' + 'a');
      }
    }
    for (const auto& w : kMonWords) {
      if (strcmp(tok, w.word) == 0) {
        if (w.chan == 0x3F) {
          return 0x3F;   // "all" 直接全开
        }
        mask |= static_cast<uint8_t>(1u << w.chan);
        break;
      }
    }
  }
  return mask;
}

// 读一个通道的当前值。返回 false = 无数据(没收到帧/该项无效/帧太旧)。
// ★ NPK 取 N/P/K 三项里有效的最小值 —— 任何一项养分低于下限都该报警。
static bool MonReadValue(int ch, float* out) {
  if (!g_terminal_all_valid || out == nullptr) {
    return false;
  }
  const Display::TerminalData& d = g_terminal_all;
  if ((uint32_t)(millis() - d.rx_ms) > kMonDataTtlMs) {
    return false;
  }
  const float values[11] = {d.t, d.h, d.pr, d.co2, d.lx, d.st, d.sh, d.n, d.p, d.k, d.ph};
  if (ch == kMonNpk) {
    float m = 0.0f;
    bool any = false;
    for (int i = 7; i <= 9; ++i) {
      if (!d.ok[i]) {
        continue;
      }
      m = any ? ((values[i] < m) ? values[i] : m) : values[i];
      any = true;
    }
    if (!any) {
      return false;
    }
    *out = m;
    return true;
  }
  const int idx = kMonMeta[ch].term_idx;
  if (idx < 0 || !d.ok[idx]) {
    return false;
  }
  *out = values[idx];
  return true;
}

// 数值/阈值格式化精度(与传感器精度对齐, 面板不显示假精度)
static void MonFmt(int ch, float v, char* buf, size_t n) {
  switch (ch) {
    case kMonT:
    case kMonH:
    case kMonSh: snprintf(buf, n, "%.1f", v); break;
    default:     snprintf(buf, n, "%.0f", v); break;   // lx / co2 / npk
  }
}

// 发一条 ACT 指令给终端。
// ★ 故意【不】碰 g_act_ack_pending 那套标志: 那是"用户点名的指令"的回执跟踪,
//   监测的联动指令有自己的面板反馈, 混在一起会互相覆盖(回执只有最后一条)。
static void MonSendAct(const char* kv) {
  char line[96] = {0};
  snprintf(line, sizeof(line), "ACT,%s", kv);
  LinkSendLine(line);
  printf("[Mon] 下发 %s\n", line);
}

// 语音播报: 走小智协议的 listen/detect 文本(相当于"用户说了一句话"),
// 云端模型会接这句话继续对话, 于是这段事件被 TTS 说出口。
// ★ SendText 在 WebSocket 未连接时内部直接丢弃, 不会阻塞也不会报错。
// ★ 文本里绝不能含双引号/反斜杠(会打碎 JSON) —— 本模块所有文案都是纯中文+数字。
static void MonSpeakNow(const char* text) {
  if (text == nullptr || text[0] == '\0') {
    return;
  }
  if (g_display != nullptr) {
    g_display->SetChatMessage(Display::Role::kSystem, text);
  }
  printf("[Mon][播报] %s\n", text);

  // ★★ 状态门卫: AI 正在说话(kSpeaking)时【绝不】插 listen/detect。
  //   踩过的坑: TTS 播到一半被监测播报打断, 服务端会话状态容易错乱,
  //   之后的对话就唤不醒了(用户实测"监测状态下无法唤起对讲"的成因之一)。
  //   改为暂扣进 pending 队列, MonitorTick 里等 AI 说完再补发。
  if (g_chat_state == ai_vox::ChatState::kSpeaking) {
    snprintf(g_mon_speak_pending, sizeof(g_mon_speak_pending), "%s", text);
    printf("[Mon][播报] AI 说话中, 暂扣待补发\n");
    return;
  }

  // ★★ 必须带 session_id(2026-10-23 实测: 不带时屏幕显示正常但语音不出来)。
  //   服务端按 session 路由 listen/detect, 缺 session_id 会被静默丢弃。
  //   SDK 自己发这条消息时(OnJsonData 的 hello 分支)就带着 session_id_, 现已通过
  //   GetSessionId() 暴露出来。空串时也照发 —— 与 SDK 行为一致(hello 未到时 SDK 也发空)。
  const std::string mon_sid = ai_vox::Engine::GetInstance().GetSessionId();
  char json[384] = {0};
  const int n = snprintf(json, sizeof(json),
                         "{\"session_id\":\"%s\",\"type\":\"listen\",\"state\":\"detect\",\"text\":"
                         "\"监测系统事件: %s。请向听众简短播报这个动作。\"}",
                         mon_sid.c_str(), text);
  if (n > 0 && static_cast<size_t>(n) < sizeof(json)) {
    ai_vox::Engine::GetInstance().SendText(json);
  }
}

// 带节流的播报: 间隔不够时暂扣(只留最新一条), MonitorTick 里到点补发。
static void MonSpeak(const char* text) {
  const uint32_t now = millis();
  if (g_mon_last_speak_ms == 0 ||
      (uint32_t)(now - g_mon_last_speak_ms) >= kMonSpeakGapMs) {
    g_mon_last_speak_ms = now;
    MonSpeakNow(text);
    return;
  }
  snprintf(g_mon_speak_pending, sizeof(g_mon_speak_pending), "%s", text);
}

// 启用/复位一个通道
static void MonEnableChan(int ch, bool has_lo, float lo, bool has_hi, float hi) {
  MonChan& c = g_mon[ch];
  c.enabled = true;
  c.has_lo = has_lo;
  c.has_hi = has_hi;
  c.lo = lo;
  c.hi = hi;
  c.state = 3;              // 刚启动还不知道状态, 等第一次读数
  c.value = 0.0f;
  c.value_ok = false;
  c.state_ms = millis();
}

// 把 AI 给的 11 项区间套到被点名的通道上。
// 返回配置成功的通道数; missing 里列出"点了名但没给阈值"的通道。
// ★ CO2 只有上限有意义(过低不用管); NPK 取 N/P/K 给出的最低下限。
static int MonApplyRanges(const CropRange* r, uint8_t mask, char* missing, size_t missing_n) {
  if (missing != nullptr && missing_n > 0) {
    missing[0] = '\0';
  }
  int configured = 0;
  auto miss = [&](const char* name) {
    if (missing != nullptr && missing_n > 0) {
      if (missing[0] != '\0') {
        strncat(missing, " ", missing_n - strlen(missing) - 1);
      }
      strncat(missing, name, missing_n - strlen(missing) - 1);
    }
  };

  if ((mask & (1u << kMonLx)) != 0) {
    if (r[4].has) { MonEnableChan(kMonLx, true, r[4].lo, true, r[4].hi); ++configured; }
    else { miss("光照(LX)"); }
  }
  if ((mask & (1u << kMonT)) != 0) {
    if (r[0].has) { MonEnableChan(kMonT, true, r[0].lo, true, r[0].hi); ++configured; }
    else { miss("温度(T)"); }
  }
  if ((mask & (1u << kMonH)) != 0) {
    if (r[1].has) { MonEnableChan(kMonH, true, r[1].lo, true, r[1].hi); ++configured; }
    else { miss("湿度(H)"); }
  }
  if ((mask & (1u << kMonCo2)) != 0) {
    if (r[3].has) { MonEnableChan(kMonCo2, false, 0.0f, true, r[3].hi); ++configured; }
    else { miss("CO2(CO2)"); }
  }
  if ((mask & (1u << kMonSh)) != 0) {
    if (r[6].has) { MonEnableChan(kMonSh, true, r[6].lo, true, r[6].hi); ++configured; }
    else { miss("土壤湿度(SH)"); }
  }
  if ((mask & (1u << kMonNpk)) != 0) {
    float lo = 1e9f;
    bool has = false;
    for (int i = 7; i <= 9; ++i) {
      if (r[i].has && r[i].lo < lo) {
        lo = r[i].lo;
        has = true;
      }
    }
    if (has) { MonEnableChan(kMonNpk, true, lo, false, 0.0f); ++configured; }
    else { miss("养分(N/P/K)"); }
  }
  return configured;
}

// 启动监测。成功返回 true 并把确认文本写进 out(给 AI 播报), 失败返回 false 并写错误原因。
static bool MonitorStart(const char* channels_str, const char* ranges_str,
                         const char* crop_name, char* out, size_t out_n) {
  const uint8_t mask = MonParseChannels(channels_str);
  if (mask == 0) {
    snprintf(out, out_n,
             "没有认出任何监测通道。channels 要么写 all, 要么包含这些名字之一: "
             "光照/温度/湿度/co2/土壤/养分(英文 lux/temp/hum/co2/soil/npk 亦可)。");
    return false;
  }
  CropRange r[kCropItemCount];
  const int parsed = ParseCropRanges(ranges_str != nullptr ? ranges_str : "", r);
  if (parsed <= 0) {
    snprintf(out, out_n,
             "ranges 里一个有效区间都没解析出来。格式必须是 KEY=下限:上限, 多项用分号分隔, "
             "例如 LX=3000:20000;T=15:28;H=50:75。");
    return false;
  }

  for (int ch = 0; ch < kMonCount; ++ch) {
    g_mon[ch].enabled = false;   // 先全关, 再按点名开
  }
  char missing[128] = {0};
  const int configured = MonApplyRanges(r, mask, missing, sizeof(missing));
  if (configured == 0) {
    snprintf(out, out_n,
             "点名的通道一个阈值都没配上(%s)。ranges 里必须有对应 KEY: "
             "光照→LX, 温度→T, 湿度→H, co2→CO2, 土壤湿度→SH, 养分→N/P/K。",
             (missing[0] != '\0') ? missing : "通道与 KEY 对不上");
    return false;
  }

  // 复位联动状态缓存(遮阳伞默认按"收起"算 —— 上电就是收起的)
  g_mon_fan_on = g_mon_light_on = g_mon_humid_on = false;
  g_mon_lamp_on = false;
  g_mon_shade_out = false;
  g_npk_pulse_pending = false;

  // ★★ 监测接管底态: 把四路受控执行器静默关一遍。
  //   为什么必须: 缓存现在全是 false, 若上一轮监测/手动把风扇开着, 本轮监测会以为
  //   它是关的 → 条件一直正常就永远不会发 FAN=0 → 风扇一直转, 演示时像"关不掉"。
  //   先关一遍, 让"缓存=false"和"实际=关"严格一致, 之后监测才是这些执行器的唯一主人。
  //   ★ 不发 SHADE —— 遮阳伞行程 45 秒, 中途反走有顶坏风险(见 self.act.set 的警告),
  //     上电默认就是收起的, 只把缓存置 false 即可。
  MonSendAct("FAN=0");
  MonSendAct("LIGHT=0");
  MonSendAct("LAMP=0");
  MonSendAct("HUMID=0");

  // 面板标题
  const bool has_crop = (crop_name != nullptr && crop_name[0] != '\0');
  if (has_crop && strlen(crop_name) <= 21u) {
    snprintf(g_mon_title, sizeof(g_mon_title), "自动监测-%s", crop_name);
  } else {
    int cnt = 0;
    int only = -1;
    for (int ch = 0; ch < kMonCount; ++ch) {
      if (g_mon[ch].enabled) {
        ++cnt;
        only = ch;
      }
    }
    if (cnt == 1 && only >= 0) {
      snprintf(g_mon_title, sizeof(g_mon_title), "监测-%s", kMonMeta[only].name);
    } else {
      snprintf(g_mon_title, sizeof(g_mon_title), "自动监测");
    }
  }

  // 顺手把这套标准下发给终端染色(与 self.crop.profile 同一条通路)。
  // ★ 有作物名才发: B 板标题栏要画作物名, 空名会画成空白。
  char cfg_note[80] = {0};
  if (has_crop && strlen(crop_name) <= 21u && CropNameDrawable(crop_name, nullptr, 0)) {
    char cfg[256] = {0};
    if (BuildCropCfg(crop_name, r, cfg, sizeof(cfg)) > 0) {
      LinkSendLine(cfg);
      snprintf(g_crop_ack_name, sizeof(g_crop_ack_name), "%s", crop_name);
      g_crop_ack_sent_ms = millis();
      g_crop_ack_pending = true;
      snprintf(cfg_note, sizeof(cfg_note), "标准已同步终端染色; ");
    }
  }

  g_mon_active = true;
  g_mon_last_tick_ms = 0;      // 下一圈立即做第一次比对, 不用等 2 秒
  g_mon_last_speak_ms = 0;
  g_mon_speak_pending[0] = '\0';
  if (g_display != nullptr) {
    g_display->HideTerminalPage();
    g_display->ShowMonitorPage();
  }
  printf("[Mon] 监测启动: mask=0x%02X, configured=%d, 标题=%s\n",
         static_cast<unsigned>(mask), configured, g_mon_title);

  // 组确认文本(给 AI 播报)
  char list[320] = {0};
  for (int ch = 0; ch < kMonCount; ++ch) {
    if (!g_mon[ch].enabled) {
      continue;
    }
    char vb[16] = {0};
    char lb[16] = {0};
    char one[80];
    if (g_mon[ch].has_lo && g_mon[ch].has_hi) {
      MonFmt(ch, g_mon[ch].lo, lb, sizeof(lb));
      MonFmt(ch, g_mon[ch].hi, vb, sizeof(vb));
      snprintf(one, sizeof(one), "%s 宜%s~%s%s; ", kMonMeta[ch].name, lb, vb, kMonMeta[ch].unit);
    } else if (g_mon[ch].has_hi) {
      MonFmt(ch, g_mon[ch].hi, vb, sizeof(vb));
      snprintf(one, sizeof(one), "%s 上限%s%s; ", kMonMeta[ch].name, vb, kMonMeta[ch].unit);
    } else {
      MonFmt(ch, g_mon[ch].lo, lb, sizeof(lb));
      snprintf(one, sizeof(one), "%s 下限%s%s; ", kMonMeta[ch].name, lb, kMonMeta[ch].unit);
    }
    const size_t len = strlen(list);
    snprintf(list + len, sizeof(list) - len, "%s", one);
  }
  snprintf(out, out_n, "监测已启动: %s%s。超上限或低于下限会自动联动对应执行器并语音播报, "
           "主控屏幕常驻监测面板。请用一句话确认开始监测, 不要念全部数字。",
           list, cfg_note);
  if (missing[0] != '\0') {
    const size_t len = strlen(out);
    snprintf(out + len, out_n - len, "注意: 通道 %s 没给阈值, 未纳入监测。", missing);
  }
  return true;
}

// 停止监测并收起面板。
// ★★ 停止时全部执行器恢复初始位置(2026-10-23 用户要求, 答辩演示结束要干净复位):
//   风扇/暖光灯/照明灯/加湿器全关, 施肥舵机回中 90°, 展开过的遮阳伞收起。
//   ★ 遮阳伞必须判 g_mon_shade_out: 伞若本来就在收起位, 盲发 SHADE=2 会反走满 45s 行程顶坏
//     —— MonitorStart 的接管底态不发 SHADE 就是这个原因, 这里同理。
//   ★ 复位指令必须在清缓存【之前】发(要用当前缓存值判断伞是否展开过)。
//   ★ SHADE=2 是连续运动指令, 45 秒后终端自动停, 无需也不应在这里等回执。
static void MonitorStop() {
  if (!g_mon_active) {
    return;
  }
  // ---- 先复位执行器(此刻缓存还是监测期间的最新状态) ----
  const bool shade_was_out = g_mon_shade_out;
  MonSendAct("FAN=0");     // 风扇停
  MonSendAct("LIGHT=0");   // 暖光灯灭
  MonSendAct("LAMP=0");    // 照明灯灭
  MonSendAct("HUMID=0");   // 加湿器停
  MonSendAct("NPK=90");    // 施肥舵机回中(脉冲模式本来会回, 这里兜底确保)
  if (shade_was_out) {
    MonSendAct("SHADE=2"); // 监测期间展开过的遮阳伞 → 收起回初始位(45s 行程, 终端自动停)
  }
  // ---- 再清状态 ----
  g_mon_active = false;
  for (int ch = 0; ch < kMonCount; ++ch) {
    g_mon[ch].enabled = false;
    g_mon[ch].state = 3;
    g_mon[ch].value_ok = false;
  }
  g_mon_fan_on = g_mon_light_on = g_mon_humid_on = false;
  g_mon_lamp_on = false;
  g_mon_shade_out = false;
  g_npk_pulse_pending = false;
  LedAlarmExit();   // ★ 停止监测时若报警灯还亮着, 必须恢复用户灯效(否则红呼吸永远停不下来)
  g_mon_speak_pending[0] = '\0';
  if (g_display != nullptr) {
    g_display->HideMonitorPage();
  }
  printf("[Mon] 监测停止, 全部执行器已复位%s\n", shade_was_out ? "(含遮阳伞收起)" : "");
}

// 把 6 个通道的当前状态构建成面板行并交给显示层。
static void MonUpdatePanel() {
  if (g_display == nullptr || !g_display->IsMonitorPageActive()) {
    return;
  }
  Display::MonitorRow rows[Display::kMonItemCount];
  for (int ch = 0; ch < kMonCount; ++ch) {
    const MonChan& c = g_mon[ch];
    Display::MonitorRow& row = rows[ch];
    if (!c.enabled) {
      row.main_text = kMonMeta[ch].name;
      row.sub_text = "未监测";
      row.state = 4;
      continue;
    }
    char vb[16] = {0};
    if (c.value_ok) {
      MonFmt(ch, c.value, vb, sizeof(vb));
    } else {
      snprintf(vb, sizeof(vb), "--");
    }
    char range[48] = {0};
    char lb[16] = {0};
    char hb[16] = {0};
    if (c.has_lo && c.has_hi) {
      MonFmt(ch, c.lo, lb, sizeof(lb));
      MonFmt(ch, c.hi, hb, sizeof(hb));
      snprintf(range, sizeof(range), "宜 %s~%s", lb, hb);
    } else if (c.has_hi) {
      MonFmt(ch, c.hi, lb, sizeof(lb));
      snprintf(range, sizeof(range), "上限 %s", lb);
    } else if (c.has_lo) {
      MonFmt(ch, c.lo, lb, sizeof(lb));
      snprintf(range, sizeof(range), "下限 %s", lb);
    }
    char main_buf[96];
    snprintf(main_buf, sizeof(main_buf), "%s %s %s  %s",
             kMonMeta[ch].name, vb, kMonMeta[ch].unit, range);
    row.main_text = main_buf;

    switch (c.state) {
      case 0:
        row.sub_text = "正常";
        row.state = 0;
        break;
      case 1:
        row.sub_text = (kMonMeta[ch].hi_action[0] != '\0')
                           ? std::string("超上限 → ") + kMonMeta[ch].hi_action
                           : "超上限";
        row.state = 1;
        break;
      case 2:
        row.sub_text = (kMonMeta[ch].lo_action[0] != '\0')
                           ? std::string("低于下限 → ") + kMonMeta[ch].lo_action
                           : "低于下限";
        row.state = 2;
        break;
      default:
        row.sub_text = "无数据";
        row.state = 3;
        break;
    }
  }
  g_display->SetMonitorData(rows, g_mon_title);
}

// loop 里每轮调用: 监测状态机。全部非阻塞。
static void MonitorTick() {
  if (!g_mon_active) {
    return;
  }
  const uint32_t now = millis();

  // ---- 施肥舵机脉冲回收: 到点回中位(独立于监测周期, 不能被节流挡住) ----
  if (g_npk_pulse_pending && (uint32_t)(now - g_npk_pulse_ms) >= kMonNpkPulseMs) {
    g_npk_pulse_pending = false;
    MonSendAct("NPK=90");
  }
  // ---- 被节流暂扣的播报, 间隔够了就补发 ----
  if (g_mon_speak_pending[0] != '\0' &&
      (uint32_t)(now - g_mon_last_speak_ms) >= kMonSpeakGapMs) {
    g_mon_last_speak_ms = now;
    MonSpeakNow(g_mon_speak_pending);
    g_mon_speak_pending[0] = '\0';
  }
  // ---- 监测周期节流 ----
  if ((uint32_t)(now - g_mon_last_tick_ms) < kMonPeriodMs) {
    return;
  }
  g_mon_last_tick_ms = now;

  // ---- 1) 逐通道读数 + 带回差的状态机 ----
  for (int ch = 0; ch < kMonCount; ++ch) {
    MonChan& c = g_mon[ch];
    if (!c.enabled) {
      continue;
    }
    float v = 0.0f;
    c.value_ok = MonReadValue(ch, &v);
    c.value = c.value_ok ? v : c.value;

    // 回差: 越限后必须往回走 margin 才算恢复。
    // 有上下限用区间的 15%; 只有单边阈值用该值的 8%; 至少 0.5(单位)。
    float margin = 0.5f;
    if (c.has_lo && c.has_hi) {
      margin = (c.hi - c.lo) * 0.15f;
    } else if (c.has_hi) {
      margin = c.hi * 0.08f;
    } else if (c.has_lo) {
      margin = c.lo * 0.08f;
    }
    if (margin < 0.5f) {
      margin = 0.5f;
    }

    uint8_t target;
    if (!c.value_ok) {
      target = 3;
    } else if (c.state == 1) {
      // 已在超上限: 跌破下限算转成"低于下限", 回到 hi-margin 以内算恢复
      if (c.has_lo && v < c.lo) {
        target = 2;
      } else {
        target = (!c.has_hi || v >= c.hi - margin) ? 1 : 0;
      }
    } else if (c.state == 2) {
      if (c.has_hi && v > c.hi) {
        target = 1;
      } else {
        target = (!c.has_lo || v <= c.lo + margin) ? 2 : 0;
      }
    } else {
      if (c.has_hi && v > c.hi) {
        target = 1;
      } else if (c.has_lo && v < c.lo) {
        target = 2;
      } else {
        target = 0;
      }
    }

    if (target != c.state) {
      // 最小保持: 8 秒内不许翻第二次(数据无效的进出不受限, 那不是执行器动作)
      const bool hold_ok =
          (c.state == 3) || (target == 3) ||
          (uint32_t)(now - c.state_ms) >= kMonStateHoldMs;
      if (!hold_ok) {
        continue;
      }
      c.state = target;
      c.state_ms = now;
      printf("[Mon] %s 状态 -> %u (值 %.1f)\n", kMonMeta[ch].name,
             static_cast<unsigned>(target), v);
    }
  }

  // ---- 1.5) 报警灯效: 任一通道超限/偏低 = 主控灯带红色呼吸 ----
  //   ★ 放在合并执行器之前: 只看通道状态, 与联动了哪些执行器无关。
  //     "无数据"(3)不算报警 —— 传感器没上线不该让灯一直闪, 面板上已经显示无数据。
  bool any_alarm = false;
  for (int ch = 0; ch < kMonCount; ++ch) {
    const MonChan& c = g_mon[ch];
    if (c.enabled && (c.state == 1 || c.state == 2)) {
      any_alarm = true;
      break;
    }
  }
  if (any_alarm) {
    LedAlarmEnter();
  } else {
    LedAlarmExit();
  }

  // ---- 2) 多通道合并出执行器期望状态(多对一取"或") ----
  bool fan_want   = g_mon_fan_on;
  bool light_want = g_mon_light_on;
  bool lamp_want  = g_mon_lamp_on;
  bool humid_want = g_mon_humid_on;
  bool shade_want = g_mon_shade_out;
  bool npk_trigger = false;
  for (int ch = 0; ch < kMonCount; ++ch) {
    const MonChan& c = g_mon[ch];
    if (!c.enabled) {
      continue;
    }
    const uint8_t st = c.state;
    switch (ch) {
      case kMonLx:
        // 展开条件只有"超上限"; 恢复正常或光照不足都收回(遮阳是挡光, 光弱不用挡)
        if (st == 1) shade_want = true;
        else if (st == 0 || st == 2) shade_want = false;
        // ★ 光照不足 → 照明灯(白光 LAMP)补光, 不是暖光灯(2026-10-23 实测修正):
        //   补光的目的是"让植物见光", 白光照明灯才是干这个的; 暖光灯只留给"温度偏低"。
        if (st == 2) lamp_want = true;
        break;
      case kMonT:
        if (st == 1) fan_want = true;
        else if (st == 2) light_want = true;   // 温度偏低 → 暖光灯升温
        break;
      case kMonH:
        if (st == 1) fan_want = true;
        else if (st == 2) humid_want = true;
        break;
      case kMonCo2:
        if (st == 1) fan_want = true;
        break;
      case kMonSh:
        if (st == 2) humid_want = true;
        break;
      case kMonNpk:
        if (st == 2 && !g_npk_pulse_pending) npk_trigger = true;
        break;
      default:
        break;
    }
  }

  // ---- 3) 只在期望状态变化时发指令 + 播报(播报的是"动作", 不会说假话) ----
  if (shade_want != g_mon_shade_out) {
    g_mon_shade_out = shade_want;
    MonSendAct(shade_want ? "SHADE=1" : "SHADE=2");
    const MonChan& lx = g_mon[kMonLx];
    char vb[16] = {0};
    char hb[16] = {0};
    char text[160];
    if (shade_want && lx.value_ok) {
      MonFmt(kMonLx, lx.value, vb, sizeof(vb));
      MonFmt(kMonLx, lx.hi, hb, sizeof(hb));
      snprintf(text, sizeof(text), "光照 %s 勒克斯超过上限 %s, 已自动展开遮阳伞", vb, hb);
    } else {
      snprintf(text, sizeof(text), "光照已回落到正常区间, 已自动收回遮阳伞");
    }
    MonSpeak(text);
  }
  if (fan_want != g_mon_fan_on) {
    g_mon_fan_on = fan_want;
    MonSendAct(fan_want ? "FAN=2" : "FAN=0");
    MonSpeak(fan_want ? "监测到指标超上限, 已自动打开风扇通风"
                      : "相关指标已回到正常区间, 已自动关闭风扇");
  }
  if (light_want != g_mon_light_on) {
    g_mon_light_on = light_want;
    MonSendAct(light_want ? "LIGHT=1" : "LIGHT=0");
    MonSpeak(light_want ? "监测到温度偏低, 已自动打开暖光灯升温"
                        : "温度已恢复, 已自动关闭暖光灯");
  }
  if (lamp_want != g_mon_lamp_on) {
    g_mon_lamp_on = lamp_want;
    MonSendAct(lamp_want ? "LAMP=1" : "LAMP=0");
    const MonChan& lx = g_mon[kMonLx];
    char vb[16] = {0};
    char lb[16] = {0};
    char text[160];
    if (lamp_want && lx.value_ok) {
      MonFmt(kMonLx, lx.value, vb, sizeof(vb));
      MonFmt(kMonLx, lx.lo, lb, sizeof(lb));
      snprintf(text, sizeof(text), "光照 %s 勒克斯低于下限 %s, 已自动打开照明灯补光", vb, lb);
    } else if (lamp_want) {
      snprintf(text, sizeof(text), "光照不足, 已自动打开照明灯补光");
    } else {
      snprintf(text, sizeof(text), "光照已恢复, 已自动关闭照明灯");
    }
    MonSpeak(text);
  }
  if (humid_want != g_mon_humid_on) {
    g_mon_humid_on = humid_want;
    MonSendAct(humid_want ? "HUMID=1" : "HUMID=0");
    MonSpeak(humid_want ? "监测到湿度偏低, 已自动打开加湿器"
                        : "湿度已恢复, 已自动关闭加湿器");
  }
  if (npk_trigger) {
    MonSendAct("NPK=150");
    g_npk_pulse_pending = true;
    g_npk_pulse_ms = now;
    const MonChan& npk = g_mon[kMonNpk];
    char vb[16] = {0};
    char lb[16] = {0};
    char text[160];
    if (npk.value_ok) {
      MonFmt(kMonNpk, npk.value, vb, sizeof(vb));
      MonFmt(kMonNpk, npk.lo, lb, sizeof(lb));
      snprintf(text, sizeof(text), "土壤养分 %s 低于下限 %s, 施肥舵机已自动投肥", vb, lb);
    } else {
      snprintf(text, sizeof(text), "土壤养分低于下限, 施肥舵机已自动投肥");
    }
    MonSpeak(text);
  }

  // ---- 4) 刷新监测面板 ----
  MonUpdatePanel();
}

// ---------------- 终端执行器控制与回执 (2026-10-21 新增) ----------------
//  A 板语音 → 39/40 串口发 "ACT,KEY=值,..." → B 板动作 → 回 "ACT_OK,<全量状态>"
//  ★★ 为什么回执带回全量状态而不是只回 OK：
//    动作是"设成某个值"而不是"toggle"，回传实际生效值，A 板播报时就不用猜，
//    也顺手把 B 板上电默认值（舵机 90° 等）同步过来。
static constexpr uint32_t kActAckTimeoutMs = 2000;
// 判定"链路还活着"的时间窗：终端每 2 秒主动发一帧 ALL，所以 5 秒内有数据 = 通。
// ★ 只用来区分"A->B 单向不通"和"双向都不通"，见 ActAckPoll() 超时分支。
static constexpr uint32_t kLinkAliveWindowMs = 5000;
static bool     g_act_ack_pending = false;
static uint32_t g_act_ack_sent_ms = 0;

// 缓存 B 板最后一次回传的实际状态，供 self.act.status 直接读（不阻塞等回执）
// 顺序: LIGHT BRIGHT LAMP HUMID FAN SHADE NPK MIX SHADE_T
// (SHADE_T=遮阳伞累计前进秒数, 2026-10-23 新增, 供"复位退多少秒"播报)
static bool g_act_valid = false;
static int  g_act_v[9]  = {0, 255, 0, 0, 0, 0, 90, 90, 0};

// ★★ 回执绝不能只靠 g_link_last_line 判断（2026-10-20 实测踩到"看不见回执"）。
//   原因：终端每 2 秒还有一帧 ALL 主动上报。若 ACT_OK 和这一帧在同一轮被读进来，
//   g_link_last_line 只会留下后到的那一条，ACT_OK 直接被冲掉 → 误判超时 →
//   屏幕显示"终端没有回应"，可实际上动作已经做了。
//   所以改成：每收到一行就在 LinkCommitLine() 里立刻分类，只记标志位，
//   与"哪一行是最后一行"完全解耦。
static bool     g_act_got_ok    = false;
static uint32_t g_act_got_ok_ms = 0;
static bool     g_act_got_err   = false;   // 终端收到但一个键都没认出来
static bool     g_act_got_ack   = false;   // 终端回 "ACK:ACT,..." → 它不认识 ACT(旧固件)

// 把状态渲染成一句中文。cap 建议 ≥160。
// ★ 2026-10-23: 遮阳伞带累计前进秒数(SHADE_T), 暖光灯保持整体开关。
//   缓冲区给足, 防 -Werror=format-truncation。
static void ActFormatState(const int* v, char* out, size_t cap) {
  if (out == nullptr || cap == 0) {
    return;
  }
  char shade[48] = {0};
  const char* sd = v[5] == 1 ? "前进" : (v[5] == 2 ? "后退" : "停");
  if (v[8] > 0) {
    snprintf(shade, sizeof(shade), "遮阳伞%s(累计开%ds)", sd, v[8]);
  } else {
    snprintf(shade, sizeof(shade), "遮阳伞%s", sd);
  }

  snprintf(out, cap,
           "暖光灯%s 照明%s 加湿%s 风扇%d档 %s 草木灰%d 复合肥%d",
           v[0] ? "开" : "关",
           v[2] ? "开" : "关",
           v[3] ? "开" : "关",
           v[4],
           shade,
           v[6], v[7]);
}

// 在 ACT_OK 行里取一个整数键。找不到时返回 fallback（保留原值）。
static int ActPickInt(const char* line, const char* key, int fallback) {
  float f = 0.0f;
  if (!LinkPickFloat(line, key, &f)) {
    return fallback;
  }
  return static_cast<int>(f + 0.5f);
}

// 解析一条 ACT_OK 行，写进状态缓存。
// ★ 由 LinkCommitLine() 在"收到帧的那一刻"调用，不经过 g_link_last_line，
//   所以不会被随后到达的 ALL 上报帧冲掉。
static void ActParseOkLine(const char* line) {
  if (line == nullptr) {
    return;
  }
  int v[9];
  v[0] = ActPickInt(line, "LIGHT",   g_act_v[0]);
  v[1] = ActPickInt(line, "BRIGHT",  g_act_v[1]);
  v[2] = ActPickInt(line, "LAMP",    g_act_v[2]);
  v[3] = ActPickInt(line, "HUMID",   g_act_v[3]);
  v[4] = ActPickInt(line, "FAN",     g_act_v[4]);
  v[5] = ActPickInt(line, "SHADE",   g_act_v[5]);
  v[6] = ActPickInt(line, "NPK",     g_act_v[6]);
  v[7] = ActPickInt(line, "MIX",     g_act_v[7]);
  v[8] = ActPickInt(line, "SHADE_T", g_act_v[8]);
  for (int i = 0; i < 9; ++i) {
    g_act_v[i] = v[i];
  }
  g_act_valid = true;
}

// loop 里每轮调用：把 LinkCommitLine() 记下的标志位翻译成屏幕提示。
// ★ 只做判断和显示，不再解析 g_link_last_line —— 那是上一版的坑。
static void ActAckPoll() {
  if (!g_act_ack_pending) {
    return;
  }

  // ---- ① 成功（最先判：避免刚好卡在超时边界上被判成失败）----
  // 只认"发出指令之后"收到的 ACT_OK，防止把上电那条旧回执当成本次结果。
  if (g_act_got_ok && static_cast<int32_t>(g_act_got_ok_ms - g_act_ack_sent_ms) >= 0) {
    g_act_ack_pending = false;
    g_act_got_ok = false;
    char body[160] = {0};
    ActFormatState(g_act_v, body, sizeof(body));
    char msg[224] = {0};
    snprintf(msg, sizeof(msg), "终端已执行: %s", body);
    CropNotify(msg);
    printf("[Act ] 终端已执行: %s\n", body);
    return;
  }

  // ---- ② 终端收到, 但一个键都没认出来 ----
  if (g_act_got_err) {
    g_act_ack_pending = false;
    g_act_got_err = false;
    CropNotify("终端没认出这条执行器指令, 请换个说法重试");
    printf("[Act ] 终端回 ACT_ERR\n");
    return;
  }

  // ---- ③ 终端回 "ACK:ACT,..." = 它不认识 ACT, 确认是旧固件 ----
  if (g_act_got_ack) {
    g_act_ack_pending = false;
    g_act_got_ack = false;
    CropNotify("终端不认识这条指令, 它是旧固件, 请重烧 B 板");
    printf("[Act ] 终端回 ACK 兜底 -> 确认终端是旧固件\n");
    return;
  }

  // ---- ④ 超时 ----
  if (millis() - g_act_ack_sent_ms > kActAckTimeoutMs) {
    g_act_ack_pending = false;
    // ★★ 2026-10-20 改：超时提示必须能区分两种【完全不同的故障】，否则用户只能瞎试。
    //   判据只有一个：最近 kLinkAliveWindowMs 内有没有收到终端的任何一行
    //   （终端每 2 秒主动发一帧 ALL，所以"通"的话这个时间戳永远是新鲜的）。
    //
    //   · 链路活着 + 仍然没回执 → B 板【收到任何一行都会回点什么】（我们的两个终端固件
    //     都有 ACK 兜底），所以它一声不吭只可能意味着**指令根本没到 B 板** ——
    //     即【主控 → 终端】这个方向断了（主控39→终端40 那根线）。
    //     B → 主控 是通的（数据能同步），所以光看"数据有同步"会误判成"B 板是旧固件"。
    //   · 链路是死的 → 两个方向都不通，去查共地 / 供电 / 两根线是否交叉。
    const bool recently_open =
        g_link_has_line && (millis() - g_link_last_rx_ms) < kLinkAliveWindowMs;
    char buf[256] = {0};
    if (recently_open) {
      // ★ 屏幕只有 240 宽，提示必须短；详细推理放在 printf 里给串口看。
      snprintf(buf, sizeof(buf),
               "终端在发数据但不回执: 主控39到终端40这根线不通");
      printf("[Act ] ACK 超时: B->A 通(最近 %lums 有数据), 但 A->B 无任何回执。"
             "B 板对任何一行都会回一条, 所以判定 A->B 方向断了(查主控39->终端40 + 共地)\n",
             static_cast<unsigned long>(millis() - g_link_last_rx_ms));
    } else {
      snprintf(buf, sizeof(buf),
               "5秒没收到终端任何数据: 查39/40交叉、共地、终端供电");
      printf("[Act ] ACK 超时: 链路双向都不通(已 %lums 无任何数据)\n",
             static_cast<unsigned long>(g_link_has_line
                                            ? (millis() - g_link_last_rx_ms)
                                            : 0xFFFFFFFFul));
    }
    CropNotify(buf);
  }
}

// ================== 终端在线检测(2026-10-23 新增) ==================
// 需求: 串口线(模拟网线接口)插入时, 主控播报"终端设备已连接"。
// 原理: 串口线本身没有"插入信号", 但 B 板每 2 秒主动推一行 ENV,... ——
//   链路上自带心跳, 拔线/断电后心跳必然停。所以:
//   · 离线 → 在线: 短时间内收到【连续两帧】才判定上线并播报
//     (防插线瞬间接触抖动产生的垃圾单帧误报);
//   · 在线 → 离线: 超过 kLinkOfflineMs 无任何帧, 静默判离线。
//     ★ 用户只要求"插入时播报", 断开不播报(演示中频繁报断开反而吵), 只打日志。
static bool g_link_online = false;          // 当前是否判定终端在线
static uint32_t g_link_first_frame_ms = 0;  // 离线期间收到的第一帧时刻(0 = 没有未决的第一帧)
static constexpr uint32_t kLinkOnlineConfirmMs = 8000;  // 两帧间隔超过此值 → 放弃第一帧重计
static constexpr uint32_t kLinkOfflineMs = 6000;        // 在线时超过此值无帧 → 判离线(约 3 个心跳周期)

// 终端上线播报。独立于 MonSpeakNow: 不借用监测的暂扣队列(监测没开时没人补发,
// 混用还会互相覆盖)。AI 正在说话(kSpeaking)时放弃本次播报 —— 插线是人工动作,
// 丢一句不值得为它冒"插话打乱服务端会话"的风险(那坑踩过, 见 MonSpeakNow 注释)。
static void LinkAnnounceOnline() {
  const char* text = "终端设备已连接";
  if (g_display != nullptr) {
    g_display->SetChatMessage(Display::Role::kSystem, text);
  }
  printf("[Link] 播报: %s\n", text);
  if (g_chat_state == ai_vox::ChatState::kSpeaking) {
    printf("[Link] AI 说话中, 放弃本次上线播报\n");
    return;
  }
  // ★ 必须带 session_id(2026-10-23 实测踩坑, 详见 MonSpeakNow 内注释)。
  const std::string link_sid = ai_vox::Engine::GetInstance().GetSessionId();
  char json[384] = {0};
  const int n = snprintf(json, sizeof(json),
                         "{\"session_id\":\"%s\",\"type\":\"listen\",\"state\":\"detect\",\"text\":"
                         "\"系统事件: %s。请简短确认。\"}",
                         link_sid.c_str(), text);
  if (n > 0 && static_cast<size_t>(n) < sizeof(json)) {
    ai_vox::Engine::GetInstance().SendText(json);
  }
}

// 在 loop() 里 SerialLinkPoll 之后每圈调用: 推进在线/离线状态机(纯时间比较, 零阻塞)。
static void LinkPresenceTick() {
  const uint32_t now = millis();
  if (g_link_online) {
    if (g_link_last_rx_ms != 0 && now - g_link_last_rx_ms > kLinkOfflineMs) {
      g_link_online = false;
      g_link_first_frame_ms = 0;
      printf("[Link] 终端离线(>%lus 无任何数据帧)\n",
             static_cast<unsigned long>(kLinkOfflineMs / 1000));
    }
    return;
  }
  // 离线中: 若第一帧之后一直没等到第二帧, 超时放弃重来
  if (g_link_first_frame_ms != 0 && now - g_link_first_frame_ms > kLinkOnlineConfirmMs) {
    g_link_first_frame_ms = 0;
    printf("[Link] 上线确认超时(只来过一帧), 重新等待\n");
  }
}

// 把收到的一行存入缓存并打印日志。how 说明是"怎么判定这一帧结束的", 便于排障。
static void LinkCommitLine(const char* line, const char* how) {
  if (line == nullptr || line[0] == '\0') {
    return;
  }
  snprintf(g_link_last_line, sizeof(g_link_last_line), "%s", line);
  g_link_has_line = true;
  g_link_last_rx_ms = millis();

  // ---- 终端在线检测挂钩: 离线期间连续第二帧到达 → 判上线并播报 ----
  if (!g_link_online) {
    if (g_link_first_frame_ms == 0) {
      g_link_first_frame_ms = g_link_last_rx_ms;  // 第一帧: 开始计时, 还不算上线
    } else {
      g_link_online = true;                       // 短时间内第二帧 → 真上线
      g_link_first_frame_ms = 0;
      LinkAnnounceOnline();
    }
  }

  // ★★ 执行器回执在这里立刻分类（见上面 g_act_got_* 的注释）。
  //   只做前缀比较 + 一次解析，ALL/ENV 这些高频帧完全不进分支，开销可忽略。
  if (strncmp(line, "ACT_OK", 6) == 0) {
    ActParseOkLine(line);
    g_act_got_ok    = true;
    g_act_got_ok_ms = millis();
  } else if (strncmp(line, "ACT_ERR", 7) == 0) {
    g_act_got_err = true;
  } else if (strncmp(line, "ACK:", 4) == 0 && strstr(line, "ACT") != nullptr) {
    // 终端不认识这条指令, 落到兜底 ACK → 说明它跑的是没有 ACT 分支的旧固件
    g_act_got_ack = true;
  }

  printf("[Link] RX << %s  (frame end: %s)\n", g_link_last_line, how);
}

// 主循环轮询: 非阻塞地收字节, 按 '\n' 组帧; 无换行则靠空闲超时提交。
// ★ 全程不阻塞、不 delay, 不会拖慢语音响应(这是把它放 loop 的前提)。
void SerialLinkPoll() {
  const uint32_t now = millis();

  while (g_link_serial.available() > 0) {
    const int c = g_link_serial.read();
    if (c < 0) {
      break;
    }
    if (c == '\n' || c == '\r') {
      if (g_link_rx_len > 0) {
        g_link_rx_buf[g_link_rx_len] = '\0';
        LinkCommitLine(g_link_rx_buf, "newline");
        g_link_rx_len = 0;
        g_link_rx_buf[0] = '\0';
      }
      continue;
    }
    if (g_link_rx_len == 0) {
      g_link_rx_first_byte_ms = now;
    }
    if (g_link_rx_len < sizeof(g_link_rx_buf) - 1) {
      g_link_rx_buf[g_link_rx_len++] = static_cast<char>(c);
    } else {
      // 缓冲溢出: 把已收到的部分当一帧交出去再重置, 避免"死等换行"把后续数据一起吞掉
      g_link_rx_buf[g_link_rx_len] = '\0';
      LinkCommitLine(g_link_rx_buf, "overflow-truncated");
      g_link_rx_len = 0;
      g_link_rx_buf[0] = '\0';
    }
  }

  // 空闲超时提交: 对端发的内容不带换行时也能收到(短帧/单次发送场景)
  if (g_link_rx_len > 0 && (now - g_link_rx_first_byte_ms) >= kLinkFrameIdleMs) {
    g_link_rx_buf[g_link_rx_len] = '\0';
    LinkCommitLine(g_link_rx_buf, "idle-timeout");
    g_link_rx_len = 0;
    g_link_rx_buf[0] = '\0';
  }
}

// ================== 终端灯握手 (主控 → 终端 → 主控) ==================
// 需求: 对主控说"打开终端灯" →
//   ① 主控发 LED_ON 给终端;
//   ② 终端点亮自己的 RGB 白色灯, 然后回 LED_ON_OK;
//   ③ 主控【确实收到 LED_ON_OK 之后】才点亮自己第 1 颗 WS2812。
//
// ★★ 为什么必须做成"等回执"而不是"发完就点灯":
//   发完就点灯的话, 即使串口线拔了、终端没通电, 主控照样会亮 —— 那就只是
//   自己放个动画, 完全不能证明通讯打通。做成等回执以后:
//     · 线接了、终端活着 → 主控亮灯, 是"真握手成功"的物理证据;
//     · 线没接 / 终端没烧 / 板子坏了 → 主控不亮灯 + 语音回复"终端没有响应"。
//   所以【主控第 1 颗灯亮 = 双向通讯真的通了】, 这就是最直观的通断判据。
//
// ★ 为什么用状态机 + 超时不阻塞在回调里:
//   MCP 工具回调里不能 delay(会卡住音频与语音事件), 所以工具只负责"发指令 + 记下
//   开始等待的时刻", 真正的超时判定放在 loop() 里非阻塞推进。

enum class TerminalLedCmd { kNone, kOn, kOff };

struct TerminalHandshake {
  bool pending;             // 是否正在等回执
  TerminalLedCmd cmd;       // 等的是哪条指令的回执
  uint32_t sent_ms;         // 发出指令的时刻(用于超时)
  bool ready;               // 有一次【已出结果】的握手可供 AI 读取
  bool ok;                  // 那次握手是否成功
  // ★ 缓冲区给到 256: 中文 UTF-8 每字 3 字节, 一句 40 字的提示就 120 字节;
  //   128 字节装不下, 会被 -Werror=format-truncation 判为编译错误(已踩过)。
  char result[256];
};

TerminalHandshake g_terminal_hs = {false, TerminalLedCmd::kNone, 0, false, false, {0}};

// 把"握手失败/成功"的结论写进 g_terminal_hs.result, 供 AI 读取
static void TerminalSetResult(bool ok, const char* text) {
  g_terminal_hs.ready = true;
  g_terminal_hs.ok = ok;
  snprintf(g_terminal_hs.result, sizeof(g_terminal_hs.result), "%s", text != nullptr ? text : "");
}

// 把 g_led_pick_mask 的某一位设成 1/0(不动其它位), 并立即重画一帧。
// ★ 这里绕开 self.led.pick 的"替换/累加"语义, 直接用位操作:
//   握手点灯只应该点亮【第 1 颗】, 不应该把用户之前点亮的其它灯一起关掉。
static void LedSetBit(uint8_t bit_index, bool on) {
  if (bit_index >= kWs2812Count) {
    return;
  }
  const uint8_t bit = static_cast<uint8_t>(1u << bit_index);
  if (on) {
    g_led_pick_mask |= bit;
  } else {
    g_led_pick_mask &= static_cast<uint8_t>(~bit);
  }
  g_led_mode = LedMode::kCustom;  // 切到"指定灯"模式, 让 LedPickBreathUpdate 接管刷新
  if (kCustomBreath) {
    LedShowPickMaskAt(kBreathMinLevel);  // 呼吸模式: 先给最低亮度, 避免闪一下满亮
  } else {
    LedShowPickMask();
  }
}

// 发起一次握手: 发指令 + 进入等待状态。返回给 AI 的即时回执文本。
static void TerminalStartHandshake(TerminalLedCmd cmd) {
  const bool is_on = (cmd == TerminalLedCmd::kOn);
  LinkSendLine(is_on ? kCmdTerminalLedOn : kCmdTerminalLedOff);
  g_terminal_hs.pending = true;
  g_terminal_hs.cmd = cmd;
  g_terminal_hs.sent_ms = millis();
  g_terminal_hs.ready = false;  // 新一轮开始, 旧结果作废
  printf("[handshake] start: cmd=%s, waiting ack (timeout %ums)\n",
         is_on ? kCmdTerminalLedOn : kCmdTerminalLedOff, (unsigned)kTerminalAckTimeoutMs);
}

// 收到回执后的处理。由 loop 里的 TerminalHandshakePoll 推进。
static void TerminalHandshakePoll() {
  // ---- 1. 检查是否收到了匹配的回执 ----
  if (g_terminal_hs.pending && g_link_has_line) {
    // 用"已处理的收帧时刻"去重: 同一条回执不能被反复处理两次。
    // ★ 不用哨兵值 0 而用单独的 bool: 因为 g_link_last_rx_ms 真有可能等于 0
    //   (上电后 millis() 还是 0 的那一毫秒就收到数据), 用 0 当哨兵会漏掉这一帧。
    static uint32_t s_handled_rx_ms = 0;
    static bool s_handled_valid = false;
    if (!s_handled_valid || g_link_last_rx_ms != s_handled_rx_ms) {
      s_handled_rx_ms = g_link_last_rx_ms;
      s_handled_valid = true;

      const bool want_on = (g_terminal_hs.cmd == TerminalLedCmd::kOn);
      const char* expect = want_on ? kAckTerminalLedOn : kAckTerminalLedOff;
      if (strstr(g_link_last_line, expect) != nullptr) {
        g_terminal_hs.pending = false;
        // ★ 握手成功 → 点亮/熄灭主控第 1 颗 WS2812
        LedSetBit(kHandshakeLedIndex, want_on);
        char buf[256] = {0};
        snprintf(buf, sizeof(buf),
                 "终端已确认:%s。主控第%d颗灯已%s。",
                 want_on ? "白色灯已点亮" : "灯已关闭",
                 kHandshakeLedIndex + 1,
                 want_on ? "点亮" : "熄灭");
        TerminalSetResult(true, buf);
        printf("[handshake] OK: %s\n", buf);
      }
    }
  }

  // ---- 2. 超时判定 ----
  if (g_terminal_hs.pending && (millis() - g_terminal_hs.sent_ms) >= kTerminalAckTimeoutMs) {
    g_terminal_hs.pending = false;
    char buf[256] = {0};
    snprintf(buf, sizeof(buf),
             "终端没有响应(等了%u毫秒无回执)。请检查: 串口线是否交叉接好(主控39→终端40, "
             "主控40→终端39)、是否共地、终端是否通电并烧录固件。",
             (unsigned)kTerminalAckTimeoutMs);
    TerminalSetResult(false, buf);
    printf("[handshake] TIMEOUT\n");
  }
}

// ================== 终端 OLED 显示数字握手 (主控 → 终端) ==================
// 需求: 对主控说"在终端 OLED 上显示 123" →
//   ① 主控发 "OLED:123";
//   ② 终端居中大字显示 123, 然后回 "OLED_OK:123";
//   ③ 主控比对回执里的数值与发出去的一致, 才算成功。
//
// ★ 为什么回执要带数值: 终端可能显示失败(比如被别的指令覆盖),
//   回执带上实际显示的数值, 主控才能确认"屏幕上现在真的是这个数"。
struct TerminalOled {
  bool pending;        // 是否在等回执
  int32_t value;       // 本次要显示的值
  uint32_t sent_ms;
  bool ready;          // 有一次已出结果的操作
  bool ok;
  char result[256];    // ★ 中文 UTF-8 每字 3 字节, 提示文案缓冲区一律 256
};
TerminalOled g_terminal_oled = {false, 0, 0, false, false, {0}};

static void TerminalOledSetResult(bool ok, const char* text) {
  g_terminal_oled.ready = true;
  g_terminal_oled.ok = ok;
  snprintf(g_terminal_oled.result, sizeof(g_terminal_oled.result), "%s",
           text != nullptr ? text : "");
}

// 发起一次显示: 发指令 + 进入等待。不阻塞、不 delay。
static void TerminalOledStart(int32_t value) {
  char cmd[48] = {0};
  snprintf(cmd, sizeof(cmd), "%s%ld", kCmdTerminalOledPrefix, static_cast<long>(value));
  LinkSendLine(cmd);
  g_terminal_oled.pending = true;
  g_terminal_oled.value = value;
  g_terminal_oled.sent_ms = millis();
  g_terminal_oled.ready = false;
  printf("[oled] start: %s, waiting ack (timeout %ums)\n", cmd,
         (unsigned)kTerminalAckTimeoutMs);
}

static void TerminalOledPoll() {
  if (!g_terminal_oled.pending || !g_link_has_line) {
    return;
  }
  static uint32_t s_oled_handled_ms = 0;
  static bool s_oled_handled_valid = false;
  if (s_oled_handled_valid && g_link_last_rx_ms == s_oled_handled_ms) {
    return;
  }
  s_oled_handled_ms = g_link_last_rx_ms;
  s_oled_handled_valid = true;

  if (strncmp(g_link_last_line, kAckTerminalOledPrefix,
              strlen(kAckTerminalOledPrefix)) != 0) {
    return;
  }

  // 取回执里的数值, 必须与发出去的一致
  const long echoed = strtol(g_link_last_line + strlen(kAckTerminalOledPrefix), nullptr, 10);
  if (echoed != static_cast<long>(g_terminal_oled.value)) {
    printf("[oled] 回执数值 %ld 与指令 %ld 不符, 忽略\n", echoed,
           static_cast<long>(g_terminal_oled.value));
    return;
  }

  g_terminal_oled.pending = false;
  char buf[256] = {0};
  snprintf(buf, sizeof(buf), "终端 OLED 已显示 %ld(终端已回执确认)。",
           static_cast<long>(g_terminal_oled.value));
  TerminalOledSetResult(true, buf);
  printf("[oled] OK: %s\n", buf);
}

// 超时判定单独抽出来, 由 loop 里统一调用 —— 与回执检测分开,
// 避免"没收到回执就永远不判超时"这种漏网。
static void TerminalOledTimeoutCheck() {
  if (!g_terminal_oled.pending) {
    return;
  }
  if ((millis() - g_terminal_oled.sent_ms) < kTerminalAckTimeoutMs) {
    return;
  }
  g_terminal_oled.pending = false;
  char buf[256] = {0};
  snprintf(buf, sizeof(buf),
           "终端没有响应(等了%u毫秒无回执), OLED 可能没显示。请检查: 串口线是否交叉接好"
           "(主控39→终端40, 主控40→终端39)、是否共地、终端是否通电并烧录了带 OLED 的固件。",
           (unsigned)kTerminalAckTimeoutMs);
  TerminalOledSetResult(false, buf);
  printf("[oled] TIMEOUT\n");
}

// ================== 三路舵机 (IO12/13/14, LEDC 50Hz) ==================
// 标准 RC 舵机: 50Hz PWM, 脉宽 500us~2500us 对应 0~180 度。
// core 3.x API: ledcAttach(pin, freq, resolution) + ledcWrite(pin, duty)。
// 14bit 分辨率下 20ms/16384 ≈ 1.22us/tick, 500us->duty 410, 2500us->duty 2048。
// ★ 自检动作是"长动作"(好几秒), 绝不能在 MCP 回调里 delay(会卡语音/屏保/串口),
//   一律用非阻塞状态机放 loop 里轮询。

constexpr gpio_num_t kServoPins[3] = {GPIO_NUM_12, GPIO_NUM_13, GPIO_NUM_14};
constexpr uint32_t kServoFreqHz = 50;
constexpr uint8_t kServoResBits = 14;

// ================== 摄像头识别 -> 多执行器联动规则 (数据结构, 前置定义) ==================
// 一条动作: 舵机 / 电机1 / 电机2 / RGB灯 四选一
struct VisionAct {
  int type;     // 0=舵机, 1=电机1(PWM), 2=电机2(开关), 3=RGB灯
  int index;    // type=0: 舵机编号 1~3; type=3: 灯号 1~8(0=全部8颗)
  int angle;    // type=0: 角度 0~180
  int dir;      // type=1/2: 0=停, 1=正转, 2=反转, 3=刹车(仅电机2)
  int speed;    // type=1: PWM 0~255, -1=用默认150
  int hue;      // type=3: 色相 0~255
  int sat;      // type=3: 饱和度 0~255(0=白)
  bool valid;   // 是否有效
};

struct VisionLinkRule {
  char keyword[64];           // 触发关键词(可用 '|' 分隔多个, 任一命中即触发)
  char speak[64];             // 命中后的播报语(空则用默认)
  VisionAct acts[8];          // 满足条件后同时执行的动作列表(最多8个)
  int act_count;              // 有效动作数
  bool active;                // 是否占用
};
constexpr int kMaxVisionLinks = 6;
VisionLinkRule g_vision_links[kMaxVisionLinks];
static int g_vision_link_count = 0;

struct ServoCtl {
  int angle;         // 当前角度(0~180)
  bool testing;      // 自检进行中
  int phase;         // 0..4=waypoints{45,90,135,180,0}各停700ms; 5=平滑扫描0->180; 6=完成
  uint32_t next_ms;  // 下一次动作的时刻
};
ServoCtl g_servos[3];

// 角度 -> LEDC duty
static inline uint32_t ServoAngleToDuty(int angle) {
  if (angle < 0) angle = 0;
  if (angle > 180) angle = 180;
  const uint32_t us = 500 + (static_cast<uint32_t>(angle) * 2000) / 180;  // 500~2500us
  return (us * 16384) / 20000;
}

static void ServoWriteAngle(int idx, int angle) {
  if (idx < 0 || idx >= 3) return;
  if (angle < 0) angle = 0;
  if (angle > 180) angle = 180;
  ledcWrite(static_cast<uint8_t>(kServoPins[idx]), ServoAngleToDuty(angle));
  g_servos[idx].angle = angle;
}

void InitServos() {
  for (int i = 0; i < 3; i++) {
    ledcAttach(static_cast<uint8_t>(kServoPins[i]), kServoFreqHz, kServoResBits);
    g_servos[i].testing = false;
    g_servos[i].phase = 0;
    g_servos[i].next_ms = 0;
    ServoWriteAngle(i, 0);  // 上电归 0 度
  }
  // 联动规则上电清零(不写 Flash, 重启即失效)
  g_vision_link_count = 0;
  for (int i = 0; i < kMaxVisionLinks; i++) {
    g_vision_links[i].active = false;
    g_vision_links[i].keyword[0] = '\0';
    g_vision_links[i].speak[0] = '\0';
    g_vision_links[i].act_count = 0;
    for (int a = 0; a < 8; a++) {
      g_vision_links[i].acts[a].valid = false;
    }
  }
  printf("[Servo] 3 servos ready on IO12/13/14 (50Hz, 500-2500us -> 0-180deg)\n");
}

// 启动某一舵机的自检序列(异步, 立即返回):
// 先回 0 度 -> 45 -> 90 -> 135 -> 180 -> 回 0, 每步停 700ms;
// 然后从 0 度平滑扫到 180 度(每 20ms 走 1 度, 约 3.6s), 结束停在 180 度。
static void ServoStartSelftest(int idx) {
  ServoWriteAngle(idx, 0);           // 先初始化到 0 度
  g_servos[idx].testing = true;
  g_servos[idx].phase = 0;
  g_servos[idx].next_ms = millis() + 700;
}

// 非阻塞自检状态机, loop 里每圈调用
void ServoPoll() {
  const uint32_t now = millis();
  static constexpr int kWaypoints[5] = {45, 90, 135, 180, 0};
  static constexpr uint32_t kStepHoldMs = 700;
  static constexpr uint32_t kSweepStepMs = 20;  // 平滑扫描步进间隔

  for (int i = 0; i < 3; i++) {
    ServoCtl& s = g_servos[i];
    if (!s.testing) continue;
    if (static_cast<int32_t>(now - s.next_ms) < 0) continue;

    if (s.phase < 5) {
      // 阶段一: 逐点 45/90/135/180/0, 每步停 700ms
      ServoWriteAngle(i, kWaypoints[s.phase]);
      s.phase++;
      s.next_ms = now + kStepHoldMs;
      if (s.phase == 5) {
        // 五个点走完, 此时舵机在 0 度; 直接开始平滑扫描
        s.angle = 0;
        s.next_ms = now + kStepHoldMs;
      }
    } else if (s.phase == 5) {
      // 阶段二: 0 -> 180 平滑扫描, 每 20ms +1 度
      if (s.angle < 180) {
        ServoWriteAngle(i, s.angle + 1);
        s.next_ms = now + kSweepStepMs;
      } else {
        s.phase = 6;
        s.testing = false;  // 自检完成, 停在 180 度
        printf("[Servo] servo%d selftest done (rest at 180deg)\n", i + 1);
      }
    }
  }
}

// 电机方向枚举(前置到这里, 供联动执行器使用; 实现在下方电机区)
enum class MotorDir {
  kStop,   // 停止
  kFwd,    // 正转
  kRev,    // 反转
  kBrake,  // 刹车(仅电机2 有意义)
};
// 电机控制函数前置声明(实现在下方电机区)
const char* MotorDirName(MotorDir dir);
void Motor1Set(MotorDir dir, int speed);
void Motor2SetDirection(MotorDir dir);
// 电机状态全局变量前置声明(实现在下方电机区)
extern MotorDir g_motor1_dir;
extern int g_motor1_speed;
extern MotorDir g_motor2_dir;

// ================== 摄像头识别 -> 多执行器联动 (语音可配) ==================
// 用户需求(2026-11): 说"识别到风扇的时候, 舵机1转到33度", 下次摄像头识别结果
// 里出现"风扇"两个字, 就自动执行动作; 说"恢复/舵机恢复"则清空规则并回0度。
// 规则存内存, 断电丢失; 支持多条。
// ★ 支持"生长阶段条件判断": 关键词可用 '|' 分隔多个(如"结果|果实|结果期", 任一命中即触发)。
// ★ 支持"多执行器同时动作": 一条规则可同时控制 舵机1/2/3 + 电机1 + 电机2。
//   actions 字符串格式(逗号分隔多项, 冒号分隔字段):
//     servo1:90            -> 舵机1 转到 90 度
//     servo2:45            -> 舵机2 转到 45 度
//     servo3:135           -> 舵机3 转到 135 度
//     motor1:forward:150   -> 电机1 正转, PWM 速度 150(速度可省略, 默认150)
//     motor1:reverse:200   -> 电机1 反转, PWM 200
//     motor1:stop          -> 电机1 停止
//     motor2:forward       -> 电机2 正转(开关, 无速度)
//     motor2:reverse       -> 电机2 反转
//     motor2:stop          -> 电机2 停止

// 清空所有联动规则, 并让所有舵机回 0 度、电机全停
static void VisionLinkReset() {
  g_vision_link_count = 0;
  for (int i = 0; i < kMaxVisionLinks; i++) {
    g_vision_links[i].active = false;
    g_vision_links[i].keyword[0] = '\0';
    g_vision_links[i].speak[0] = '\0';
    g_vision_links[i].act_count = 0;
    for (int a = 0; a < 8; a++) {
      g_vision_links[i].acts[a].valid = false;
    }
  }
  for (int i = 0; i < 3; i++) {
    ServoWriteAngle(i, 0);
  }
  printf("[VisionLink] rules cleared, all actuators reset\n");
}

// 生长阶段 -> 特征词列表(用 '|' 分隔)。用户口述"开花期"等阶段词时,
// 摄像头识别文字只要出现任一特征词就算命中该阶段, 不要求精确出现"开花期"三个字。
// ★ 特征词可【口述动态改】(整句替换): 内置默认值只是初始词库, 用户随时用
//   self.vision.stage 工具覆盖。阶段名固定为下面 6 个。
struct StageMap {
  const char* stage;       // 阶段词(固定, 精确匹配)
  char features[96];       // 特征词列表('|' 分隔), 可运行时覆盖
};
static StageMap g_stage_maps[] = {
  {"开花期", "开花|花朵|花苞|花开|花瓣|开了花|花开了"},
  {"结果期", "结果|果实|瓜果|结了果|结果子|结瓜|长果"},
  {"幼苗期", "幼苗|小苗|嫩芽|发芽|刚发芽|苗期|秧苗"},
  {"成长期", "成长|生长期|枝叶|长叶子|茂盛|抽枝"},
  {"成熟期", "成熟|黄了|变红|可以采摘|采摘期"},
  {"枯萎",   "枯萎|枯黄|蔫了|干枯|落叶|凋谢"},
};
static constexpr int kStageCount = sizeof(g_stage_maps) / sizeof(g_stage_maps[0]);

// 按阶段名查找词库下标, 找不到返回 -1
static int VisionFindStage(const char* stage) {
  if (stage == nullptr) return -1;
  for (int i = 0; i < kStageCount; i++) {
    if (strcmp(stage, g_stage_maps[i].stage) == 0) return i;
  }
  return -1;
}

// 口述整句覆盖某阶段的特征词列表(用 '|' 分隔)。返回是否成功。
static bool VisionSetStageFeatures(const char* stage, const char* features) {
  const int idx = VisionFindStage(stage);
  if (idx < 0 || features == nullptr || features[0] == '\0') return false;
  snprintf(g_stage_maps[idx].features, sizeof(g_stage_maps[idx].features), "%s", features);
  printf("[VisionLink] stage '%s' features = '%s'\n", stage, g_stage_maps[idx].features);
  return true;
}

// 判断 keyword 字段(可能含多个 '|' 分隔词)是否命中 text, 任一命中即 true。
// ★ 支持阶段归类: 若某词是阶段词(如"开花期"), 自动展开成特征词列表去匹配。
static bool VisionLinkMatch(const char* keywords, const char* text) {
  if (keywords == nullptr || text == nullptr) return false;
  char buf[128];
  snprintf(buf, sizeof(buf), "%s", keywords);
  char* saveptr = nullptr;
  for (char* tok = strtok_r(buf, "|", &saveptr); tok != nullptr;
       tok = strtok_r(nullptr, "|", &saveptr)) {
    while (*tok == ' ' || *tok == '\t') tok++;
    if (*tok == '\0') continue;

    // 先看这个 token 是不是阶段词, 是就用特征词列表去匹配
    const int sidx = VisionFindStage(tok);
    if (sidx >= 0) {
      char fbuf[128];
      snprintf(fbuf, sizeof(fbuf), "%s", g_stage_maps[sidx].features);
      char* fsave = nullptr;
      bool matched = false;
      for (char* f = strtok_r(fbuf, "|", &fsave); f != nullptr;
           f = strtok_r(nullptr, "|", &fsave)) {
        if (strstr(text, f) != nullptr) {
          matched = true;
          printf("[VisionLink] stage '%s' hit feature '%s'\n", tok, f);
          break;
        }
      }
      if (matched) return true;
      continue;  // 阶段词没命中, 不再做普通子串匹配(阶段词本身不会出现在识别文字里)
    }

    // 不是阶段词, 普通子串匹配
    if (strstr(text, tok) != nullptr) return true;
  }
  return false;
}

// 解析电机方向字符串 -> dir(0停 1正 2反 3刹车)。返回是否识别成功。
static bool VisionParseDir(const char* s, int* dir_out) {
  if (s == nullptr) return false;
  if (strcmp(s, "forward") == 0 || strcmp(s, "fwd") == 0 ||
      strcmp(s, "正转") == 0 || strcmp(s, "前进") == 0) {
    *dir_out = 1; return true;
  }
  if (strcmp(s, "reverse") == 0 || strcmp(s, "rev") == 0 ||
      strcmp(s, "反转") == 0 || strcmp(s, "后退") == 0) {
    *dir_out = 2; return true;
  }
  if (strcmp(s, "stop") == 0 || strcmp(s, "停") == 0 || strcmp(s, "停止") == 0) {
    *dir_out = 0; return true;
  }
  if (strcmp(s, "brake") == 0 || strcmp(s, "刹车") == 0) {
    *dir_out = 3; return true;
  }
  return false;
}

// 解析 actions 字符串(逗号分隔多项), 填进 rule.acts[0..7]。返回解析出的动作数。
static int VisionParseActions(VisionLinkRule& rule, const char* actions) {
  rule.act_count = 0;
  if (actions == nullptr || actions[0] == '\0') return 0;

  char buf[192];
  snprintf(buf, sizeof(buf), "%s", actions);
  char* saveptr = nullptr;
  for (char* item = strtok_r(buf, ",", &saveptr); item != nullptr;
       item = strtok_r(nullptr, ",", &saveptr)) {
    while (*item == ' ' || *item == '\t') item++;
    if (*item == '\0') continue;
    if (rule.act_count >= 8) break;

    // 拆冒号字段
    char fields[4][32];
    int nf = 0;
    char fbuf[64];
    snprintf(fbuf, sizeof(fbuf), "%s", item);
    char* fsave = nullptr;
    for (char* f = strtok_r(fbuf, ":", &fsave); f != nullptr && nf < 4;
         f = strtok_r(nullptr, ":", &fsave)) {
      snprintf(fields[nf], sizeof(fields[nf]), "%s", f);
      nf++;
    }
    if (nf == 0) continue;

    VisionAct act;
    memset(&act, 0, sizeof(act));
    act.valid = false;
    act.speed = -1;

    if (strcmp(fields[0], "servo1") == 0 || strcmp(fields[0], "servo2") == 0 ||
        strcmp(fields[0], "servo3") == 0) {
      // 舵机: servoN:角度
      if (nf < 2) continue;
      const int idx = fields[0][5] - '1';  // 'servo1' 的第5个字符(下标5)是数字
      const int angle = atoi(fields[1]);
      if (idx < 0 || idx > 2) continue;
      if (angle < 0 || angle > 180) continue;
      act.type = 0;
      act.index = idx + 1;
      act.angle = angle;
      act.valid = true;
    } else if (strcmp(fields[0], "motor1") == 0) {
      // 电机1: motor1:方向[:速度]
      if (nf < 2) continue;
      int dir = 0;
      if (!VisionParseDir(fields[1], &dir)) continue;
      int speed = -1;
      if (nf >= 3) {
        speed = atoi(fields[2]);
        if (speed < 0) speed = 0;
        if (speed > 255) speed = 255;
      }
      act.type = 1;
      act.dir = dir;
      act.speed = speed;
      act.valid = true;
    } else if (strcmp(fields[0], "motor2") == 0) {
      // 电机2: motor2:方向
      if (nf < 2) continue;
      int dir = 0;
      if (!VisionParseDir(fields[1], &dir)) continue;
      act.type = 2;
      act.dir = dir;
      act.valid = true;
    } else if (strncmp(fields[0], "led", 3) == 0) {
      // RGB 灯: led:颜色(全亮) / ledN:颜色(第N颗, N=1~8)
      // 例: led:red / led1:红 / led3:green
      if (nf < 2) continue;
      int led_index = 0;  // 0 = 全部8颗
      if (fields[0][3] != '\0') {  // 带灯号, 如 led1 / led8
        led_index = atoi(fields[0] + 3);
        if (led_index < 1 || led_index > 8) continue;
      }
      uint8_t hue = 0, sat = 0;
      if (!LedColorFromName(fields[1], hue, sat)) continue;  // 颜色不认识就跳过
      act.type = 3;
      act.index = led_index;
      act.hue = hue;
      act.sat = sat;
      act.valid = true;
    } else {
      continue;  // 未知动作类型, 跳过
    }

    if (act.valid) {
      rule.acts[rule.act_count] = act;
      rule.act_count++;
    }
  }
  return rule.act_count;
}

// 新增/覆盖一条联动规则(同关键词覆盖, 否则追加)
static bool VisionLinkAdd(const char* keyword, const char* actions, const char* speak) {
  if (keyword == nullptr || keyword[0] == '\0') return false;
  if (actions == nullptr || actions[0] == '\0') return false;

  // 先解析动作; 一个动作都解析不出来就失败
  VisionLinkRule tmp;
  memset(&tmp, 0, sizeof(tmp));
  tmp.active = false;
  tmp.act_count = 0;
  if (VisionParseActions(tmp, actions) <= 0) return false;

  // 已存在同名关键词 -> 覆盖
  for (int i = 0; i < g_vision_link_count; i++) {
    if (g_vision_links[i].active && strcmp(g_vision_links[i].keyword, keyword) == 0) {
      snprintf(g_vision_links[i].keyword, sizeof(g_vision_links[i].keyword), "%s", keyword);
      snprintf(g_vision_links[i].speak, sizeof(g_vision_links[i].speak), "%s",
               (speak != nullptr && speak[0] != '\0') ? speak : "");
      for (int a = 0; a < 8; a++) g_vision_links[i].acts[a] = tmp.acts[a];
      g_vision_links[i].act_count = tmp.act_count;
      printf("[VisionLink] update: '%s' -> %d acts (speak='%s')\n",
             keyword, tmp.act_count, g_vision_links[i].speak);
      return true;
    }
  }
  // 追加新规则
  if (g_vision_link_count >= kMaxVisionLinks) return false;
  VisionLinkRule& r = g_vision_links[g_vision_link_count];
  snprintf(r.keyword, sizeof(r.keyword), "%s", keyword);
  snprintf(r.speak, sizeof(r.speak), "%s", (speak != nullptr && speak[0] != '\0') ? speak : "");
  for (int a = 0; a < 8; a++) r.acts[a] = tmp.acts[a];
  r.act_count = tmp.act_count;
  r.active = true;
  g_vision_link_count++;
  printf("[VisionLink] add: '%s' -> %d acts (speak='%s', total %d)\n",
         keyword, r.act_count, r.speak, g_vision_link_count);
  return true;
}

// 执行一条规则里的全部动作
static void VisionExecActs(const VisionLinkRule& r) {
  for (int a = 0; a < r.act_count; a++) {
    const VisionAct& act = r.acts[a];
    if (!act.valid) continue;
    if (act.type == 0) {
      // 舵机
      ServoWriteAngle(act.index - 1, act.angle);
      printf("[VisionLink]   servo%d -> %ddeg (IO%d)\n",
             act.index, act.angle, static_cast<int>(kServoPins[act.index - 1]));
    } else if (act.type == 1) {
      // 电机1 (PWM)
      MotorDir dir = MotorDir::kStop;
      if (act.dir == 1) dir = MotorDir::kFwd;
      else if (act.dir == 2) dir = MotorDir::kRev;
      Motor1Set(dir, act.speed);  // speed<0 表示沿用默认
      printf("[VisionLink]   motor1 -> %s speed=%d\n", MotorDirName(g_motor1_dir), g_motor1_speed);
    } else if (act.type == 2) {
      // 电机2 (开关)
      MotorDir dir = MotorDir::kStop;
      if (act.dir == 1) dir = MotorDir::kFwd;
      else if (act.dir == 2) dir = MotorDir::kRev;
      else if (act.dir == 3) dir = MotorDir::kBrake;
      Motor2SetDirection(dir);
      printf("[VisionLink]   motor2 -> %s\n", MotorDirName(g_motor2_dir));
    } else if (act.type == 3) {
      // RGB 灯: index=0 全亮, 1~8 指定单颗; hue/sat 是颜色
      if (act.index == 0) {
        // 全部 8 颗同色
        g_led_pick_mask = 0xFF;
        for (uint32_t i = 0; i < kWs2812Count; ++i) {
          g_led_pick_hue[i] = static_cast<uint8_t>(act.hue);
          g_led_pick_sat[i] = static_cast<uint8_t>(act.sat);
        }
        LedShowPickMask();
        printf("[VisionLink]   led all -> hue=%d sat=%d\n", act.hue, act.sat);
      } else {
        // 指定单颗: 保留其它灯现状, 只改这一颗
        const int li = act.index - 1;
        if (li >= 0 && li < static_cast<int>(kWs2812Count)) {
          g_led_pick_mask |= (1u << li);
          g_led_pick_hue[li] = static_cast<uint8_t>(act.hue);
          g_led_pick_sat[li] = static_cast<uint8_t>(act.sat);
          LedShowPickMask();
          printf("[VisionLink]   led%d -> hue=%d sat=%d\n", act.index, act.hue, act.sat);
        }
      }
    }
  }
}

// 拿识别结果文字去匹配规则, 命中就执行动作并播报。返回命中的规则条数。
static int VisionLinkCheck(const char* text) {
  if (text == nullptr || text[0] == '\0') return 0;
  if (g_vision_link_count == 0) return 0;
  int hit = 0;
  for (int i = 0; i < g_vision_link_count; i++) {
    const VisionLinkRule& r = g_vision_links[i];
    if (!r.active || r.keyword[0] == '\0') continue;
    if (VisionLinkMatch(r.keyword, text)) {
      printf("[VisionLink] HIT '%s' -> %d acts\n", r.keyword, r.act_count);
      VisionExecActs(r);
      // 命中后播报: 有自定义播报语就用它, 否则给默认
      char speak_buf[128] = {0};
      if (r.speak[0] != '\0') {
        snprintf(speak_buf, sizeof(speak_buf), "%s", r.speak);
      } else {
        snprintf(speak_buf, sizeof(speak_buf), "识别到%s, 已执行联动动作", r.keyword);
      }
      MonSpeakNow(speak_buf);
      hit++;
    }
  }
  return hit;
}

// ================== 定时任务 (借鉴 mimiclaw cron_service 移植) ==================
// 用户需求(2026-11-08): 从 GitHub 项目 mimiclaw 借鉴移植"定时任务"能力。
// 支持两类定时规则, 全部口述配置:
//   · 周期重复: "每 30 分钟提醒我浇水" / "每 5 分钟舵机1转90度"
//   · 定点一次: "下午 3 点开灯" / "晚上 8 点舵机2转45度"
// 触发后 = 语音播报 + 执行器动作(复用 VisionExecActs 的舵机/电机/灯动作)。
// 内存存储(断电丢失), 不写 Flash; 由 loop() 每秒轮询一次。

struct CronJob {
  int kind;             // 0=周期(every), 1=定点(at)
  char name[48];        // 任务名(可选, 便于回显)
  char speak[96];       // 触发后的播报语
  char actions[192];    // 触发后的执行器动作(复用 VisionLink 动作格式)
  uint32_t interval_s;  // kind=0: 间隔秒
  int at_hour;          // kind=1: 触发小时(0~23)
  int at_min;           // kind=1: 触发分钟(0~59)
  int at_last_day;      // kind=1: 上次触发的"天"(防止同一分钟重复触发)
  uint32_t next_ms;     // kind=0: 下次触发时刻(millis)
  bool active;
};
constexpr int kMaxCronJobs = 16;
CronJob g_cron_jobs[kMaxCronJobs];
static int g_cron_count = 0;

// 清空所有定时任务
static void CronReset() {
  g_cron_count = 0;
  for (int i = 0; i < kMaxCronJobs; i++) {
    g_cron_jobs[i].active = false;
    g_cron_jobs[i].name[0] = '\0';
    g_cron_jobs[i].speak[0] = '\0';
    g_cron_jobs[i].actions[0] = '\0';
  }
  printf("[Cron] all jobs cleared\n");
}

// 新增一个周期任务: 每 interval_s 秒触发一次
static bool CronAddEvery(const char* name, const char* speak, const char* actions, uint32_t interval_s) {
  if (interval_s < 5) interval_s = 5;  // 最小 5 秒, 防误触发刷屏
  if (g_cron_count >= kMaxCronJobs) return false;
  CronJob& j = g_cron_jobs[g_cron_count];
  memset(&j, 0, sizeof(j));
  j.kind = 0;
  j.interval_s = interval_s;
  j.next_ms = millis() + interval_s * 1000UL;
  snprintf(j.name, sizeof(j.name), "%s", (name && name[0]) ? name : "");
  snprintf(j.speak, sizeof(j.speak), "%s", (speak && speak[0]) ? speak : "");
  snprintf(j.actions, sizeof(j.actions), "%s", (actions && actions[0]) ? actions : "");
  j.active = true;
  g_cron_count++;
  printf("[Cron] add every %us: name='%s' speak='%s' actions='%s'\n",
         (unsigned)interval_s, j.name, j.speak, j.actions);
  return true;
}

// 新增一个定点任务: 每天 at_hour:at_min 触发一次
static bool CronAddAt(const char* name, const char* speak, const char* actions, int hour, int minute) {
  if (hour < 0 || hour > 23 || minute < 0 || minute > 59) return false;
  if (g_cron_count >= kMaxCronJobs) return false;
  CronJob& j = g_cron_jobs[g_cron_count];
  memset(&j, 0, sizeof(j));
  j.kind = 1;
  j.at_hour = hour;
  j.at_min = minute;
  j.at_last_day = -1;  // 尚未触发过
  snprintf(j.name, sizeof(j.name), "%s", (name && name[0]) ? name : "");
  snprintf(j.speak, sizeof(j.speak), "%s", (speak && speak[0]) ? speak : "");
  snprintf(j.actions, sizeof(j.actions), "%s", (actions && actions[0]) ? actions : "");
  j.active = true;
  g_cron_count++;
  printf("[Cron] add at %02d:%02d: name='%s' speak='%s' actions='%s'\n",
         hour, minute, j.name, j.speak, j.actions);
  return true;
}

// 执行一个定时任务: 播报 + 动作
static void CronFire(const CronJob& j) {
  printf("[Cron] FIRE: '%s'\n", j.name[0] ? j.name : "(unnamed)");

  // 1) 执行器动作(复用联动动作解析 + 执行)
  if (j.actions[0] != '\0') {
    VisionLinkRule tmp;
    memset(&tmp, 0, sizeof(tmp));
    tmp.active = false;
    tmp.act_count = 0;
    if (VisionParseActions(tmp, j.actions) > 0) {
      VisionExecActs(tmp);
    }
  }

  // 2) 语音播报
  if (j.speak[0] != '\0') {
    MonSpeakNow(j.speak);
  }
}

// loop 里每秒调用一次: 检查到期任务
static void CronPoll() {
  const uint32_t now = millis();
  if (g_cron_count == 0) return;
  // 周期任务: 每约 200ms 检查一次(loop 高频, 不额外延时)
  for (int i = 0; i < g_cron_count; i++) {
    CronJob& j = g_cron_jobs[i];
    if (!j.active) continue;
    if (j.kind == 0) {
      // 周期: 比较 millis
      if ((int32_t)(now - j.next_ms) >= 0) {
        CronFire(j);
        j.next_ms = now + j.interval_s * 1000UL;
      }
    } else {
      // 定点: 比较当前时间(需 NTP 已对时)
      time_t t = time(nullptr);
      if (t < 1000000000) continue;  // 时间未同步(1970~2001 间视为无效)
      struct tm tmv;
      localtime_r(&t, &tmv);
      if (tmv.tm_hour == j.at_hour && tmv.tm_min == j.at_min &&
          tmv.tm_mday != j.at_last_day) {
        j.at_last_day = tmv.tm_mday;
        CronFire(j);
      }
    }
  }
}

// ================== 智谱 GLM ReAct 自主控硬件 (借鉴 mimiclaw) ==================
// 仿 mimiclaw 的 agent_loop.c ReAct 循环: 把用户原话 + 硬件工具清单发给 GLM,
// GLM 返回 tool_calls → 本地执行 → 结果回填 → 再问, 直到 GLM 不再要工具。

// 执行 GLM 返回的单个工具调用, 结果写进 out(JSON 字符串)。返回是否执行成功。
static bool GlmExecTool(const char* name, cJSON* args, char* out, size_t out_n) {
  if (strcmp(name, "servo_set") == 0) {
    cJSON* sv = cJSON_GetObjectItem(args, "servo");
    cJSON* ag = cJSON_GetObjectItem(args, "angle");
    int idx = cJSON_IsNumber(sv) ? (int)sv->valuedouble : 0;
    int angle = cJSON_IsNumber(ag) ? (int)ag->valuedouble : 0;
    if (idx < 1 || idx > 3) { snprintf(out, out_n, "{\"ok\":false,\"msg\":\"servo 需 1~3\"}"); return false; }
    ServoWriteAngle(idx - 1, angle);
    snprintf(out, out_n, "{\"ok\":true,\"msg\":\"舵机%d 已转到 %d 度\"}", idx, angle);
    printf("[GLM] servo_set: servo%d -> %d\n", idx, angle);
    return true;
  }
  if (strcmp(name, "motor1_set") == 0) {
    cJSON* dirj = cJSON_GetObjectItem(args, "direction");
    const char* dir = (dirj && cJSON_IsString(dirj)) ? dirj->valuestring : nullptr;
    cJSON* sp = cJSON_GetObjectItem(args, "speed");
    int speed = cJSON_IsNumber(sp) ? (int)sp->valuedouble : -1;
    MotorDir d = MotorDir::kStop;
    if (dir && (strcmp(dir, "forward") == 0 || strcmp(dir, "正转") == 0)) d = MotorDir::kFwd;
    else if (dir && (strcmp(dir, "reverse") == 0 || strcmp(dir, "反转") == 0)) d = MotorDir::kRev;
    Motor1Set(d, speed);
    snprintf(out, out_n, "{\"ok\":true,\"msg\":\"电机1 %s speed=%d\"}", MotorDirName(g_motor1_dir), g_motor1_speed);
    printf("[GLM] motor1_set -> %s\n", MotorDirName(g_motor1_dir));
    return true;
  }
  if (strcmp(name, "motor2_set") == 0) {
    cJSON* dirj = cJSON_GetObjectItem(args, "direction");
    const char* dir = (dirj && cJSON_IsString(dirj)) ? dirj->valuestring : nullptr;
    MotorDir d = MotorDir::kStop;
    if (dir && (strcmp(dir, "forward") == 0 || strcmp(dir, "正转") == 0)) d = MotorDir::kFwd;
    else if (dir && (strcmp(dir, "reverse") == 0 || strcmp(dir, "反转") == 0)) d = MotorDir::kRev;
    else if (dir && (strcmp(dir, "brake") == 0 || strcmp(dir, "刹车") == 0)) d = MotorDir::kBrake;
    Motor2SetDirection(d);
    snprintf(out, out_n, "{\"ok\":true,\"msg\":\"电机2 %s\"}", MotorDirName(g_motor2_dir));
    printf("[GLM] motor2_set -> %s\n", MotorDirName(g_motor2_dir));
    return true;
  }
  if (strcmp(name, "led_set") == 0) {
    cJSON* colorj = cJSON_GetObjectItem(args, "color");
    const char* color = (colorj && cJSON_IsString(colorj)) ? colorj->valuestring : nullptr;
    cJSON* led = cJSON_GetObjectItem(args, "led");
    int li = cJSON_IsNumber(led) ? (int)led->valuedouble : 0;
    if (color && strcmp(color, "off") == 0) {
      LedShowOff();
      snprintf(out, out_n, "{\"ok\":true,\"msg\":\"灯已关\"}");
      return true;
    }
    uint8_t hue = 0, sat = 0;
    if (!LedColorFromName(color, hue, sat)) {
      snprintf(out, out_n, "{\"ok\":false,\"msg\":\"未知颜色 %s\"}", color ? color : "");
      return false;
    }
    if (li == 0) {
      g_led_pick_mask = 0xFF;
      for (uint32_t i = 0; i < kWs2812Count; ++i) { g_led_pick_hue[i] = hue; g_led_pick_sat[i] = sat; }
    } else if (li >= 1 && li <= 8) {
      g_led_pick_mask |= (1u << (li - 1));
      g_led_pick_hue[li - 1] = hue; g_led_pick_sat[li - 1] = sat;
    }
    LedShowPickMask();
    snprintf(out, out_n, "{\"ok\":true,\"msg\":\"灯已设色\"}");
    printf("[GLM] led_set -> color=%s led=%d\n", color ? color : "", li);
    return true;
  }
  if (strcmp(name, "fan_set") == 0) {
    cJSON* lv = cJSON_GetObjectItem(args, "level");
    int level = cJSON_IsNumber(lv) ? (int)lv->valuedouble : 0;
    if (level < 0) level = 0; if (level > 3) level = 3;
    char kv[16] = {0};
    snprintf(kv, sizeof(kv), "FAN=%d", level);
    MonSendAct(kv);
    snprintf(out, out_n, "{\"ok\":true,\"msg\":\"风扇已设 %d 档\"}", level);
    printf("[GLM] fan_set -> %d\n", level);
    return true;
  }
  snprintf(out, out_n, "{\"ok\":false,\"msg\":\"未知工具 %s\"}", name);
  return false;
}

// 智谱单次 chat 调用(带 tools), 返回响应 JSON 字符串(调用者负责 cJSON_Delete)。
static cJSON* GlmChat(const char* system_prompt, cJSON* messages) {
  cJSON* body = cJSON_CreateObject();
  cJSON_AddStringToObject(body, "model", kGlmModel);
  cJSON_AddStringToObject(body, "system", system_prompt);
  cJSON_AddItemToObject(body, "messages", cJSON_Duplicate(messages, 1));
  cJSON* tools = cJSON_Parse(kGlmToolsJson);
  if (tools) cJSON_AddItemToObject(body, "tools", tools);
  cJSON_AddStringToObject(body, "tool_choice", "auto");
  cJSON_AddNumberToObject(body, "temperature", 0.2);

  char* post = cJSON_PrintUnformatted(body);
  cJSON_Delete(body);
  if (!post) return nullptr;

  WiFiClientSecure tls;
  tls.setInsecure();
  tls.setTimeout(30000);
  HTTPClient http;
  cJSON* result = nullptr;
  if (http.begin(tls, kGlmUrl)) {
    http.addHeader("Content-Type", "application/json");
    http.addHeader("Authorization", String("Bearer ") + String(GLM_API_KEY));
    int code = http.POST((uint8_t*)post, strlen(post));
    if (code == 200) {
      String resp = http.getString();
      result = cJSON_Parse(resp.c_str());
    } else {
      printf("[GLM] chat http=%d\n", code);
    }
  }
  http.end();
  free(post);
  return result;
}

// 智谱 ReAct 主循环: 输入用户原话, 自主调用硬件工具, 最终把结果播报出来。
static void GlmReActLoop(const char* user_text) {
  if (GLM_API_KEY[0] == '\0') return;  // 没配 key, 通道关闭
  if (user_text == nullptr || user_text[0] == '\0') return;

  printf("[GLM] ReAct start: '%s'\n", user_text);

  const char* sys = "你是智能种植系统的硬件调度助手。根据用户的话，自主决定是否需要调用硬件工具"
                    "（舵机/电机/风扇/RGB灯）来实现目标。能调工具就调，调完用一句简短的话告诉用户结果；"
                    "如果用户的话跟硬件无关，就正常简短回答，不要硬调工具。";

  cJSON* messages = cJSON_CreateArray();
  cJSON* um = cJSON_CreateObject();
  cJSON_AddStringToObject(um, "role", "user");
  cJSON_AddStringToObject(um, "content", user_text);
  cJSON_AddItemToArray(messages, um);

  char final_text[256] = {0};

  for (int iter = 0; iter < 5; iter++) {  // 最多 5 轮工具调用
    cJSON* resp = GlmChat(sys, messages);
    if (!resp) { printf("[GLM] chat failed\n"); break; }

    cJSON* choices = cJSON_GetObjectItem(resp, "choices");
    cJSON* c0 = (choices && cJSON_IsArray(choices)) ? cJSON_GetArrayItem(choices, 0) : nullptr;
    cJSON* msg = c0 ? cJSON_GetObjectItem(c0, "message") : nullptr;

    // 收集文本
    cJSON* content = msg ? cJSON_GetObjectItem(msg, "content") : nullptr;
    if (content && cJSON_IsString(content) && content->valuestring[0]) {
      snprintf(final_text, sizeof(final_text), "%s", content->valuestring);
    }

    cJSON* tool_calls = msg ? cJSON_GetObjectItem(msg, "tool_calls") : nullptr;
    bool has_tool = tool_calls && cJSON_IsArray(tool_calls) && cJSON_GetArraySize(tool_calls) > 0;

    if (!has_tool) {
      cJSON_Delete(resp);
      break;  // 不再要工具, 结束
    }

    // 先把 assistant(含 tool_calls) 加进历史
    cJSON* asst = cJSON_CreateObject();
    cJSON_AddStringToObject(asst, "role", "assistant");
    cJSON_AddStringToObject(asst, "content", content && cJSON_IsString(content) ? content->valuestring : "");
    cJSON_AddItemToObject(asst, "tool_calls", cJSON_Duplicate(tool_calls, 1));
    cJSON_AddItemToArray(messages, asst);

    // 逐条执行工具, 回填 role=tool
    cJSON* tc;
    cJSON_ArrayForEach(tc, tool_calls) {
      cJSON* id = cJSON_GetObjectItem(tc, "id");
      cJSON* fn = cJSON_GetObjectItem(tc, "function");
      cJSON* fname = fn ? cJSON_GetObjectItem(fn, "name") : nullptr;
      cJSON* fargs = fn ? cJSON_GetObjectItem(fn, "arguments") : nullptr;

      const char* tname = cJSON_IsString(fname) ? fname->valuestring : "";
      cJSON* args = cJSON_IsString(fargs) ? cJSON_Parse(fargs->valuestring) : cJSON_CreateObject();
      if (!args) args = cJSON_CreateObject();

      char out[256] = {0};
      GlmExecTool(tname, args, out, sizeof(out));

      cJSON* tool = cJSON_CreateObject();
      cJSON_AddStringToObject(tool, "role", "tool");
      cJSON_AddStringToObject(tool, "tool_call_id", cJSON_IsString(id) ? id->valuestring : "");
      cJSON_AddStringToObject(tool, "content", out);
      cJSON_AddItemToArray(messages, tool);

      cJSON_Delete(args);
    }

    cJSON_Delete(resp);
  }

  cJSON_Delete(messages);

  // 播报最终结论
  if (final_text[0] != '\0') {
    printf("[GLM] ReAct final: %s\n", final_text);
    MonSpeakNow(final_text);
  }
}

// ================== 双直流电机 (电机1=PWM, 电机2=纯开关) ==================
// ★★ 2026-10-16 用户实测定案: 【两个电机控制方式不同, 必须分开实现】
//
//   电机1 (IO1/IO2): 【能 PWM 调速】
//     - 方向 + 速度都靠 PWM 表达。用户原话:
//       "听到把pwm调成150, 那么 io1 io2 pwm 就是150; 正反转也是一样的"。
//     - 落地方式(用户已确认): 正转时【给前进脚(IO1)输出 PWM, 后退脚(IO2)恒 0】;
//       反转时反过来(IO1=0, IO2 输出 PWM); 停止时两个都 0。
//       即"方向决定哪个脚给 PWM, 速度就是那个 PWM 值"。
//     - PWM 取值范围 0~255 (8bit, 与 Arduino analogWrite 同量纲, 语音"调成150"直接用)。
//
//   电机2 (IO47/IO48): 【只能纯高低电平开关, 不能 PWM】
//     - 用户实测: IO47/IO48 用 LEDC(PWM) 控不了这台电机, digitalWrite 可以。
//       故本路【绝不调用 ledcAttach/ledcWrite】, 只做 正转/反转/停止 三个开关状态。
//
//   ★ 为什么两路实现方式完全不同: 这是硬件实测结论, 不是设计取舍。见上。
//   ★ 硬件提醒: 电机驱动(RZ7889 等)峰值 5A/持续 3A, 电机 VCC 必须【独立供电 + 与 ESP32 共地】,
//     绝不可从板子 3V3/5V 取电(会把 ESP32 拉复位)。

// ---- 电机1: PWM 调速 (IO1=前进, IO2=后退) ----
constexpr gpio_num_t kMotor1FwdPin = GPIO_NUM_1;   // 电机1 前进脚 (PWM)
constexpr gpio_num_t kMotor1RevPin = GPIO_NUM_2;   // 电机1 后退脚 (PWM)
constexpr uint32_t kMotor1PwmFreqHz = 20000;       // PWM 频率: 20kHz(超出人耳可闻范围, 电机不啸叫)
constexpr uint8_t kMotor1PwmResBits = 8;           // 8bit 分辨率 -> duty 0~255
constexpr int kMotor1PwmMax = 255;                 // PWM 上限(8bit)
constexpr int kMotor1DefaultSpeed = 150;           // 默认速度: 语音只说方向未说速度时用它

// ---- 电机2: 纯开关 (IO47=FI, IO48=BI) ----
constexpr gpio_num_t kMotor2FwdPin = GPIO_NUM_47;  // 电机2 前进输入 (FI)
constexpr gpio_num_t kMotor2RevPin = GPIO_NUM_48;  // 电机2 后退输入 (BI)

// 电机1 状态: 方向 + 速度(0~255)
MotorDir g_motor1_dir = MotorDir::kStop;
int g_motor1_speed = kMotor1DefaultSpeed;  // 记忆当前速度: 只调方向时沿用上次速度
// 电机2 状态: 方向
MotorDir g_motor2_dir = MotorDir::kStop;

const char* MotorDirName(MotorDir dir) {
  switch (dir) {
    case MotorDir::kStop: return "stop";
    case MotorDir::kFwd: return "forward";
    case MotorDir::kRev: return "reverse";
    case MotorDir::kBrake: return "brake";
  }
  return "unknown";
}

// ---- 电机1: 把"方向 + 速度"落到两个 PWM 脚上 ----
// ★ 方向决定哪个脚给 PWM, 另一个脚恒 0 —— 这是用户确认的语义。
static void Motor1Apply(MotorDir dir, int speed) {
  const uint8_t fwd = static_cast<uint8_t>(kMotor1FwdPin);
  const uint8_t rev = static_cast<uint8_t>(kMotor1RevPin);
  if (speed < 0) speed = 0;
  if (speed > kMotor1PwmMax) speed = kMotor1PwmMax;

  switch (dir) {
    case MotorDir::kFwd:
      ledcWrite(fwd, static_cast<uint32_t>(speed));
      ledcWrite(rev, 0);
      break;
    case MotorDir::kRev:
      ledcWrite(fwd, 0);
      ledcWrite(rev, static_cast<uint32_t>(speed));
      break;
    case MotorDir::kStop:
    case MotorDir::kBrake:  // 电机1 无刹车态, 按停止处理(两脚 0)
    default:
      ledcWrite(fwd, 0);
      ledcWrite(rev, 0);
      break;
  }
  g_motor1_dir = (dir == MotorDir::kBrake) ? MotorDir::kStop : dir;
  g_motor1_speed = speed;
}

// 电机1 方向+速度设置(供 MCP 调用)。speed 传负数表示"不改速度, 沿用当前"。
void Motor1Set(MotorDir dir, int speed) {
  const bool keep_speed = (speed < 0);
  const int use_speed = keep_speed ? g_motor1_speed : speed;
  Motor1Apply(dir, use_speed);
  printf("[T+%lums] [Motor1] -> %s, speed=%d%s (IO1=%d, IO2=%d)\n", (unsigned long)millis(),
         MotorDirName(g_motor1_dir), g_motor1_speed, keep_speed ? " (kept prev speed)" : "",
         (g_motor1_dir == MotorDir::kFwd) ? g_motor1_speed : 0,
         (g_motor1_dir == MotorDir::kRev) ? g_motor1_speed : 0);
}

// ---- 电机2: 纯高低电平 ----
// ★ 用 digitalWrite 而非 ledcWrite —— 用户实测本路 PWM 控不动, 只有高低电平有效。
static void Motor2Apply(MotorDir dir) {
  // 显式 static_cast<uint8_t>: digitalWrite/pinMode 期望 uint8_t, gpio_num_t 是 enum,
  // 工程开了 -Wall -Werror, 显式转换可避免潜在类型警告。
  const uint8_t fi = static_cast<uint8_t>(kMotor2FwdPin);
  const uint8_t bi = static_cast<uint8_t>(kMotor2RevPin);
  switch (dir) {
    case MotorDir::kFwd:   digitalWrite(fi, HIGH); digitalWrite(bi, LOW);  break;
    case MotorDir::kRev:   digitalWrite(fi, LOW);  digitalWrite(bi, HIGH); break;
    case MotorDir::kBrake: digitalWrite(fi, HIGH); digitalWrite(bi, HIGH); break;
    case MotorDir::kStop:
    default:               digitalWrite(fi, LOW);  digitalWrite(bi, LOW);  break;
  }
  g_motor2_dir = dir;
}

// 电机2 方向设置(供 MCP 调用)。立即生效, 无延时。
void Motor2SetDirection(MotorDir dir) {
  Motor2Apply(dir);
  printf("[T+%lums] [Motor2] -> %s (FI IO%d=%d, BI IO%d=%d)\n", (unsigned long)millis(),
         MotorDirName(g_motor2_dir), static_cast<int>(kMotor2FwdPin),
         (g_motor2_dir == MotorDir::kFwd || g_motor2_dir == MotorDir::kBrake) ? 1 : 0,
         static_cast<int>(kMotor2RevPin),
         (g_motor2_dir == MotorDir::kRev || g_motor2_dir == MotorDir::kBrake) ? 1 : 0);
}

void InitMotors() {
  // ---- 电机1: PWM (IO1/IO2) ----
  // ★ ledcAttach 绑引脚到 LEDC 通道; 舵机已占用 3 个通道(IO12/13/14),
  //   S3 共 8 个通道, 再加 2 个(IO1/IO2) 足够。
  ledcAttach(static_cast<uint8_t>(kMotor1FwdPin), kMotor1PwmFreqHz, kMotor1PwmResBits);
  ledcAttach(static_cast<uint8_t>(kMotor1RevPin), kMotor1PwmFreqHz, kMotor1PwmResBits);
  Motor1Apply(MotorDir::kStop, kMotor1DefaultSpeed);  // 上电即停, 防止乱动

  // ---- 电机2: 纯 GPIO 数字输出 (IO47/IO48), 不用 ledcAttach ----
  pinMode(static_cast<uint8_t>(kMotor2FwdPin), OUTPUT);
  pinMode(static_cast<uint8_t>(kMotor2RevPin), OUTPUT);
  Motor2Apply(MotorDir::kStop);  // 上电即停止

  printf("[Motor] motor1 ready on IO1(F)/IO2(R) PWM %ukHz 0-255; motor2 ready on IO47(FI)/IO48(BI) on-off\n",
         (unsigned)(kMotor1PwmFreqHz / 1000));
}

#if ENABLE_USB_CAMERA
// ============================================================================
// USB 摄像头拍照 → 屏幕显示 (2026-10-31, 移植自 AI_VOX_S3CAM_KZB 的 UVC 方案)
// ----------------------------------------------------------------------------
// 硬件: 摄像头 D- → IO19, D+ → IO20, 5V/GND 接电源。走 USB Host + UVC,
//       不额外占一个 IO(舵机/电机口全都留得住)。
// 流程: 语音"拍照" → MCP 工具 self.camera.capture → 懒启动 UVC → 等设备枚举
//       → 抓一帧 MJPEG 进 PSRAM → TJpgDec 解码成 RGB565 → 居中裁 3:4 铺满 240x320
//       → 常显在屏幕上, 直到用户说"返回"(self.display.home)或按 BOOT 才收起;
//         收起后释放像素缓冲(= 删除照片), 数据流继续跑着等下一次。
//       → 同时把这一帧 JPEG 以 multipart/form-data 上传到【服务器下发的 vision 地址】
//         (小智官方识别路径, 见下方"照片上传识别"段), 识别结果随 MCP 回执念给用户。
//
// 四个刻意的设计选择(全是实机踩出来的, 别想当然改):
//   ① 不落盘: JPEG 直接放 PSRAM。省掉一次 flash 写(约 100ms, 写盘时 cache 关闭会
//      卡音频), 也不依赖 LittleFS/分区表; 上传时直接拿这块缓冲拼 multipart 请求体。
//   ② 上传分两级, 尽量不碰语音链路: 先按现状(语音保持连接)直接开第二路 TLS 试上传;
//      只有当连 HTTP 层都没走到(典型症状: 内部 RAM 分不出 TLS 的 16KB+4KB 收发缓冲,
//      http.POST 返回 <=0)才 SuspendVoiceLink 让出内存重试一次, 传完再重连。
//      ★ 重连失败就得 esp_restart(S3CAM 版实测教训), 所以第一级能成功就绝不挂断。
//      相机起不来只回一条错误, 对话毫发无损 —— 失败模式小得多。
//   ③ ★ 相机【只初始化一次、终身常驻、永不停止/挂起】。这是两次实机炸机换来的:
//      · 第一次: 拍完就 usb_streaming_stop + 释放缓冲, 第二张必然失败
//        (_usb_urb_alloc: urb data_buffer alloc failed / CORRUPT HEAP: Bad head)。
//        → 该库的 stop 只回收一部分资源, USB 主机的 DMA 内存没还堆;
//          "停了流还立刻 free xfer/frame" 还会与 USB 任务抢指针、写坏 PSRAM 堆头。
//      · 第二次: 改成"拍完 SUSPEND, 下次 RESUME", 连拍 5 次后
//        (uvc_stream_start: sample task create failed → assert xTaskGenericNotify)。
//        → 该库 SUSPEND 会 uvc_stream_stop() 销毁 sample 任务, RESUME 再新建;
//          而任务栈是静态创建的, 任务退出时不释放 → 每轮净漏 ~4.2KB 内部 RAM。
//      → 现在的做法: 连接时 RESUME 一次, 之后【既不停也不挂起】。帧回调没有拍照
//        请求时第一行就 return, 常态开销只是 USB 等时传输的一点 CPU
//        (USB 任务优先级 4 < 音频 5, 抢不到音频的 CPU)。
//      代价: USB 主机那点内部 RAM(约 15KB)不归还, 缓冲全在 PSRAM(约 1.1MB / 8MB)。
//   ④ 大缓冲一律优先 PSRAM: 35KB×2 + 128KB 若占内部 RAM, 会把 USB 的 URB 堆挤干
//      (S3CAM 实测: urb data buffer alloc failed)。内部 RAM 只留给音频/USB 的 DMA。
// ============================================================================
constexpr uint32_t kCamXferBufSize = 35 * 1024;    // outbuf/holdbuf: 只被 CPU memcpy 读写, 不走 DMA
constexpr uint32_t kCamFrameBufSize = 128 * 1024;  // 完整一帧 JPEG (PSRAM 大, 720P 也装得下)
constexpr uint16_t kCamMaxW = 640;                 // 帧分辨率上限 (USB 1.1 Full-Speed 带宽)
constexpr uint16_t kCamMaxH = 480;
constexpr uint32_t kCamCaptureTimeoutMs = 1500;    // 等一帧的兜底超时 (30fps 下正常 <100ms)
constexpr uint32_t kCamConnectTimeoutMs = 3000;    // 等设备枚举完成 (插好后一般 <2 秒)
// ★★ 2026-11-24 加固: "第一次拍照卡死"的根因是整条链路【同步串行跑在 loop 任务里】,
//    而且每一步的等待上限都很宽(枚举 5s + 抓帧 3×3s + 上传 8s+30s + 重连 10s > 90s)。
//    这期间 loop 完全停摆: 屏幕定格、语音不应答、按键无反应 —— 用户看到的就是"卡死"。
//    本轮改动只做三件事, 不改流程结构, 风险最小:
//      ① 每一步打点(阶段名 + t+毫秒 + 当时的堆), 真卡住时一眼看出停在哪;
//      ② 收紧每个等待的上限, 让最坏总时长从 90s+ 压到 ~25s;
//      ③ 去掉"超时即 esp_restart()"—— 自动重启在现场表现成"黑屏重启", 比卡住更吓人。
constexpr uint32_t kCamBudgetMs = 25000;           // 整条链路总预算, 超了立刻放弃并回执
static uint32_t s_cam_t0 = 0;                      // 本次拍照的起点(millis)

// 阶段打点: 每次拍照至少打 8 行, 卡死时最后一行就是死点。
static void CamMark(const char* stage) {
  const uint32_t dt = (s_cam_t0 == 0) ? 0 : (millis() - s_cam_t0);
  printf("[cam][t+%5ums] %-22s | int free=%6u largest=%6u | psram free=%7u\n", (unsigned)dt, stage,
         (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
         (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL),
         (unsigned)heap_caps_get_free_size(MALLOC_CAP_SPIRAM));
  Serial.flush();
}
constexpr int kCamShotW = 240;                     // = kDisplayWidth: 照片铺满整屏
constexpr int kCamShotH = 320;                     // = kDisplayHeight

static uint8_t* s_cam_xfer_a = nullptr;             // UVC 传输缓冲 A
static uint8_t* s_cam_xfer_b = nullptr;             // UVC 传输缓冲 B
static uint8_t* s_cam_frame_buf = nullptr;          // UVC 组帧缓冲(库内部拼一帧用)
static uint8_t* s_cam_dma_reserve = nullptr;        // DMA 预留块: 枚举期间锁住, 给 URB 留连续块
static uint8_t* s_cam_jpeg = nullptr;               // 抓下来的这一帧 JPEG (PSRAM)
static size_t s_cam_jpeg_len = 0;
static bool s_cam_ready = false;                    // usb_streaming_start 已成功(成功后就终身常驻)
static volatile bool s_cam_connected = false;       // 设备枚举完成(可出图)
static bool s_cam_suspended = false;                // false = 流在跑(库 start 后即运行); 只有我们主动 SUSPEND 过才为 true
static bool s_cam_init_failed = false;              // 本次开机初始化失败过 → 不再重试(见 ③)
static volatile bool s_cam_capture_req = false;     // 拍照请求: 来了下一帧就抓
static volatile bool s_cam_capture_done = false;    // 本张已处理完(无论成败)
static volatile bool s_cam_capture_ok = false;
// ★ 2026-11-24: memcpy 互锁。帧回调跑在 USB 任务里, CamCapture 超时后会重新置请求;
//   若上一帧的 128KB memcpy 还没拷完(USB 任务被抢占), 两次 memcpy 会同时写同一块缓冲,
//   抓出来的就是半帧拼接数据 → 解码必失败, 还会把 s_cam_jpeg_len 写成中间值。
static volatile bool s_cam_copying = false;
// ---- 上传识别 (2026-10-31) ----
static char s_cam_question[192] = {0};       // 本次要问视觉模型的问题(AI 经 MCP 参数传入, 空=默认提示词)
static char g_cam_notice_json[1600] = {0};   // 会话重建后让 AI 播报的手拼 detect 消息(见 CamRespondAndResume)

// 只从 PSRAM 分配 (见上面第 ④ 条)。
// ★★ 2026-11-24: 删掉了原来的"内部 RAM 兜底"。那行兜底是定时炸弹 —— 内部 RAM 实测只有
//   33KB 可用, 万一 PSRAM 分配失败, 它会把 35KB/128KB 的缓冲塞进内部 RAM, 直接把
//   音频/TLS/USB 的堆挤干, 现象是"能拍一次, 之后整机哑掉"。宁可明确失败返回 false。
static bool CamAllocBuf(uint32_t size, uint8_t** p) {
  *p = (uint8_t*)heap_caps_malloc(size, MALLOC_CAP_8BIT | MALLOC_CAP_SPIRAM);
  if (*p == nullptr) {
    printf("[cam] psram alloc %u failed (internal fallback disabled on purpose)\n", (unsigned)size);
  }
  return *p != nullptr;
}

// UVC 帧回调 (跑在 usb 任务里)。平时第一行就返回, 零开销; 拍照请求置位时拷走这一帧。
// ★ 必须在回调里 memcpy: frame->data 指向库内部的组帧缓冲, 回调返回后随时会被下一帧覆盖。
static void camera_frame_cb(uvc_frame_t* frame, void* arg) {
  (void)arg;
  if (!s_cam_capture_req) return;
  if (s_cam_jpeg == nullptr) return;
  if (frame->frame_format != UVC_FRAME_FORMAT_MJPEG) return;
  if (frame->data_bytes < 200 || frame->data_bytes > kCamFrameBufSize) return;
  if (s_cam_copying) return;      // 上一帧还在 memcpy, 这帧直接丢(见 s_cam_copying 说明)
  s_cam_capture_req = false;      // 只拍一张, 先清请求防重入
  s_cam_copying = true;
  memcpy(s_cam_jpeg, frame->data, frame->data_bytes);
  s_cam_jpeg_len = frame->data_bytes;
  s_cam_capture_ok = true;
  s_cam_copying = false;          // ★ 必须在 done 之前清, 否则 CamCapture 可能抢先读到 done
  s_cam_capture_done = true;
}

// USB 设备状态回调 (枚举完成/断开): 选分辨率 → 释放 DMA 预留块 → 恢复出图
static void camera_state_cb(usb_stream_state_t event, void* arg) {
  (void)arg;
  if (event == STREAM_CONNECTED) {
    // 选分辨率: 优先"刚好够铺满 240x320"的最小者(宽>=240 且 高>=320),
    //          没有就退到 >=240x240, 再没有就取最小的。
    uvc_frame_size_t list[48];
    size_t num = 0;
    uvc_frame_size_list_get(nullptr, &num, nullptr);
    if (num > 48) num = 48;
    if (num > 0) uvc_frame_size_list_get(list, &num, nullptr);
    size_t best = 0;
    uint32_t best_area = 0;
    bool have = false;
    for (size_t i = 0; i < num; ++i) {
      const uint32_t area = (uint32_t)list[i].width * list[i].height;
      if (list[i].width > kCamMaxW || list[i].height > kCamMaxH) continue;
      if (list[i].width >= 240 && list[i].height >= kCamShotH) {
        if (!have || area < best_area) { have = true; best = i; best_area = area; }
      }
    }
    if (!have) {
      for (size_t i = 0; i < num; ++i) {
        const uint32_t area = (uint32_t)list[i].width * list[i].height;
        if (list[i].width > kCamMaxW || list[i].height > kCamMaxH) continue;
        if (list[i].width >= 240 && list[i].height >= 240) {
          if (!have || area < best_area) { have = true; best = i; best_area = area; }
        }
      }
    }
    if (!have) {
      for (size_t i = 0; i < num; ++i) {
        const uint32_t area = (uint32_t)list[i].width * list[i].height;
        if (!have || area < best_area) { have = true; best = i; best_area = area; }
      }
    }
    if (have) {
      printf("[cam] set resolution %ux%u\n", (unsigned)list[best].width, (unsigned)list[best].height);
      const esp_err_t err = uvc_frame_size_reset(list[best].width, list[best].height, FPS2INTERVAL(30));
      if (err != ESP_OK) {
        // 不能在这里 abort: 本回调跑在 usb 任务里, abort 会拖垮整机
        printf("[cam] uvc_frame_size_reset failed err=0x%x\n", (unsigned)err);
      }
    } else {
      printf("[cam] camera has no usable resolution\n");
    }
    // ★ 就在这一刻之后库要分 URB(512B)。枚举期间语音会话会把堆打散,
    //   预留块保证 URB 必然分得到连续块。
    if (s_cam_dma_reserve) {
      heap_caps_free(s_cam_dma_reserve);
      s_cam_dma_reserve = nullptr;
      printf("[cam] dma reserve freed for urb alloc\n");
    }
    s_cam_connected = true;
    // ★ 这一行是排障关键: 库在 RESUME 里才会为 UVC 分配 URB(内部 DMA, 512 字节起),
    //   分配失败就整条流起不来(日志: urb alloc failed! [UVC] urb_num=1 bpp=512)。
    //   把 RESUME 之前的堆状态打出来, 一眼能看出是不是"内部 RAM 被别的功能挤干"。
    printf("[cam] connected heap: DMA free=%u largest=%u | internal free=%u\n",
           (unsigned)heap_caps_get_free_size(MALLOC_CAP_DMA),
           (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_DMA),
           (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL));
    // ★★ 2026-10-31 修正: 这里【不再】调 CTRL_RESUME。
    //   usb_streaming_start() 之后库里流本来就处于运行态, 再 RESUME 一次会被库判为
    //   "already resume", 并连带返回一个 ESP_FAIL("Reset transfer failed/timeout");
    //   我们据此把挂起标志置 true, 于是拍照时又 RESUME 一次 → 状态彻底打架,
    //   最终表现成"摄像头起不来(内部内存不足)" —— 其实跟内存毫无关系。
    //   正确语义: 连接成功 = 流在跑 = 没挂起。要挂起只有我们自己调 CTRL_SUSPEND 才成立。
    s_cam_suspended = false;
    printf("[cam] camera connected (stream running)\n");
  } else if (event == STREAM_DISCONNECTED) {
    s_cam_connected = false;
    printf("[cam] camera disconnected\n");
  }
}

// 懒启动 UVC。每一步都手工检查: 失败只禁用摄像头, 语音/屏/传感器照常。
static void InitCamera() {
  printf("[cam] init uvc (USB Host, D-=IO19 D+=IO20)\n");
  CamMark("uvc init begin");
  printf("[cam] heap: DMA free=%u largest=%u | internal free=%u | psram free=%u\n",
         (unsigned)heap_caps_get_free_size(MALLOC_CAP_DMA),
         (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_DMA),
         (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
         (unsigned)heap_caps_get_free_size(MALLOC_CAP_SPIRAM));
  Serial.flush();

  // ★★ 内存门槛 (2026-11-24, 第二版)
  //   实机基线(本机空载测量): internal free≈33KB / largest≈23.5KB, DMA largest≈23.5KB,
  //   PSRAM 7.84MB —— 这套配置拍得好好的, 说明 23.5KB 对 UVC 完全够用
  //   (URB 只要 512B 起的 DMA 连续块, 样本任务栈 4KB)。
  //   ★ 第一版门槛写成 internal largest >= 24KB, 结果 23540 差 1036 字节被拦下,
  //     现象是"什么都没改, 烧个固件摄像头就起不来了" —— 教训: 门槛必须留足余量, 宁可低。
  //   现在只防两个真正的极端: PSRAM 被占光(4 个大缓冲分不出) / DMA 连 10KB 都拿不出。
  const size_t psram_free = heap_caps_get_free_size(MALLOC_CAP_SPIRAM);
  const size_t dma_largest = heap_caps_get_largest_free_block(MALLOC_CAP_DMA);
  if (psram_free < (size_t)1500 * 1024 || dma_largest < (size_t)10 * 1024) {
    printf("[cam] ABORT: psram free=%u (need >=1536000), dma largest=%u (need >=10240)\n",
           (unsigned)psram_free, (unsigned)dma_largest);
    Serial.flush();
    s_cam_init_failed = true;  // 内存不足属于"本次开机不可恢复", 别反复重试把堆切得更碎
    return;
  }

  if (!CamAllocBuf(kCamXferBufSize, &s_cam_xfer_a) || !CamAllocBuf(kCamXferBufSize, &s_cam_xfer_b) ||
      !CamAllocBuf(kCamFrameBufSize, &s_cam_frame_buf) || !CamAllocBuf(kCamFrameBufSize, &s_cam_jpeg)) {
    printf("[cam] buffer alloc failed (PSRAM 不足?), camera disabled\n");
    Serial.flush();
    return;
  }

  uvc_config_t uvc_config = {
      .frame_width = FRAME_RESOLUTION_ANY,
      .frame_height = FRAME_RESOLUTION_ANY,
      .frame_interval = FRAME_INTERVAL_FPS_30,
      .xfer_buffer_size = kCamXferBufSize,
      .xfer_buffer_a = s_cam_xfer_a,
      .xfer_buffer_b = s_cam_xfer_b,
      .frame_buffer_size = kCamFrameBufSize,
      .frame_buffer = s_cam_frame_buf,
      .frame_cb = &camera_frame_cb,
      .frame_cb_arg = nullptr,
      .flags = FLAG_UVC_SUSPEND_AFTER_START,  // 先挂起, 连接后选好分辨率再 RESUME
  };
  // ★ DMA 预留块: 此刻堆最宽裕, 先锁一块 4KB 连续内存; 枚举的几秒里语音会话会持续
  //   分配打散堆, 到 camera_state_cb 里再释放, 让 URB 必然分到。
  s_cam_dma_reserve = (uint8_t*)heap_caps_malloc(4096, MALLOC_CAP_8BIT | MALLOC_CAP_DMA);
  printf("[cam] dma reserve: %s\n", s_cam_dma_reserve ? "ok (4KB held)" : "failed (skip)");
  Serial.flush();
  delay(50);

  esp_err_t cam_err = uvc_streaming_config(&uvc_config);
  if (cam_err != ESP_OK) {
    printf("[cam] uvc config failed err=0x%x, camera disabled\n", (unsigned)cam_err);
    Serial.flush();
    return;
  }
  cam_err = usb_streaming_state_register(&camera_state_cb, nullptr);
  if (cam_err != ESP_OK) {
    printf("[cam] state register failed err=0x%x, camera disabled\n", (unsigned)cam_err);
    Serial.flush();
    return;
  }
  // ★ 先记账再 start: 摄像头如果本来就插着, CONNECTED 回调可能在 start() 返回前就
  //   把 s_cam_suspended 改成 false; 若放到 start 之后再赋值, 会把这个状态又抹回 true。
  s_cam_suspended = true;  // FLAG_UVC_SUSPEND_AFTER_START: start 完是挂起态, 等 CONNECTED 再 RESUME
  // ★ 这两行之间是 USB Host 起来 + 起 usb 任务的地方, 也是"卡在第一次拍照"最常见的中招点
  //   (库内部要 install host / 建任务, 内部 RAM 紧张时会重试)。前后各打一点即可判定。
  CamMark("usb_streaming_start ...");
  cam_err = usb_streaming_start();
  CamMark("usb_streaming_start done");
  printf("[cam] usb_streaming_start err=0x%x (%s)\n", (unsigned)cam_err, esp_err_to_name(cam_err));
  Serial.flush();
  if (cam_err != ESP_OK) {
    return;  // 摄像头不可用, 语音等功能照常
  }
  s_cam_ready = true;
  printf("[cam] init done, heap after: internal free=%u largest=%u | psram free=%u\n",
         (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
         (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL),
         (unsigned)heap_caps_get_free_size(MALLOC_CAP_SPIRAM));
  // 不在这里阻塞等连接: 摄像头插上后 CONNECTED 事件自动选分辨率并恢复出图
}

// 恢复出图(见文件头第 ③ 条)。★ 只在设备连接时被调用一次, 之后数据流一直跑着。
// ★ 绝不 usb_streaming_stop / free 缓冲: 这个库的 stop 回收不干净, 二次 start 必失败;
//   而"停流后立刻 free xfer/frame"还有与 USB 任务抢指针、写坏 PSRAM 堆头的风险。
// ★★ 为什么不挂起(2026-10-31 实机第 2 次踩坑): 这个库的 SUSPEND → resume 会
//   `uvc_stream_stop()`(销毁 sample 任务, taskh 置 NULL) + `uvc_stream_close()`,
//   恢复时 `uvc_stream_start()` 再 `_task_create_psram_stack()` 新建任务 ——
//   而那个任务栈是**静态创建**(heap_caps_malloc 4096B 内部 RAM + TCB), 任务自己
//   `vTaskDelete(NULL)` 时【不会释放】。于是每挂起/恢复一轮就净漏 ~4.2KB 内部 RAM:
//   实测连拍 5 次后内部 RAM 见底 →
//     uvc_stream_start: sample task create failed → _uvc_streaming_resume 失败
//     → 库的 free_stream_ 里 uvc_stream_stop(NULL) → xTaskNotifyGive(NULL)
//     → assert failed: xTaskGenericNotify 直接炸机
//   所以: 只在连接时 RESUME 一次, 之后永不挂起。帧回调在没有拍照请求时第一行就返回,
//   常态代价只是 USB 等时传输的一点 CPU(USB 任务优先级 4, 低于音频的 5, 不抢音频)。
//   ★ 也正因为失败即炸机(库内部没有判空), 这里只试一次, 不做无意义的重试。
static bool CamStreamResume() {
  if (!s_cam_ready) return false;
  if (!s_cam_suspended) return true;
  const esp_err_t err = usb_streaming_control(STREAM_UVC, CTRL_RESUME, nullptr);
  if (err == ESP_OK) {
    s_cam_suspended = false;
    vTaskDelay(pdMS_TO_TICKS(150));  // 恢复出图后先放掉几帧陈旧/半帧数据
    return true;
  }
  if (err == ESP_ERR_INVALID_STATE) {
    // 库用 INVALID_STATE 表示 "Streaming is running" —— 本来就在跑, 等价于成功
    s_cam_suspended = false;
    return true;
  }
  // ★ 2026-10-31 实机: 库里流已经恢复, 但控制调用返回 ESP_FAIL, 同时打印
  //     "stream resume: already resume" + "Reset transfer failed/timeout"
  //   —— 这是"库状态=RESUMED、我们的标志=挂起"的不同步, 不是内存不足, 也不是真失败。
  //   既然库认为在跑, 我们也按"在跑"处理, 让后面的抓帧去证伪;
  //   抓不到时 CamTakePhoto 还有"重试 + 硬复位"兜底。
  printf("[cam] resume returned err=0x%x (likely already-resume, assume running) | "
         "DMA free=%u largest=%u\n",
         (unsigned)err,
         (unsigned)heap_caps_get_free_size(MALLOC_CAP_DMA),
         (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_DMA));
  s_cam_suspended = false;
  vTaskDelay(pdMS_TO_TICKS(150));
  return true;
}

// 等设备枚举完成(非阻塞式轮询, 不卡死任何东西 —— 期间音频任务照常跑)
static bool CamWaitConnected(uint32_t timeout_ms) {
  const uint32_t t0 = millis();
  while (millis() - t0 < timeout_ms) {
    if (s_cam_connected) return true;
    vTaskDelay(pdMS_TO_TICKS(50));
  }
  return s_cam_connected;
}

// 同步抓一帧: 置请求 → 等帧回调 memcpy 完 → 返回结果。
// 30fps 下正常 <100ms, 超时 kCamCaptureTimeoutMs 兜底, 不会把 loop 卡死。
static bool CamCapture() {
  if (!s_cam_ready || !s_cam_jpeg) return false;
  // ★ 上一帧的 memcpy 还没结束就先等它(最多 200ms), 否则新请求会让帧回调跳过这一帧,
  //   白白多等一个 1.5s 超时。
  for (int i = 0; i < 20 && s_cam_copying; ++i) {
    vTaskDelay(pdMS_TO_TICKS(10));
  }
  s_cam_capture_done = false;
  s_cam_capture_ok = false;
  s_cam_jpeg_len = 0;
  s_cam_capture_req = true;
  const uint32_t t0 = millis();
  while (!s_cam_capture_done && (millis() - t0) < kCamCaptureTimeoutMs) {
    vTaskDelay(pdMS_TO_TICKS(20));
  }
  if (!s_cam_capture_done) {
    s_cam_capture_req = false;
    printf("[cam] capture timeout (%ums) connected=%d req=%d copying=%d\n",
           (unsigned)kCamCaptureTimeoutMs, (int)s_cam_connected, (int)s_cam_capture_req,
           (int)s_cam_copying);
    return false;
  }
  printf("[cam] frame got: %u bytes in %ums\n", (unsigned)s_cam_jpeg_len, (unsigned)(millis() - t0));
  return s_cam_capture_ok;
}

// ---- 解码: JPEG → RGB565 → 裁成 3:4 → 缩放到 240x320 铺满 ----
// 照片怎么摆到屏上(两个只用改数字的旋钮)
// CAM_ROT: 摄像头装歪了 90° 时用来救场。0 = 不转(默认); 1 = 顺时针 90°; -1 = 逆时针 90°。
//          USB 摄像头平放在桌面上拍, 一般不用转; 竖着夹在支架上拍才需要。
#define CAM_ROT 0
// CAM_SHOT_FIT: 0 = 居中裁成 3:4 铺满整屏(默认, 像相册, 观感好, 但会裁掉左右两侧);
//               1 = 整张等比装进屏幕并留黑边(一点内容都不裁, 画面比屏幕小)。
#define CAM_SHOT_FIT 0

static uint8_t* s_cam_rgb_full = nullptr;    // 解码输出整图 (PSRAM, 最大 640x480x2 = 600KB)
static uint8_t* s_cam_rgb_screen = nullptr;  // 最终上屏的 240x320 (PSRAM, 150KB; 要长期留着给 LVGL 读)
#if CAM_ROT != 0
static uint8_t* s_cam_rgb_rot = nullptr;     // 旋转缓冲(只在 CAM_ROT != 0 时分配)
#endif

struct CamJpegIn {
  const uint8_t* data;
  size_t len;
  size_t pos;
};

static size_t cam_jpeg_in(JDEC* jd, uint8_t* buf, size_t n) {
  CamJpegIn* ctx = (CamJpegIn*)jd->device;
  if (ctx->pos + n > ctx->len) n = ctx->len - ctx->pos;
  if (n) {
    if (buf) memcpy(buf, ctx->data + ctx->pos, n);
    ctx->pos += n;
  }
  return n;
}

// 输出回调: 把解码出的 RGB565 矩形块拷进整图缓冲
static int cam_jpeg_out(JDEC* jd, void* bitmap, JRECT* rect) {
  static int s_out_calls = 0;
  const uint16_t out_w = (uint16_t)(jd->width >> jd->scale);  // 缩放后的输出宽度
  uint8_t* src = (uint8_t*)bitmap;
  const int rows = rect->bottom - rect->top + 1;
  const int cols = rect->right - rect->left + 1;
  for (int y = 0; y < rows; y++) {
    uint16_t* dst = (uint16_t*)(s_cam_rgb_full + (size_t)(rect->top + y) * out_w * 2) + rect->left;
    memcpy(dst, src + (size_t)y * cols * 2, (size_t)cols * 2);
  }
  // ★ 解码在 loop 任务里同步跑约 0.5~1s, 每 32 块让出 1ms,
  //   给同优先级任务(串口链路)和看门狗喘息机会
  if (++s_out_calls % 32 == 0) {
    vTaskDelay(pdMS_TO_TICKS(1));
  }
  return 1;
}

// 把 s_cam_jpeg 解出来铺满屏幕。返回 false 只代表"没显示成", 不影响拍照本身。
static bool CamShowPhotoOnScreen() {
  if (g_display == nullptr) return false;
  if (s_cam_jpeg == nullptr || s_cam_jpeg_len < 100) {
    printf("[cam] show: no frame\n");
    return false;
  }
  if (s_cam_rgb_full == nullptr) {
    s_cam_rgb_full = (uint8_t*)heap_caps_malloc((size_t)kCamMaxW * kCamMaxH * 2, MALLOC_CAP_8BIT | MALLOC_CAP_SPIRAM);
  }
  if (s_cam_rgb_screen == nullptr) {
    s_cam_rgb_screen = (uint8_t*)heap_caps_malloc((size_t)kCamShotW * kCamShotH * 2, MALLOC_CAP_8BIT | MALLOC_CAP_SPIRAM);
  }
  static uint8_t* s_work = nullptr;
  if (s_work == nullptr) s_work = (uint8_t*)heap_caps_malloc(16 * 1024, MALLOC_CAP_8BIT | MALLOC_CAP_SPIRAM);
  if (s_cam_rgb_full == nullptr || s_cam_rgb_screen == nullptr || s_work == nullptr) {
    printf("[cam] show: psram buf alloc failed\n");
    return false;
  }

  CamJpegIn in_ctx = {s_cam_jpeg, s_cam_jpeg_len, 0};
  JDEC jd;
  JRESULT r = jd_prepare(&jd, cam_jpeg_in, s_work, 16 * 1024, &in_ctx);
  if (r != JDR_OK) {
    printf("[cam] show: jd_prepare err=%d\n", (int)r);
    return false;
  }
  // 选最大的缩放档, 但要保证缩完仍不小于屏幕(不够就在下一步放大, 太糊)
  uint8_t scale = 0;
  while (scale < 3 && (jd.width >> (scale + 1)) >= (uint16_t)kCamShotW && (jd.height >> (scale + 1)) >= (uint16_t)kCamShotH) {
    scale++;
  }
  const int out_w0 = jd.width >> scale;
  const int out_h0 = jd.height >> scale;
  if (out_w0 > kCamMaxW || out_h0 > kCamMaxH) {
    printf("[cam] show: decoded %dx%d too big\n", out_w0, out_h0);
    return false;
  }
  r = jd_decomp(&jd, cam_jpeg_out, scale);
  if (r != JDR_OK) {
    printf("[cam] show: jd_decomp err=%d\n", (int)r);
    return false;
  }
  int out_w = out_w0;
  int out_h = out_h0;

#if CAM_ROT != 0
  // 旋转 90° 补偿安装方向(宽高互换)。只有整屏重排一次, 开销可忽略。
  if (s_cam_rgb_rot == nullptr) {
    s_cam_rgb_rot = (uint8_t*)heap_caps_malloc((size_t)kCamMaxW * kCamMaxH * 2, MALLOC_CAP_8BIT | MALLOC_CAP_SPIRAM);
  }
  if (s_cam_rgb_rot != nullptr) {
    const int rot_w = out_h;
    const int rot_h = out_w;
    const uint16_t* src16 = (const uint16_t*)s_cam_rgb_full;
    uint16_t* dst16 = (uint16_t*)s_cam_rgb_rot;
    for (int y = 0; y < out_h; y++) {
      for (int x = 0; x < out_w; x++) {
#if CAM_ROT == 1
        dst16[(size_t)x * rot_w + (size_t)(rot_w - 1 - y)] = src16[(size_t)y * out_w + x];  // 顺时针
#else
        dst16[(size_t)(rot_h - 1 - x) * rot_w + y] = src16[(size_t)y * out_w + x];          // 逆时针
#endif
      }
      if ((y & 63) == 0) vTaskDelay(1);
    }
    uint8_t* tmp = s_cam_rgb_full;
    s_cam_rgb_full = s_cam_rgb_rot;
    s_cam_rgb_rot = tmp;  // 指针互换, 下次旋转复用旧缓冲
    out_w = rot_w;
    out_h = rot_h;
  }
#endif

  int cw = 0;  // 实际参与显示的源区域(只给日志用)
  int ch = 0;
#if CAM_SHOT_FIT
  // ---- 完整装入: 等比缩到屏幕内, 黑底补边, 一点内容都不裁 ----
  memset(s_cam_rgb_screen, 0, (size_t)kCamShotW * kCamShotH * 2);
  int dw = kCamShotW;
  int dh = (int)((int64_t)out_h * kCamShotW / out_w);
  if (dh > kCamShotH) {
    dh = kCamShotH;
    dw = (int)((int64_t)out_w * kCamShotH / out_h);
  }
  if (dw < 1) dw = 1;
  if (dh < 1) dh = 1;
  const int ox = (kCamShotW - dw) / 2;
  const int oy = (kCamShotH - dh) / 2;
  for (int y = 0; y < dh; y++) {
    const int sy = (int)((int64_t)y * out_h / dh);
    const uint16_t* srow = (const uint16_t*)s_cam_rgb_full + (size_t)sy * (size_t)out_w;
    uint16_t* drow = (uint16_t*)s_cam_rgb_screen + (size_t)(oy + y) * (size_t)kCamShotW + (size_t)ox;
    for (int x = 0; x < dw; x++) {
      drow[x] = srow[(int)((int64_t)x * out_w / dw)];
    }
    if ((y & 31) == 0) vTaskDelay(1);
  }
  cw = dw;
  ch = dh;
#else
  // ---- 铺满: 居中裁成 3:4 (240:320), 再最近邻缩放到 240x320。单趟完成, 不需要中间缓冲 ----
  cw = out_w;
  ch = out_h;
  if ((int64_t)out_w * kCamShotH >= (int64_t)out_h * kCamShotW) {  // 源偏宽 → 以高为准
    ch = out_h;
    cw = (int)((int64_t)out_h * kCamShotW / kCamShotH);
  } else {  // 源偏高/正好 → 以宽为准
    cw = out_w;
    ch = (int)((int64_t)out_w * kCamShotH / kCamShotW);
  }
  if (cw < 1) cw = 1;
  if (ch < 1) ch = 1;
  const int sx0 = (out_w - cw) / 2;
  const int sy0 = (out_h - ch) / 2;
  for (int y = 0; y < kCamShotH; y++) {
    const int sy = sy0 + (int)((int64_t)y * ch / kCamShotH);
    const uint16_t* srow = (const uint16_t*)s_cam_rgb_full + (size_t)sy * (size_t)out_w + (size_t)sx0;
    uint16_t* drow = (uint16_t*)s_cam_rgb_screen + (size_t)y * (size_t)kCamShotW;
    for (int x = 0; x < kCamShotW; x++) {
      drow[x] = srow[(int)((int64_t)x * cw / kCamShotW)];
    }
    if ((y & 31) == 0) vTaskDelay(1);  // 7.7 万像素走完要十几毫秒, 分 10 次让出 CPU 保音频
  }
#endif

  // ★ TJpgDec(JD_FORMAT=1) 输出 uint16 RGB565, 与 LVGL v9 的 RGB565 原生格式一致;
  //   esp_lvgl_port 的 swap_bytes 在 flush 时统一换序, 这里不要自己翻转字节。
  static lv_image_dsc_t s_photo_dsc;
  memset(&s_photo_dsc, 0, sizeof(s_photo_dsc));
  s_photo_dsc.header.magic = LV_IMAGE_HEADER_MAGIC;
  s_photo_dsc.header.cf = LV_COLOR_FORMAT_RGB565;
  s_photo_dsc.header.w = kCamShotW;
  s_photo_dsc.header.h = kCamShotH;
  s_photo_dsc.data_size = (uint32_t)kCamShotW * kCamShotH * 2;
  s_photo_dsc.data = s_cam_rgb_screen;
  g_display->ShowPhotoPage(&s_photo_dsc);
  printf("[cam] show: %ux%u -> %dx%d, crop %dx%d -> 240x320 on screen\n", (unsigned)jd.width,
         (unsigned)jd.height, out_w, out_h, cw, ch);
  return true;
}

// "删除"这张照片: 释放上屏用的像素缓冲(PSRAM), 下次拍照再重新分配。
// ★ 只能等照片页真正收起之后再调 —— LVGL 绘制时直接读这块内存, 提前 free 就是
//   use-after-free; 释放时机由 loop 里那段(收起后再等 200ms)控制。
// ★ 只释放 s_cam_rgb_screen(就是"这张照片"); 解码用的 s_cam_rgb_full / s_work 是
//   草稿纸, 留着复用, 免得每次拍照都申请 600KB(碎片化风险)。
static void CamReleasePhotoBuffers() {
  if (s_cam_rgb_screen != nullptr) {
    heap_caps_free(s_cam_rgb_screen);
    s_cam_rgb_screen = nullptr;
    s_cam_jpeg_len = 0;  // 逻辑上这张照片也没了, 别让谁误以为还有一帧在手里
    printf("[cam] photo deleted (pixel buffer released)\n");
  }
}

// ============ 照片上传识别 (小智官方 vision 方案) ============
// 对齐 xiaozhi-esp32 的 Esp32Camera::Explain:
//   · POST 到服务器下发的 explain url (capabilities.vision, 见 FilteredObserver)
//   · Content-Type: multipart/form-data; boundary=----ESP32_CAMERA_BOUNDARY
//   · 两个字段: question(AI 传入的问句, 决定识别模型怎么答) + file(camera.jpg, image/jpeg)
//   · 头: Device-Id(带冒号 MAC) / Client-Id(无冒号 MAC) / Authorization: Bearer <token>
//   · 响应: {"success":true,"result":"..."} 或 {"success":false,"message":"..."}
// ★ 官方用 chunked + 独立编码线程; 这里直接整包 POST, 语义等价且代码少一半
//   (整包约 130KB, 放 PSRAM, 内部 RAM 一点不占 —— 本板内部 RAM 有多紧见文件头 ④)。
static const char* kCamDefaultQuestion = "请用一到两句简短的中文描述这张照片的主要内容";
static const char* kCamBoundary = "----ESP32_CAMERA_BOUNDARY";

// ★★ 2026-10-31 实机教训: 服务器下发的 vision 地址是
//      http://api.xiaozhi.me/vision/explain     ← 明文 HTTP, 不是 https!
//   我一开始不分青红皂白用 WiFiClientSecure 去连, POST 必然返回 <=0(约等于
//   "连接被对端直接掐断"), 还误判成"内部内存不足开不了第二路 TLS"。
//   所以：按 URL 前缀分流 —— http:// 用明文 WiFiClient, https:// 才用 TLS。
//   明文还有个巨大好处: 完全不吃 TLS 那 16KB+4KB 内部 RAM,
//   上传期间语音链路可以一直活着, 连"挂断让路"这一整套都不需要了。
// 把一份 multipart 请求体 POST 出去并解析识别结果。
static bool CamUploadHttp(NetworkClient& client, const char* url, uint8_t* body, size_t body_len,
                          char* out, size_t out_n, int* http_code) {
  // 设备身份头: 与官方同格式(Device-Id 保留冒号, Client-Id 去掉冒号)
  String mac = WiFi.macAddress();  // AA:BB:CC:DD:EE:FF
  String client_id = mac;
  client_id.replace(":", "");

  HTTPClient http;
  // ★ 2026-11-24: 原 8000ms。连接超时压在 5s —— 网络不通时这 5 秒是纯锁死 loop 的时间,
  //   现场等 8 秒已经够难受了。
  http.setConnectTimeout(5000);
  if (!http.begin(client, url)) {
    snprintf(out, out_n, "HTTP 初始化失败(地址非法)");
    return false;
  }
  http.addHeader("Device-Id", mac.c_str());
  http.addHeader("Client-Id", client_id.c_str());
  if (!s_explain_token.empty()) {
    http.addHeader("Authorization", String("Bearer ") + String(s_explain_token.c_str()));
  }
  http.addHeader("Content-Type", String("multipart/form-data; boundary=") + kCamBoundary);
  // ★ 2026-11-24: 原 30000ms(弱网给足 30 秒)。但这段全在 loop 里同步跑, 30 秒 = 屏幕
  //   定格 30 秒 = 用户以为死机。改 12 秒: 130KB 在 2.4G 上 2~4 秒就传完, 12 秒足够,
  //   真传不完说明网络已经废了, 早放弃早恢复交互。
  http.setTimeout(12000);

  const int code = http.POST(body, body_len);
  if (code <= 0) {
    // <=0 是 HTTPClient 的错误码(-1 连接失败 / -32512 TLS 分配失败 ...),
    // 正常服务器状态码不可能为负。http 地址却走 TLS 客户端, 就是典型死法。
    http.end();
    printf("[cam] upload POST failed code=%d\n", code);
    snprintf(out, out_n, "连接失败(%d)", code);
    return false;
  }
  *http_code = code;
  if (code != 200) {
    http.end();
    printf("[cam] upload http=%d\n", code);
    snprintf(out, out_n, "服务器返回 HTTP %d", code);
    return false;
  }
  const String resp = http.getString();
  http.end();

  cJSON* root = cJSON_Parse(resp.c_str());
  if (root == nullptr) {
    printf("[cam] upload response parse failed: %.160s\n", resp.c_str());
    snprintf(out, out_n, "响应解析失败");
    return false;
  }
  // 官方响应实测(2026-10-31):
  //   {"success":true,"filename":"xxx.jpg","text":"照片从低角度仰拍室内天花板……"}
  // ★ 识别结果是放在 "text" 里的, 不是 "result" —— 之前只认 result/message,
  //   于是拿到 200 却说"未获得识别结果"。三个字段全认, 谁都兜得住:
  //   result(某些实现) / text(官方实测) / message(错误信息或纯文本)
  cJSON* success = cJSON_GetObjectItem(root, "success");
  cJSON* result = cJSON_GetObjectItem(root, "result");
  cJSON* text = cJSON_GetObjectItem(root, "text");
  cJSON* message = cJSON_GetObjectItem(root, "message");
  bool ok = false;
  if (cJSON_IsBool(success) && !cJSON_IsTrue(success)) {
    snprintf(out, out_n, "%s",
             (cJSON_IsString(message) && message->valuestring) ? message->valuestring : "服务器返回失败");
  } else if (cJSON_IsString(result) && result->valuestring) {
    snprintf(out, out_n, "%s", result->valuestring);
    ok = true;
  } else if (cJSON_IsString(text) && text->valuestring) {
    snprintf(out, out_n, "%s", text->valuestring);  // ★ 官方实测走这条
    ok = true;
  } else if (cJSON_IsString(message) && message->valuestring) {
    snprintf(out, out_n, "%s", message->valuestring);
    ok = true;
  } else {
    printf("[cam] upload response unexpected: %.200s\n", resp.c_str());
    snprintf(out, out_n, "未获得识别结果");
  }
  cJSON_Delete(root);
  // ★ 识别结果拿到后, 先做联动检查: 命中规则关键词 -> 转舵机
  if (ok) {
    const int hit = VisionLinkCheck(out);
    if (hit > 0) {
      printf("[cam] vision link triggered %d rule(s)\n", hit);
    }
  }
  return ok;
}

// 单次上传尝试(不碰语音链路)。返回 true = 拿到识别结果。
//   out/out_n : 识别结果, 或失败原因
//   http_code : 0 = 连 HTTP 层都没走到(连接/TLS/内存问题); >0 = 服务器状态码
static bool CamUploadOnce(const char* question, char* out, size_t out_n, int* http_code) {
  *http_code = 0;
  if (out_n > 1) out[0] = '\0';
  const char* url = CamExplainUrl();
  if (url[0] == '\0') {
    snprintf(out, out_n, "服务器没下发视觉识别地址");
    return false;
  }
  if (s_cam_jpeg == nullptr || s_cam_jpeg_len < 100) {
    snprintf(out, out_n, "没有可上传的照片");
    return false;
  }

  // 请求体缓冲: 只申请一次, 之后常驻复用(PSRAM 8MB, 200KB 不值一提)
  static const size_t kBodyCap = 200 * 1024;
  static uint8_t* s_body = nullptr;
  if (s_body == nullptr) {
    s_body = (uint8_t*)heap_caps_malloc(kBodyCap, MALLOC_CAP_8BIT | MALLOC_CAP_SPIRAM);
    if (s_body == nullptr) {
      snprintf(out, out_n, "上传缓冲分配失败");
      return false;
    }
  }
  if (s_cam_jpeg_len + 512 > kBodyCap) {  // 512 够放两段边界和两个字段头
    snprintf(out, out_n, "照片太大, 放不进请求体");
    return false;
  }

  size_t n = 0;
  n += (size_t)snprintf((char*)s_body + n, kBodyCap - n,
                        "--%s\r\n"
                        "Content-Disposition: form-data; name=\"question\"\r\n"
                        "\r\n"
                        "%s\r\n"
                        "--%s\r\n"
                        "Content-Disposition: form-data; name=\"file\"; filename=\"camera.jpg\"\r\n"
                        "Content-Type: image/jpeg\r\n"
                        "\r\n",
                        kCamBoundary, question, kCamBoundary);
  memcpy(s_body + n, s_cam_jpeg, s_cam_jpeg_len);
  n += s_cam_jpeg_len;
  n += (size_t)snprintf((char*)s_body + n, kBodyCap - n, "\r\n--%s--\r\n", kCamBoundary);

  // 设备身份头在 CamUploadHttp 里现取(那边按 scheme 分流后就地加 header)

  printf("[cam] upload start: jpeg=%u body=%u | internal free=%u largest=%u | DMA free=%u largest=%u\n",
         (unsigned)s_cam_jpeg_len, (unsigned)n,
         (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
         (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL),
         (unsigned)heap_caps_get_free_size(MALLOC_CAP_DMA),
         (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_DMA));

  // ★ 按 scheme 分流(见上面对 http:// 的说明)。两个客户端都放局部作用域:
  //   函数一返回就析构 —— https 那条路占的 TLS 内部 RAM(16KB+4KB)立刻还堆,
  //   这是"轻装尝试失败后还能再挂断重试一次"的前提。
  bool ret = false;
  const bool is_https = (strncmp(url, "https://", 8) == 0);
  printf("[cam] upload scheme: %s\n", is_https ? "https (TLS)" : "http (plain, no TLS)");
  if (is_https) {
    WiFiClientSecure tls;
    tls.setInsecure();            // 演示项目跳过证书链校验(和官方固件一致)
    tls.setHandshakeTimeout(15);  // 秒; 默认 120s 太长, 演示现场不能干等
    ret = CamUploadHttp(tls, url, s_body, n, out, out_n, http_code);
  } else {
    WiFiClient plain;  // 明文: 不吃内部 RAM, 上传期间语音链路可以一直活着
    ret = CamUploadHttp(plain, url, s_body, n, out, out_n, http_code);
  }
  printf("[cam] upload done ok=%d code=%d result=%.100s\n", (int)ret, *http_code, out);
  return ret;
}

// 挂断语音后要等引擎任务真正处理完 DISCONNECTED(异步)并把音频引擎/TLS 还给堆,
// 否则立刻开第二路 TLS 必然又失败(-0x7F00 = MBEDTLS_ERR_SSL_ALLOC_FAILED)。
// ★ SuspendVoiceLink 自己只调 esp_websocket_client_close(), 释放发生在
//   OnWebSocketDisconnected() 里(engine 任务队列异步执行) —— 所以必须轮询等。
static void CamWaitVoiceSuspended(uint32_t timeout_ms) {
  auto& engine = ai_vox::Engine::GetInstance();
  const uint32_t t0 = millis();
  bool closed_seen = false;
  while (millis() - t0 < timeout_ms) {
    vTaskDelay(pdMS_TO_TICKS(200));
    if (!engine.VoiceLinkConnected()) {
      closed_seen = true;
      vTaskDelay(pdMS_TO_TICKS(400));  // 再给引擎任务一拍, 让它把音频引擎 reset 掉
      break;
    }
  }
  printf("[cam] suspend %s after %ums | internal free=%u largest=%u\n", closed_seen ? "settled" : "timeout",
         (unsigned)(millis() - t0),
         (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
         (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL));
}

// 语音链路重连: SuspendVoiceLink 断开后 DISCONNECTED 事件在引擎任务里异步处理,
// 状态落定 kStandby 要一点时间; Advance() 在状态不对时是 no-op, 所以轮询调用
// 直到 websocket 重建成功。
// ★ 超时只给 8 秒(原 20 秒): 连不回来时语音已经是死的, 现场干等 20 秒最难受,
//   不如早点重启恢复 —— 重启后是干净堆, 一定能连上。
static void CamResumeVoiceLink() {
  auto& engine = ai_vox::Engine::GetInstance();
  printf("[cam] pre-resume: internal free=%u largest=%u\n",
         (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
         (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL));

  // ① 先静等 1.2 秒: DISCONNECTED 事件要在引擎任务里落定 kStandby, 音频引擎也要 reset 完。
  //    ★ 这段【故意不推 Advance】—— 每推一次都可能在 kStandby 里新建 websocket client,
  //      每新建一次就想再要 20KB TLS; 连不上就白把堆切碎一次, 一次比一次难成功。
  //    ★ 2026-11-24: 原 20×100ms。缩短是因为这段时间整机是"失联"的, 越短越好。
  for (int i = 0; i < 12; i++) {
    vTaskDelay(pdMS_TO_TICKS(100));
    if (engine.VoiceLinkConnected()) {
      printf("[cam] voice link resumed during settle\n");
      return;
    }
  }

  // ② 之后每 0.6 秒推一次 Advance(共 4 次, 合计约 2.4 秒)。
  for (int i = 0; i < 4; i++) {
    printf("[cam] resume push #%d | largest=%u internal=%u\n", i + 1,
           (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL),
           (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL));
    engine.Advance();
    for (int k = 0; k < 6; k++) {
      vTaskDelay(pdMS_TO_TICKS(100));
      if (engine.VoiceLinkConnected()) {
        printf("[cam] voice link resumed at push #%d\n", i + 1);
        return;
      }
    }
  }

  // ★★ 2026-11-24 改: 这里原来是 esp_restart()。
  //   现场表现是"卡住十来秒 → 黑屏重启", 比单纯卡住更吓人; 而且重启后语音要重新握手、
  //   摄像头要重新枚举, 演示时等于白丢半分钟。更要命的是: 如果摄像头一插就占满内部 RAM,
  //   重启后第一次拍照会再走同一条路 —— 变成无限重启循环。
  //   现在只报告不重启: 语音链路交给 loop 里每圈都在跑的 Advance() 慢慢恢复,
  //   照片已经拍到、屏幕上也显示着, 用户照样看得到画面。
  printf("[cam] voice link NOT back after ~3.6s, leave it to loop Advance() (no restart)\n");
  Serial.flush();
  return;
}

// 统一收尾: 先恢复语音链路, 再按"会话是否重建"选回执通道。
//   · sid 未变: 走正常 MCP 回执, 服务端会把它当成工具结果交给大模型继续对话;
//   · sid 已变: 旧 call id 随旧会话作废, MCP 回执会被服务端丢弃 —— 改用手拼
//     detect 消息(带【新】session_id)让 AI 播报, 与自动监测播报同一套已验证模式。
static void CamRespondAndResume(int64_t call_id, const char* ack, bool is_error,
                                const std::string& sid_before) {
  auto& engine = ai_vox::Engine::GetInstance();
  CamResumeVoiceLink();
  const std::string sid_after = engine.GetSessionId();
  if (sid_after == sid_before) {
    if (is_error) {
      engine.SendMcpCallError(call_id, ack);
    } else {
      engine.SendMcpCallResponse(call_id, std::string(ack));
    }
    return;
  }
  // detect 消息是手拼 JSON, 文本里的双引号/反斜杠会破坏结构, 统一替换成中文逗号
  std::string safe(ack != nullptr ? ack : "");
  for (auto& c : safe) {
    if (c == '"' || c == '\\') c = ',';
  }
  const int jn = snprintf(g_cam_notice_json, sizeof(g_cam_notice_json),
                          "{\"session_id\":\"%s\",\"type\":\"listen\",\"state\":\"detect\",\"text\":\"%s\"}",
                          sid_after.c_str(), safe.c_str());
  if (jn > 0 && (size_t)jn < sizeof(g_cam_notice_json)) {
    engine.SendText(g_cam_notice_json);
    printf("[cam] ack via detect message (session rebuilt)\n");
  } else {
    printf("[cam] detect message too long (%d), ack dropped\n", jn);
  }
}

// 拍照成功后的"上传识别 + 回执"总调度(两级策略, 见文件头 ②)。
// cap_ack: CamTakePhoto 给的原始回执(含"照片已在屏幕上"等说明), 失败时兜底用它。
static void CamUploadAndRespond(int64_t call_id, const char* cap_ack, const std::string& sid_before) {
  auto& engine = ai_vox::Engine::GetInstance();
  CamMark("upload begin");
  (void)cap_ack;  // 回执已改由本函数统一拼, 原 cap_ack 不再使用

  // ★★ 2026-11-24 顺序重构(第二次拍照"能拍不能识别"的根治):
  //   旧顺序 抓帧 → 解码 → 上屏 → 上传:
  //         解码上屏先把 LVGL 绘制层缓冲吃掉, 到上传时内部最大连续块只剩 1460 字节,
  //         http.POST 连 socket 都建不起来(返回 -3) → "照片有、识别无"。
  //   新顺序 抓帧 → 上传 → 解码 → 上屏:
  //         上传那一刻解码缓冲还没申请、屏幕还没画大图, 内部 RAM 最宽裕。
  //   代价: 照片晚 2~3 秒出现(先听到识别结果, 再看到画面), 换来识别必定走得通。
  // ★ 下面每条提前返回都必须走 showAndReply(), 否则照片就不显示了。
  auto showAndReply = [&](const char* reason) {
    CamMark("decode begin");
    const bool shown = CamShowPhotoOnScreen();
    CamMark(shown ? "shown on screen" : "decode FAILED");
    char a[768] = {0};
    if (shown) {
      snprintf(a, sizeof(a),
               "拍照成功, 但这次没做识别（%s）。照片已经显示在屏幕上, 看完说一声返回就行。", reason);
    } else {
      snprintf(a, sizeof(a), "拍照成功, 但这次没做识别（%s）, 照片也没能显示到屏幕上。", reason);
    }
    printf("[cam] ack(no-recog): %s\n", a);
    engine.SendMcpCallResponse(call_id, std::string(a));
  };

  // ---- 分支 1: 服务器没下发 vision 地址(设备未激活/服务端没开视觉) → 只上屏 ----
  if (CamExplainUrl()[0] == '\0') {
    printf("[cam] SKIP recognition: no vision url captured (server never sent capabilities.vision)\n");
    showAndReply("服务器没下发识别地址");
    return;
  }

  // ---- 闸门 1: 总预算 ----
  if (s_cam_t0 != 0 && (millis() - s_cam_t0) > kCamBudgetMs) {
    printf("[cam] upload SKIPPED: budget %ums already used\n", (unsigned)(millis() - s_cam_t0));
    showAndReply("本次耗时已超预算");
    return;
  }

  // ---- 闸门 2: 内部 RAM(只防"绝对是 0 的极端情况", 不拦 3KB 这种临界值) ----
  //   ★ 2026-11-24 第二次调整: 原来卡 3072, 实测第二次拍照时连这都过不去, 结果 AI 只能
  //     说"内存不足" —— 门槛反而成了拦路虎。改成 1024: 只要不是完全枯竭就让它去试,
  //     真失败也能从 -3 里看出真实原因; 而且下面"失败就挂断语音重试"会立刻腾出 20KB+。
  const size_t int_largest_up = heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL);
  if (int_largest_up < (size_t)1024) {
    printf("[cam] upload SKIPPED: internal largest=%u < 1024 (ram exhausted)\n",
           (unsigned)int_largest_up);
    showAndReply("内部内存已耗尽");
    return;
  }

  const char* question = (s_cam_question[0] != '\0') ? s_cam_question : kCamDefaultQuestion;
  char res[640] = {0};
  int code = 0;
  bool ok = CamUploadOnce(question, res, sizeof(res), &code);

  bool suspended = false;
  // ★★ 2026-11-25 回退到"只对 https 挂断": 昨天改成"明文也挂断", 实测重连时
  //     E esp-aes: Failed to allocate memory → WEBSOCKET_EVENT_ERROR → 语音彻底掉线。
  //   根因: 内部 RAM 33KB, UVC 常驻吃掉约 15KB, 上传结束后剩不下重建 TLS 所需的 20KB。
  //   挂断语音等于"借高利贷", Upload 完成了却还不回去。
  //   已从根本解决: LVGL 分配器改为 PSRAM 优先(见 lv_mem_core_clib.c), 内部 RAM 腾出来了,
  //   明文上传现在直接就能成功, 不需要挂断语音。保留 https 分支仅为兼容。
  const bool url_is_https_retry = (strncmp(CamExplainUrl(), "https://", 8) == 0);
  if (!ok && code == 0 && url_is_https_retry) {
    printf("[cam] attempt#1 failed w/o HTTP, pre-suspend: internal free=%u largest=%u\n",
           (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
           (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL));
    printf("[cam] retry upload with voice link suspended...\n");
    engine.SuspendVoiceLink();
    suspended = true;
    // ★ 2026-11-24: 4000 → 2500ms。这 2.5 秒里屏幕是定格的, 而实际断开通常 1 秒内就落定。
    CamWaitVoiceSuspended(2500);
    // 明文只要 4KB(建 socket + 拼 header); https 才需要 18KB(TLS 收发缓冲)。
    const bool need_tls_buf = (strncmp(CamExplainUrl(), "https://", 8) == 0);
    const size_t kNeedLargest = need_tls_buf ? (size_t)(18 * 1024) : (size_t)(4 * 1024);
    const size_t largest = heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL);
    printf("[cam] after suspend: largest=%u need=%u\n", (unsigned)largest, (unsigned)kNeedLargest);
    if (largest < kNeedLargest) {
      printf("[cam] skip attempt#2: largest=%u < %u\n", (unsigned)largest, (unsigned)kNeedLargest);
    } else {
      ok = CamUploadOnce(question, res, sizeof(res), &code);
    }
  }

  // ★★ 2026-11-24: 重连语音必须排在解码上屏【之前】。
  //   上传刚结束, socket 已释放, 内部 RAM 处于最宽裕状态, 重连要的 20KB TLS 这时候最拿得出;
  //   一旦先解码上屏, LVGL 又把这波内存吃掉, 语音就再也连不回来(AI 直接哑掉)。
  //   末尾的 CamRespondAndResume 内部还会再调一次 CamResumeVoiceLink, 但那时已连接, 会秒返回。
  if (suspended) {
    printf("[cam] reconnect voice BEFORE decode (decode would eat the RAM)\n");
    CamResumeVoiceLink();
  }

  // ★★ 顺序重构的另一半: 上传彻底结束了, 现在才解码上屏。
  //   此时 socket / HTTPClient 都已析构还堆, 解码和 LVGL 渲染要吃多少内部 RAM 都不影响上传。
  CamMark("upload done, now decode");
  const bool photo_on_screen = CamShowPhotoOnScreen();
  CamMark(photo_on_screen ? "shown on screen" : "decode FAILED");
  const char* tail;
  if (photo_on_screen) {
    tail = "照片现在显示在屏幕上了, 看完说一声返回就行。";
  } else if (ok) {
    tail = "照片这次没能显示到屏幕上, 但画面已经识别过了。";
  } else {
    tail = "照片这次也没能显示到屏幕上, 请检查摄像头是否插好。";
  }

  // ★ 回执长度: 官方识别结果是整段中文描述(实测一条 60~150 字), 1024 才够,
  //   768 会在句中截断 —— 截在 UTF-8 中间还会出现乱码尾巴。
  char ack[1024] = {0};
  if (ok) {
    snprintf(ack, sizeof(ack), "拍照成功。识别结果：%s。%s", res, tail);
  } else if (code == 0) {
    snprintf(ack, sizeof(ack), "拍照成功, 但识别失败（上行连接打不开 %s）。%s", res, tail);
  } else {
    snprintf(ack, sizeof(ack), "拍照成功, 但识别失败（%s）。%s", res, tail);
  }
  printf("[cam] ack: %s\n", ack);

  if (suspended) {
    CamRespondAndResume(call_id, ack, false, sid_before);  // 内含重连 + sid 兜底
  } else {
    engine.SendMcpCallResponse(call_id, std::string(ack));
  }
}

// 一次完整拍照(语音"MCP 工具调用"的落地动作)。ack 里是给 AI 念的结果。
static bool CamTakePhoto(char* ack, size_t ack_n) {
  s_cam_t0 = millis();  // 本次拍照计时起点: 总预算闸门 + 阶段打点共用
  CamMark("capture begin");

  // ★★ 2026-11-24 第二次拍照内存枯竭的修复:
  //   照片页是【常显】的, 拍第二张时上一张还在屏上。LVGL 用的是 CLIB malloc(内部 RAM 优先),
  //   为这张 240x320 RGB565 图持有的绘制资源 + 像素缓冲会一直占着, 实测第二次拍照时
  //   内部 RAM 只剩 6244 字节、最大连续块 1460 —— http.POST 连 socket 都建不起来(返回 -3),
  //   LVGL 也刷不动屏幕(看着像死机, 其实 loop 还在跑 Env 日志)。
  //   做法: 拍新照片前先把旧照片收起并释放像素缓冲, 让 LVGL 把内部 RAM 还回来。
  //   代价: 屏幕上旧照片会消失约 0.5 秒, 然后新照片顶上来, 观感可接受。
  if (g_display != nullptr && g_display->IsPhotoPageActive()) {
    printf("[cam] previous photo active -> hide & release before capture\n");
    g_display->HidePhotoPage();
    vTaskDelay(pdMS_TO_TICKS(350));  // 给 LVGL 渲染任务一轮时间把对象/缓冲落定
    CamReleasePhotoBuffers();        // 释放 s_cam_rgb_screen(PSRAM), 内部判空
    CamMark("old photo released");
  }

  if (!s_cam_ready) {
    if (s_cam_init_failed) {
      // 首次初始化就没起来: 库内部可能停在半初始化状态, 反复重试只会越搅越乱,
      // 老实告诉用户重启设备(重启后是干净状态, 会重新走一次 InitCamera)。
      snprintf(ack, ack_n, "摄像头初始化失败过, 请重启设备后再拍");
      return false;
    }
    InitCamera();  // 懒启动: 第一次拍照才枚举 USB(2~4 秒), 之后终身常驻
    CamMark("uvc init returned");
    if (!s_cam_ready) {
      s_cam_init_failed = true;
      snprintf(ack, ack_n, "摄像头启动失败, 请检查摄像头供电与 19/20 接线, 然后重启设备再试");
      return false;
    }
  }
  if (!s_cam_connected && !CamWaitConnected(kCamConnectTimeoutMs)) {
    printf("[cam] wait connected TIMEOUT (%ums)\n", (unsigned)kCamConnectTimeoutMs);
    snprintf(ack, ack_n, "没有检测到摄像头, 请检查 IO19/IO20 接线与摄像头 5V 供电");
    return false;
  }
  CamMark("camera connected");
  if (!CamStreamResume()) {
    // 只有 s_cam_ready/s_cam_connected 都不成立才会走到这里(正常不会)。
    snprintf(ack, ack_n, "摄像头起不来, 请重启设备后再拍");
    return false;
  }
  CamMark("stream resumed");
  vTaskDelay(pdMS_TO_TICKS(300));  // 刚出图的头几帧自动曝光还没稳, 等 0.3 秒颜色才正常

  // ★ 抓帧改成"抓两次 + 最后硬复位一次"(2026-10-31 实机):
  //   之前只要 CamStreamResume 报错就整条放弃, 结果一张都拍不了。可日志显示
  //     "stream resume: already resume" + "Reset transfer failed/timeout"
  //   是库状态与我们的挂起标志不同步, 流往往其实在跑 —— 直接抓一帧就能证伪,
  //   不该凭一个控制调用的返回值就判死刑。
  bool got = CamCapture();
  if (!got) {
    printf("[cam] capture attempt#1 failed, retry once\n");
    vTaskDelay(pdMS_TO_TICKS(400));
    got = CamCapture();
  }
  CamMark(got ? "frame captured" : "frame FAILED");
  if (!got && (millis() - s_cam_t0) < kCamBudgetMs / 2) {
    // 硬复位: 流可能卡在"库以为在跑、实际不出图"。SUSPEND→RESUME 是唯一能救回来的手段;
    // ★ 代价是该库每轮净漏 ~4.2KB 内部 RAM(样本任务栈静态创建、退出不还),
    //   所以只在真失败时用, 且一轮拍照最多用一次。
    printf("[cam] hard cycle: suspend+resume once (DMA largest=%u)\n",
           (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_DMA));
    usb_streaming_control(STREAM_UVC, CTRL_SUSPEND, nullptr);
    vTaskDelay(pdMS_TO_TICKS(400));
    s_cam_suspended = true;  // 与库对齐后再走一次标准 RESUME
    if (CamStreamResume()) {
      vTaskDelay(pdMS_TO_TICKS(400));
      got = CamCapture();
    }
  }
  if (!got) {
    s_cam_suspended = true;  // 下次拍照重新走 RESUME, 别停在"以为在跑"
    snprintf(ack, ack_n, "抓取画面失败, 摄像头没有输出画面, 请检查连接");
    return false;
  }
  // ★★ 2026-11-24 顺序重构(第二次拍照"能拍不能识别"的根治):
  //   这里【不再】解码上屏, 抓到手就返回。解码+上屏改由 CamUploadAndRespond 在上传【之后】做。
  //   原因: 解码上屏会占掉 LVGL 的绘制层缓冲, 等轮到上传时内部最大连续块只剩 1460 字节,
  //        http.POST 连 socket 都建不起来(返回 -3) —— 表现就是"照片能拍能显示, 但识别不出来"。
  //   现在上传那一刻刚抓完帧, 解码缓冲还没申请、屏幕还没画大图, 内部 RAM 最宽裕。
  // ★ 拍完【不挂起】: 见 CamStreamResume 上面那段 —— 挂起/恢复每轮漏 4.2KB 内部 RAM,
  //   连拍几次就把内存漏干并炸机。数据流一直跑着, 帧回调无请求时零开销。
  snprintf(ack, ack_n, "拍照成功");
  return true;
}
#endif  // ENABLE_USB_CAMERA

// 语音"退出/待命"统一入口: 收起所有全屏覆盖层 → 上屏保。
// ★ 必须先收覆盖层: ScreenSaverTick() 里有"照片页激活时不进屏保"的保护
//   (见 display.cpp), 照片常显时说"退出"会被它挡回去, 时钟根本出不来。
// ★ Hide* 只动画面不动业务: 监测照跑、相机照待命; 照片像素缓冲仍由 loop 里
//   那段"收起后 200ms"负责释放, 这里不用管。
static void EnterSaverFromVoice() {
  if (!g_display) return;
  g_display->HidePhotoPage();
  g_display->HideMonitorPage();
  g_display->HideTerminalPage();
  g_display->HideEnvPage();
  g_display->EnterScreenSaver();
}

void InitMcpTools() {
  auto& engine = ai_vox::Engine::GetInstance();
  engine.AddMcpTool("self.audio_speaker.set_volume",         // tool name
                    "Set the volume of the audio speaker.",  // tool description
                    {
                        {
                            "volume",  // parameter name

                            ai_vox::ParamSchema<int64_t>{
                                // parameter type can be bool, std::string or int64_t
                                .default_value = std::nullopt,  // default value, set to std::nullopt if not specified
                                .min = 0,                       // minimum value, set to std::nullopt if not specified
                                .max = 100,                     // maximum value, set to std::nullopt if not specified
                            },
                        },
                        // add more parameter schema as needed
                    }  // parameter schema
  );

  engine.AddMcpTool("self.audio_speaker.get_volume",         // tool name
                    "Get the volume of the audio speaker.",  // tool description
                    {
                        // empty
                    }  // parameter schema
  );

  engine.AddMcpTool("self.led.set",
                    "控制设备 RGB 灯链(共 8 颗, 同步显示)。可选模式: "
                    "'white' 表示开启并常亮白色灯光(用户说 '开灯'、'打开灯'、'把灯打开'、'打开白灯' 时使用); "
                    "'breath' 表示开启呼吸灯效果(用户说 '呼吸灯'、'打开呼吸灯'、'来个呼吸效果' 时使用); "
                    "'rainbow' 表示开启彩色彩虹流水灯效果(用户说 '打开彩灯'、'开彩灯'、'彩色灯'、'彩虹灯'、'流光' 时使用); "
                    "'chase' 表示开启彩灯跑马灯效果 —— 3 颗不同颜色的亮点在 8 颗灯上等间隔排布、"
                    "整体一起循环跑动(用户说 '跑马灯'、'跑马灯效果'、'彩灯跑'、'3颗彩灯一起跑'、"
                    "'流水灯'、'来一个跑马灯'、'灯跑起来' 时使用); "
                    "'off' 表示关灯(用户说 '关灯'、'把灯关掉' 时使用)。"
                    "注意: 若用户说的是'打开第N个灯'、'点亮第3个灯'、'打开3 6 8号灯'这类点名具体灯珠的需求, "
                    "请不要用本工具, 改用 self.led.pick。",
                    {
                        {
                            "mode",  // parameter name
                            ai_vox::ParamSchema<std::string>{
                                // parameter type can be bool, std::string or int64_t
                                .default_value = std::nullopt,  // default value, set to std::nullopt if not specified
                            },                                  // parameter type
                        },
                        // add more parameter schema as needed
                    }  // parameter schema
  );

  engine.AddMcpTool("self.led.pick",
                    "点亮灯链上【指定的若干颗】灯(共 8 颗, 编号 1~8), 其余灯熄灭。"
                    "被点亮的灯会做呼吸效果(淡入淡出), 颜色可以逐颗指定。"
                    "参数 leds 是被点亮的灯号列表, 用英文逗号分隔, 例如 '3' 表示只亮第 3 颗, "
                    "'3,6,8' 表示同时亮第 3、6、8 颗; 传 '0' 或空字符串表示全部熄灭。"
                    "参数 pairs 用于【逐灯指定颜色】, 格式 '灯号:颜色,灯号:颜色', "
                    "例如 '1:red,2:green' 表示第1颗红、第2颗绿; '3:blue,5:yellow,7:white' 同理。"
                    "参数 color 用于【所有被点名的灯统一一个颜色】, 例如 'green'。"
                    "颜色名只接受: red(红) / green(绿) / blue(蓝) / yellow(黄) / "
                    "cyan(青) / purple(紫) / white(白)。"
                    "★★【重要】当用户一句话里【给不同的灯指定了不同的颜色】时"
                    "(例如'让第一个变红第二个变绿'、'1号红2号绿3号蓝'、'第三个蓝第四个黄'), "
                    "必须用 pairs 参数传递, 格式 '1:red,2:green'; "
                    "同时 leds 也要列出这些灯号(如 '1,2')。"
                    "若用户只说了一个颜色给多颗灯(如'打开3 6 8号黄灯'), 用 color='yellow' 即可。"
                    "★ 绝对不能把'1号红2号绿'拆成两次工具调用, 也不要只传一个颜色，"
                    "那样后一次会覆盖前一次, 导致前面的灯被熄灭。"
                    "以下说法【都属于】本工具的触发场景: "
                    "'打开第3个灯'、'点亮第3个灯'、'把第3个灯打开'、'3号灯亮'、'打开3号'、"
                    "'3 6 8'、'打开3 6 8号灯'、'只开第5盏灯'、'点亮1和4'、"
                    "'来第7个'、'第2个和第6个'、'打开第3个绿灯'、'让5号灯变红'、"
                    "'第一个红第二个绿'、'1号红2号绿'。"
                    "★ 注意: 用户【只说灯号】而没提别的动作时(例如单独一句'3号'、'6号'、'3'、'第5个'), "
                    "同样是要点亮这些灯, 必须调用本工具, 不要当成闲聊或问句。"
                    "★ 与 self.led.set 的区别: 本工具是'点名哪几颗亮哪几颗', self.led.set 是'整条链一起变色'。",
                    {
                        {
                            "leds",
                            ai_vox::ParamSchema<std::string>{
                                .default_value = std::nullopt,
                            },
                        },
                        {
                            "pairs",
                            ai_vox::ParamSchema<std::string>{
                                .default_value = std::nullopt,
                            },
                        },
                        {
                            "color",
                            ai_vox::ParamSchema<std::string>{
                                .default_value = std::nullopt,
                            },
                        },
                    }  // parameter schema
  );

  engine.AddMcpTool("self.led.get",
                    "获取设备 RGB 灯链的当前状态, 返回 'off'、'white'、'breath'、'rainbow'、'chase' 或 'custom'。",
                    {
                        // empty
                    }  // parameter schema
  );

  // ---------------- 环境传感器 MCP 工具 (语音查询温湿度气压) ----------------
  // ★ 为什么要拆成 4 个工具而不是 1 个"返回全部"的工具:
  //   只给 1 个返回三个值的工具, 用户问"温度多少"时, 大模型拿到三个数可能
  //   全报一遍、也可能只报一个 —— 取决于临场发挥, 不够稳。
  //   拆成单项工具后, 描述里写死"只问一项就调对应单项工具",
  //   问什么答什么, 输出确定。
  // ★ 数据来源: 全部读 SensorPoll() 每 3 秒刷新的缓存, 不在回调里现场读
  //   (AHT20 一次测量约 80ms, 现场读会拖慢语音响应)。

  // 综合查询: 用户一次问多项或笼统问"环境怎么样"时用
  engine.AddMcpTool("self.sensor.get_environment",
                    "获取当前环境的全部读数(温度、湿度、气压三个值一起返回)。"
                    "仅当用户一次性询问多个环境数据、或笼统地问'环境怎么样'、'现在环境如何'时使用。"
                    "如果用户只问其中一项(例如只问温度), 请改用对应的单项工具 "
                    "self.sensor.get_temperature / self.sensor.get_humidity / self.sensor.get_pressure。"
                    "返回形如 '温度 25.3°C, 湿度 46%, 气压 1008.2 hPa' 的文本。",
                    {
                        // empty
                    }  // parameter schema
  );

  // 单项: 温度
  engine.AddMcpTool("self.sensor.get_temperature",
                    "获取当前环境温度。当用户询问'现在温度多少'、'多少度'、'屋里几度'、"
                    "'热不热'、'冷不冷'、'温度' 等只涉及温度的问题时使用。"
                    "返回形如 '25.3 摄氏度' 的文本。",
                    {
                        // empty
                    }  // parameter schema
  );

  // 单项: 湿度
  engine.AddMcpTool("self.sensor.get_humidity",
                    "获取当前环境湿度。当用户询问'现在湿度多少'、'湿度'、'潮不潮'、"
                    "'干不干'、'空气湿度' 等只涉及湿度的问题时使用。"
                    "返回形如 '46%' 的文本。",
                    {
                        // empty
                    }  // parameter schema
  );

  // 单项: 气压
  engine.AddMcpTool("self.sensor.get_pressure",
                    "获取当前大气气压。当用户询问'现在气压多少'、'气压'、'大气压'、"
                    "'海拔' 等只涉及气压的问题时使用。"
                    "返回形如 '1008.2 百帕' 的文本。",
                    {
                        // empty
                    }  // parameter schema
  );

  // ★ 2026-11-06 新增: 土壤温湿度(SHT40 插土里)。只针对土壤, 与空气温湿度分开。
  engine.AddMcpTool("self.sensor.get_soil",
                    "获取当前【土壤】的温湿度(土壤温度、土壤湿度)。"
                    "当用户询问'土壤温度'、'土壤湿度'、'土温'、'土湿'、'土壤温湿度'、"
                    "'土里多少度'、'土壤干不干' 等【明确指土壤】的问题时使用。"
                    "返回形如 '土壤温度 23.1°C, 土壤湿度 45%' 的文本。",
                    {
                        // empty
                    }  // parameter schema
  );

  // ★ 2026-11-06 新增: 完整数据(空气温湿度气压 + 土壤温湿度), 一条工具全部返回。
  engine.AddMcpTool("self.sensor.get_all",
                    "获取【完整环境数据】: 空气温度、空气湿度、气压、土壤温度、土壤湿度, "
                    "共五项一次返回。"
                    "当用户说'显示完整数据'、'所有数据'、'全部数据'、'各项数据'、"
                    "'完整的环境数据'、'空气和土壤的数据都告诉我' 等【要一次看全部】时使用。"
                    "返回形如 '空气温度 25.3°C, 空气湿度 46%, 气压 1008.2 hPa, "
                    "土壤温度 23.1°C, 土壤湿度 45%' 的文本。",
                    {
                        // empty
                    }  // parameter schema
  );

  // ---------------- 屏幕控制 MCP 工具 (语音进/出屏保) ----------------
  // ★ 用户 2026-10-14 需求: "当听到待机、退出、锁屏之类的词语, 直接进入屏保显示"。
  // ★ 为什么用 MCP 工具而不是在固件里做关键词匹配:
  //   语音的识别、分词、近义表达(睡觉/休眠/黑屏/关屏…)交给云端大模型判断,
  //   固件只负责"被调用就进屏保"这一个动作。这样新增同义词不用改固件、不用重烧。
  // ★★ 2026-10-31 拆分(用户要求): "退出" 和 "待命" 现在是两件事 ——
  //   · 退出 → 只关屏保, 语音会话【保持在线】, 说一句"唤醒"立刻回来;
  //   · 待命 → 屏保 + 【断开语音会话】, 只能按 BOOT 键唤醒(离线省电)。
  //   原来一个工具管所有近义词, 模型分不清"退出"该关屏还是该断开, 结果两个都不准。
  engine.AddMcpTool("self.display.sleep",
                    "只关闭屏幕: 进入屏保(大时钟+日期+温湿度)。★ 语音会话保持在线, "
                    "用户说一句'唤醒'/'亮屏'就能立刻回来。"
                    "当用户【单独】说'退出'、'退出吧'、'锁屏'、'息屏'、'黑屏'、'关屏'、"
                    "'睡觉'、'进入屏保'、'休息' 等【只想关掉屏幕】的话时调用。"
                    "★ 不要与另外两个工具混淆: "
                    "说'待命/待机/挂断/不聊了'→ 用 self.display.standby(会断开连接); "
                    "说'退出数据页/退出画面/不看了'→ 用 self.terminal.hide 或 self.display.home"
                    "(那是收起画面, 不是关屏)。"
                    "调用成功后简短回复, 例如'好的, 屏幕关了, 说声唤醒就回来'。",
                    {
                        // empty
                    }  // parameter schema
  );

  // ★ 2026-10-31 新增: 【真·待命】—— 屏保 + 断开语音会话。
  //   用户原话: "当听到待命时候程序进去屏保 主程序进去待命状态 再次按唤醒按键唤醒"。
  //   断开用 SuspendVoiceLink(): 关掉 websocket 并释放其 TLS/音频内存(状态落 kStandby),
  //   之后【语音叫不醒】(麦克风链路也停了), 必须按 BOOT 键 —— BOOT 分支在 kStandby 会
  //   调 Advance() 重连, 这条路径已实测可用。
  engine.AddMcpTool("self.display.standby",
                    "让设备进入【待命】: 屏幕变成屏保(大时钟), 同时【断开语音会话】进入离线待命, "
                    "省电且不占网络。之后【只能按设备上的 BOOT/唤醒按键】才能再唤醒, "
                    "说话叫不醒。"
                    "当用户说'待命'、'进入待命'、'待机'、'进入待机'、'挂断'、'断开'、"
                    "'下线'、'先不聊了'、'你歇着吧' 等【要求彻底休息/断开】的话时调用。"
                    "★ 只是想关屏幕、还要随时叫醒的, 用 self.display.sleep, 不要用本工具。"
                    "调用成功后简短回复, 例如'好的, 已进入待命, 按唤醒键再叫我'。",
                    {
                        // empty
                    }  // parameter schema
  );

  engine.AddMcpTool("self.display.wake",
                    "把设备屏幕从待机/屏保状态唤醒, 恢复到正常的对话界面。"
                    "当用户说'唤醒'、'亮屏'、'打开屏幕'、'退出屏保'、'回来' 等"
                    "【要求屏幕恢复显示】的意思时调用。",
                    {
                        // empty
                    }  // parameter schema
  );

  // ★ 2026-10-31 新增: 语音"返回"回主界面。照片现在是常显的(不自动关),
  //   必须有这条指令才能收起来; 同样也用于收起监测面板/终端数据页。
  engine.AddMcpTool("self.display.home",
                    "把屏幕上的全屏画面(照片、监测面板、终端数据页)全部收起, 回到对话主界面。"
                    "当用户说'返回'、'退回主界面'、'回到对话'、'回到主界面'、'关掉照片'、"
                    "'收起照片'、'不看了'、'退出画面'、'关掉面板' 等【要求回到对话界面】的话时调用。"
                    "★ 只收起画面, 不停止后台业务: 自动监测仍在跑, 摄像头仍待命。"
                    "拍照后照片会一直显示, 用户说这句话才收起并删除照片。",
                    {
                        // empty
                    }  // parameter schema
  );

  // ---------------- 板间串口通讯 MCP 工具 (IO39/IO40, UART1) ----------------
  // ★ 面向"另一块单片机/开发板"的通讯, 不是给用户朗读的。
  //   典型链路: 用户说"通知另一块板打开水泵" → AI 调 self.serial.send("PUMP_ON")
  //            → 本机从 IO39 发出 → 对方板收到并执行。
  engine.AddMcpTool("self.serial.send",
                    "通过预留串口(IO39/IO40, UART1)向【另一块单片机/开发板】发送一行文本指令。"
                    "当用户要求把某个指令、命令或消息转发给另一块板子时调用, 例如"
                    "'告诉另一块板打开水泵'、'通知从机开始采集'、'给对面发 xxx'、'让从机重启'。"
                    "参数 text 是要发送的文本内容(不需要自己加换行, 本工具会自动补一个换行)。"
                    "注意: 这不是朗读给用户听, 而是发给外接的单片机。",
                    {
                        {
                            "text",
                            ai_vox::ParamSchema<std::string>{
                                .default_value = std::nullopt,
                            },
                        },
                    }  // parameter schema
  );

  engine.AddMcpTool("self.serial.get_last",
                    "读取【另一块单片机/开发板】最近通过串口发来的一行文本。"
                    "当用户询问'另一块板说了什么'、'有没有收到从机的数据'、'串口收到什么了'、"
                    "'对面发来什么' 等需要查看串口接收内容的问题时调用。"
                    "若至今还没收到任何数据, 工具会返回错误提示。",
                    {
                        // empty
                    }  // parameter schema
  );

  engine.AddMcpTool("self.terminal.led",
                    "打开或关闭【终端设备(另一块板)】上的照明灯(终端 IO10)。"
                    "这是一个【带握手确认】的老式操作: 发指令 → 等终端回执 → 主控亮第 1 颗灯表示链路正常。"
                    "当用户说'打开终端灯'、'打开终端的灯'、'把终端灯打开'、'点亮终端灯'、"
                    "'关掉终端灯'、'关闭终端灯'、'终端灯灭了'、'把终端那个灯关掉' 等【笼统】说法时调用。"
                    "参数 on: true = 打开, false = 关闭。"
                    "★★★ 只要用户说出了【具体设备名】—— 照明灯、照明、暖光灯、加湿器、风扇、"
                    "遮阳伞、草木灰、复合肥、舵机 —— 【必须】改用 self.act.set, 不要用本工具。"
                    "  例如'打开终端照明' → self.act.set cmd='LAMP=1'; "
                    "'打开终端的暖光灯' → self.act.set cmd='LIGHT=1'。"
                    "★ 注意: 如果返回'终端没有响应', 说明串口通讯不通或终端是旧固件, 请如实告诉用户。",
                    {
                        {
                            "on",
                            ai_vox::ParamSchema<bool>{
                                .default_value = std::nullopt,
                            },
                        },
                    }  // parameter schema
  );

  engine.AddMcpTool("self.terminal.last_result",
                    "读取【上一次与终端握手】的结果。"
                    "当用户问'终端响应了吗'、'刚才那次操作成功了吗'、'握手成功没有'、"
                    "'终端确认了吗' 等需要查看上一次终端操作结果的问句时调用。",
                    {
                        // empty
                    }  // parameter schema
  );

  engine.AddMcpTool("self.terminal.oled",
                    "让【终端设备(另一块板)】的 OLED 小屏幕上显示一个整数。"
                    "当用户说'在终端 OLED 上显示 123'、'终端屏幕显示 456'、"
                    "'让终端显示 88'、'终端 OLED 显示个 666'、'把 2025 显示到终端屏上' 等"
                    "要求在终端小屏幕显示数字的话时调用。"
                    "参数 value: 要显示的整数, 范围 0~999999。"
                    "★ 终端会把数字用大字号居中显示, 位数越少字号越大。"
                    "★ 这是【带握手确认】的操作: 工具会向终端发指令并等待终端回执, "
                    "如果返回'终端没有响应', 说明串口通讯不通, 请如实告诉用户并提示检查接线。"
                    "★ 只能显示数字, 不能显示中文或字母 —— 若用户要求显示文字, "
                    "请告知终端屏幕只能显示数字。",
                    {
                        {
                            "value",
                            ai_vox::ParamSchema<int64_t>{
                                .default_value = std::nullopt,
                            },
                        },
                    }  // parameter schema
  );

  engine.AddMcpTool("self.terminal.env",
                    "读取【终端设备(另一块板)】上 DHT11 温湿度传感器的最新读数。"
                    "当用户说'终端现在多少度'、'终端那边温湿度是多少'、'终端环境怎么样'、"
                    "'另一个板子的温度'、'终端温度湿度报一下'、'终端测到的温度' 等"
                    "询问终端环境数据的问句时调用。"
                    "★ 本工具【立即返回】缓存的最新读数(终端每 2 秒主动上报一次), 不会等待。"
                    "返回内容包含温度和湿度。你需要把这个真实读数组织成自然语言播报给用户。",
                    {
                        // empty
                    }  // parameter schema
  );

  engine.AddMcpTool("self.terminal.all",
                    "把【终端设备(另一块板)】上报的全部 11 项传感器数据【整屏显示在主控屏幕上】,"
                    "同时把数值返回给你播报。显示会覆盖当前对话界面, 30 秒后自动回到对话界面。"
                    "当用户说'显示终端的全部数据'、'帮我显示终端的全部数据'、'把终端的数据显示出来'、"
                    "'显示终端传感器数据'、'看一下终端的感应器数据'、'获取终端的感应器数据'、"
                    "'终端现在所有数据是多少'、'终端环境怎么样' 等"
                    "要求【查看或显示终端全部数据】的话时, 【必须】调用本工具。"
                    "★ 显示内容固定 11 项: 空气温度、空气湿度、气压、空气CO2、光照强度、"
                    "土壤温度、土壤湿度、土壤氮、土壤磷、土壤钾、土壤PH。"
                    "★ 播报时不要逐项念完 11 个数字: 挑用户关心的重点说 2~4 句, "
                    "并告诉用户'完整的 11 项数据已经显示在屏幕上了, 30 秒后自动返回, "
                    "想提前退出说一声就行'。"
                    "★★ 严格区分: 本工具只用于【查看/显示终端当前的实时读数】。"
                    "如果用户是在【设置 / 同步 / 下发作物生长标准、报警阈值、适宜区间】"
                    "(例如'把土豆的标准同步到终端'、'给终端设置番茄的生长条件'), "
                    "【必须改用 self.crop.profile】, 绝对不要调用本工具。"
                    "★ 若返回'还没收到终端数据', 说明串口不通或终端没通电, "
                    "请如实告知用户, 并提示检查接线(主控39→终端40, 主控40→终端39, 必须共地)。",
                    {
                        // empty
                    }  // parameter schema
  );

  // 关闭终端数据页, 回到对话界面。
  // ★ 为什么必须要这个工具: 数据页是整屏覆盖层, 会挡住对话内容。
  //   虽然有 30 秒自动关闭兜底, 但用户"想马上回去"时必须有一个立刻生效的通道,
  //   不能让他干等 30 秒(用户 2026-10-20 反馈过"无法恢复到对话界面")。
  engine.AddMcpTool("self.terminal.hide",
                    "关闭【终端数据展示页】, 把主控屏幕恢复回原本的对话界面。"
                    "当用户说'退出数据页'、'关闭数据页'、'退出终端数据'、'把数据页关掉'、"
                    "'数据不看了' 等【明确指数据页】的话时【必须】调用本工具。"
                    "★ 只在数据页正显示时才有意义; 用户没提'数据页'时不要调用 —— "
                    "单独说'退出'是关屏屏保(self.display.sleep), '返回'是回主界面(self.display.home)。"
                    "★ 调用后屏幕立刻恢复对话界面, 数据页里的数值只是隐藏, 不会丢失。",
                    {
                        // empty
                    }  // parameter schema
  );

  // 作物生长标准：从模型知识整理出区间 → 下发并让终端记住 → 终端按标准染色
  engine.AddMcpTool("self.crop.profile",
                    "把某种【作物适宜生长的环境区间】整理好后下发给终端, 让终端记住这套标准, "
                    "之后终端屏幕上每一项数据都会按标准染色(淡绿=在范围内, 淡红=超限), "
                    "作为后续数据报警的依据。终端会把这套标准存进 Flash, 断电不丢。"
                    "★★ 只要用户的话里出现【同步到终端 / 下发给终端 / 让终端记住 / 设置标准 / "
                    "报警阈值 / 适宜区间 / 生长条件 / 适合什么环境】并涉及某种作物, "
                    "就【必须】调用本工具。这里的'终端'指的是另一块板, 不是屏幕显示。"
                    "典型说法: '查一下土豆生长需要什么温度湿度'、'土豆的适宜范围是多少'、"
                    "'从网上查一下XX的生长条件'、'把XX的标准同步到终端'、'把XX的标准下发给终端'、"
                    "'让终端记住XX的标准'、'设置XX的报警标准'、'给终端设定XX的生长区间'。"
                    "★ 你必须自己根据农业常识把 11 项区间算出来填进 ranges, 不要反问用户。"
                    "★ 取值范围: 只填你确定有意义的项, 不确定的项【直接省略】, 不要瞎填。"
                    "★ 这些是通用参考区间, 播报时必须告诉用户'这是通用参考值, "
                    "具体请以当地农技标准为准'。"
                    "★ 调用后主控屏幕会退出数据页回到对话界面, 并在状态栏显示"
                    "'终端已记住 XX' 或 '终端没有回应'(表示串口不通), 请把结果如实告诉用户。"
                    "参数 crop: 作物名(中文, 7 个汉字以内, 例如 土豆 / 番茄 / 生菜)。"
                    "参数 ranges: 区间串, 格式为 KEY=下限:上限, 多个用分号分隔。"
                    "可用 KEY: T(空气温度℃) H(空气湿度%) PR(气压hPa) CO2(ppm) LX(光照lx) "
                    "ST(土壤温度℃) SH(土壤湿度%) N(氮) P(磷) K(钾) PH(土壤酸碱度)。"
                    "示例(土豆): T=10:25;H=60:80;PR=950:1050;CO2=300:800;"
                    "LX=15000:60000;ST=10:20;SH=60:80;N=60:150;P=30:80;K=80:200;PH=5.0:6.5",
                    {
                        {
                            "crop",
                            ai_vox::ParamSchema<std::string>{
                                .default_value = std::nullopt,
                            },
                        },
                        {
                            "ranges",
                            ai_vox::ParamSchema<std::string>{
                                .default_value = std::nullopt,
                            },
                        },
                    }  // parameter schema
  );

  engine.AddMcpTool("self.act.set",
                    "控制【终端设备(另一块板 B)】上的现场执行器: 暖光灯、照明灯、加湿器、"
                    "风扇、遮阳伞、草木灰舵机、复合肥舵机。指令经串口下发, 终端执行后回传实际状态。"
                    "★★★ 先分清是哪块板: 本工具控制的是【另一块板 B / 终端 / 温室那边】的设备。"
                    "如果用户说的是【主控自己 / 这块板 / 你身上的】电机、舵机、灯带, "
                    "请改用 self.motor1.set / self.motor2.set / self.servo.set / self.led.pick。"
                    "★★ 典型说法: '把终端的暖光灯打开'、'打开加湿器'、'风扇开到2档'、"
                    "'把遮阳伞放下来'、'收起遮阳伞'、'遮阳伞往前'、'遮阳伞往回退'、"
                    "'遮阳板收起来'、'加湿器关掉'、'温室的风扇调大一点'、"
                    "'让草木灰舵机转到150度'、'复合肥投放一下'、'把终端所有设备都关掉'。"
                    "参数 cmd 是键值串, 格式 KEY=值, 多项用英文逗号分隔。"
                    "可用 KEY 与取值: "
                    "LIGHT=0/1 (暖光灯, 暖黄色灯带, 开/关); "
                    "BRIGHT=0~255 (暖光灯亮度, 不写默认 255); "
                    "LAMP=0/1 (照明灯, 白色, 开/关); "
                    "HUMID=0/1 (加湿器, 开/关); "
                    "FAN=0/1/2/3 (风扇档位: 0 停 / 1 低速 / 2 中速 / 3 高速); "
                    "SHADE=1/2/0 (遮阳伞: 1 前进=把伞放下来遮阳 / 2 后退=把伞收回去 / 0 立刻停; "
                    "前进和后退都是走满 45 秒后自动停, 不用管); "
                    "NPK=0~180 (草木灰舵机角度, 90 是中位); "
                    "MIX=0~180 (复合肥舵机角度, 90 是中位)。"
                    "★ 示例: 'FAN=2'、'LIGHT=1,BRIGHT=180'、'SHADE=1'、'LIGHT=0,FAN=0,HUMID=0'。"
                    "★ 口语对应: '打开终端照明'/'打开照明灯'/'开白灯' → cmd='LAMP=1'; "
                    "'打开终端暖光灯'/'开暖光' → cmd='LIGHT=1'; "
                    "'风扇开到2档' → cmd='FAN=2'; '加湿器打开' → cmd='HUMID=1'。"
                    "★ 遮阳伞按秒控制('打开N秒'/'再前进N秒'/'复位')不要用本工具, "
                    "改用 self.shade.forward / self.shade.reset, 它们有累计时长保护。"
                    "★ 用户说'关掉所有设备'时用 cmd='LIGHT=0,LAMP=0,HUMID=0,FAN=0'。"
                    "★ 舵机没有开/关, 只有角度。用户说'撒草木灰/放复合肥'时给一个能让漏斗倾泻的角度, "
                    "例如 NPK=150 或 MIX=150; 并提醒用户做完说'舵机复位', 那时再用 NPK=90 / MIX=90 回中位。"
                    "★★ 遮阳伞每次启动都按满行程 45 秒走, 没有限位开关。所以用户刚说过'停'、"
                    "紧接着又要反方向时, 先提醒一句'伞会按满行程反着走 45 秒, 可能顶到位', "
                    "等用户确认后再发 SHADE=1 或 SHADE=2。"
                    "★ 用户只说'灯'并且指的是补充光照/暖光时用 LIGHT; 说'照明灯/白灯/亮一点'用 LAMP。"
                    "★ 调用后返回值里带终端回传的实际状态, 请按那个状态播报, 不要凭想象。",
                    {
                        {
                            "cmd",
                            ai_vox::ParamSchema<std::string>{
                                .default_value = std::nullopt,
                            },
                        },
                    }  // parameter schema
  );

  // 遮阳伞按秒控制(2026-10-23 新增): 前进 N 秒自动停, 终端侧累计总前进时长(上限45秒)。
  // 复位由终端按累计时长退回, 不用 AI 记账。
  engine.AddMcpTool("self.shade.forward",
                    "让【终端的遮阳伞】前进(展开)指定的秒数后自动停。"
                    "用户说'遮阳打开N秒'、'遮阳伞前进N秒'、'再前进N秒'、'遮阳再走N秒' 时调用, "
                    "seconds = 用户说出的那个数字, 可以是 1~45 之间的任意整数(5秒、6秒、13秒…都行), "
                    "不要默认成某个固定值, 听到多少就传多少。"
                    "★ 多次调用是【累加】关系: 先前进5秒再说前进6秒, 终端共前进11秒;"
                    "总前进时长被终端限制在45秒(全行程)以内, 超出的部分会被自动截断, 不会顶坏伞。"
                    "★ 用户说'遮阳复位/收回遮阳/回到初始位置'时不要用本工具, 用 self.shade.reset。",
                    {
                        {
                            "seconds",
                            ai_vox::ParamSchema<int64_t>{
                                .default_value = std::nullopt,
                            },
                        },
                    }  // parameter schema
  );

  engine.AddMcpTool("self.shade.reset",
                    "让【终端的遮阳伞】复位: 终端会按此前【累计前进的总时长】往后退同样的秒数, "
                    "回到最初的位置, 累计值清零。"
                    "用户说'遮阳复位'、'遮阳伞收回'、'遮阳回到初始位置'、'伞退回去' 时调用。"
                    "★ 若终端此前没有前进过, 终端会忽略复位(伞本来就在初始位), 这是正常的。",
                    {
                        // empty
                    }  // parameter schema
  );

  engine.AddMcpTool("self.act.status",
                    "查询【终端设备(另一块板 B)】上现场执行器的当前状态: "
                    "暖光灯开关与亮度、照明灯、加湿器、风扇档位、遮阳伞、草木灰舵机角度、复合肥舵机角度。"
                    "当用户说'终端执行器现在什么状态'、'灯是开的吗'、'风扇几档'、"
                    "'遮阳伞收了没'、'加湿器关了没有'、'终端设备都什么情况' 等时调用。"
                    "★ 返回的是终端最近一次回传的实际状态。若回答'还没收到终端执行器状态', "
                    "说明终端没上电或串口不通, 或还没下发过任何执行器指令。",
                    {
                        // empty
                    }  // parameter schema
  );

  engine.AddMcpTool("self.monitor.start",
                    "启动【环境自动监测】(报警+联动执行器): 主控用终端上报的传感器数据实时比对阈值, "
                    "超上限或低于下限就自动联动对应执行器并语音播报, 主控屏幕常驻监测面板。"
                    "★★ 触发说法与 channels 取值: "
                    "用户说'全自动检测'/'全自动监测'/'开始监测' → channels='all', "
                    "按用户提到的作物把全部通道的阈值算出来; "
                    "用户说'打开光照监测' → channels='光照', 只给 LX 一项区间; "
                    "'温度监测' → channels='温度'; '湿度监测' → channels='湿度'; "
                    "'CO2监测'/'二氧化碳监测' → channels='co2'; "
                    "'土壤监测'/'土壤湿度监测' → channels='土壤'; '养分监测'/'施肥监测' → channels='养分'。"
                    "★ channels 也可以多项并列, 例如 '光照,温度'。"
                    "★ 参数 ranges 格式与 self.crop.profile 完全相同: KEY=下限:上限, 分号分隔。"
                    "KEY 对应通道: LX=光照 T=空气温度 H=空气湿度 CO2=二氧化碳 SH=土壤湿度 "
                    "N/P/K=土壤养分(取三者最低的下限)。"
                    "★★ 你必须根据农业常识(结合用户说的作物)自己把阈值算出来填进 ranges, "
                    "不要反问用户。channels 里点名的每个通道都必须有对应 KEY, 缺了会报错。"
                    "★ 固定联动规则: 光照超上限→自动展开遮阳伞, 光照低于下限→自动开照明灯(白光LAMP)补光; "
                    "空气温度超上限→自动开风扇, 低于下限→自动开暖光灯(LIGHT); "
                    "空气湿度超上限→自动开风扇, 低于下限→自动开加湿器; "
                    "CO2超上限→自动开风扇; 土壤湿度低于下限→自动开加湿器; "
                    "土壤养分低于下限→施肥舵机自动投肥一次。"
                    "★ 启动后监测面板常驻屏幕, 用户说'停止监测'才收起(由 self.monitor.stop 负责)。"
                    "★ 参数 crop 可选: 作物名(中文, 7 字以内)。给了就同步终端染色并显示在面板标题里。",
                    {
                        {
                            "channels",
                            ai_vox::ParamSchema<std::string>{
                                .default_value = std::nullopt,
                            },
                        },
                        {
                            "ranges",
                            ai_vox::ParamSchema<std::string>{
                                .default_value = std::nullopt,
                            },
                        },
                        {
                            "crop",
                            ai_vox::ParamSchema<std::string>{
                                .default_value = std::nullopt,
                            },
                        },
                    }  // parameter schema
  );

  engine.AddMcpTool("self.monitor.stop",
                    "停止环境自动监测并收起监测面板。"
                    "用户说'停止监测'/'关闭监测'/'退出监测'/'不要监测了'时调用。"
                    "★ 停止时会自动把所有执行器恢复初始位置: 风扇/暖光灯/照明灯/加湿器全部关闭, "
                    "施肥舵机回中, 监测期间展开过的遮阳伞自动收起(约45秒行程)。"
                    "不需要也不应该再调用 self.act.set 逐个关闭。",
                    {
                        // empty
                    }  // parameter schema
  );

  engine.AddMcpTool("self.monitor.status",
                    "查询当前环境自动监测的运行状态: 各通道的当前值/阈值/正常或超限, "
                    "以及监测是否在运行。用户说'现在监测到什么情况'/'监测状态'/'有没有超限'等时调用。",
                    {
                        // empty
                    }  // parameter schema
  );

  engine.AddMcpTool("self.monitor.panel",
                    "显示或隐藏监测面板(【监测继续在后台运行, 不会停止】)。"
                    "用户说'显示监测面板'/'打开监测面板'/'回到监测界面'/'看看监测面板' → show='1'; "
                    "用户说'隐藏监测面板'/'收起面板' → show='0'。"
                    "★ 用户按了 BOOT 键回到对话界面后, 想再看监测数据就说'显示监测面板'。"
                    "★ 监测未启动时本工具只会提示先启动监测。",
                    {
                        {
                            "show",
                            ai_vox::ParamSchema<std::string>{
                                .default_value = std::nullopt,
                            },
                        },
                    }  // parameter schema
  );

  engine.AddMcpTool("self.servo.set",
                    "控制三路舵机中的一路转到指定角度。"
                    "舵机1 接在 IO12, 舵机2 接在 IO13, 舵机3 接在 IO14。"
                    "当用户说'舵机1 转到30度'、'舵机2 转到69度'、'舵机三转到90度'、"
                    "'舵机1 回到0度'、'转动2号舵机到120度' 等舵机转向指令时调用。"
                    "参数 servo 是舵机编号(1~3), 参数 angle 是目标角度(0~180, 整数)。",
                    {
                        {
                            "servo",
                            ai_vox::ParamSchema<int64_t>{
                                .default_value = std::nullopt,
                            },
                        },
                        {
                            "angle",
                            ai_vox::ParamSchema<int64_t>{
                                .default_value = std::nullopt,
                            },
                        },
                    }  // parameter schema
  );

  engine.AddMcpTool("self.servo.selftest",
                    "让三路舵机中的一路执行硬件自检动作: 先回0度, 再依次转到45度、90度、135度、180度, "
                    "然后回到0度, 最后从0度缓慢平滑地转向180度, 用于检测舵机硬件是否正常。"
                    "当用户说'舵机1 自检一下硬件是否正常'、'舵机二自检'、'舵机3 检测一下' 等时调用。"
                    "自检是异步执行的, 调用后立即返回'自检已开始', 动作会持续几秒钟。"
                    "参数 servo 是舵机编号(1~3)。",
                    {
                        {
                            "servo",
                            ai_vox::ParamSchema<int64_t>{
                                .default_value = std::nullopt,
                            },
                        },
                    }  // parameter schema
  );

  engine.AddMcpTool("self.vision.link",
                    "设置一条\"摄像头识别联动\"规则: 当摄像头识别结果文字里出现指定关键词时, "
                    "自动同时执行一组执行器动作(舵机1/2/3 + 电机1 + 电机2 + RGB灯 可任意组合), 并可选地播报一句话。"
                    "当用户说'识别到风扇的时候舵机1转到90度'、'看到风扇时舵机1转90度舵机2转45度'、"
                    "'识别到结果期的时候舵机1转90度电机1启动红灯亮' 等时调用。判断词和执行器都是用户口述的, "
                    "不要写死, 用户说什么词就用什么词。"
                    "参数 keyword 是触发关键词(可用竖线|分隔多个, 如'风扇|电风扇', 识别文字出现任意一个即命中)。"
                    "★ 内置生长阶段归类: 若 keyword 直接用阶段词, 会自动匹配对应特征词, 建议用户说"
                    "'开花期''结果期''幼苗期''成长期''成熟期''枯萎'时, keyword 就填这几个词本身: "
                    "开花期=开花/花朵/花苞/花瓣; 结果期=结果/果实/瓜果; 幼苗期=幼苗/小苗/嫩芽/发芽; "
                    "成长期=枝叶/长叶子/茂盛; 成熟期=成熟/黄了/变红; 枯萎=枯黄/蔫了/凋谢。"
                    "参数 actions 是动作列表, 用逗号分隔多项, 冒号分隔字段, 格式如下: "
                    "servo1:90=舵机1转到90度; servo2:45=舵机2转到45度; servo3:135=舵机3转到135度; "
                    "motor1:forward:150=电机1正转速度150(速度可省略默认150, 方向可为 forward正转/reverse反转/stop停); "
                    "motor2:forward=电机2正转(方向 forward/reverse/stop/brake); "
                    "led:red=全部8颗灯亮红色; led1:绿=第1颗灯亮绿色(led1~led8, 颜色可为 red/红 green/绿 blue/蓝 yellow/黄 cyan/青 purple/紫 white/白)。"
                    "例: 舵机1转90度舵机2转45度电机1正转红灯亮 -> actions=\"servo1:90,servo2:45,motor1:forward,led:red\"。"
                    "参数 speak 是命中后要播报的话(可选, 如'植物已到结果期'; 不传则默认播报'识别到XX,已执行联动动作')。"
                    "可设置多条规则, 同关键词会覆盖。",
                    {
                        {
                            "keyword",
                            ai_vox::ParamSchema<std::string>{
                                .default_value = std::nullopt,
                            },
                        },
                        {
                            "actions",
                            ai_vox::ParamSchema<std::string>{
                                .default_value = std::nullopt,
                            },
                        },
                        {
                            "speak",
                            ai_vox::ParamSchema<std::string>{
                                .default_value = std::nullopt,
                            },
                        },
                    }  // parameter schema
  );

  engine.AddMcpTool("self.vision.reset",
                    "清空所有\"摄像头识别联动\"规则, 并让三路舵机全部回到0度(初始位置)。"
                    "当用户说'恢复'、'舵机恢复'、'恢复初始位置'、'取消识别联动'、"
                    "'清空联动规则' 等时调用。",
                    {}  // parameter schema
  );

  engine.AddMcpTool("self.vision.stage",
                    "口述设置某个生长阶段对应的特征词列表(整句替换, 覆盖旧的)。"
                    "阶段词固定为: 开花期/结果期/幼苗期/成长期/成熟期/枯萎 这六个。"
                    "当用户说'开花期的特征是开花花朵花苞'、'结果期对应结果和果实'、"
                    "'幼苗期就是嫩芽小苗'、'把枯萎的特征改成枯黄蔫了' 等时调用。"
                    "参数 stage 是阶段词(开花期/结果期/幼苗期/成长期/成熟期/枯萎之一), "
                    "参数 features 是特征词列表, 用竖线|分隔(如'开花|花朵|花苞'), "
                    "识别文字里出现任意一个特征词就算命中该阶段。",
                    {
                        {
                            "stage",
                            ai_vox::ParamSchema<std::string>{
                                .default_value = std::nullopt,
                            },
                        },
                        {
                            "features",
                            ai_vox::ParamSchema<std::string>{
                                .default_value = std::nullopt,
                            },
                        },
                    }  // parameter schema
  );

  engine.AddMcpTool("self.cron.add",
                    "设置一个定时任务, 到点后语音播报并可选地执行一组执行器动作(舵机/电机/灯)。"
                    "支持两类: (1)周期重复——'每30分钟提醒我浇水'、'每5分钟舵机1转90度'; "
                    "(2)定点一次——'下午3点开灯'、'晚上8点舵机2转45度'。"
                    "参数 kind 取 'every'(周期)或 'at'(定点)。"
                    "kind=every 时用 interval_s(间隔秒); kind=at 时用 hour(0~23)和 minute(0~59)。"
                    "参数 speak 是到点后播报的话(必填, 如'该浇水了')。"
                    "参数 actions 是可选执行器动作, 格式同 self.vision.link 的 actions: "
                    "servo1:90/servo2:45/servo3:135/motor1:forward:150/motor2:forward/led:red/led1:绿 等, 逗号分隔多项。"
                    "不传 actions 则只播报不动作。",
                    {
                        {
                            "kind",
                            ai_vox::ParamSchema<std::string>{
                                .default_value = std::nullopt,
                            },
                        },
                        {
                            "interval_s",
                            ai_vox::ParamSchema<int64_t>{
                                .default_value = std::nullopt,
                            },
                        },
                        {
                            "hour",
                            ai_vox::ParamSchema<int64_t>{
                                .default_value = std::nullopt,
                            },
                        },
                        {
                            "minute",
                            ai_vox::ParamSchema<int64_t>{
                                .default_value = std::nullopt,
                            },
                        },
                        {
                            "speak",
                            ai_vox::ParamSchema<std::string>{
                                .default_value = std::nullopt,
                            },
                        },
                        {
                            "actions",
                            ai_vox::ParamSchema<std::string>{
                                .default_value = std::nullopt,
                            },
                        },
                    }  // parameter schema
  );

  engine.AddMcpTool("self.cron.reset",
                    "清空所有定时任务。当用户说'取消所有定时任务'、'清空定时'、'把定时任务都删掉' 等时调用。",
                    {}  // parameter schema
  );

  engine.AddMcpTool("self.motor1.set",
                    "控制【1号直流电机】(接在 IO1/IO2)。这台电机【支持调速】。"
                    "参数 direction 取值: 'forward' 正转, 'reverse' 反转, 'stop' 停止。"
                    "参数 speed 是 PWM 速度值, 范围 0~255(整数), 数值越大转得越快; "
                    "可以不传(表示保持当前速度不变)。"
                    "当用户说'1号电机正转'、'电机1正转'、'一号电机反转'、'1号电机停' 等时调用; "
                    "当用户说'把1号电机的pwm调成150'、'电机1速度调到200'、'电机一转速设为100' 等时, "
                    "保持当前方向不变, 只把 speed 设为用户说的数值; "
                    "当用户说'电机1正转 速度150'、'1号电机反转 pwm 200' 等同时含方向与速度时, "
                    "direction 和 speed 一起传。"
                    "★ 注意: 2号电机的速度不能调, 不要给 2 号电机传 speed。",
                    {
                        {
                            "direction",
                            ai_vox::ParamSchema<std::string>{
                                .default_value = std::nullopt,
                            },
                        },
                        {
                            "speed",
                            ai_vox::ParamSchema<int64_t>{
                                .default_value = std::nullopt,
                            },
                        },
                    }  // parameter schema
  );

  engine.AddMcpTool("self.motor2.set",
                    "控制【2号直流电机】(接在 IO47/IO48)。这台电机【只能正转/反转/停止, 不能调速】。"
                    "参数 direction 取值: 'forward' 正转, 'reverse' 反转, 'stop' 停止。"
                    "当用户说'2号电机正转'、'电机2反转'、'二号电机停下'、'电机2转起来' 等时调用。"
                    "★ 注意: 本接口只有 direction 一个参数, 没有速度参数 —— "
                    "2号电机是纯开关控制, 若用户提到'速度/PWM'请告知只能开或关, 不要尝试传速度值。",
                    {
                        {
                            "direction",
                            ai_vox::ParamSchema<std::string>{
                                .default_value = std::nullopt,
                            },
                        },
                    }  // parameter schema
  );

#if ENABLE_USB_CAMERA
  engine.AddMcpTool("self.camera.capture",
                    "用本机摄像头拍一张照片, 【上传给视觉模型识别】, 并把照片显示在屏幕上"
                    "(会一直显示到你让它收起)。"
                    "当用户说'拍照'、'拍一张照'、'拍个照'、'照一张'、'给我拍张照片'、"
                    "'看看这是什么'、'识别一下'、'这是什么植物/虫子/东西'、'把现在画面拍下来' 等"
                    "【需要看画面或想知道画面里是什么】的请求时调用。"
                    "参数 question: 用户具体想知道的内容, 原样填用户的问题(如'这是什么植物'、"
                    "'叶子是不是发黄了'、'画面里有几个人'); 用户只是让拍照、没提具体问题时不要填, "
                    "留空会用默认提示词做通篇描述。"
                    "★ 工具返回里【带有识别结果的文字】。请把识别结果用一两句口语念给用户听, "
                    "不要念字段名、不要逐字照读, 也不要再追问'要不要识别'。"
                    "★ 若返回里说识别失败, 就把失败原因如实转告, 并说明照片已经显示在屏幕上; "
                    "不要立刻重复调用重拍。",
                    {
                        {
                            "question",  // parameter name
                            ai_vox::ParamSchema<std::string>{
                                .default_value = std::nullopt,  // 不填 = 用默认描述提示词
                            },
                        },
                    }  // parameter schema
  );
#endif
}
}  // namespace

// ---------------- 音频任务提权的前置声明 (必须在 namespace 之外) ----------------
// ★ 为什么声明放这里: setup()/loop() 定义在匿名 namespace 之外(Arduino 要求它们是全局符号),
//   而被它们调用的 EnsureAudioTaskPriority() 定义在 setup() 之后。
//   声明与定义必须同命名空间 —— 若把声明写进上面的 namespace, 就会变成
//   `(anonymous namespace)::EnsureAudioTaskPriority`, 与实际定义(全局)不是同一个符号,
//   链接期报 undefined reference。这是 2026-10-16 实际踩过的编译错误。
//   函数实现见 setup() 之后。
void EnsureAudioTaskPriority();

// ★ 2026-11-08 复位原因取证:
//   现象是"串口打到半截就跳回 entry 0x..., 然后 from setup 重来", 但复位瞬间
//   USB CDC 会重新枚举, 最后几条日志必然丢字 —— 光看日志永远看不出是"供电跌落"
//   还是"看门狗"还是"代码崩溃"。这里把复位原因 + 连续启动次数打在 setup 最前面,
//   一开机就知道上一次是怎么死的。
static const char* ResetReasonName(esp_reset_reason_t r) {
  switch (r) {
    case ESP_RST_POWERON:   return "POWERON 上电/插USB";
    case ESP_RST_EXT:       return "EXT 外部RST脚";
    case ESP_RST_SW:        return "SW 软件esp_restart";
    case ESP_RST_PANIC:     return "PANIC 代码崩溃/abort";
    case ESP_RST_INT_WDT:   return "INT_WDT 中断看门狗";
    case ESP_RST_TASK_WDT:  return "TASK_WDT 任务阻塞>5s";
    case ESP_RST_WDT:       return "WDT 其它看门狗";
    case ESP_RST_DEEPSLEEP: return "DEEPSLEEP 深睡唤醒";
    case ESP_RST_BROWNOUT:  return "BROWNOUT 供电电压跌落";
    case ESP_RST_SDIO:      return "SDIO";
    default:                return "UNKNOWN/其它";
  }
}

// 连续启动计数放 RTC 不掉电内存: 只有真断电才会归零。
// 记住它就能区分"偶发一次"和"反复重启死循环"。
RTC_NOINIT_ATTR static uint32_t g_boot_magic;
RTC_NOINIT_ATTR static uint32_t g_boot_seq;

void setup() {
  Serial.begin(115200);
  constexpr uint32_t kBootMagic = 0x5A17C0DEu;
  if (g_boot_magic != kBootMagic) {  // 魔数不对 = RTC 内存被清 = 真的断过电
    g_boot_magic = kBootMagic;
    g_boot_seq = 0;
  }
  ++g_boot_seq;
  printf("\n=== [BOOT] seq=%u reason=%s | internal free=%u largest=%u ===\n", (unsigned)g_boot_seq,
         ResetReasonName(esp_reset_reason()),
         (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
         (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL));
  printf("setup\n");

  // 板载 WS2812: 初始化后保持熄灭, 等语音指令
  InitWs2812();

  printf("init button\n");
  const button_config_t btn_cfg = {
      .long_press_time = 1000,
      .short_press_time = 50,
  };

  const button_gpio_config_t gpio_cfg = {
      .gpio_num = kButtonBoot,
      .active_level = 0,
      .enable_power_save = false,
      .disable_pull = false,
  };

  ESP_ERROR_CHECK(iot_button_new_gpio_device(&btn_cfg, &gpio_cfg, &g_button_boot_handle));

  InitDisplay();
  g_display->ShowStatus("初始化");

  // 环境传感器 (AHT20 温湿度 + BMP280 气压, I2C: SDA=IO41 SCL=IO42)
  // 放在 WiFi 之前: I2C 初始化不依赖网络, 失败也只打印日志不阻塞
  InitSensors();

  // 板间串口通讯 (UART1: TX=IO39, RX=IO40), 与另一块 ESP32/S3 点对点相连。
  // 这里只是把 UART 通道开好; 接收在 loop() 里非阻塞轮询(SerialLinkPoll)
  InitSerialLink();
  InitServos();
  // 双直流电机: 电机1(IO1/IO2, PWM 调速) + 电机2(IO47/IO48, 纯开关)
  InitMotors();

  ConfigureWifi();

  InitMcpTools();

#if AUDIO_INPUT_DEVICE_TYPE == AUDIO_INPUT_DEVICE_TYPE_I2S_STD
  auto audio_input_device = std::make_shared<ai_vox::AudioInputDeviceI2sStd>(kMicPinSck, kMicPinWs, kMicPinSd);
#elif AUDIO_INPUT_DEVICE_TYPE == AUDIO_INPUT_DEVICE_TYPE_PDM
  auto audio_input_device = std::make_shared<ai_vox::PdmAudioInputDevice>(kMicPinSck, kMicPinSd);
#endif

  auto& ai_vox_engine = ai_vox::Engine::GetInstance();
  ai_vox_engine.SetObserver(g_observer);
  ai_vox_engine.SetOtaUrl("https://api.tenclass.net/xiaozhi/ota/");
  ai_vox_engine.ConfigWebsocket("wss://api.tenclass.net/xiaozhi/v1/",
                                {
                                    {"Authorization", "Bearer test-token"},
                                });
  printf("engine starting\n");
  g_display->ShowStatus("AI引擎启动中");

  ai_vox_engine.Start(audio_input_device, g_audio_output_device);

  printf("engine started\n");

  ESP_ERROR_CHECK(iot_button_register_cb(
      g_button_boot_handle,
      BUTTON_PRESS_DOWN,
      nullptr,
      [](void* button_handle, void* usr_data) {
        printf("boot button pressed (state=%d)\n", (int)g_chat_state);
        // ★★ 2026-11-02 需求: 聆听/说话状态下按 BOOT → 进入待命状态。
        //   "待命" = 断开语音会话进入 kStandby, 屏幕【顶部状态栏】显示"待命"二字,
        //   不进屏保、不黑屏(用户 2026-11-02 明确纠正: 不是进屏保)。
        //   断链后引擎会发 StateChangedEvent→kStandby, ShowStatus("待命") 自动把
        //   顶部状态栏刷成"待命"。延后 2.5s 是为了等 AI 那句没说完的话落定。
        if (g_chat_state == ai_vox::ChatState::kListening ||
            g_chat_state == ai_vox::ChatState::kSpeaking) {
          printf("[standby] BOOT during listening/speaking -> enter standby\n");
          g_standby_from_boot = true;  // 按键待命: 只断链, 不进屏保
          g_standby_pending = true;
          g_standby_at_ms = millis() + kStandbySuspendDelayMs;
          return;  // 走待命, 不再执行下面的"唤醒+恢复对讲"逻辑
        }
        // ★ 若此刻正排队等"待命断链"(屏保已上、2.5 秒后才断), 按键就是反悔:
        //   撤销它。否则用户按了唤醒键反而会被紧接着的断链踢回离线。
        g_standby_pending = false;
        // ★★ 任何界面按下 BOOT 都回到对话界面(2026-10-23):
        //   监测面板 / 终端数据页 / 屏保 —— 三个全屏覆盖层全部收起。
        //   Hide* 内部自己判空/判激活, 没显示时调用是零开销 no-op。
        //   ★ 监测不会停: 只是收起画面, 监测和语音播报继续在后台跑,
        //     想再看面板说"显示监测面板"(self.monitor.panel)即可。
        if (g_display) {
          g_display->HideMonitorPage();
          g_display->HideTerminalPage();
          g_display->HidePhotoPage();  // 照片页也是全屏覆盖层, 一并收起(没显示时是 no-op)
          g_display->HideEnvPage();    // 环境完整数据页同样收起(没显示时是 no-op)
          // 按键是最明确的人工唤醒: 解锁睡觉锁 + 唤醒屏保, 不走宽限期。
          // ★★ ReleaseSleepLock() 之后必须再显式 WakeUp() 一次(2026-10-23 用户实测踩坑):
          //   屏保有两条进入路径 ——
          //     a) 语音"待机" → EnterScreenSaver() 上睡觉锁 → ReleaseSleepLock 内部会 WakeUp, 正常;
          //     b) 闲置 30 秒自动进 → ScreenSaverTick() 直接置 active, 【没有锁】
          //        → ReleaseSleepLock() 在 !sleep_lock_ 时直接 return, 是空操作!
          //   不补 WakeUp 的现象就是"自动屏保下按 IO0 永远唤不醒"。
          //   WakeUp() 对有锁场景是安全的: 锁已在上一行解除; 无锁场景正好关掉屏保。
          g_display->ReleaseSleepLock();
          g_display->WakeUp();
        }
        // ★★ 对讲恢复必须【按状态】调 Advance, 不能盲目调。踩过的坑:
        //   Advance() 在 kListening(麦克风开着、服务端正在听)分支的语义是
        //   【断开连接=挂断】—— 盲目调的结果就是"按 BOOT 想说话反而把会话挂了",
        //   表现正是"监测状态下无法唤起对讲"。
        //   正确语义: 说话中→打断它回到聆听; 掉线待命→重连; 正在聆听→什么都不用做。
        switch (g_chat_state) {
          case ai_vox::ChatState::kSpeaking:   // AI 播报中 → 打断, 回到聆听
          case ai_vox::ChatState::kStandby:    // 掉线待命 → 重新连接
          case ai_vox::ChatState::kIdle:       // 引擎未跑 → 推进
          case ai_vox::ChatState::kInitted:
          case ai_vox::ChatState::kLoadingFailed:
            ai_vox::Engine::GetInstance().Advance();
            break;
          case ai_vox::ChatState::kListening:  // 麦克风本来就开着, 无需任何动作
          case ai_vox::ChatState::kConnecting:
          case ai_vox::ChatState::kLoading:
          default:
            break;
        }
      },
      nullptr));

  g_display->ShowStatus("AI引擎已启动");

  // ★★ 2026-10-16 残余卡顿根治(第 1 刀): 把音频任务提到高优先级。
  //
  //   真因(有实证的推导链):
  //     ① SDK 的音频解码+写 I2S 跑在独立任务里, 见 src/core/audio_output_engine.cpp:
  //          task_queue_ = new ActiveTaskQueue("AudioOutput", 9<<10, tskIDLE_PRIORITY + 1);
  //        即该任务优先级 = 1。
  //     ② Arduino 的 loopTask 优先级也是 1, 见 cores/esp32/main.cpp:
  //          xTaskCreateUniversal(loopTask, "loopTask", ..., 1, &loopTaskHandle, ...);
  //     ③ 两者【同优先级】→ FreeRTOS 只能靠时间片轮转(tick=1ms)切换, 谁都无法抢占对方。
  //     ④ 而 I2S 输出的缓冲极小, 见 src/audio_device/audio_output_device_i2s_std.h:
  //          .dma_desc_num = 2, .dma_frame_num = 480
  //        24000Hz 单声道 → 每个 desc = 480/24000 = 20ms, 总共只有约 40ms 余量。
  //        AudioOutput 任务必须每 20ms 内被调度到并喂满 DMA, 否则 I2S 下溢 → 播报卡顿。
  //     ⑤ 我方 loop() 单圈里有若干"同步阻塞点"(LED 的 RMT 刷新 led_strip_refresh、
  //        PopEvents 搬整个事件队列、传感器 I2C 等), 一旦累计跑满 20ms 不让步,
  //        音频任务就断供 —— 这就是"偶尔还会卡顿, 但频率已降低"的原因
  //        (上一轮砍掉 printf 是砍掉了大头, 但优先级这个结构性因素还在)。
  //
  //   解法: 把整条音频链路的任务优先级提到 loopTask 之上(设为 5, 高于 loopTask 的 1、
  //     也高于 taskLVGL 的 4)。音频任务一旦有活干就能立即抢占 loop 与 LVGL,
  //     保证 DMA 永不饿死; loop 侧被打断的只是几十 µs 的灯效/界面刷新, 肉眼无感。
  //   ★ 为什么不改 SDK 源码: src/ 是多个示例共享的库代码, 动它会影响别的工程;
  //     在应用侧用 vTaskPrioritySet 按任务名查句柄后提权, 是零侵入做法。
  EnsureAudioTaskPriority();

  // ★ 开机内存基线(2026-10-31): 内部 RAM 是本板最稀缺的资源 —— 摄像头常驻要吃 ~15KB,
  //   而一次 TLS 会话就需要 16KB+4KB 连续块。这行基线是排查"拍照后语音连不回来"的
  //   参照点: 只有知道开机时有多少, 才能算清后面每一步花掉/还回多少。
  printf("[heap] boot baseline: internal free=%u largest=%u | DMA free=%u largest=%u | psram free=%u\n",
         (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
         (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL),
         (unsigned)heap_caps_get_free_size(MALLOC_CAP_DMA),
         (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_DMA),
         (unsigned)heap_caps_get_free_size(MALLOC_CAP_SPIRAM));
}

// ★ 按任务名查找到 SDK 的音频任务并提高其优先级(见 setup 末尾的详细说明)。
//   任务名与各模块 ActiveTaskQueue 的构造名一致, 已 grep 核实:
//   "AudioOutput" / "AudioInput" / "WakeNetFeed" / "WakeNetDetect"。
//   找不到某个任务时只打印一条提示并跳过, 不影响其它功能(SDK 版本变动改任务名也安全)。
void EnsureAudioTaskPriority() {
  // ★ 优先级取值 5:
  //   · 必须高于 loopTask(1) —— 这样音频任务能抢占 loop 里的同步阻塞(本刀的核心)。
  //   · 也要高于 taskLVGL(4) —— 见 display.cpp 的 port_cfg.task_priority = 4。
  //     LVGL 是"软实时"(动画卡一帧无感), 音频是"硬实时"(断一帧就听见咔声),
  //     所以音频必须能抢占 LVGL。二者同优先级只会退化成时间片轮转, 仍可能互相拖累。
  //   · FreeRTOS 最大优先级 configMAX_PRIORITIES 一般为 25, WiFi/TCPIP 等内核任务
  //     在 18~23 一带, 取 5 距离它们很远, 不会干扰网络栈。
  constexpr UBaseType_t kAudioTaskPriority = 5;
  // ★ 用 xTaskGetHandle 按【精确任务名】查句柄, 而不是 uxTaskGetSystemState 枚举。
  //   理由:
  //     ① xTaskGetHandle 只依赖 FreeRTOS 内核本身, 不需开启 configUSE_TRACE_FACILITY;
  //        uxTaskGetSystemState 依赖该开关, 换个 core 版本/配置就可能编译不过。
  //     ② 不需要在栈上开 24 个 TaskStatus_t(每项含指针等, 占用可观)。
  //
  // ★ 一次性把【整条音频链路的任务】都提权, 而不只是输出任务。
  //   这些任务名来自 SDK 各模块里 ActiveTaskQueue 的构造名(已 grep 核实):
  //     · "AudioOutput"   src/core/audio_output_engine.cpp:33  —— 解码 + 写 I2S(播报)
  //     · "AudioInput"    src/core/audio_input_engine.cpp:52   —— 读麦克风(拾音)
  //     · "WakeNetFeed"   src/core/wake_net/wake_net.cpp:76    —— 唤醒词音频喂入
  //     · "WakeNetDetect" src/core/wake_net/wake_net.cpp:77    —— 唤醒词检测
  //   它们全都是 tskIDLE_PRIORITY + 1 = 1, 与 loopTask 同优先级 —— 只要 loop 单圈
  //   跑满 20ms 不让步, 这几条实时链路都会饿死(表现为播报卡顿 / 拾音丢帧 / 唤醒迟钝)。
  //   一并提权才治本; 提升它们不会与 loop 抢资源, 因为提权后它们能抢占 loop。
  const char* kRealtimeTaskNames[] = {"AudioOutput", "AudioInput", "WakeNetFeed", "WakeNetDetect"};
  constexpr int kTaskCount = static_cast<int>(sizeof(kRealtimeTaskNames) / sizeof(kRealtimeTaskNames[0]));
  // ★★ 2026-11-08 改为【幂等】: 同一个任务只提一次权, not-found 提示也只打一次。
  //   原因(实测日志): 开机在 Engine::Start() 之后立刻调用时, AudioOutput / AudioInput
  //   这两条任务还没创建 —— 它们分别在【第一次播报】(StartSpeaking) 和
  //   【开始聆听】(StartListening) 时才 new 出来, 所以那次只提到 WakeNet 两个,
  //   日志就是 "task 'AudioOutput' not found, skip"。
  //   而恰恰这两条是硬实时链路(播报 I2S / 麦克风), 不提权就和 loopTask 同级(1),
  //   loop 卡一圈 20ms 就断音。所以本函数会在状态变化时被反复调用补提,
  //   必须做去重, 否则日志刷屏。
  static uint32_t s_prio_done_mask = 0;    // 已成功提权的位
  static uint32_t s_notfound_logged = 0;   // not-found 已提示过的位
  for (int i = 0; i < kTaskCount; ++i) {
    if (s_prio_done_mask & (1u << i)) {
      continue;  // 已经提过了, 静默跳过
    }
    const char* name = kRealtimeTaskNames[i];
    TaskHandle_t task = xTaskGetHandle(name);
    if (task == nullptr) {
      if (!(s_notfound_logged & (1u << i))) {
        printf("[audio-prio] task '%s' not found (will retry on state change)\n", name);
        s_notfound_logged |= (1u << i);
      }
      continue;
    }
    const UBaseType_t old_prio = uxTaskPriorityGet(task);
    vTaskPrioritySet(task, kAudioTaskPriority);
    s_prio_done_mask |= (1u << i);
    printf("[audio-prio] task '%s' priority: %u -> %u\n",
           name, (unsigned)old_prio, (unsigned)kAudioTaskPriority);
  }
}

void loop() {
#ifdef PRINT_HEAP_INFO_INTERVAL
  static uint32_t s_print_heap_info_time = 0;
  if (s_print_heap_info_time == 0 || millis() - s_print_heap_info_time >= PRINT_HEAP_INFO_INTERVAL) {
    s_print_heap_info_time = millis();
    PrintMemInfo();
  }
#endif

  // 灯效刷新。★ 报警期间(监测发现终端数据超限)红呼吸接管全部常规灯效(2026-10-23):
  //   LedAlarmUpdate 自己带 33ms 节流; 常规灯效此刻必须停 —— 不然 breath/rainbow 会
  //   和红色呼吸交替写同一串灯珠, 颜色乱闪。
  if (g_led_alarm) {
    LedAlarmUpdate();
  } else {
    // 呼吸灯效果刷新(仅在 breath 模式下真正动作, 其它模式直接返回)
    LedBreathUpdate();
    // 彩虹流水灯效果刷新(仅在 rainbow 模式下真正动作)
    LedRainbowUpdate();
    // 单点跑马灯效果刷新(仅在 chase 模式下真正动作)
    LedChaseUpdate();
    // 指定灯的呼吸效果刷新(仅在 custom 模式且 kCustomBreath 打开时真正动作)
    LedPickBreathUpdate();
  }

  // ★ 2026-10-14 二次修正: 【已删除】这里原来的"每 5 秒补发清灯帧"兜底。
  //   删除理由:
  //     · 该兜底是为了对付"上电伪数据残留", 但后来查明故障的真因是我自己在
  //       InitWs2812() 里加的 pinMode() 毛刺(已删除), 不是残留 —— 兜底治错了病。
  //     · 灯处于 off 时本来就该【什么都不做】, 让 IO38 静止。周期性重发帧反而是
  //       一种"持续活动", 在长线上可能被邻近走线耦合, 弊大于利。
  //     · 若真需要清灯, 语音指令 LedSetMode(kOff) 已经会调用 LedShowOff()。

  // ---------------- 语音"待命"的延迟断链 (2026-10-31) ----------------
  // 屏保已经在 MCP 回调里立刻显示了, 这里只负责到点把语音会话真正断开。
  // ★ 用 (int32_t)(millis() - 目标) >= 0 判到期, 无符号回绕也正确。
  // ★ 排在 ScreenSaverTick() 之前: SuspendVoiceLink() 会触发 DISCONNECTED 事件,
  //   引擎任务随后把状态落成 kStandby; 我们这边只关心时机, 顺序无强依赖,
  //   但放在最前面可以保证"说待命"的那一圈就把断链动作排掉。
  if (g_standby_pending && (int32_t)(millis() - g_standby_at_ms) >= 0) {
    g_standby_pending = false;
    const bool from_boot = g_standby_from_boot;
    g_standby_from_boot = false;
    printf("[standby] suspending voice link -> engine standby (press BOOT to wake)\n");
    ai_vox::Engine::GetInstance().SuspendVoiceLink();
    // 断链后引擎发 StateChangedEvent→kStandby, 顶部状态栏自动显示"待命"。
    // ★ 语音"待命"路径: 这 2.5 秒里若被"唤醒"过(语音/按键), 屏保可能已关, 再上一次。
    // ★ BOOT 键待命路径: 用户要的是【顶部显示"待命"】而不是屏保, 不补这一次。
    if (!from_boot) {
      EnterSaverFromVoice();
    }
  }

  // 照片页调度(2026-10-31 改常显): 拍照后照片【一直留在屏幕上】, 不自动关;
  //   直到用户说"返回"(MCP self.display.home)或按 BOOT 键才收起。
  //   PhotoPageTick() 在常显模式下是空的(kPhotoPageAutoHideMs == 0), 保留调用只为
  //   将来改回定时关闭时不用动这里。
  // ★ 与 TerminalPageTick 同理, 必须排在 ScreenSaverTick 之前 —— HidePhotoPage()
  //   内部会 WakeUp() 重置闲置计时, 顺序反了会"照片页一关就变屏保时钟"。
  g_display->PhotoPageTick();
#if ENABLE_USB_CAMERA
  // ★ "收起即删除照片": 照片页收起(说"返回"或按 BOOT)之后, 再等 200ms
  //   把像素缓冲释放掉。等这 200ms 是为了避开 LVGL 渲染任务的最后一帧 ——
  //   收起后对象已是 HIDDEN 不再绘制, 但要给渲染线程留一轮时间落定才最安全。
  {
    static bool s_photo_was_active = false;
    static uint32_t s_photo_hidden_ms = 0;
    if (g_display->IsPhotoPageActive()) {
      s_photo_was_active = true;
      s_photo_hidden_ms = 0;
    } else if (s_photo_was_active) {
      if (s_photo_hidden_ms == 0) {
        s_photo_hidden_ms = millis();
      } else if (millis() - s_photo_hidden_ms >= 400) {
        CamReleasePhotoBuffers();  // 内部判空, 没有照片时是 no-op
        s_photo_was_active = false;
        s_photo_hidden_ms = 0;
      }
    }
  }
#endif

  // 终端数据页超时调度(2026-10-20 新增): 显示满 30 秒自动关掉, 回到对话界面。
  // ★★ 必须排在 ScreenSaverTick() 【之前】。原因: 数据页与屏保都占满全屏,
  //   而"数据页显示 30 秒"和"闲置 30 秒"是同一时刻到期的。
  //   先关数据页 → HideTerminalPage() 内部 WakeUp() 重置闲置计时 →
  //   紧接着运行的 ScreenSaverTick() 看到 idle≈0, 就不会把屏保又盖上来。
  //   顺序反了的话, 用户看到的是"数据页变成屏保时钟", 而不是回到对话界面。
  // 非显示态只做一次 bool 判断, 零开销。
  g_display->TerminalPageTick();

  // 屏保调度: 闲置满 30s 自动显示时钟; 期间也会驱动时钟每秒走字
  //   - 内部会判断 kind_==kChat 与是否已进入屏保, 非聊天屏/非屏保态几乎零开销
  //   - 唤醒靠 ShowStatus/SetChatMessage/SetEmotion 内部的 WakeUp(), 无需在此写事件分支
  g_display->ScreenSaverTick();

  // 环境传感器轮询: 内部 3 秒节流, 不到点直接返回(几乎零开销)
  SensorPoll();

  // 板间串口接收轮询: 非阻塞(while available 收字节), 有数据才处理, 空闲时几乎零开销
  SerialLinkPoll();
  // 终端在线检测(2026-10-23 新增): 必须在 SerialLinkPoll 之后 —— 它消费本轮刚更新的
  // g_link_last_rx_ms, 负责离线判定与"上线确认超时"复位。上线播报本身在 LinkCommitLine 里触发。
  LinkPresenceTick();
  // 终端灯握手推进: 必须在 SerialLinkPoll 之后 —— 它要读本轮刚收到的回执。
  // 非阻塞: 只做字符串比对和超时判定, 没在等回执时开销仅一次 bool 判断。
  TerminalHandshakePoll();
  // 终端 OLED 显示回执 + 超时判定(2026-10-17 新增), 同样必须在 SerialLinkPoll 之后
  TerminalOledPoll();
  TerminalOledTimeoutCheck();
  // 终端温湿度上报解析: 把最新 ENV,... 行写进 g_terminal_env 缓存
  TerminalEnvPoll();
  // 终端全量数据上报解析(2026-10-20 新增): 把最新 ALL,... 行喂给屏幕数据页与 MCP 缓存。
  // 非 ALL 帧时只做一次前缀比较, 几乎零开销。
  TerminalAllPoll();
  // 作物标准下发回执(2026-10-20 新增): 把"终端已记住/没回应"显示到状态栏。
  // 必须在 SerialLinkPoll 之后 —— 它要读本轮刚收到的回执。
  CropAckPoll();
  // 终端执行器回执(2026-10-21 新增): 把 ACT_OK 的实际状态显示到状态栏+对话区。
  ActAckPoll();
  // 自动监测状态机(2026-10-23 新增): 必须在 TerminalAllPoll/ActAckPoll 之后 ——
  //   它读的是本轮刚更新的 ALL 缓存。内部 2 秒节流 + 全程非阻塞, 未启动时零开销。
  MonitorTick();
  ServoPoll();
  // 定时任务轮询(2026-11-08 新增, 借鉴 mimiclaw): 内部判断到期, 未到点几乎零开销
  CronPoll();
  // 智谱兜底判定(2026-11-08 新增): 官方没理解模糊指令时, 交给智谱 ReAct 自主控硬件
  if (g_glm_recheck_ms != 0 && (int32_t)(millis() - g_glm_recheck_ms) >= 0) {
    g_glm_recheck_ms = 0;
    if (!g_official_handled_hw && g_last_user_text[0] != '\0' &&
        g_chat_state != ai_vox::ChatState::kSpeaking &&
        g_chat_state != ai_vox::ChatState::kListening) {
      char buf[256] = {0};
      snprintf(buf, sizeof(buf), "%s", g_last_user_text);
      g_last_user_text[0] = '\0';  // 只兜底一次, 防止循环重试
      GlmReActLoop(buf);
    }
  }

  // 把最新环境读数喂给屏保面板的温湿度行(2026-10-14 新增)。
  // 放在 SensorPoll 之后: 保证读到的是本轮最新值。
  // 内部会做"变化才刷"判断, 数据没变时零开销; 也不会 WakeUp 把用户踢出屏保。
  if (g_display) {
    g_display->SetEnvForScreenSaver(g_env_temp_c, g_env_hum, g_env_valid);
  }

  auto& engine = ai_vox::Engine::GetInstance();

  const auto events = g_observer->PopEvents();

  // ★ 语音链路耗时打点: 用于定位"说完话到舵机动"的延时到底在哪一段。
  //   看串口日志: [T+...ms] 前缀就是每个事件到达的绝对毫秒数, 相减即得各段耗时。
  //   预期耗时分布: kUser(识别出你说的话) -> MCP tools/call(云端模型决定调工具) 这段最长,
  //   它就是"大模型思考"的时间, 改不了; 之后 MCP 事件到舵机动作是同一函数内同步执行, 耗时 ~0ms。
  const uint32_t evt_now_ms = millis();

  // ★★ 2026-10-15 卡顿优化 —— 这是头号真凶, 说明如下:
  //   TextReceivedEvent 的 content 是 WebSocket 【每一个文本帧】的完整 JSON 原文。
  //   (SDK 源码: OnWebsocketEvent -> kWebsocketTextFrame -> PushEvent(TextReceivedEvent{整帧}))
  //   一次对话里服务端会下发【几十~上百个】JSON 帧:
  //   唤醒词确认 / STT 中间结果 / LLM 文本流(每句或每 chunk 一帧) / TTS 音频元数据 /
  //   MCP 工具调用 / 情感标记 / 结束帧 ...
  //   原先这里【每一帧都完整 printf】出去, 单帧几百字节~几 KB, 一次对话累计几万字节。
  //   串口输出是同步阻塞的: 115200bps 满打满算每秒约 11KB, 这几万字节意味着
  //   对话过程中 loop() 被反复卡住几十毫秒 —— 表现就是"一说话就卡顿"。
  //   改为默认关闭。需要抓协议原文排查问题时, 把下面的 false 改成 true 重新编译即可。
  static constexpr bool kLogWebSocketFrames = false;

  // 日志截断: 超过 kLogMaxChars 个字符就只留前段 + "...(N chars omitted)"。
  // ★ 目的: 串口 printf 是同步阻塞的(115200bps ≈ 11.5 字节/毫秒),
  //   一条几百字的 AI 回复整段打出去就要阻塞几十毫秒, 会打断音频与灯效时序。
  //   日志只需能辨识内容, 不必全文 —— 所以统一截断。
  static constexpr size_t kLogMaxChars = 160;
  auto trunc = [](const std::string& s) -> std::string {
    if (s.size() <= kLogMaxChars) {
      return s;
    }
    char tail[32] = {0};
    snprintf(tail, sizeof(tail), "...(+%u chars)", (unsigned)(s.size() - kLogMaxChars));
    return s.substr(0, kLogMaxChars) + tail;
  };

  for (auto& event : events) {
    if (auto text_received_event = std::get_if<ai_vox::TextReceivedEvent>(&event)) {
      // (void) 显式引用一次: 避免 kLogWebSocketFrames=false 时该变量在部分
      // 编译路径下被认为"未使用"(工程开了 -Werror, 未使用变量会直接报错)
      (void)text_received_event;
      if (kLogWebSocketFrames) {
        printf("[T+%lums] on text received: %s\n", (unsigned long)evt_now_ms,
               text_received_event->content.c_str());
      }
    } else if (auto activation_event = std::get_if<ai_vox::ActivationEvent>(&event)) {
      printf("[T+%lums] activation code: %s, message: %s\n", (unsigned long)evt_now_ms,
             activation_event->code.c_str(), activation_event->message.c_str());
      g_display->ShowStatus("激活设备");
      g_display->SetChatMessage(Display::Role::kSystem, activation_event->message);
    } else if (auto state_changed_event = std::get_if<ai_vox::StateChangedEvent>(&event)) {
      // ★ 记录当前会话状态: BOOT 键按状态决定是否调 Advance、监测播报的状态门卫都靠它
      g_chat_state = state_changed_event->new_state;
      switch (state_changed_event->new_state) {
        case ai_vox::ChatState::kIdle: {
          printf("Idle\n");
          break;
        }
        case ai_vox::ChatState::kInitted: {
          printf("Initted\n");
          g_display->ShowStatus("初始化完成");
          break;
        }
        case ai_vox::ChatState::kLoading: {
          printf("Loading...\n");
          g_display->ShowStatus("加载协议中");
          break;
        }
        case ai_vox::ChatState::kLoadingFailed: {
          printf("Loading failed, please retry\n");
          g_display->ShowStatus("加载协议失败，请重试");
          break;
        }
        case ai_vox::ChatState::kStandby: {
          printf("Standby\n");
          g_display->ShowStatus("待命");
          break;
        }
        case ai_vox::ChatState::kConnecting: {
          printf("Connecting...\n");
          g_display->ShowStatus("连接中...");
          break;
        }
        case ai_vox::ChatState::kListening: {
          printf("Listening...\n");
          // ★★ 注意: 这里【不解除】语音睡觉锁。
          //   踩过的坑(2026-10-14): 最初在这里调 ReleaseSleepLock(), 结果"待机"功能没用 ——
          //   因为本固件是【连续对话】模式, AI 说完话后 AFE 会自动重新进入 kListening,
          //   时间点在"待机工具调用"之后约 1 秒 → 锁刚上就被自己解开 → 屏保立刻被后续
          //   的 SetChatMessage/ShowStatus 事件关掉。用户看到的就是"显示了一下又唤醒"。
          //   kListening 只表示"麦克风开了", 不表示"用户说了新话", 不能作为解锁依据。
          //   正确的解锁点是【用户真的说出新的一句话】(ChatMessageEvent/kUser), 见下方。
          g_display->ShowStatus("聆听中");
          // ★ 补音频提权(幂等): "AudioInput" 任务是 StartListening() 里才 new 的,
          //   开机那次提权抓不到它。此刻它一定已经存在, 正好补上。
          EnsureAudioTaskPriority();
          break;
        }
        case ai_vox::ChatState::kSpeaking: {
          printf("Speaking...\n");
          g_display->ShowStatus("说话中");
          // ★ 补音频提权(幂等): "AudioOutput" 任务是 StartSpeaking() 里才 new 的,
          //   这是它第一次出现的时机, 不提权就会和 loopTask 同级(1) → 播报断音。
          EnsureAudioTaskPriority();
          break;
        }
        default: {
          break;
        }
      }
    } else if (auto emotion_event = std::get_if<ai_vox::EmotionEvent>(&event)) {
      printf("emotion: %s\n", emotion_event->emotion.c_str());
      g_display->SetEmotion(emotion_event->emotion);
    } else if (auto chat_message_event = std::get_if<ai_vox::ChatMessageEvent>(&event)) {
      switch (chat_message_event->role) {
        case ai_vox::ChatRole::kAssistant: {
          // ★ 截断后再打印: AI 回复可能几百字, 整段 printf 会阻塞串口几十毫秒。
          //   日志只需能辨识内容, 不必全文。
          printf("[T+%lums] role: assistant, content: %s\n", (unsigned long)evt_now_ms,
                 trunc(chat_message_event->content).c_str());
          g_display->SetChatMessage(Display::Role::kAssistant, chat_message_event->content);
          // ★★ 智谱兜底判定(2026-11-08): AI 回复到达 = 官方这轮已答完。
          //   若本轮官方【没调任何硬件工具】且用户原话还在, 说明可能是模糊指令没被理解,
          //   交给智谱 ReAct 兜底。延迟到 loop 里执行(避免在事件回调里做阻塞 HTTP)。
          if (!g_official_handled_hw && g_last_user_text[0] != '\0') {
            g_glm_recheck_ms = millis() + 300;  // 300ms 后再判, 给 MCP 事件留出到达窗口
          }
          break;
        }
        case ai_vox::ChatRole::kUser: {
          printf("[T+%lums] role: user, content: %s\n", (unsigned long)evt_now_ms,
                 trunc(chat_message_event->content).c_str());
          // ★★ 记录用户原话(智谱兜底用) + 重置"官方是否调了硬件"标记。
          snprintf(g_last_user_text, sizeof(g_last_user_text), "%s", chat_message_event->content.c_str());
          g_official_handled_hw = false;
          g_glm_recheck_ms = 0;
          // ★★ 语音睡觉锁的正确解锁点(2026-10-14)。
          //   这是"用户真的说出了新的一句话"的可靠信号 —— 覆盖了所有情况:
          //     · 待机后重新叫"小智小智"+ 说新话
          //     · 待机后直接说"亮屏"/"唤醒"(这句话会先到这里, 再触发 wake 工具)
          //   时序保证: 发出"待机"这个指令的那句用户语音, 到达时间【早于】工具调用,
          //     所以此刻锁还没上, 不会误放行"待机"自身。若万一乱序晚到,
          //     CanReleaseSleepLock() 的宽限期会挡住。
          if (g_display && g_display->CanReleaseSleepLock()) {
            printf("[sleep-lock] user spoke again -> releasing\n");
            g_display->ReleaseSleepLock();
          }
          g_display->SetChatMessage(Display::Role::kUser, chat_message_event->content.c_str());
          break;
        }
      }
    } else if (auto mcp_tool_call_event = std::get_if<ai_vox::McpToolCallEvent>(&event)) {
      // ToString() 会把工具名和所有参数拼成完整字符串(可能很长), 同样截断
      printf("[T+%lums] on mcp tool call: %s\n", (unsigned long)evt_now_ms,
             trunc(mcp_tool_call_event->ToString()).c_str());

      // ★★ 智谱兜底标记(2026-11-08): 官方若调了任何硬件工具, 说明它已理解用户意图,
      //   本轮就不需要智谱兜底了。硬件工具名见下。
      if (mcp_tool_call_event->name == "self.servo.set" ||
          mcp_tool_call_event->name == "self.servo.selftest" ||
          mcp_tool_call_event->name == "self.motor1.set" ||
          mcp_tool_call_event->name == "self.motor2.set" ||
          mcp_tool_call_event->name == "self.led.set" ||
          mcp_tool_call_event->name == "self.led.pick" ||
          mcp_tool_call_event->name == "self.act.set" ||
          mcp_tool_call_event->name == "self.shade.forward" ||
          mcp_tool_call_event->name == "self.shade.reset") {
        g_official_handled_hw = true;
        g_glm_recheck_ms = 0;
      }

      if ("self.audio_speaker.set_volume" == mcp_tool_call_event->name) {
        const auto volume_ptr = mcp_tool_call_event->param<int64_t>("volume");
        if (volume_ptr != nullptr) {
          printf("on mcp tool call: self.audio_speaker.set_volume, volume: %" PRId64 "\n", *volume_ptr);
          g_audio_output_device->set_volume(*volume_ptr);
          engine.SendMcpCallResponse(mcp_tool_call_event->id, true);
        } else {
          engine.SendMcpCallError(mcp_tool_call_event->id, "Missing valid argument: volume");
        }
      } else if ("self.audio_speaker.get_volume" == mcp_tool_call_event->name) {
        const auto volume = g_audio_output_device->volume();
        printf("on mcp tool call: self.audio_speaker.get_volume, volume: %" PRIu16 "\n", volume);
        engine.SendMcpCallResponse(mcp_tool_call_event->id, volume);
      } else if ("self.led.set" == mcp_tool_call_event->name) {
        const auto mode_ptr = mcp_tool_call_event->param<std::string>("mode");
        if (mode_ptr != nullptr) {
          printf("on mcp tool call: self.led.set, mode: %s\n", mode_ptr->c_str());
          if (*mode_ptr == "white") {
            LedSetMode(LedMode::kWhite);
          } else if (*mode_ptr == "breath") {
            LedSetMode(LedMode::kBreath);
          } else if (*mode_ptr == "rainbow") {
            LedSetMode(LedMode::kRainbow);
          } else if (*mode_ptr == "chase") {
            LedSetMode(LedMode::kChase);
          } else if (*mode_ptr == "off") {
            LedSetMode(LedMode::kOff);
          } else {
            printf("[LED] unknown mode: %s, fallback to white\n", mode_ptr->c_str());
            LedSetMode(LedMode::kWhite);
          }
          engine.SendMcpCallResponse(mcp_tool_call_event->id, true);
        } else {
          engine.SendMcpCallError(mcp_tool_call_event->id, "Missing valid argument: mode");
        }
      } else if ("self.led.pick" == mcp_tool_call_event->name) {
        const auto leds_ptr = mcp_tool_call_event->param<std::string>("leds");
        if (leds_ptr == nullptr) {
          engine.SendMcpCallError(mcp_tool_call_event->id, "Missing valid argument: leds");
        } else {
          // 解析 leds 字符串, 提取其中的 1~8 灯号, 支持四种写法混用:
          //   ① ASCII 阿拉伯数字   "3,6,8" / "3号6号"
          //   ② 全角阿拉伯数字     "３号"
          //   ③ 中文数字           "三号" / "六" / "第八个"
          //   ④ 任意分隔符/无关汉字 逗号、空格、"号"、"灯"、"个" —— 一律跳过
          // ★ 为什么要兼容中文/全角: 语音转写或大模型可能把"三号"原样塞进 leds,
          //   只认 ASCII 的话会解析成 mask=0(灯不亮), 表现为"说了几号灯但灯没反应"。
          uint8_t mask = 0x00;
          const char* p = leds_ptr->c_str();
          auto apply_num = [&mask](int num) {
            if (num >= 1 && num <= static_cast<int>(kWs2812Count)) {
              mask |= static_cast<uint8_t>(1u << (num - 1));
            } else if (num != 0) {
              printf("[LED] pick: ignore out-of-range led number %d\n", num);
            }
          };
          while (*p != '\0') {
            const unsigned char c = static_cast<unsigned char>(*p);
            if (c >= '0' && c <= '9') {
              // ① ASCII 数字: 连续读取成多位数
              int num = 0;
              while (*p >= '0' && *p <= '9') {
                num = num * 10 + (*p - '0');
                ++p;
                if (num > 999) {  // 防超长数字溢出, 与越界同样处理
                  break;
                }
              }
              apply_num(num);
            } else if (c == 0xEF && p[1] != '\0') {
              // ② 全角数字 U+FF10~U+FF19 = EF BC 90 ~ EF BC 99
              //    简化处理: 全角数字第三字节固定落在 0x90~0x99
              const unsigned char c3 = static_cast<unsigned char>(p[2]);
              if (c3 >= 0x90 && c3 <= 0x99) {
                apply_num(c3 - 0x90);
                p += 3;
                continue;
              }
              ++p;
            } else if (c == 0xE4 || c == 0xE5 || c == 0xE6 || c == 0xE7 || c == 0xE8 || c == 0xE9) {
              // ③ 中文数字: 一~十 的 UTF-8 编码
              //    一 E4B880 / 二 E4BA8C / 三 E4B889 / 四 E59B9B / 五 E4BA94
              //    六 E585AD / 七 E4B883 / 八 E585AB / 九 E4B99D / 十 E58D81
              static const struct { const char* utf8; int num; } kCnNum[] = {
                  {"一", 1}, {"二", 2}, {"三", 3}, {"四", 4}, {"五", 5},
                  {"六", 6}, {"七", 7}, {"八", 8}, {"九", 9}, {"十", 10},
              };
              bool matched = false;
              for (const auto& item : kCnNum) {
                if (strncmp(p, item.utf8, 3) == 0) {
                  apply_num(item.num);
                  p += 3;
                  matched = true;
                  break;
                }
              }
              if (!matched) {
                ++p;  // 不是中文数字(可能是"号"、"灯"等), 跳过这个字节
              }
            } else {
              ++p;  // 逗号/空格/其他无关字符
            }
          }
          // ---- 灯号/位图更新策略 ----
          // ★★ 2026-10-15 关键修正: 区分"整体替换"和"逐灯累加"两种语义。
          //   问题: AI 有时会把"第一个变红 第二个变绿"拆成两次调用(leds='1'、leds='2')。
          //     若第二次仍然整体替换 mask, 第一次点的第1颗就被【熄灭】了 ——
          //     这正是用户报的"第一个灯不亮"。
          //   解决: 用 pairs 逐灯指定颜色时, 视为【累加】(点名的灯并入现有 mask,
          //     未点名的保持原状态); 不用 pairs 时, 保持原来的【整体替换】语义
          //     (这样"打开第3个灯"仍然只亮第3颗, 符合直觉)。
          const auto pairs_ptr = mcp_tool_call_event->param<std::string>("pairs");
          const auto color_ptr = mcp_tool_call_event->param<std::string>("color");
          const bool has_pairs = (pairs_ptr != nullptr && !pairs_ptr->empty());
          if (has_pairs) {
            g_led_pick_mask |= mask;  // 累加: 把这次点名的灯"加"进来
          } else {
            g_led_pick_mask = mask;   // 替换: 整批重设
          }

          // ---- 颜色处理 ----
          // 两种模式:
          //   A. pairs = "1:red,2:green" —— 【逐灯指定颜色】, 每颗灯自己的色不一样(推荐)
          //   B. color = "green"        —— 全部被点名的灯统一一个色(向后兼容)
          // 优先级: 同时给了就 pairs 生效(因为它表达力更强)。
          // ★★ 为什么要有 pairs: "让第一变红 第二变绿" 这种一句多色的需求, 单一 color 表达不了。
          bool colored = false;
          if (has_pairs) {
            // 解析 "1:red,2:green,5:blue" 这种列表: 按 ',' 切段, 每段是 "灯号:颜色名"
            const std::string& spec = *pairs_ptr;
            // 特殊情况: 整串里一个冒号都没有 → 说明只给了颜色没给灯号, 当"全链统一色"
            // ★★ 2026-10-16 修正: 全角冒号"："是 UTF-8 三字节字符(U+FF1A = EF BC 9A),
            //   【绝不能写成单引号字符常量 '：'】—— 那是"多字符常量",
            //   在 -Wall -Werror 下会报 `multi-character character constant [-Werror=multichar]`
            //   直接编译失败; 即便侥幸编过, 也会被截断成单个字节去匹配, 逻辑是错的。
            //   正确做法: 用双引号字符串字面量, 走 std::string::find(const char*) 重载。
            if (spec.find(':') == std::string::npos && spec.find("：") == std::string::npos) {
              LedPickSetAllColor(spec.c_str());
              colored = true;
            } else {
              size_t pos = 0;
              while (pos < spec.size()) {
                size_t comma = spec.find(',', pos);
                std::string seg = spec.substr(pos, (comma == std::string::npos) ? std::string::npos : comma - pos);
                size_t colon = seg.find(':');
                if (colon == std::string::npos) {
                  // ★ 兼容全角冒号: 用字符串字面量, 不能用 '：'(多字符常量, 见上方说明)
                  colon = seg.find("：");
                }
                if (colon != std::string::npos) {
                  // 灯号部分: 提取冒号前的数字(兼容 "1" / "1号" / "第1个")
                  int led_no = -1;
                  for (size_t k = 0; k < colon; ++k) {
                    const char ch = seg[k];
                    if (ch >= '0' && ch <= '9') {
                      led_no = (led_no < 0 ? 0 : led_no) * 10 + (ch - '0');
                    } else if (led_no >= 0) {
                      break;  // 数字读完了
                    }
                  }
                  // 颜色部分: 取冒号之后的非空白内容
                  std::string color_name = seg.substr(colon + 1);
                  size_t b = color_name.find_first_not_of(" \t");
                  size_t e = color_name.find_last_not_of(" \t");
                  if (b != std::string::npos && e != std::string::npos) {
                    color_name = color_name.substr(b, e - b + 1);
                  }
                  if (led_no >= 1 && led_no <= static_cast<int>(kWs2812Count)) {
                    LedPickSetOneColor(led_no - 1, color_name.c_str());
                    colored = true;
                  } else {
                    printf("[LED] pick: pairs segment '%s' has invalid led number\n", seg.c_str());
                  }
                } else {
                  printf("[LED] pick: pairs segment '%s' missing ':'\n", seg.c_str());
                }
                if (comma == std::string::npos) {
                  break;
                }
                pos = comma + 1;
              }
            }
          } else if (color_ptr != nullptr) {
            LedPickSetAllColor(color_ptr->c_str());
            colored = true;
          }
          if (!colored) {
            // 完全没说颜色: 被点亮的灯保持各自原有的颜色(首次点亮默认是绿色, 见 g_led_pick_hue 初值)
            printf("[LED] pick: no color given, keep per-led default\n");
          }

          LedSetMode(LedMode::kCustom);

          // 回执文本: 列出【当前实际亮着】的全部灯号(用 g_led_pick_mask 而非 mask,
          // 因为 pairs 模式下 mask 只是本次增量, 累加后的才是真实状态)
          char ack[200] = {0};
          if (g_led_pick_mask == 0x00) {
            snprintf(ack, sizeof(ack), "已关闭全部灯");
          } else {
            int written = snprintf(ack, sizeof(ack), "已点亮第");
            for (uint32_t i = 0; i < kWs2812Count && written > 0 && written < (int)sizeof(ack); ++i) {
              if ((g_led_pick_mask >> i) & 0x01) {
                written += snprintf(ack + written, sizeof(ack) - written, "%u ", (unsigned)(i + 1));
              }
            }
            snprintf(ack + (written < (int)sizeof(ack) ? written : (int)sizeof(ack) - 1),
                     sizeof(ack) - (written < (int)sizeof(ack) ? written : (int)sizeof(ack) - 1), "颗灯");
          }
          printf("[T+%lums] on mcp tool call: self.led.pick, leds='%s' color='%s' pairs='%s'"
                 " -> this_time_mask=0x%02X all_mask=0x%02X\n",
                 (unsigned long)millis(), leds_ptr->c_str(),
                 color_ptr != nullptr ? color_ptr->c_str() : "(none)",
                 pairs_ptr != nullptr ? pairs_ptr->c_str() : "(none)",
                 (unsigned)mask, (unsigned)g_led_pick_mask);
          engine.SendMcpCallResponse(mcp_tool_call_event->id, std::string(ack));
        }
      } else if ("self.led.get" == mcp_tool_call_event->name) {
        printf("on mcp tool call: self.led.get, mode: %s\n", LedModeName(g_led_mode));
        engine.SendMcpCallResponse(mcp_tool_call_event->id, std::string(LedModeName(g_led_mode)));
      } else if ("self.sensor.get_environment" == mcp_tool_call_event->name) {
        // 环境传感器综合查询: 返回最近一次轮询的缓存(最多 3 秒旧), 无需现场重读
        // (AHT20 一次测量要 ~80ms, 在 MCP 回调里现场读会拖慢响应)
        if (!g_env_valid) {
          engine.SendMcpCallError(mcp_tool_call_event->id, "环境传感器未连接或尚未完成首次读数");
        } else {
          char env_buf[96] = {0};
          if (g_aht_ok && g_bmp_ok) {
            snprintf(env_buf, sizeof(env_buf), "温度 %.1f°C, 湿度 %.0f%%, 气压 %.1f hPa",
                     g_env_temp_c, g_env_hum, g_env_press_hpa);
          } else if (g_aht_ok) {
            snprintf(env_buf, sizeof(env_buf), "温度 %.1f°C, 湿度 %.0f%%", g_env_temp_c, g_env_hum);
          } else {
            snprintf(env_buf, sizeof(env_buf), "气压 %.1f hPa", g_env_press_hpa);
          }
          printf("on mcp tool call: self.sensor.get_environment -> %s\n", env_buf);
          engine.SendMcpCallResponse(mcp_tool_call_event->id, std::string(env_buf));
        }
      } else if ("self.sensor.get_temperature" == mcp_tool_call_event->name) {
        // 单项: 温度 (读缓存)
        if (!g_env_valid || !g_aht_ok) {
          engine.SendMcpCallError(mcp_tool_call_event->id, "温度传感器未连接或尚未完成首次读数");
        } else {
          char buf[48] = {0};
          snprintf(buf, sizeof(buf), "%.1f 摄氏度", g_env_temp_c);
          printf("on mcp tool call: self.sensor.get_temperature -> %s\n", buf);
          engine.SendMcpCallResponse(mcp_tool_call_event->id, std::string(buf));
        }
      } else if ("self.sensor.get_humidity" == mcp_tool_call_event->name) {
        // 单项: 湿度 (读缓存)
        if (!g_env_valid || !g_aht_ok) {
          engine.SendMcpCallError(mcp_tool_call_event->id, "湿度传感器未连接或尚未完成首次读数");
        } else {
          char buf[48] = {0};
          snprintf(buf, sizeof(buf), "%.0f%%", g_env_hum);
          printf("on mcp tool call: self.sensor.get_humidity -> %s\n", buf);
          engine.SendMcpCallResponse(mcp_tool_call_event->id, std::string(buf));
        }
      } else if ("self.sensor.get_pressure" == mcp_tool_call_event->name) {
        // 单项: 气压 (读缓存)
        if (!g_env_valid || !g_bmp_ok) {
          engine.SendMcpCallError(mcp_tool_call_event->id, "气压传感器未连接或尚未完成首次读数");
        } else {
          char buf[48] = {0};
          snprintf(buf, sizeof(buf), "%.1f 百帕", g_env_press_hpa);
          printf("on mcp tool call: self.sensor.get_pressure -> %s\n", buf);
          engine.SendMcpCallResponse(mcp_tool_call_event->id, std::string(buf));
        }
      } else if ("self.sensor.get_soil" == mcp_tool_call_event->name) {
        // ★ 2026-11-06 新增: 土壤温湿度 (SHT40, 读缓存)
        if (!g_soil_valid || !g_sht_ok) {
          engine.SendMcpCallError(mcp_tool_call_event->id, "土壤传感器未连接或尚未完成首次读数");
        } else {
          char buf[96] = {0};
          snprintf(buf, sizeof(buf), "土壤温度 %.1f°C, 土壤湿度 %.0f%%", g_soil_temp_c, g_soil_hum);
          printf("on mcp tool call: self.sensor.get_soil -> %s\n", buf);
          engine.SendMcpCallResponse(mcp_tool_call_event->id, std::string(buf));
        }
      } else if ("self.sensor.get_all" == mcp_tool_call_event->name) {
        // ★ 2026-11-06 新增: 完整数据(空气温湿度气压 + 土壤温湿度)。
        //   ★★ 用户要求: 听到"显示完整数据"时【列表显示】在屏幕上。
        //   手段: 调 ShowEnvPage 整屏列表显示 5 项数据; 同时返回云端让 AI 念出来。
        //   数据都是缓存, 不在回调里现场读(不拖慢响应)。
        char buf[160] = {0};
        int used = 0;
        // 空气部分
        if (g_aht_ok || g_bmp_ok) {
          char air[96] = {0};
          if (g_aht_ok && g_bmp_ok) {
            snprintf(air, sizeof(air), "空气温度 %.1f°C 湿度 %.0f%% 气压 %.1f hPa",
                     g_env_temp_c, g_env_hum, g_env_press_hpa);
          } else if (g_aht_ok) {
            snprintf(air, sizeof(air), "空气温度 %.1f°C 湿度 %.0f%%", g_env_temp_c, g_env_hum);
          } else {
            snprintf(air, sizeof(air), "气压 %.1f hPa", g_env_press_hpa);
          }
          used = snprintf(buf, sizeof(buf), "%s", air);
        }
        // 土壤部分
        if (g_sht_ok) {
          if (g_soil_valid) {
            used += snprintf(buf + used, sizeof(buf) - used, "%s土壤温度 %.1f°C 湿度 %.0f%%",
                             used > 0 ? ", " : "", g_soil_temp_c, g_soil_hum);
          } else {
            used += snprintf(buf + used, sizeof(buf) - used, "%s土壤传感器尚未读数",
                             used > 0 ? ", " : "");
          }
        }
        if (used == 0) {
          snprintf(buf, sizeof(buf), "环境传感器未连接");
        }
        printf("on mcp tool call: self.sensor.get_all -> %s\n", buf);

        // ★ 列表显示到屏幕: 每项一行(名称 + 数值)
        const char* lines[Display::kEnvItemCount];
        int line_count = 0;
        // 用固定长度缓冲拼每行, 避免栈上太多小数组
        static char line0[40], line1[40], line2[40], line3[40], line4[40];
        if (g_aht_ok) {
          snprintf(line0, sizeof(line0), "空气温度   %.1f°C", g_env_temp_c);
          snprintf(line1, sizeof(line1), "空气湿度   %.0f%%", g_env_hum);
          lines[line_count++] = line0;
          lines[line_count++] = line1;
        }
        if (g_bmp_ok) {
          snprintf(line2, sizeof(line2), "气压       %.1f hPa", g_env_press_hpa);
          lines[line_count++] = line2;
        }
        if (g_sht_ok && g_soil_valid) {
          snprintf(line3, sizeof(line3), "土壤温度   %.1f°C", g_soil_temp_c);
          snprintf(line4, sizeof(line4), "土壤湿度   %.0f%%", g_soil_hum);
          lines[line_count++] = line3;
          lines[line_count++] = line4;
        }
        if (line_count > 0 && g_display) {
          g_display->ShowEnvPage("环境完整数据", lines, line_count);
        }

        // ★ 同时返回云端让 AI 把完整数据念给用户听
        engine.SendMcpCallResponse(mcp_tool_call_event->id, std::string(buf));
      } else if ("self.display.sleep" == mcp_tool_call_event->name) {
        // 语音进屏保: 立刻显示大时钟屏保, 并上"睡觉锁"防止 AI 随后的回复语音把它吵醒。
        // ★ 先发响应再进屏保: SendMcpCallResponse 只是把结果交回云端, 不阻塞;
        //   进屏保是纯本地 UI 动作。顺序无强依赖, 但先回响应语义更清晰。
        printf("on mcp tool call: self.display.sleep -> entering screen saver\n");
        engine.SendMcpCallResponse(mcp_tool_call_event->id,
                                   std::string("好的, 屏幕关上了, 说声'唤醒'就回来"));
        EnterSaverFromVoice();  // 先收覆盖层再上屏保, 否则照片页会把屏保挡回去
      } else if ("self.display.standby" == mcp_tool_call_event->name) {
        // 真·待命: 屏保立刻上(用户马上看到反应), 语音会话延后断开(等 AI 把确认话说完)。
        printf("on mcp tool call: self.display.standby -> screen saver + pending suspend\n");
        if (g_mon_active) {
          // ★ 监测在跑时不真断链: 报警要靠 engine.SendText 播报, 链路一断所有报警
          //   都变成哑的(用户以为在监控, 其实什么都不会喊)。只关屏保, 如实说明。
          EnterSaverFromVoice();
          engine.SendMcpCallResponse(
              mcp_tool_call_event->id,
              std::string("监测还在跑, 只关了屏幕没有断开连接(断了报警就播不出来了)。"
                          "说声'唤醒'就回来。"));
        } else {
          EnterSaverFromVoice();
          engine.SendMcpCallResponse(mcp_tool_call_event->id,
                                     std::string("好的, 已进入待命, 按唤醒键再叫我"));
          g_standby_from_boot = false;  // 语音待命: 屏保已上, 断链时仍补屏保
          g_standby_pending = true;
          g_standby_at_ms = millis() + kStandbySuspendDelayMs;
        }
      } else if ("self.display.wake" == mcp_tool_call_event->name) {
        // 语音出屏保: 解除睡觉锁并唤醒(正常界面)。
        printf("on mcp tool call: self.display.wake -> waking up\n");
        g_standby_pending = false;  // 用户已经在叫醒, 撤销还排着队的"待命断链"
        engine.SendMcpCallResponse(mcp_tool_call_event->id, std::string("已唤醒屏幕"));
        if (g_display) {
          // ★ 同 BOOT 键: ReleaseSleepLock() 在无锁(闲置自动屏保)时是空操作,
          //   必须再显式 WakeUp() 才能保证任何路径进的屏保都被关掉(2026-10-23)。
          g_display->ReleaseSleepLock();
          g_display->WakeUp();
        }
      } else if ("self.display.home" == mcp_tool_call_event->name) {
        // 语音"返回": 收起所有全屏覆盖层, 回到对话主界面。
        // ★ 只动画面, 不停业务 —— 与 BOOT 键同一套语义(监测照跑, 相机照待命)。
        // ★ 照片页收起后, loop 里那段会在 200ms 后释放像素缓冲(= 删除照片)。
        printf("on mcp tool call: self.display.home -> back to chat ui\n");
        g_standby_pending = false;  // 还在跟设备互动, 撤销排着队的"待命断链"
        if (g_display) {
          g_display->HidePhotoPage();
          g_display->HideMonitorPage();
          g_display->HideTerminalPage();
          g_display->HideEnvPage();  // 环境完整数据页一并收起(没显示时是 no-op)
          g_display->WakeUp();  // 万一此刻在屏保, 一并退回对话界面
        }
        engine.SendMcpCallResponse(mcp_tool_call_event->id, std::string("已返回对话界面"));
      } else if ("self.serial.send" == mcp_tool_call_event->name) {
        // 向另一块板发一行文本(行协议, 自动补 '\n')
        const auto text_ptr = mcp_tool_call_event->param<std::string>("text");
        if (text_ptr == nullptr) {
          engine.SendMcpCallError(mcp_tool_call_event->id, "Missing valid argument: text");
        } else {
          LinkSendLine(text_ptr->c_str());
          char ack[96] = {0};
          snprintf(ack, sizeof(ack), "已通过串口发出: %s", text_ptr->c_str());
          engine.SendMcpCallResponse(mcp_tool_call_event->id, std::string(ack));
        }
      } else if ("self.serial.get_last" == mcp_tool_call_event->name) {
        // 返回最近收到的完整一行
        if (!g_link_has_line) {
          engine.SendMcpCallError(mcp_tool_call_event->id, "尚未收到另一块板发来的数据");
        } else {
          printf("on mcp tool call: self.serial.get_last -> %s\n", g_link_last_line);
          engine.SendMcpCallResponse(mcp_tool_call_event->id, std::string(g_link_last_line));
        }
      } else if ("self.terminal.led" == mcp_tool_call_event->name) {
        // 打开/关闭终端灯 —— 带握手: 发指令后由 TerminalHandshakePoll() 在 loop 里
        // 等回执, 收到 LED_ON_OK / LED_OFF_OK 才点亮主控第 1 颗灯。
        // ★ 这里【绝不 delay 等回执】: MCP 回调里阻塞会卡住音频和语音事件。
        const auto on_ptr = mcp_tool_call_event->param<bool>("on");
        if (on_ptr == nullptr) {
          engine.SendMcpCallError(mcp_tool_call_event->id, "Missing valid argument: on");
        } else {
          TerminalStartHandshake(*on_ptr ? TerminalLedCmd::kOn : TerminalLedCmd::kOff);
          char ack[160] = {0};
          snprintf(ack, sizeof(ack),
                   "已向终端发出%s指令, 正在等待终端回执确认(最多%u毫秒)。"
                   "收到确认后主控第%d颗灯会自动点亮。",
                   *on_ptr ? "打开灯" : "关闭灯",
                   (unsigned)kTerminalAckTimeoutMs,
                   kHandshakeLedIndex + 1);
          engine.SendMcpCallResponse(mcp_tool_call_event->id, std::string(ack));
        }
      } else if ("self.terminal.last_result" == mcp_tool_call_event->name) {
        // 读取上一次握手的结果(成功/超时)
        if (!g_terminal_hs.ready) {
          engine.SendMcpCallError(mcp_tool_call_event->id, "还没有执行过终端操作");
        } else {
          printf("on mcp tool call: self.terminal.last_result -> %s\n", g_terminal_hs.result);
          engine.SendMcpCallResponse(mcp_tool_call_event->id, std::string(g_terminal_hs.result));
        }
      } else if ("self.terminal.oled" == mcp_tool_call_event->name) {
        // 终端 OLED 显示数字 —— 带握手: 发 OLED:123, 由 TerminalOledPoll() 在 loop 里
        // 等回执 OLED_OK:123, 比对数值一致才算成功。
        // ★ 同样【绝不 delay 等回执】: MCP 回调里阻塞会卡住音频和语音事件。
        const auto value_ptr = mcp_tool_call_event->param<int64_t>("value");
        if (value_ptr == nullptr) {
          engine.SendMcpCallError(mcp_tool_call_event->id, "Missing valid argument: value");
        } else if (*value_ptr < kTerminalOledMin || *value_ptr > kTerminalOledMax) {
          char err[256] = {0};
          snprintf(err, sizeof(err),
                   "数值 %ld 超出范围, 终端只能显示 %ld~%ld 之间的整数",
                   static_cast<long>(*value_ptr), static_cast<long>(kTerminalOledMin),
                   static_cast<long>(kTerminalOledMax));
          engine.SendMcpCallError(mcp_tool_call_event->id, err);
        } else {
          TerminalOledStart(static_cast<int32_t>(*value_ptr));
          char ack[256] = {0};
          snprintf(ack, sizeof(ack),
                   "已向终端发出显示 %ld 的指令, 正在等待终端回执确认(最多%u毫秒)。",
                   static_cast<long>(*value_ptr), (unsigned)kTerminalAckTimeoutMs);
          engine.SendMcpCallResponse(mcp_tool_call_event->id, std::string(ack));
        }
      } else if ("self.terminal.env" == mcp_tool_call_event->name) {
        // 终端温湿度 —— 直接读缓存秒回, 不等串口。
        // ★ 数据过期(终端断电/线松)必须如实说, 不能拿旧值骗用户。
        char buf[256] = {0};
        if (!g_terminal_env.valid) {
          snprintf(buf, sizeof(buf),
                   "还没有收到终端的温湿度数据。可能是终端没有通电、串口线没接好"
                   "(主控39→终端40, 主控40→终端39, 且必须共地), "
                   "或终端固件没有每 2 秒上报 ENV 数据。");
          engine.SendMcpCallResponse(mcp_tool_call_event->id, std::string(buf));
        } else {
          const uint32_t age = millis() - g_terminal_env.rx_ms;
          if (age > kTerminalEnvStaleMs) {
            snprintf(buf, sizeof(buf),
                     "终端最后一次上报是 %lu 秒前(超过 %u 秒视为过期), 数据可能不准。"
                     "最后读到的值是: 温度 %.1f 摄氏度, 湿度 %.1f 百分比。"
                     "建议提醒用户检查终端供电和串口接线。",
                     static_cast<unsigned long>(age / 1000),
                     static_cast<unsigned>(kTerminalEnvStaleMs / 1000),
                     g_terminal_env.temperature, g_terminal_env.humidity);
          } else {
            snprintf(buf, sizeof(buf),
                     "终端当前环境: 温度 %.1f 摄氏度, 湿度 %.1f 百分比。"
                     "(数据来自终端 DHT11 传感器, %lu 毫秒前更新)",
                     g_terminal_env.temperature, g_terminal_env.humidity,
                     static_cast<unsigned long>(age));
          }
          printf("on mcp tool call: self.terminal.env -> %s\n", buf);
          engine.SendMcpCallResponse(mcp_tool_call_event->id, std::string(buf));
        }
      } else if ("self.terminal.all" == mcp_tool_call_event->name) {
        // 终端全量数据 —— ① 立刻整屏显示(读的是刚收到的那一帧缓存) ② 把数值组织成文本返回。
        // ★ 绝不 delay 等串口: 终端本来每 2 秒主动上报, 缓存里永远有最新值。
        //   如果改成"发 ALL? 再等回帧", MCP 回调就得阻塞, 会卡住音频与语音事件。
        if (g_display) {
          g_display->ShowTerminalPage();
          g_display->SetTerminalData(g_terminal_all);
        }

        char buf[640] = {0};
        if (!g_terminal_all_valid) {
          snprintf(buf, sizeof(buf),
                   "还没有收到终端的传感器数据。屏幕上的数据页已经打开, 各项目前显示 --。"
                   "可能的原因: 终端没有通电、串口线没接好"
                   "(主控39→终端40, 主控40→终端39, 且必须共地), "
                   "或者终端固件没有每 2 秒上报 ALL 数据。请如实告诉用户并提示检查接线。");
          engine.SendMcpCallResponse(mcp_tool_call_event->id, std::string(buf));
        } else {
          const uint32_t age = millis() - g_terminal_all.rx_ms;
          const Display::TerminalData& d = g_terminal_all;
          // 无效项统一写 "--", 不能显示 0 —— 0 会被当成真实读数
          char s_t[16], s_h[16], s_pr[16], s_co2[16], s_lx[16], s_st[16], s_sh[16], s_n[16], s_p[16],
              s_k[16], s_ph[16];
          auto fmt_or_dash = [](char* dst, size_t n, bool ok, const char* f, float v) {
            if (ok) {
              snprintf(dst, n, f, v);
            } else {
              snprintf(dst, n, "--");
            }
          };
          fmt_or_dash(s_t, sizeof(s_t), d.ok[0], "%.1f", d.t);
          fmt_or_dash(s_h, sizeof(s_h), d.ok[1], "%.1f", d.h);
          fmt_or_dash(s_pr, sizeof(s_pr), d.ok[2], "%.1f", d.pr);
          fmt_or_dash(s_co2, sizeof(s_co2), d.ok[3], "%.0f", d.co2);
          fmt_or_dash(s_lx, sizeof(s_lx), d.ok[4], "%.0f", d.lx);
          fmt_or_dash(s_st, sizeof(s_st), d.ok[5], "%.1f", d.st);
          fmt_or_dash(s_sh, sizeof(s_sh), d.ok[6], "%.1f", d.sh);
          fmt_or_dash(s_n, sizeof(s_n), d.ok[7], "%.0f", d.n);
          fmt_or_dash(s_p, sizeof(s_p), d.ok[8], "%.0f", d.p);
          fmt_or_dash(s_k, sizeof(s_k), d.ok[9], "%.0f", d.k);
          fmt_or_dash(s_ph, sizeof(s_ph), d.ok[10], "%.1f", d.ph);

          snprintf(buf, sizeof(buf),
                   "终端全部传感器数据(已显示在主控屏幕上): "
                   "空气温度 %s 摄氏度, 空气湿度 %s 百分比, 气压 %s 百帕, "
                   "空气CO2 %s ppm, 光照强度 %s 勒克斯, "
                   "土壤温度 %s 摄氏度, 土壤湿度 %s 百分比, "
                   "土壤氮 %s, 土壤磷 %s, 土壤钾 %s, 土壤PH %s。"
                   "(数据 %lu 毫秒前更新%s)",
                   s_t, s_h, s_pr, s_co2, s_lx, s_st, s_sh, s_n, s_p, s_k, s_ph,
                   static_cast<unsigned long>(age),
                   (age > kTerminalEnvStaleMs) ? ", 已超过 10 秒未更新, 数据可能过期" : "");
          printf("on mcp tool call: self.terminal.all -> shown=%d age=%lums\n",
                 g_display && g_display->IsTerminalPageActive() ? 1 : 0,
                 static_cast<unsigned long>(age));
          engine.SendMcpCallResponse(mcp_tool_call_event->id, std::string(buf));
        }
      } else if ("self.terminal.hide" == mcp_tool_call_event->name) {
        // 关闭数据页, 回到对话界面。★ 这是用户"想马上回去"的立刻生效通道,
        //   不依赖 30 秒自动关闭的兜底。
        const bool was_active = g_display && g_display->IsTerminalPageActive();
        if (g_display) {
          g_display->HideTerminalPage();
        }
        printf("on mcp tool call: self.terminal.hide -> was_active=%d\n", was_active ? 1 : 0);
        engine.SendMcpCallResponse(mcp_tool_call_event->id,
                                   was_active ? std::string("已关闭终端数据页, 屏幕已恢复对话界面。")
                                              : std::string("屏幕当前就是对话界面, 没有数据页需要关闭。"));
      } else if ("self.crop.profile" == mcp_tool_call_event->name) {
        // 作物标准下发。★ 绝不 delay: 发完就返回, 终端收到后会回 CFG_OK(只用于日志)。
        const auto crop_ptr = mcp_tool_call_event->param<std::string>("crop");
        const auto ranges_ptr = mcp_tool_call_event->param<std::string>("ranges");

        if (crop_ptr == nullptr || crop_ptr->empty()) {
          engine.SendMcpCallError(mcp_tool_call_event->id, "缺少参数 crop(作物名)");
        } else if (ranges_ptr == nullptr || ranges_ptr->empty()) {
          engine.SendMcpCallError(mcp_tool_call_event->id, "缺少参数 ranges(区间串)");
        } else {
          char bad[8] = {0};
          if (!CropNameDrawable(crop_ptr->c_str(), bad, sizeof(bad))) {
            // ★ 先拦字库：不拦的话终端会把这个字画成红框，用户以为是屏坏了
            char err[384] = {0};
            snprintf(err, sizeof(err),
                     "终端字库里没有「%s」这个字, 显示出来会是一个红方块。"
                     "请换一个常见作物名重试, 例如: 土豆、马铃薯、番茄、黄瓜、辣椒、茄子、"
                     "生菜、白菜、菠菜、芹菜、韭菜、大葱、大蒜、生姜、萝卜、胡萝卜、"
                     "西瓜、甜瓜、玉米、小麦、水稻、大豆、花生、棉花、向日葵、葡萄、"
                     "苹果、梨、桃、香蕉、山药、红薯、西兰花、豆角、南瓜、冬瓜、苦瓜、"
                     "蘑菇、油菜、甘蓝、洋葱、豌豆、绿豆、高粱、谷子、芝麻、烟草、茶叶、"
                     "花卉、多肉、绿萝、月季。实在没有合适的, 就用「作物」两个字代替。",
                     bad);
            engine.SendMcpCallError(mcp_tool_call_event->id, err);
          } else if (crop_ptr->size() > 21u) {
            engine.SendMcpCallError(mcp_tool_call_event->id, "作物名太长, 请控制在 7 个汉字以内");
          } else {
            CropRange r[kCropItemCount];
            const int parsed = ParseCropRanges(ranges_ptr->c_str(), r);
            if (parsed <= 0) {
              engine.SendMcpCallError(
                  mcp_tool_call_event->id,
                  "ranges 里一个有效区间都没解析出来。格式必须是 KEY=下限:上限, 多项用分号分隔, "
                  "例如 T=10:25;H=60:80;PH=5.5:6.5。可用 KEY: T H PR CO2 LX ST SH N P K PH");
            } else {
              char cfg[256] = {0};
              const int n = BuildCropCfg(crop_ptr->c_str(), r, cfg, sizeof(cfg));
              if (n <= 0) {
                engine.SendMcpCallError(mcp_tool_call_event->id, "区间数据太长, 请减少项数");
              } else {
                // ★★ 先关掉终端数据页。踩过的坑（2026-10-20 用户实测）：
                //   上一句可能刚说过"显示终端数据"，数据页正占着整屏，
                //   这时再下发标准，屏幕还停在数据页上 → 用户以为"同步没生效"。
                //   关掉之后用户才能看到状态栏的"终端已记住 XX"。
                if (g_display) {
                  g_display->HideTerminalPage();
                }
                LinkSendLine(cfg);
                // 开始等 B 板回执，结果由 CropAckPoll() 在 3 秒内显示到状态栏
                snprintf(g_crop_ack_name, sizeof(g_crop_ack_name), "%s", crop_ptr->c_str());
                g_crop_ack_sent_ms = millis();
                g_crop_ack_pending = true;
                printf("on mcp tool call: self.crop.profile -> %s\n", cfg);

                char ack[640] = {0};
                if (parsed == kCropItemCount) {
                  snprintf(ack, sizeof(ack),
                           "已把%s的 11 项生长区间同步给终端, 终端已存进 Flash(断电不丢)。"
                           "现在终端屏幕上每一项都会按标准染色: 淡绿=在范围内, 淡红+红字=超限。"
                           "请挑 2~3 个重点向用户复述, 并说明: 这是通用参考区间, "
                           "不是当地农技站的实测标准, 实际种植请结合本地情况。",
                           crop_ptr->c_str());
                } else {
                  snprintf(ack, sizeof(ack),
                           "已把%s的 %d 项生长区间同步给终端(另有 %d 项没给区间, 那几项不会染色), "
                           "终端已存进 Flash(断电不丢)。屏幕上: 淡绿=在范围内, 淡红+红字=超限。"
                           "请挑 2~3 个重点向用户复述, 并说明这是通用参考区间, "
                           "不是当地农技站的实测标准。",
                           crop_ptr->c_str(), parsed, kCropItemCount - parsed);
                }
                engine.SendMcpCallResponse(mcp_tool_call_event->id, std::string(ack));
              }
            }
          }
        }
      } else if ("self.act.set" == mcp_tool_call_event->name) {
        // 终端执行器控制。★ 绝不 delay 等回执: 会卡住音频/语音事件。
        const auto cmd_ptr = mcp_tool_call_event->param<std::string>("cmd");
        if (cmd_ptr == nullptr || cmd_ptr->empty()) {
          engine.SendMcpCallError(mcp_tool_call_event->id,
                                  "缺少参数 cmd, 例如 FAN=2 或 LIGHT=1,HUMID=1");
        } else if (cmd_ptr->size() > 120u) {
          engine.SendMcpCallError(mcp_tool_call_event->id,
                                  "cmd 太长, 请把指令拆成几条分别发送");
        } else {
          // ★★ 先关掉终端数据页。踩过的坑(2026-10-20 用户实测):
          //   上一句刚说过"显示终端数据", 数据页正占着整屏, 这时再下发执行器指令,
          //   用户看不到状态栏和对话区的回执, 就会以为"说了没反应"。
          if (g_display) {
            g_display->HideTerminalPage();
          }
          char line[160] = {0};
          snprintf(line, sizeof(line), "ACT,%s", cmd_ptr->c_str());
          // ★ 先把上一轮的三面标志全部清掉，否则旧回执会把这次误判成成功
          g_act_got_ok  = false;
          g_act_got_err = false;
          g_act_got_ack = false;
          g_act_ack_sent_ms = millis();
          g_act_ack_pending = true;
          LinkSendLine(line);
          printf("on mcp tool call: self.act.set -> %s\n", line);

          char ack[384] = {0};
          snprintf(ack, sizeof(ack),
                   "指令已发给终端: %s。终端收到后会立刻动作, "
                   "主控屏幕上会出现一条'终端已执行: ...'的实际状态。"
                   "请用一句话向用户确认动作已下发, 不要逐项念参数。"
                   "若屏幕上出现'终端没有回应', 说明串口不通或终端没上电, 请如实告知用户, "
                   "并提示检查接线(主控39→终端40, 主控40→终端39, 必须共地)。",
                   cmd_ptr->c_str());
          engine.SendMcpCallResponse(mcp_tool_call_event->id, std::string(ack));
        }
      } else if ("self.shade.forward" == mcp_tool_call_event->name) {
        // 遮阳伞按秒前进(2026-10-23 新增)。终端侧负责累计总时长(≤45s)与自动停,
        // A 板只透传秒数, 复位也由终端按累计值执行 —— 两边都不用记账。
        const auto sec = mcp_tool_call_event->param<int64_t>("seconds");
        if (sec == nullptr || *sec < 1 || *sec > 45) {
          engine.SendMcpCallError(mcp_tool_call_event->id,
                                  "seconds 必须是 1~45 的整数秒, 请把用户说的秒数原样传入");
        } else {
          if (g_display) {
            g_display->HideTerminalPage();
          }
          char line[160] = {0};
          snprintf(line, sizeof(line), "ACT,SHADE_FWD=%lld", static_cast<long long>(*sec));
          g_act_got_ok  = false;
          g_act_got_err = false;
          g_act_got_ack = false;
          g_act_ack_sent_ms = millis();
          g_act_ack_pending = true;
          LinkSendLine(line);
          printf("on mcp tool call: self.shade.forward -> %s\n", line);
          char ack[384] = {0};
          snprintf(ack, sizeof(ack),
                   "已让遮阳伞前进 %lld 秒后自动停(终端累计前进时长不超过45秒, "
                   "若已到全开位终端会忽略)。可继续说'再前进N秒'累加, "
                   "说'遮阳复位'会按累计时长退回初始位。",
                   static_cast<long long>(*sec));
          engine.SendMcpCallResponse(mcp_tool_call_event->id, std::string(ack));
        }
      } else if ("self.shade.reset" == mcp_tool_call_event->name) {
        // 遮阳伞复位: 终端按累计前进总时长后退同样的秒数, 回到初始位。
        if (g_display) {
          g_display->HideTerminalPage();
        }
        g_act_got_ok  = false;
        g_act_got_err = false;
        g_act_got_ack = false;
        g_act_ack_sent_ms = millis();
        g_act_ack_pending = true;
        LinkSendLine("ACT,SHADE_RST=1");
        printf("on mcp tool call: self.shade.reset -> ACT,SHADE_RST=1\n");
        engine.SendMcpCallResponse(
            mcp_tool_call_event->id,
            std::string("已下发遮阳伞复位: 终端会按此前累计前进的时长退回, "
                        "然后累计值清零。若终端此前没有前进过, 伞不会动(本来就在初始位)。"));
      } else if ("self.act.status" == mcp_tool_call_event->name) {
        if (!g_act_valid) {
          engine.SendMcpCallResponse(
              mcp_tool_call_event->id,
              std::string("还没收到终端执行器的状态。可能终端没上电、串口不通, "
                          "或者还没下发过任何执行器指令。"));
        } else {
          char body[128] = {0};
          ActFormatState(g_act_v, body, sizeof(body));
          char ack[256] = {0};
          snprintf(ack, sizeof(ack), "终端执行器当前状态: %s", body);
          engine.SendMcpCallResponse(mcp_tool_call_event->id, std::string(ack));
        }
      } else if ("self.monitor.start" == mcp_tool_call_event->name) {
        // 启动自动监测。解析失败用 SendMcpCallError 让 AI 自己改参数重发。
        const auto chan_ptr = mcp_tool_call_event->param<std::string>("channels");
        const auto ranges_ptr = mcp_tool_call_event->param<std::string>("ranges");
        const auto crop_ptr = mcp_tool_call_event->param<std::string>("crop");
        if (chan_ptr == nullptr || chan_ptr->empty()) {
          engine.SendMcpCallError(mcp_tool_call_event->id,
                                  "缺少参数 channels, 例如 all 或 光照 / 光照,温度");
        } else if (ranges_ptr == nullptr || ranges_ptr->empty()) {
          engine.SendMcpCallError(mcp_tool_call_event->id,
                                  "缺少参数 ranges(阈值区间串), 格式 KEY=下限:上限, 分号分隔");
        } else {
          char out[640] = {0};
          const char* crop = (crop_ptr != nullptr && !crop_ptr->empty()) ? crop_ptr->c_str() : nullptr;
          if (MonitorStart(chan_ptr->c_str(), ranges_ptr->c_str(), crop,
                           out, sizeof(out))) {
            engine.SendMcpCallResponse(mcp_tool_call_event->id, std::string(out));
          } else {
            engine.SendMcpCallError(mcp_tool_call_event->id, out);
          }
        }
      } else if ("self.monitor.stop" == mcp_tool_call_event->name) {
        if (!g_mon_active) {
          engine.SendMcpCallResponse(
              mcp_tool_call_event->id,
              std::string("当前没有正在运行的监测。"));
        } else {
          MonitorStop();
          engine.SendMcpCallResponse(
              mcp_tool_call_event->id,
              std::string("监测已停止, 监测面板已收起。所有执行器已恢复初始位置: "
                          "风扇/暖光灯/照明灯/加湿器已全部关闭, 施肥舵机已回中"
                          "(监测期间若展开过遮阳伞, 正在自动收起, 约45秒走完)。请向用户简短确认即可。"));
        }
      } else if ("self.monitor.status" == mcp_tool_call_event->name) {
        if (!g_mon_active) {
          engine.SendMcpCallResponse(
              mcp_tool_call_event->id,
              std::string("当前没有启动监测。用户要开始监测时用 self.monitor.start。"));
        } else {
          char body[512] = {0};
          for (int ch = 0; ch < kMonCount; ++ch) {
            const MonChan& c = g_mon[ch];
            if (!c.enabled) {
              continue;
            }
            char vb[16] = {0};
            MonFmt(ch, c.value, vb, sizeof(vb));
            char one[96];
            if (!c.value_ok) {
              snprintf(one, sizeof(one), "%s 无数据; ", kMonMeta[ch].name);
            } else {
              const char* st = (c.state == 1) ? "超上限" : (c.state == 2) ? "低于下限" : "正常";
              snprintf(one, sizeof(one), "%s 当前%s%s(%s); ",
                       kMonMeta[ch].name, vb, kMonMeta[ch].unit, st);
            }
            const size_t len = strlen(body);
            snprintf(body + len, sizeof(body) - len, "%s", one);
          }
          char ack[640] = {0};
          snprintf(ack, sizeof(ack), "监测运行中(%s): %s。请按通道逐条向用户播报, "
                   "超限的通道重点说, 并说明已联动什么执行器。",
                   g_mon_title, body);
          engine.SendMcpCallResponse(mcp_tool_call_event->id, std::string(ack));
        }
      } else if ("self.monitor.panel" == mcp_tool_call_event->name) {
        // 显示/隐藏监测面板。★ 只是画面切换, 监测状态机照常跑。
        const auto show_ptr = mcp_tool_call_event->param<std::string>("show");
        if (!g_mon_active) {
          engine.SendMcpCallResponse(
              mcp_tool_call_event->id,
              std::string("当前没有启动监测, 面板无从显示。请先说'全自动检测'或"
                          "'打开XX监测'来启动。"));
        } else if (show_ptr != nullptr && *show_ptr == "0") {
          if (g_display) {
            g_display->HideMonitorPage();
          }
          engine.SendMcpCallResponse(
              mcp_tool_call_event->id,
              std::string("监测面板已隐藏, 监测继续在后台运行。"));
        } else {
          if (g_display) {
            g_display->HideTerminalPage();
            g_display->ShowMonitorPage();
          }
          engine.SendMcpCallResponse(
              mcp_tool_call_event->id,
              std::string("监测面板已显示。"));
        }
      } else if ("self.servo.set" == mcp_tool_call_event->name) {
        const auto servo_ptr = mcp_tool_call_event->param<int64_t>("servo");
        const auto angle_ptr = mcp_tool_call_event->param<int64_t>("angle");
        if (servo_ptr == nullptr || angle_ptr == nullptr) {
          engine.SendMcpCallError(mcp_tool_call_event->id, "Missing valid argument: servo / angle");
        } else {
          const int idx = static_cast<int>(*servo_ptr) - 1;
          int angle = static_cast<int>(*angle_ptr);
          if (idx < 0 || idx >= 3) {
            engine.SendMcpCallError(mcp_tool_call_event->id, "servo must be 1~3");
          } else if (angle < 0 || angle > 180) {
            engine.SendMcpCallError(mcp_tool_call_event->id, "angle must be 0~180");
          } else {
            g_servos[idx].testing = false;  // 新指令取消该舵机正在进行的自检
            ServoWriteAngle(idx, angle);
            printf("[T+%lums] on mcp tool call: self.servo.set -> servo%d IO%d to %d deg\n",
                   (unsigned long)millis(), idx + 1, static_cast<int>(kServoPins[idx]), angle);
            char ack[96] = {0};
            snprintf(ack, sizeof(ack), "舵机%d 已转到 %d 度", idx + 1, angle);
            engine.SendMcpCallResponse(mcp_tool_call_event->id, std::string(ack));
          }
        }
      } else if ("self.servo.selftest" == mcp_tool_call_event->name) {
        const auto servo_ptr = mcp_tool_call_event->param<int64_t>("servo");
        if (servo_ptr == nullptr) {
          engine.SendMcpCallError(mcp_tool_call_event->id, "Missing valid argument: servo");
        } else {
          const int idx = static_cast<int>(*servo_ptr) - 1;
          if (idx < 0 || idx >= 3) {
            engine.SendMcpCallError(mcp_tool_call_event->id, "servo must be 1~3");
          } else {
            ServoStartSelftest(idx);
            printf("[T+%lums] on mcp tool call: self.servo.selftest -> servo%d IO%d\n",
                   (unsigned long)millis(), idx + 1, static_cast<int>(kServoPins[idx]));
            char ack[96] = {0};
            snprintf(ack, sizeof(ack), "舵机%d 自检已开始", idx + 1);
            engine.SendMcpCallResponse(mcp_tool_call_event->id, std::string(ack));
          }
        }
      } else if ("self.vision.link" == mcp_tool_call_event->name) {
        const auto kw_ptr = mcp_tool_call_event->param<std::string>("keyword");
        const auto acts_ptr = mcp_tool_call_event->param<std::string>("actions");
        const auto speak_ptr = mcp_tool_call_event->param<std::string>("speak");
        if (kw_ptr == nullptr || acts_ptr == nullptr) {
          engine.SendMcpCallError(mcp_tool_call_event->id, "Missing valid argument: keyword / actions");
        } else {
          const char* speak = (speak_ptr != nullptr) ? speak_ptr->c_str() : nullptr;
          if (!VisionLinkAdd(kw_ptr->c_str(), acts_ptr->c_str(), speak)) {
            engine.SendMcpCallError(mcp_tool_call_event->id,
                                    "规则已满(最多6条), 或关键词/动作为空, 或动作格式无法解析");
          } else {
            char ack[192] = {0};
            snprintf(ack, sizeof(ack), "已设置: 识别到\"%s\"时, 执行 %s",
                     kw_ptr->c_str(), acts_ptr->c_str());
            printf("[T+%lums] on mcp tool call: self.vision.link -> '%s' actions='%s'\n",
                   (unsigned long)millis(), kw_ptr->c_str(), acts_ptr->c_str());
            engine.SendMcpCallResponse(mcp_tool_call_event->id, std::string(ack));
          }
        }
      } else if ("self.vision.reset" == mcp_tool_call_event->name) {
        VisionLinkReset();
        printf("[T+%lums] on mcp tool call: self.vision.reset\n", (unsigned long)millis());
        engine.SendMcpCallResponse(mcp_tool_call_event->id,
                                   std::string("已清空所有识别联动规则, 执行器已复位"));
      } else if ("self.vision.stage" == mcp_tool_call_event->name) {
        const auto stage_ptr = mcp_tool_call_event->param<std::string>("stage");
        const auto feat_ptr = mcp_tool_call_event->param<std::string>("features");
        if (stage_ptr == nullptr || feat_ptr == nullptr) {
          engine.SendMcpCallError(mcp_tool_call_event->id, "Missing valid argument: stage / features");
        } else if (!VisionSetStageFeatures(stage_ptr->c_str(), feat_ptr->c_str())) {
          engine.SendMcpCallError(mcp_tool_call_event->id,
                                  "阶段词无效(应为 开花期/结果期/幼苗期/成长期/成熟期/枯萎 之一), 或特征词为空");
        } else {
          char ack[160] = {0};
          snprintf(ack, sizeof(ack), "已设置 %s 的特征词: %s", stage_ptr->c_str(), feat_ptr->c_str());
          printf("[T+%lums] on mcp tool call: self.vision.stage -> '%s'='%s'\n",
                 (unsigned long)millis(), stage_ptr->c_str(), feat_ptr->c_str());
          engine.SendMcpCallResponse(mcp_tool_call_event->id, std::string(ack));
        }
      } else if ("self.cron.add" == mcp_tool_call_event->name) {
        const auto kind_ptr = mcp_tool_call_event->param<std::string>("kind");
        const auto speak_ptr = mcp_tool_call_event->param<std::string>("speak");
        const auto actions_ptr = mcp_tool_call_event->param<std::string>("actions");
        const auto interval_ptr = mcp_tool_call_event->param<int64_t>("interval_s");
        const auto hour_ptr = mcp_tool_call_event->param<int64_t>("hour");
        const auto minute_ptr = mcp_tool_call_event->param<int64_t>("minute");
        const char* speak = (speak_ptr != nullptr) ? speak_ptr->c_str() : "";
        const char* actions = (actions_ptr != nullptr) ? actions_ptr->c_str() : "";
        if (kind_ptr == nullptr) {
          engine.SendMcpCallError(mcp_tool_call_event->id, "Missing valid argument: kind");
        } else if (*kind_ptr == "every") {
          if (interval_ptr == nullptr) {
            engine.SendMcpCallError(mcp_tool_call_event->id, "every 需要 interval_s(秒)");
          } else if (!CronAddEvery("", speak, actions, static_cast<uint32_t>(*interval_ptr))) {
            engine.SendMcpCallError(mcp_tool_call_event->id, "定时任务已满(最多16条)");
          } else {
            char ack[160] = {0};
            snprintf(ack, sizeof(ack), "已设置定时: 每 %lld 秒%s", (long long)*interval_ptr,
                     actions[0] ? "并执行动作" : "播报");
            printf("[T+%lums] on mcp tool call: self.cron.add every %llds\n",
                   (unsigned long)millis(), (long long)*interval_ptr);
            engine.SendMcpCallResponse(mcp_tool_call_event->id, std::string(ack));
          }
        } else if (*kind_ptr == "at") {
          if (hour_ptr == nullptr || minute_ptr == nullptr) {
            engine.SendMcpCallError(mcp_tool_call_event->id, "at 需要 hour(0~23) 和 minute(0~59)");
          } else if (!CronAddAt("", speak, actions, static_cast<int>(*hour_ptr), static_cast<int>(*minute_ptr))) {
            engine.SendMcpCallError(mcp_tool_call_event->id, "定时任务已满(最多16条)或时间无效");
          } else {
            char ack[160] = {0};
            snprintf(ack, sizeof(ack), "已设置定时: 每天 %02lld:%02lld%s", (long long)*hour_ptr,
                     (long long)*minute_ptr, actions[0] ? "并执行动作" : "播报");
            printf("[T+%lums] on mcp tool call: self.cron.add at %02lld:%02lld\n",
                   (unsigned long)millis(), (long long)*hour_ptr, (long long)*minute_ptr);
            engine.SendMcpCallResponse(mcp_tool_call_event->id, std::string(ack));
          }
        } else {
          engine.SendMcpCallError(mcp_tool_call_event->id, "kind 应为 every 或 at");
        }
      } else if ("self.cron.reset" == mcp_tool_call_event->name) {
        CronReset();
        printf("[T+%lums] on mcp tool call: self.cron.reset\n", (unsigned long)millis());
        engine.SendMcpCallResponse(mcp_tool_call_event->id, std::string("已清空所有定时任务"));
      } else if ("self.motor1.set" == mcp_tool_call_event->name) {
        // ---- 1号电机: 方向 + 速度(PWM 0~255) ----
        const auto dir_ptr = mcp_tool_call_event->param<std::string>("direction");
        const auto speed_ptr = mcp_tool_call_event->param<int64_t>("speed");
        if (dir_ptr == nullptr && speed_ptr == nullptr) {
          engine.SendMcpCallError(mcp_tool_call_event->id, "Missing valid argument: direction / speed");
        } else {
          // 方向: 给了就解析, 没给就保持当前方向(便于"只调速"的指令)
          MotorDir target = g_motor1_dir;
          bool dir_ok = true;
          if (dir_ptr != nullptr) {
            std::string dir = *dir_ptr;
            for (char& c : dir) {
              if (c >= 'A' && c <= 'Z') c = static_cast<char>(c - 'A' + 'a');
            }
            if (dir == "forward" || dir == "fwd" || dir == "正转" || dir == "正着转" || dir == "前进") {
              target = MotorDir::kFwd;
            } else if (dir == "reverse" || dir == "rev" || dir == "backward" || dir == "反转" || dir == "倒转" || dir == "后退") {
              target = MotorDir::kRev;
            } else if (dir == "stop" || dir == "off" || dir == "停" || dir == "停止" || dir == "关") {
              target = MotorDir::kStop;
            } else {
              dir_ok = false;
            }
          }
          // 速度: 给了就取用户值(夹到 0~255), 没给传 -1 表示沿用当前速度
          int speed = -1;
          if (speed_ptr != nullptr) {
            speed = static_cast<int>(*speed_ptr);
            if (speed > kMotor1PwmMax) speed = kMotor1PwmMax;
            if (speed < 0) speed = 0;
          }
          if (!dir_ok) {
            engine.SendMcpCallError(mcp_tool_call_event->id,
                                    "Invalid direction, must be forward / reverse / stop");
          } else {
            Motor1Set(target, speed);
            char ack[96] = {0};
            const char* zh = (g_motor1_dir == MotorDir::kFwd)   ? "正转"
                             : (g_motor1_dir == MotorDir::kRev) ? "反转"
                                                                : "已停止";
            if (g_motor1_dir == MotorDir::kStop) {
              snprintf(ack, sizeof(ack), "1号电机已停止");
            } else {
              snprintf(ack, sizeof(ack), "1号电机%s, PWM=%d", zh, g_motor1_speed);
            }
            engine.SendMcpCallResponse(mcp_tool_call_event->id, std::string(ack));
          }
        }
      } else if ("self.motor2.set" == mcp_tool_call_event->name) {
        // ---- 2号电机: 只有方向(纯开关, 无速度) ----
        const auto dir_ptr = mcp_tool_call_event->param<std::string>("direction");
        if (dir_ptr == nullptr) {
          engine.SendMcpCallError(mcp_tool_call_event->id, "Missing valid argument: direction");
        } else {
          std::string dir = *dir_ptr;
          for (char& c : dir) {
            if (c >= 'A' && c <= 'Z') c = static_cast<char>(c - 'A' + 'a');
          }
          MotorDir target = MotorDir::kStop;
          bool ok = true;
          if (dir == "forward" || dir == "fwd" || dir == "正转" || dir == "正着转" || dir == "前进") {
            target = MotorDir::kFwd;
          } else if (dir == "reverse" || dir == "rev" || dir == "backward" || dir == "反转" || dir == "倒转" || dir == "后退") {
            target = MotorDir::kRev;
          } else if (dir == "stop" || dir == "off" || dir == "停" || dir == "停止" || dir == "关") {
            target = MotorDir::kStop;
          } else if (dir == "brake" || dir == "刹车") {
            target = MotorDir::kBrake;
          } else {
            ok = false;
          }
          if (!ok) {
            engine.SendMcpCallError(mcp_tool_call_event->id,
                                    "Invalid direction, must be forward / reverse / stop");
          } else {
            Motor2SetDirection(target);
            char ack[96] = {0};
            const char* zh = (g_motor2_dir == MotorDir::kFwd)   ? "正转"
                             : (g_motor2_dir == MotorDir::kRev) ? "反转"
                             : (g_motor2_dir == MotorDir::kBrake) ? "刹车"
                                                                  : "已停止";
            snprintf(ack, sizeof(ack), "2号电机%s", zh);
            engine.SendMcpCallResponse(mcp_tool_call_event->id, std::string(ack));
          }
        }
#if ENABLE_USB_CAMERA
      } else if ("self.camera.capture" == mcp_tool_call_event->name) {
        // 语音"拍照" → 抓一帧 MJPEG → 解码铺满屏幕(常显) → 上传官方 vision 接口识别
        //   → 回执里带上识别结果, 让 AI 念给用户。
        // ★ 相机懒启动在 CamTakePhoto 里; 上传的两级策略(轻装试 → 必要时挂断语音重试)
        //   在 CamUploadAndRespond 里, 这里只负责取参数和分流。
        {
          const std::string* q = mcp_tool_call_event->param<std::string>("question");
          if (q != nullptr && !q->empty()) {
            std::string qs = *q;
            for (auto& c : qs) {  // multipart 字段里 CR/LF 会截断字段边界, 换成空格
              if (c == '\r' || c == '\n') c = ' ';
            }
            snprintf(s_cam_question, sizeof(s_cam_question), "%s", qs.c_str());
            printf("[cam] question: %s\n", s_cam_question);
          } else {
            s_cam_question[0] = '\0';  // 用默认提示词
          }
        }
        const std::string cam_sid_before = engine.GetSessionId();
        printf("[cam] capture requested\n");
        char cam_ack[192] = {0};
        if (CamTakePhoto(cam_ack, sizeof(cam_ack))) {
          printf("[cam] capture ok: %s\n", cam_ack);
          CamUploadAndRespond(mcp_tool_call_event->id, cam_ack, cam_sid_before);
        } else {
          // 没拍到就别上传了, 直接把失败原因交给 AI
          printf("[cam] capture failed: %s\n", cam_ack);
          engine.SendMcpCallResponse(mcp_tool_call_event->id, std::string(cam_ack));
        }
#endif
      }
    }
  }

  // ★★ 2026-10-16 残余卡顿根治(第 2 刀): loop 末尾主动让出 1 个 tick。
  //
  //   为什么需要: 本函数上方所有工作都是"同步执行"的 —— LED 的 led_strip_refresh()
  //   会阻塞等 RMT 把整条灯链的比特流发完, PopEvents() 会把整个事件队列搬走,
  //   I2C 读传感器也要等总线。若这些累计占用连续 >= 20ms, 优先级同为 1 的
  //   AudioOutput 任务就会错过一次 DMA 喂数窗口 → I2S 下溢 → 播报卡顿。
  //
  //   vTaskDelay(1) 让出 1ms: FreeRTOS 会立刻把 CPU 交给就绪的其它任务
  //   (AudioOutput 提权后为 4, 一定先跑)。1ms 的代价对本工程的实时性无影响
  //   (屏保时钟按整秒刷新、传感器 3s 一次、灯效 20ms 一帧), 但足以保证
  //   DMA 每 20ms 必被喂一次。
  //
  //   ★ 注意: 必须放在 loop() 的【最末尾】, 且每圈都执行。不要写进条件分支,
  //     否则某条路径绕过这行时又会回到"跑满不让步"的老样子。
  vTaskDelay(1);
}
