/**
 * WS2812 RGB 灯链独立测试程序
 *
 * 硬件: ESP32-S3 (N16R8) + IO38 级联 6 颗 WS2812 (IO38 -> R8 330R -> RGB1 -> ... -> RGB6)
 * 说明: 本程序不依赖 ai_vox / WiFi / 音频, 上电即自动循环跑测试, 用于验证:
 *       1. 灯链是否点亮、颜色顺序 (GRB) 是否正确
 *       2. 灯珠数量是否为 6 颗、级联顺序是否正确
 *       3. 亮度渐变 (呼吸) 是否平滑
 *
 * 串口: 115200, 每个测试项开始时会打印日志
 */

#include <Arduino.h>

#include "led_strip.h"

// ---------------- 硬件配置 ----------------
constexpr gpio_num_t kWs2812Pin = GPIO_NUM_38;  // 灯链数据脚
constexpr uint32_t kWs2812Count = 6;            // 灯珠数量 (RGB1~RGB6)

// ---------------- 测试参数 ----------------
constexpr uint8_t kColorLevel = 80;     // 三原色测试亮度 (0~255), 别太高, 刺眼
constexpr uint8_t kWhiteLevel = 150;    // 白色测试亮度
constexpr uint32_t kColorHoldMs = 600;  // 每种颜色保持时间
constexpr uint32_t kPerLedHoldMs = 500; // 逐颗点亮每步时间
constexpr uint32_t kMarqueeStepMs = 120;  // 跑马灯每步时间
constexpr uint32_t kBreathPeriodMs = 2400;  // 一次完整呼吸周期
constexpr uint8_t kBreathMinLevel = 4;
constexpr uint8_t kBreathMaxLevel = 150;
constexpr uint32_t kBreathCycles = 3;   // 呼吸重复次数

led_strip_handle_t g_strip = nullptr;

// ---------------- 基础操作 ----------------

// 初始化 RMT 驱动的 WS2812 灯链
void StripInit() {
  led_strip_config_t strip_config = {
      .strip_gpio_num = static_cast<int>(kWs2812Pin),
      .max_leds = kWs2812Count,
      .led_model = LED_MODEL_WS2812,
      .color_component_format = LED_STRIP_COLOR_COMPONENT_FMT_GRB,  // WS2812 数据顺序 = GRB
      .flags = {.invert_out = false},
  };

  led_strip_rmt_config_t rmt_config = {
      .clk_src = RMT_CLK_SRC_DEFAULT,
      .resolution_hz = 10 * 1000 * 1000,  // 10MHz, WS2812 时序要求
      .mem_block_symbols = 0,
      .flags = {.with_dma = 0},
  };

  ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_config, &rmt_config, &g_strip));
  printf("[WS2812] init ok: pin=%d, leds=%u\n", static_cast<int>(kWs2812Pin), static_cast<unsigned>(kWs2812Count));
}

// 全部灯珠设置为同一颜色
void ShowAll(uint8_t red, uint8_t green, uint8_t blue) {
  for (uint32_t idx = 0; idx < kWs2812Count; ++idx) {
    ESP_ERROR_CHECK(led_strip_set_pixel(g_strip, idx, red, green, blue));
  }
  ESP_ERROR_CHECK(led_strip_refresh(g_strip));
}

// 全部熄灭
void ShowOff() {
  ESP_ERROR_CHECK(led_strip_clear(g_strip));
}

// ---------------- 测试项 ----------------

// 测试1: 三原色 + 白色, 验证颜色通道顺序 (若红绿互换说明格式不是 GRB)
void TestColors() {
  printf("\n===== [1/5] 三原色 + 白色 =====\n");
  struct ColorCase {
    const char* name;
    uint8_t red;
    uint8_t green;
    uint8_t blue;
  };
  const ColorCase cases[] = {
      {"红 RED", kColorLevel, 0, 0},
      {"绿 GREEN", 0, kColorLevel, 0},
      {"蓝 BLUE", 0, 0, kColorLevel},
      {"白 WHITE", kWhiteLevel, kWhiteLevel, kWhiteLevel},
  };
  for (const auto& item : cases) {
    printf("  - %s\n", item.name);
    ShowAll(item.red, item.green, item.blue);
    delay(kColorHoldMs);
  }
  ShowOff();
  delay(200);
}

// 测试2: 逐颗点亮, 验证灯珠数量与级联顺序
void TestEachLed() {
  printf("\n===== [2/5] 逐颗点亮 (应为 %u 颗) =====\n", static_cast<unsigned>(kWs2812Count));
  for (uint32_t idx = 0; idx < kWs2812Count; ++idx) {
    ShowOff();
    ESP_ERROR_CHECK(led_strip_set_pixel(g_strip, idx, kWhiteLevel, kWhiteLevel, kWhiteLevel));
    ESP_ERROR_CHECK(led_strip_refresh(g_strip));
    printf("  - 第 %u 颗亮 (索引 %u)\n", static_cast<unsigned>(idx + 1), static_cast<unsigned>(idx));
    delay(kPerLedHoldMs);
  }
  ShowOff();
  delay(200);
}

// 测试3: 跑马灯 (带拖尾), 验证连续刷新与亮度控制
void TestMarquee() {
  printf("\n===== [3/5] 跑马灯 x3 =====\n");
  for (uint32_t round = 0; round < 3; ++round) {
    for (uint32_t pos = 0; pos < kWs2812Count; ++pos) {
      ShowOff();
      ESP_ERROR_CHECK(led_strip_set_pixel(g_strip, pos, kWhiteLevel, kWhiteLevel, kWhiteLevel));
      if (pos >= 1) {
        ESP_ERROR_CHECK(led_strip_set_pixel(g_strip, pos - 1, 40, 40, 40));
      }
      if (pos >= 2) {
        ESP_ERROR_CHECK(led_strip_set_pixel(g_strip, pos - 2, 12, 12, 12));
      }
      ESP_ERROR_CHECK(led_strip_refresh(g_strip));
      delay(kMarqueeStepMs);
    }
  }
  ShowOff();
  delay(200);
}

// 测试4: 呼吸灯 (三角波), 验证亮度渐变是否平滑无跳动
void TestBreath() {
  printf("\n===== [4/5] 呼吸灯 x%u =====\n", static_cast<unsigned>(kBreathCycles));
  const uint32_t start_tick = millis();
  const uint32_t half = kBreathPeriodMs / 2;
  const uint32_t span = kBreathMaxLevel - kBreathMinLevel;

  while (millis() - start_tick < kBreathPeriodMs * kBreathCycles) {
    const uint32_t phase = (millis() - start_tick) % kBreathPeriodMs;  // 0 ~ period-1
    // 三角波: 0 ~ half 上升, 再回落 (phase < half 时取 phase, 否则取 period - phase)
    const uint32_t ramp = (phase < half) ? phase : (kBreathPeriodMs - phase);
    const uint8_t level = static_cast<uint8_t>(kBreathMinLevel + ramp * span / half);
    ShowAll(level, level, level);
    delay(20);
  }
  ShowOff();
  delay(200);
}

// 测试5: 熄灭收尾
void TestOff() {
  printf("\n===== [5/5] 全部熄灭 =====\n");
  ShowOff();
  delay(1000);
}

// ---------------- Arduino 入口 ----------------

void setup() {
  Serial.begin(115200);
  delay(500);
  printf("\n\n########## WS2812 灯链测试程序 ##########\n");
  printf("引脚 IO%d, 灯珠 %u 颗\n", static_cast<int>(kWs2812Pin), static_cast<unsigned>(kWs2812Count));
  printf("提示: 若板载灯与第 1 颗同步亮, 属正常(数据并联)\n");

  StripInit();
  ShowOff();
  delay(500);
}

void loop() {
  static uint32_t s_cycle = 0;
  ++s_cycle;
  printf("\n>>>>>>>>>> 第 %u 轮测试开始 >>>>>>>>>>\n", static_cast<unsigned>(s_cycle));

  TestColors();
  TestEachLed();
  TestMarquee();
  TestBreath();
  TestOff();

  printf(">>>>>>>>>> 第 %u 轮测试结束, 即将重复 >>>>>>>>>>\n", static_cast<unsigned>(s_cycle));
  delay(500);
}
