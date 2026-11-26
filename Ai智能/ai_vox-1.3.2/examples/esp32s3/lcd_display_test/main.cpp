/*********************************************************************
 * ESP32-S3 LCD 屏幕硬件测试 (单屏版, 八针 ST7789 带 CS)
 *
 * 与主项目 ai_voice_display_lcd 完全一致的驱动参数:
 *   - ST7789, esp_lcd 官方驱动 esp_lcd_new_panel_st7789
 *   - SPI mode 3 (CPOL=1, CPHA=1), 20MHz (排线屏 40MHz 常不稳)
 *   - CS 由 ESP 侧管理 (esp_lcd 自动拉低/拉高)
 *   - reset 后 delay 120ms, invert_color = true
 *
 * 不依赖 LVGL / WiFi / AI 引擎, 纯硬件测试, 用于定位:
 *   1. 坏点 / 亮线        -> 纯色测试
 *   2. 颜色反了 (RGB/BGR) -> 纯色测试 (看 RED 是否显示红)
 *   3. 偏移 / 裁切 / 分辨率错 -> 网格测试 (红框是否贴满四边)
 *   4. 色阶是否正常       -> 灰阶渐变
 *   5. 字体与像素对齐     -> 文字测试
 *
 * 测试项 (自动循环, 按 BOOT 键立即跳到下一项):
 *   0. 纯色循环  红->绿->蓝->白->黑->黄->青->紫 (各 800ms)
 *   1. RGB 渐变条纹
 *   2. 黑白灰阶渐变
 *   3. 网格 + 红色外框 (检查偏移/裁切)
 *   4. 文字显示 (分辨率 + OK)
 *
 * ============================ 接线 (2026-10 最新) ============================
 *   屏幕丝印      接到 ESP32-S3        说明
 *   GND           GND
 *   3V3           3V3
 *   SCL           GPIO16              SPI 时钟
 *   SDA           GPIO17              SPI 数据 (MOSI)
 *   DC            GPIO8               数据/命令选择
 *   RES           GPIO18              复位
 *   CS            GPIO21              片选 (新屏比七针版多这一根)
 *   BLK           3V3 (直连, 不占IO)   背光常亮, 因此本测试不驱动背光脚
 * ===========================================================================
 * 注意:
 *   - 本屏已确认是 240x320 竖屏 (kPanelWidth/kPanelHeight = 240/320);
 *     若画面上下有彩边或整体偏移, 试 kGapX/kGapY = (0,80) 或 (80,0)
 *   - 若白屏/花屏/能亮不显示内容, 先把 kDisplaySpiMode 从 3 改成 0 再试
 *   - 副屏已取消, 原 SPI2(47/13/14/21/12) 引脚已改作电机/舵机/屏CS 用途
 *********************************************************************/

#include <Arduino.h>
#include <driver/spi_common.h>
#include <esp_heap_caps.h>
#include <esp_lcd_panel_io.h>
#include <esp_lcd_panel_ops.h>
#include <esp_lcd_panel_vendor.h>

#ifndef ARDUINO_ESP32S3_DEV
#error "This test only supports ESP32S3-Dev board."
#endif

namespace {

// ================= 屏幕引脚 (与主项目一致) =================
constexpr gpio_num_t kDisplayMosiPin = GPIO_NUM_17;  // SDA
constexpr gpio_num_t kDisplayClkPin = GPIO_NUM_16;   // SCL
constexpr gpio_num_t kDisplayDcPin = GPIO_NUM_8;     // DC
constexpr gpio_num_t kDisplayRstPin = GPIO_NUM_18;   // RES
constexpr gpio_num_t kDisplayCsPin = GPIO_NUM_21;    // CS (新屏新增)
// 背光 BLK 直连 3V3 (常亮), 不再占用 GPIO; 保留开关以便需要软件调光时改回
constexpr bool kUseBacklightGpio = false;
constexpr gpio_num_t kDisplayBacklightPin = GPIO_NUM_NC;

// ================= 分辨率 =================
constexpr int kPanelWidth = 240;
constexpr int kPanelHeight = 320;  // 已确认: 240x320 竖屏
// 240x320 屏一般不需要 gap (0,0); 若画面上下有彩边/偏移, 试 (0,80) 或 (80,0)
constexpr int kGapX = 0;
constexpr int kGapY = 0;

// ================= 公共时序参数 (踩坑调出的稳定值, 勿乱改) =================
constexpr auto kDisplaySpiMode = 3;                    // 七针/八针排线屏一般用 mode 3; 点不亮可试 0
constexpr uint32_t kDisplayPclkHz = 20 * 1000 * 1000;  // 40MHz 常不稳定, 先 20MHz
constexpr uint32_t kResetStableMs = 120;               // reset 后至少等 120ms
constexpr bool kInvertColor = false;  // ★ 实测确认(2026-10-13 主工程照片): 本屏 true 时所有颜色变补色, 必须 false

constexpr gpio_num_t kButtonBoot = GPIO_NUM_0;  // BOOT 键, 低电平按下

// 5x7 字模 (只含测试文字用到的字符), 每字符 5 列, 每列 1 字节, bit0 在最上行
struct Glyph5x7 {
  char ch;
  uint8_t cols[5];
};
constexpr Glyph5x7 kFont[] = {
    {' ', {0x00, 0x00, 0x00, 0x00, 0x00}},
    {'-', {0x08, 0x08, 0x08, 0x08, 0x08}},
    {'.', {0x00, 0x60, 0x60, 0x00, 0x00}},
    {'0', {0x3E, 0x51, 0x49, 0x45, 0x3E}},
    {'1', {0x00, 0x42, 0x7F, 0x40, 0x00}},
    {'2', {0x42, 0x61, 0x51, 0x49, 0x46}},
    {'3', {0x21, 0x41, 0x45, 0x4B, 0x31}},
    {'4', {0x18, 0x14, 0x12, 0x7F, 0x10}},
    {'5', {0x27, 0x45, 0x45, 0x45, 0x39}},
    {'6', {0x3C, 0x4A, 0x49, 0x49, 0x30}},
    {'7', {0x01, 0x71, 0x09, 0x05, 0x03}},
    {'8', {0x36, 0x49, 0x49, 0x49, 0x36}},
    {'9', {0x06, 0x49, 0x49, 0x29, 0x1E}},
    {'A', {0x7E, 0x11, 0x11, 0x11, 0x7E}},
    {'B', {0x7F, 0x49, 0x49, 0x49, 0x36}},
    {'C', {0x3E, 0x41, 0x41, 0x41, 0x22}},
    {'D', {0x7F, 0x41, 0x41, 0x22, 0x1C}},
    {'E', {0x7F, 0x49, 0x49, 0x49, 0x41}},
    {'F', {0x7F, 0x09, 0x09, 0x09, 0x01}},
    {'G', {0x3E, 0x41, 0x49, 0x49, 0x7A}},
    {'H', {0x7F, 0x08, 0x08, 0x08, 0x7F}},
    {'I', {0x00, 0x41, 0x7F, 0x41, 0x00}},
    {'K', {0x7F, 0x08, 0x14, 0x22, 0x41}},
    {'L', {0x7F, 0x40, 0x40, 0x40, 0x40}},
    {'M', {0x7F, 0x02, 0x0C, 0x02, 0x7F}},
    {'N', {0x7F, 0x04, 0x08, 0x10, 0x7F}},
    {'O', {0x3E, 0x41, 0x41, 0x41, 0x3E}},
    {'P', {0x7F, 0x09, 0x09, 0x09, 0x06}},
    {'R', {0x7F, 0x09, 0x19, 0x29, 0x46}},
    {'S', {0x46, 0x49, 0x49, 0x49, 0x31}},
    {'T', {0x01, 0x01, 0x7F, 0x01, 0x01}},
    {'U', {0x3F, 0x40, 0x40, 0x40, 0x3F}},
    {'V', {0x1F, 0x20, 0x40, 0x20, 0x1F}},
    {'W', {0x3F, 0x40, 0x38, 0x40, 0x3F}},
    {'X', {0x63, 0x14, 0x08, 0x14, 0x63}},
    {'Y', {0x07, 0x08, 0x70, 0x08, 0x07}},
};
constexpr size_t kFontCount = sizeof(kFont) / sizeof(kFont[0]);

struct TestScreen {
  const char* name;
  esp_lcd_panel_handle_t panel;
  int w;
  int h;
};

TestScreen g_screen{"LCD(main)", nullptr, kPanelWidth, kPanelHeight};

// 全屏 framebuffer, 优先放 PSRAM
uint16_t* g_fb = nullptr;
size_t g_fb_pixels = 0;

constexpr uint8_t kStepCount = 5;
constexpr uint32_t kStepShowMs = 2500;   // 每个测试项停留时间
constexpr uint32_t kSolidColorMs = 800;  // 纯色测试每种颜色停留时间

uint16_t Color565(uint8_t r, uint8_t g, uint8_t b) {
  return (static_cast<uint16_t>(r & 0xF8) << 8) |
         (static_cast<uint16_t>(g & 0xFC) << 3) |
         (static_cast<uint16_t>(b >> 3));
}

// ---------------- 屏幕初始化 (与主项目 ai_voice_display_lcd 参数一致) ----------------

esp_lcd_panel_handle_t InitPanel() {
  if (kUseBacklightGpio && kDisplayBacklightPin != GPIO_NUM_NC) {
    pinMode(kDisplayBacklightPin, OUTPUT);
    digitalWrite(kDisplayBacklightPin, HIGH);
    printf("backlight gpio %d -> HIGH\n", static_cast<int>(kDisplayBacklightPin));
  } else {
    printf("backlight: tied to 3V3 (no gpio driven)\n");
  }

  spi_bus_config_t buscfg{};
  buscfg.mosi_io_num = kDisplayMosiPin;
  buscfg.miso_io_num = GPIO_NUM_NC;
  buscfg.sclk_io_num = kDisplayClkPin;
  buscfg.quadwp_io_num = GPIO_NUM_NC;
  buscfg.quadhd_io_num = GPIO_NUM_NC;
  buscfg.max_transfer_sz = 4096;  // 逐行刷屏, 单行最大 480 字节, 4096 足够
  ESP_ERROR_CHECK(spi_bus_initialize(SPI3_HOST, &buscfg, SPI_DMA_CH_AUTO));
  printf("spi3 ready (mosi=%d clk=%d)\n", static_cast<int>(kDisplayMosiPin), static_cast<int>(kDisplayClkPin));

  esp_lcd_panel_io_handle_t panel_io = nullptr;
  esp_lcd_panel_io_spi_config_t io_config = {};
  io_config.cs_gpio_num = kDisplayCsPin;
  io_config.dc_gpio_num = kDisplayDcPin;
  io_config.spi_mode = kDisplaySpiMode;
  io_config.pclk_hz = kDisplayPclkHz;
  io_config.trans_queue_depth = 10;
  io_config.lcd_cmd_bits = 8;
  io_config.lcd_param_bits = 8;
  ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi(SPI3_HOST, &io_config, &panel_io));

  esp_lcd_panel_dev_config_t panel_config = {};
  panel_config.reset_gpio_num = kDisplayRstPin;
  panel_config.rgb_ele_order = LCD_RGB_ELEMENT_ORDER_RGB;
  panel_config.bits_per_pixel = 16;
  esp_lcd_panel_handle_t panel = nullptr;
  ESP_ERROR_CHECK(esp_lcd_new_panel_st7789(panel_io, &panel_config, &panel));

  esp_lcd_panel_reset(panel);
  delay(kResetStableMs);  // 部分克隆 ST7789 要求 reset 后等 >=120ms
  esp_lcd_panel_init(panel);
  esp_lcd_panel_invert_color(panel, kInvertColor);
  if (kGapX != 0 || kGapY != 0) {
    esp_lcd_panel_set_gap(panel, kGapX, kGapY);
    printf("panel gap set to (%d, %d)\n", kGapX, kGapY);
  }
  esp_lcd_panel_display_on(panel);
  printf("panel init ok (cs=%d dc=%d rst=%d mode=%d %uHz invert=%d)\n",
         static_cast<int>(kDisplayCsPin),
         static_cast<int>(kDisplayDcPin),
         static_cast<int>(kDisplayRstPin),
         static_cast<int>(kDisplaySpiMode),
         static_cast<unsigned>(kDisplayPclkHz),
         kInvertColor ? 1 : 0);
  return panel;
}

// ---------------- 基于全屏 framebuffer 的绘图原语 ----------------

// 逐行刷屏 (避开大块 DMA buffer, 稳)
void FlushScreen(TestScreen& s) {
  for (int y = 0; y < s.h; ++y) {
    esp_lcd_panel_draw_bitmap(s.panel, 0, y, s.w, y + 1, &g_fb[y * s.w]);
  }
}

void FillRect(TestScreen& s, int x, int y, int rw, int rh, uint16_t color) {
  for (int j = y; j < y + rh && j < s.h; ++j) {
    for (int i = x; i < x + rw && i < s.w; ++i) {
      if (i < 0 || j < 0) {
        continue;
      }
      g_fb[j * s.w + i] = color;
    }
  }
}

void ClearScreen(TestScreen& s, uint16_t color) {
  FillRect(s, 0, 0, s.w, s.h, color);
}

void DrawPixel(TestScreen& s, int x, int y, uint16_t color) {
  if (x >= 0 && x < s.w && y >= 0 && y < s.h) {
    g_fb[y * s.w + x] = color;
  }
}

// 放大 scale 倍画一个 5x7 字符
void DrawChar(TestScreen& s, char ch, int x, int y, uint16_t color, int scale) {
  const Glyph5x7* glyph = nullptr;
  for (size_t i = 0; i < kFontCount; ++i) {
    if (kFont[i].ch == ch) {
      glyph = &kFont[i];
      break;
    }
  }
  if (glyph == nullptr) {
    return;  // 字模表没有的字符直接跳过
  }
  for (int cx = 0; cx < 5; ++cx) {
    const uint8_t col = glyph->cols[cx];
    for (int cy = 0; cy < 7; ++cy) {
      if (col & (1u << cy)) {
        for (int sy = 0; sy < scale; ++sy) {
          for (int sx = 0; sx < scale; ++sx) {
            DrawPixel(s, x + cx * scale + sx, y + cy * scale + sy, color);
          }
        }
      }
    }
  }
}

void DrawText(TestScreen& s, const char* text, int x, int y, uint16_t color, int scale) {
  int cursor_x = x;
  for (const char* p = text; *p != '\0'; ++p) {
    DrawChar(s, *p, cursor_x, y, color, scale);
    cursor_x += 6 * scale;  // 5 列字模 + 1 列间距
  }
}

void DelayWithButton(uint32_t ms) {
  const uint32_t start = millis();
  while (millis() - start < ms) {
    // BOOT 键按下 -> 提前结束当前测试项, 跳到下一个
    if (digitalRead(kButtonBoot) == LOW) {
      delay(30);  // 简单消抖
      if (digitalRead(kButtonBoot) == LOW) {
        printf("boot pressed: skip to next test\n");
        while (digitalRead(kButtonBoot) == LOW) {
          delay(10);  // 等松开
        }
        return;
      }
    }
    delay(10);
  }
}

// ---------------- 五个测试项 ----------------

// 0. 纯色循环: 测坏点/亮线/颜色顺序
void TestSolidColors(TestScreen& s) {
  struct ColorItem {
    const char* label;
    uint8_t r, g, b;
  };
  const ColorItem colors[] = {
      {"RED", 255, 0, 0},       {"GREEN", 0, 255, 0},   {"BLUE", 0, 0, 255},
      {"WHITE", 255, 255, 255}, {"BLACK", 0, 0, 0},     {"YELLOW", 255, 255, 0},
      {"CYAN", 0, 255, 255},    {"MAGENTA", 255, 0, 255},
  };
  for (const auto& c : colors) {
    printf("[%s] solid color %s\n", s.name, c.label);
    ClearScreen(s, Color565(c.r, c.g, c.b));
    FlushScreen(s);
    DelayWithButton(kSolidColorMs);
  }
  printf("check: RED must look red (blue -> RGB/BGR wrong), no dead pixels / bright lines\n");
}

// 1. RGB 渐变条纹: 左->右 红/绿/蓝三段渐变
void TestRgbGradient(TestScreen& s) {
  printf("[%s] rgb gradient bars\n", s.name);
  const int third = s.w / 3;
  for (int y = 0; y < s.h; ++y) {
    for (int x = 0; x < s.w; ++x) {
      uint8_t r = 0, g = 0, b = 0;
      if (x < third) {
        r = static_cast<uint8_t>(255 * x / third);
      } else if (x < third * 2) {
        g = static_cast<uint8_t>(255 * (x - third) / third);
      } else {
        b = static_cast<uint8_t>(255 * (x - third * 2) / third);
      }
      g_fb[y * s.w + x] = Color565(r, g, b);
    }
  }
  FlushScreen(s);
  DelayWithButton(kStepShowMs);
}

// 2. 黑白灰阶渐变: 上黑下白, 数色阶是否平滑
void TestGrayScale(TestScreen& s) {
  printf("[%s] grayscale ramp\n", s.name);
  const int kLevels = 16;
  const int band = s.h / kLevels;
  for (int lv = 0; lv < kLevels; ++lv) {
    const uint8_t v = static_cast<uint8_t>(255 * lv / (kLevels - 1));
    FillRect(s, 0, lv * band, s.w, band, Color565(v, v, v));
  }
  FlushScreen(s);
  DelayWithButton(kStepShowMs);
}

// 3. 网格 + 红色外框: 测偏移/裁切 (红框必须贴满四边, 网格不能歪)
void TestGrid(TestScreen& s) {
  printf("[%s] grid + red border (check gap/offset)\n", s.name);
  ClearScreen(s, Color565(0, 0, 0));
  const uint16_t green = Color565(0, 255, 0);
  const uint16_t red = Color565(255, 0, 0);
  const uint16_t yellow = Color565(255, 255, 0);
  // 每 20 像素一条绿网格线
  for (int x = 0; x < s.w; x += 20) {
    for (int y = 0; y < s.h; ++y) {
      DrawPixel(s, x, y, green);
    }
  }
  for (int y = 0; y < s.h; y += 20) {
    for (int x = 0; x < s.w; ++x) {
      DrawPixel(s, x, y, green);
    }
  }
  // 3 像素粗红框, 必须贴满四边
  for (int t = 0; t < 3; ++t) {
    for (int x = t; x < s.w - t; ++x) {
      DrawPixel(s, x, t, red);
      DrawPixel(s, x, s.h - 1 - t, red);
    }
    for (int y = t; y < s.h - t; ++y) {
      DrawPixel(s, t, y, red);
      DrawPixel(s, s.w - 1 - t, y, red);
    }
  }
  // 四角十字标记
  const int corners[4][2] = {{4, 4}, {s.w - 5, 4}, {4, s.h - 5}, {s.w - 5, s.h - 5}};
  for (const auto& corner : corners) {
    for (int d = -3; d <= 3; ++d) {
      DrawPixel(s, corner[0] + d, corner[1], yellow);
      DrawPixel(s, corner[0], corner[1] + d, yellow);
    }
  }
  FlushScreen(s);
  DelayWithButton(kStepShowMs);
}

// 4. 文字: 黑底白字, 测字体渲染与像素对齐
void TestText(TestScreen& s) {
  printf("[%s] text rendering\n", s.name);
  const uint16_t white = Color565(255, 255, 255);
  const uint16_t cyan = Color565(0, 255, 255);
  const uint16_t yellow = Color565(255, 255, 0);
  ClearScreen(s, Color565(0, 0, 0));

  char title[24];
  snprintf(title, sizeof(title), "LCD %s OK", s.w >= 200 ? "240" : "");
  char res[24];
  snprintf(res, sizeof(res), "%d X %d", s.w, s.h);

  const int scale = s.w >= 200 ? 3 : 2;
  const int title_w = static_cast<int>(strlen(title)) * 6 * scale;
  const int res_w = static_cast<int>(strlen(res)) * 6 * scale;
  const int foot_w = static_cast<int>(strlen("DISPLAY TEST")) * 6 * 2;

  DrawText(s, title, (s.w - title_w) / 2, s.h / 2 - 16 * scale, white, scale);
  DrawText(s, res, (s.w - res_w) / 2, s.h / 2 + 2 * scale, cyan, scale);
  DrawText(s, "DISPLAY TEST", (s.w - foot_w) / 2, s.h - 20, yellow, 2);
  FlushScreen(s);
  DelayWithButton(kStepShowMs);
}

}  // namespace

void setup() {
  Serial.begin(115200);
  delay(300);
  printf("\n\n===== ESP32-S3 LCD DISPLAY TEST (single panel, with CS) =====\n");
  printf("pins: SCL=%d SDA=%d DC=%d RES=%d CS=%d BLK=3V3\n",
         static_cast<int>(kDisplayClkPin),
         static_cast<int>(kDisplayMosiPin),
         static_cast<int>(kDisplayDcPin),
         static_cast<int>(kDisplayRstPin),
         static_cast<int>(kDisplayCsPin));

  pinMode(kButtonBoot, INPUT_PULLUP);

  g_fb_pixels = static_cast<size_t>(kPanelWidth) * kPanelHeight;
  const size_t fb_bytes = g_fb_pixels * sizeof(uint16_t);
  g_fb = static_cast<uint16_t*>(heap_caps_malloc(fb_bytes, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
  if (g_fb == nullptr) {
    g_fb = static_cast<uint16_t*>(heap_caps_malloc(fb_bytes, MALLOC_CAP_8BIT));
  }
  if (g_fb == nullptr) {
    printf("FATAL: framebuffer malloc failed (%u bytes)\n", static_cast<unsigned>(fb_bytes));
    while (true) {
      delay(1000);
    }
  }
  printf("framebuffer: %u px (%u KB)\n", static_cast<unsigned>(g_fb_pixels), static_cast<unsigned>(fb_bytes >> 10));

  g_screen.panel = InitPanel();

  printf("===== display ready, start test loop =====\n");
  printf("press BOOT to skip current test\n");
}

void loop() {
  static uint8_t s_step = 0;
  printf("\n----- test step %u/%u -----\n", static_cast<unsigned>(s_step) + 1, static_cast<unsigned>(kStepCount));
  switch (s_step) {
    case 0:
      TestSolidColors(g_screen);
      break;
    case 1:
      TestRgbGradient(g_screen);
      break;
    case 2:
      TestGrayScale(g_screen);
      break;
    case 3:
      TestGrid(g_screen);
      break;
    case 4:
      TestText(g_screen);
      break;
    default:
      break;
  }
  s_step = static_cast<uint8_t>((s_step + 1) % kStepCount);
}
