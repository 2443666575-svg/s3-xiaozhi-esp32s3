#pragma once

#ifndef _DISPLAY_H_
#define _DISPLAY_H_

#include <esp_lcd_panel_ops.h>

#include <string>

#include "lvgl.h"

class Display {
 public:
  enum class Role : uint8_t {
    kSystem,
    kAssistant,
    kUser,
  };
  enum class Kind : uint8_t {
    kChat,
    kSensor,
  };
  Display(esp_lcd_panel_io_handle_t panel_io,
          esp_lcd_panel_handle_t panel,
          int width,
          int height,
          int offset_x,
          int offset_y,
          bool mirror_x,
          bool mirror_y,
          bool swap_xy,
          Kind kind = Kind::kChat);
  ~Display();
  void Start();
  void SetChatMessage(const Role role, const std::string& content);
  void ShowStatus(const char* status);
  void SetEmotion(const std::string& emotion);
  // Sensor-mode only: 第二个屏用, 主屏 (kChat) 调用无效
  void SetSensorData(float temperature, float humidity, bool valid, const char* msg = nullptr);

  // ---------------- 屏保 (时钟) ----------------
  // ScreenSaverTick(): 由 loop 周期调用(建议 100~200ms)。内部自行判定闲置时长:
  //   - 闲置满 kScreenSaverIdleMs 自动进入屏保
  //   - 屏保中每秒刷新一次时钟
  //   - 任何"有活动"的事件(说话/回复/状态变化/按键)都会先调用 WakeUp() 重置计时
  // WakeUp(): 手动唤醒(清屏保 + 重置闲置计时)。所有会改变界面的公开方法内部已自动调用,
  //           外部(如按键回调)也可直接调。
  void ScreenSaverTick();
  void WakeUp();
  bool IsScreenSaverActive() const { return screen_saver_active_; }

  // EnterScreenSaver(): 手动【立刻】进入屏保(不等 30 秒闲置)。
  // 用途: 语音指令 —— 用户说 "待机"、"退出"、"锁屏"、"休眠"、"黑屏" 等词时,
  //       AI 调 self.display.sleep 工具 → main.cpp 调本方法 → 立刻显示屏保。
  // 实现: 把 last_activity_ms_ 拨到"刚满闲置时长"的位置, 再直接复用进入逻辑。
  //       为什么不全复制一份进入代码 —— 屏保状态机只有一条路径, 复制会分叉出
  //       "手动进"和"自动进"两套状态, 以后改一处忘一处。
  // ★★ 会同时上锁(sleep_lock_) —— 见 ReleaseSleepLock() 的说明, 这是必需的。
  void EnterScreenSaver();

  // ★★ 语音"睡觉锁" —— 解决"说完待机立刻被自己的语音回复吵醒"的死结。
  // 问题现场: 用户说"待机" → AI 调 sleep 工具 → 屏保亮起 → 但 AI 紧接着要回一句
  //   "好的, 已进入待机" → TTS 触发 EmotionEvent/ChatMessageEvent → 那些方法内部调
  //   WakeUp() → 【屏保瞬间又被关掉】。现象就是"屏保闪一下没了"。
  // 解法: EnterScreenSaver() 上一把锁, 锁住期间 WakeUp() 只重置闲置计时、
  //   不真正关闭屏保; 等【用户真的说出新的一句话】时由 main.cpp 调 ReleaseSleepLock() 解锁。
  // 为什么用"锁"而不是"延时唤醒": 延时是猜时长(AI 回复长短不一), 锁是精确的语义。
  // ★★ 解锁点必须是 ChatMessageEvent/kUser (用户说了新话), 【不是】kListening ——
  //   连续对话模式下 AFE 每轮都会自动进 kListening, 用它解锁等于没锁(踩过, 见 main.cpp 注释)。
  // ★ 还带一道"上锁宽限期"(kSleepLockGraceMs): 刚上锁后的极短时间内忽略解锁请求,
  //   防止"待机"那句用户语音因事件乱序晚于工具调用到达而误解锁。
  void ReleaseSleepLock();
  bool IsSleepLocked() const { return sleep_lock_; }
  bool CanReleaseSleepLock() const;  // 是否已过上锁宽限期(供 main.cpp 判断)

  // 屏保温湿度行用: 把环境传感器的最新读数喂给屏保。
  // 主循环(SensorPoll 之后)调用, 与 SetSensorData 的区别:
  //   - SetSensorData 是【第二个屏 Sensor 模式】用的静态数据页
  //   - SetEnvForScreenSaver 是【主屏屏保面板】用的温湿度行
  // 两者互不影响。valid=false 时屏保行显示 "温 -- 湿 --"。
  void SetEnvForScreenSaver(float temperature, float humidity, bool valid);

  // ================= 终端数据页 (全屏覆盖层) =================
  // 用途: 语音说"获取终端传感器数据" → 整屏切到 B 板(终端)上报的 11 项数据。
  // 实现: 和屏保一样是独立的全屏覆盖层, 默认 HIDDEN; 显示时 clear_flag。
  //       ★ 不碰聊天界面的任何对象, 出问题也只是少一个覆盖层, 不会影响原有功能。
  //
  // 数据来源: main.cpp 从板间串口(UART1, 39/40)收到 "ALL,T=..,H=.." 后调 SetTerminalData。
  // 自动关闭: 显示满 kTerminalPageAutoHideMs 后由 TerminalPageTick() 关掉。
  // ★ 每 2 秒收到新数据时会顺带 WakeUp(), 因此屏保不会在数据页显示期间抢屏。
  static constexpr int kTermItemCount = 11;
  struct TerminalData {
    float t;      // 空气温度 C
    float h;      // 空气湿度 %
    float pr;     // 气压 hPa
    float co2;    // 空气 CO2 ppm
    float lx;     // 光照强度 lx
    float st;     // 土壤温度 C
    float sh;     // 土壤湿度 %
    float n;      // 土壤氮
    float p;      // 土壤磷
    float k;      // 土壤钾
    float ph;     // 土壤 PH
    bool  ok[kTermItemCount];   // 每项是否有效(无效显示 --)
    uint32_t rx_ms;
  };
  void ShowTerminalPage();
  void HideTerminalPage();
  void SetTerminalData(const TerminalData& d);
  bool IsTerminalPageActive() const { return terminal_page_active_; }
  // 由 loop 周期调用: 显示超时自动关闭
  void TerminalPageTick();

  // ================= 自动监测面板 (全屏覆盖层, 2026-10-23 新增) =================
  // 用途: 语音说"全自动检测"/"打开光照监测"后, 主控持续监测终端传感器并联动执行器,
  //       这个面板整屏显示 6 个通道的 当前值/阈值/状态/已联动动作, 答辩时就是演示面板。
  // 实现: 与终端数据页同一套模式 —— 挂在 screen 上的全屏覆盖层, 默认 HIDDEN。
  // ★ 与终端数据页的区别: 【没有超时自动关闭】。监测运行期间面板一直显示,
  //       说"停止监测"(main.cpp 调 HideMonitorPage)才收起 —— 演示时不能让它自己消失。
  //       代价是监测期间屏保永远不会进来(SetMonitorData 每次都 WakeUp), 这是故意的。
  static constexpr int kMonItemCount = 5;   // 独立版: 空气温度/空气湿度/气压/土壤温度/土壤湿度
  struct MonitorRow {
    std::string main_text;  // 第一行: "光照 12800 lx  宜 3000~20000"
    std::string sub_text;   // 第二行: "正常" / "超上限 → 遮阳伞展开" / "无数据" / "未监测"
    uint8_t state;          // 0 正常 1 超上限 2 低于下限 3 无数据 4 未监测
  };
  void ShowMonitorPage();
  void HideMonitorPage();
  bool IsMonitorPageActive() const { return monitor_page_active_; }
  // 由 main.cpp 的监测状态机每 2 秒调用一次。内部只重写"文本变了"的 label。
  // ★ 面板未显示时直接忽略(不锁 LVGL, 零开销); 显示时会顺带 WakeUp() 防屏保抢屏。
  void SetMonitorData(const MonitorRow* rows, const char* title);

  // ================= 环境完整数据页 (全屏覆盖层, 2026-11-06 新增) =================
  // 用途: 语音"显示完整数据/全部数据"时, 整屏列表显示 空气温湿度气压 + 土壤温湿度。
  // 实现: 与终端数据页/监测面板同一套模式 —— 挂在 screen 上的全屏覆盖层, 默认 HIDDEN。
  // 数据: main.cpp 调 ShowEnvPage(title, lines, count), 一次传入 5 行文本。
  // 收起: 说"返回"(self.display.home)或按 BOOT 键, 由 main.cpp 调 HideEnvPage。
  static constexpr int kEnvItemCount = 6;  // 最多 6 行(5 项数据 + 余量)
  void ShowEnvPage(const char* title, const char* const* lines, int count);
  void HideEnvPage();
  bool IsEnvPageActive() const { return env_page_active_; }

  // ================= 照片回显页 (全屏覆盖层, 2026-10-24 新增) =================
  // 用途: 语音"拍一张照"成功后, 把刚拍的 JPEG 解码成 RGB565 整屏显示几秒。
  // 数据: main.cpp 用 TJpgDec 解码到 PSRAM, 以 lv_image_dsc_t(RGB565) 传入。
  //       尺寸正好 240x320 (= width_ x height_), 居中摆放即整屏铺满。
  // 自动关闭: kPhotoPageAutoHideMs 毫秒后由 PhotoPageTick() 关掉(恢复对话界面);
  //   该常量为 0 时 = 常显, PhotoPageTick() 直接返回, 一切交给用户主动收起。
  // ★ 2026-10-31 用户要求改为【常显】: 拍照后照片一直留在屏幕上, 直到用户说"返回"
  //   (MCP 工具 self.display.home) 或按 BOOT 键才收起。0 = 不自动关闭。
  //   收起后 main.cpp 会把照片像素缓冲释放掉(= 删除照片), 下次拍照重新分配。
  static constexpr uint32_t kPhotoPageAutoHideMs = 0;
  void ShowPhotoPage(const lv_image_dsc_t* img);
  void HidePhotoPage();
  bool IsPhotoPageActive() const { return photo_page_active_; }
  // 由 main.cpp 的 loop 每轮调用(与 TerminalPageTick 并排), 超时自动关闭。
  void PhotoPageTick();

 private:
  void StartChatUi();
  void StartSensorUi();
  struct ThemeColors {
    lv_color_t background;
    lv_color_t text;
    lv_color_t chat_background;
    lv_color_t user_bubble;
    lv_color_t assistant_bubble;
    lv_color_t assistant_text;  // AI 回复文字颜色(白底上用蓝色)
    lv_color_t system_bubble;
    lv_color_t system_text;
    lv_color_t border;
    lv_color_t low_battery;
  };
  uint32_t width_ = 0;
  uint32_t height_ = 0;
  Kind kind_ = Kind::kChat;
  lv_display_t* display_ = nullptr;
  // Chat-mode UI
  lv_obj_t* container_ = nullptr;
  lv_obj_t* status_bar_ = nullptr;
  lv_obj_t* content_ = nullptr;
  lv_obj_t* content_left_ = nullptr;
  lv_obj_t* content_right_ = nullptr;
  lv_obj_t* emotion_label_ = nullptr;
  lv_obj_t* chat_message_label_ = nullptr;
  lv_obj_t* network_label_ = nullptr;
  lv_obj_t* notification_label_ = nullptr;
  lv_obj_t* status_label_ = nullptr;
  lv_obj_t* mute_label_ = nullptr;
  // Sensor-mode UI
  lv_obj_t* sensor_title_label_ = nullptr;
  lv_obj_t* sensor_temp_label_ = nullptr;
  lv_obj_t* sensor_hum_label_ = nullptr;
  lv_obj_t* sensor_msg_label_ = nullptr;
  // Screen-saver UI (chat mode only)
  lv_obj_t* screen_saver_ = nullptr;       // 全屏覆盖层, 默认 HIDDEN
  lv_obj_t* saver_clock_label_ = nullptr;  // 大时钟 "HH:MM"(普通文本, 72px)
  lv_obj_t* saver_date_label_ = nullptr;   // 日期 2026-10-13 周二(26px)
  lv_obj_t* saver_env_label_ = nullptr;    // 温湿度 "温 32C 湿 33%"(26px, 日期下方)
  // Terminal-data UI (chat mode only): 全屏覆盖层, 默认 HIDDEN
  lv_obj_t* terminal_page_ = nullptr;                 // 容器: 铺满全屏
  lv_obj_t* term_val_[kTermItemCount] = {};           // 11 个数值 label(26px)
  bool      terminal_page_active_ = false;            // 当前是否显示着
  uint32_t  terminal_shown_ms_ = 0;                   // 显示起始时刻(用于超时)
  // 上次写入的数值, 用于"只在数字变化时才刷 label"(避免每 2 秒无谓重绘)
  float     term_last_[kTermItemCount] = {};
  bool      term_last_ok_[kTermItemCount] = {};
  bool      term_last_inited_ = false;
  // 自动监测面板 UI (chat mode only): 全屏覆盖层, 默认 HIDDEN
  lv_obj_t* monitor_page_ = nullptr;                 // 容器: 铺满全屏
  lv_obj_t* monitor_title_label_ = nullptr;          // 标题 "自动监测-草莓"
  lv_obj_t* mon_main_[kMonItemCount] = {};           // 每通道第一行: 值 + 阈值
  lv_obj_t* mon_sub_[kMonItemCount] = {};            // 每通道第二行: 状态 + 动作
  std::string mon_main_last_[kMonItemCount];         // 上次文本(变化才重写)
  std::string mon_sub_last_[kMonItemCount];
  std::string mon_title_last_;                       // 上次标题
  uint8_t   mon_state_last_[kMonItemCount] = {};     // 上次状态(颜色变化也要重写)
  bool      mon_last_inited_ = false;
  bool      monitor_page_active_ = false;            // 当前是否显示着
  // 环境完整数据页 UI (chat mode only): 全屏覆盖层, 默认 HIDDEN (2026-11-06 新增)
  lv_obj_t* env_page_ = nullptr;                     // 容器: 铺满全屏
  lv_obj_t* env_title_label_ = nullptr;              // 标题 "环境完整数据"
  lv_obj_t* env_name_[kEnvItemCount] = {};           // 每行名称(左)
  lv_obj_t* env_val_[kEnvItemCount] = {};            // 每行数值(右)
  bool      env_page_active_ = false;                // 当前是否显示着
  // 照片回显页 UI (chat mode only): 全屏覆盖层, 懒创建
  lv_obj_t* photo_page_ = nullptr;                   // 容器: 黑底铺满全屏
  lv_obj_t* photo_img_ = nullptr;                    // 照片 image 控件
  bool      photo_page_active_ = false;              // 当前是否显示着
  uint32_t  photo_shown_ms_ = 0;                     // 显示起始时刻(超时自动关闭)
  // ★ 2026-10-13 清理: 原 7 段数码管相关成员已删除(saver_clock_row_ / saver_segs_[4][7] /
  //   saver_colon_dot_[2]), 底部提示行也按用户要求整行去掉(saver_hint_label_)。
  // 屏保温湿度数据缓存(由 SetEnvForScreenSaver 更新, 只在数据变化时刷 label)
  float saver_env_temp_ = 0.0f;
  float saver_env_hum_ = 0.0f;
  bool saver_env_valid_ = false;
  bool saver_env_has_value_ = false;  // 是否曾收到过有效值(用于首次强制刷新)
  bool screen_saver_active_ = false;                 // 当前是否处于屏保
  // ★★ 语音睡觉锁: 语音指令进屏保后置 true, WakeUp() 在此标志下【只重置计时不关屏保】。
  // 由 main.cpp 在检测到"用户说了新话"(ChatMessageEvent/kUser)时调 ReleaseSleepLock() 解除。
  // 没有它的话, AI 回的那句 "好的已进入待机" 的 TTS 事件会立刻把刚点亮的屏保关掉。
  bool sleep_lock_ = false;
  uint32_t sleep_lock_armed_ms_ = 0;  // 上锁时刻, 用于"上锁宽限期"判定
  // 计时基准。初值用哨兵(见 display.cpp 的 kNoActivitySentinel = 0xFFFFFFFF),
  // 不可用 0: esp_timer_get_time() 开机极早期可能返回 <1000us, 使 NowMs() 恰好落在 0 而误判。
  uint32_t last_activity_ms_ = 0xFFFFFFFFu;          // 最近一次"有活动"的时间戳
  uint32_t saver_last_clock_update_ms_ = 0xFFFFFFFFu;  // 屏保时钟上次刷新时间戳
  ThemeColors current_theme_;
};

#endif
