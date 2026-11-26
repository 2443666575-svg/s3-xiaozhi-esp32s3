#pragma once

#include <stdint.h>
#include "lvgl.h"

#define WALLPAPER_W 240
#define WALLPAPER_H 320

#ifdef __cplusplus
extern "C" {
#endif

// 240x320 RGB565 照片数据(小端序), 153600 字节, 存放在固件 flash(DROM)
extern const unsigned char kWallpaperMap[WALLPAPER_W * WALLPAPER_H * 2];

// 图片描述符。定义在 wallpaper_data.c 中(C 编译),
// 因为 lv_image_dsc_t 的居中指定初始化器是 C 语法, C++ 编译不支持。
extern const lv_image_dsc_t kWallpaperDsc;

#ifdef __cplusplus
}
#endif
