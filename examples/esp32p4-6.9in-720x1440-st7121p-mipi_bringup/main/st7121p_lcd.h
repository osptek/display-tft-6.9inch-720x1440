#pragma once

#include "esp_err.h"
#include "esp_lcd_panel_io.h"

/* 发送 docs/ST7121P+TM6.9_2LANE.c 里的初始化序列，最后 Sleep Out / Display On。 */
esp_err_t st7121p_send_init(esp_lcd_panel_io_handle_t io);
