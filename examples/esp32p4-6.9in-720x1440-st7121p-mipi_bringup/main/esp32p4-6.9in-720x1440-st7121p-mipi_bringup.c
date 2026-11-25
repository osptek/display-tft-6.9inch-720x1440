#include "st7121p_lcd.h"

#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "esp_lcd_mipi_dsi.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_touch_st7123.h"
#include "esp_clk_tree.h"
#include "esp_ldo_regulator.h"
#include "esp_private/esp_clk_tree_common.h"
#include "esp_log.h"
#include "esp_lvgl_port.h"
#include "esp_rom_sys.h"
#include "hal/mipi_dsi_host_ll.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "lvgl.h"
#include "widgets/lv_demo_widgets.h"

static const char *TAG = "bringup";

/* YDP690TE001-V1 / ST7121P，RGB 时序来自 docs 里的 SSD2828 截图。
 * H: 2 + 40 + 720 + 40，V: 2 + 25 + 1440 + 60，PCLK 74 MHz，约 60 Hz。
 * 规格书颜色数为 16.7M，DSI 线上用 RGB888；LVGL 用 RGB565，由 DPI 桥转换。 */
#define LCD_H_RES              720
#define LCD_V_RES              1440
#define LCD_DPI_CLK_MHZ        74
#define LCD_HSYNC              2
#define LCD_HBP                40
#define LCD_HFP                40
#define LCD_VSYNC              2
#define LCD_VBP                25
#define LCD_VFP                280
#define LCD_LANE_NUM           2
/* 2 lane 下 non-burst 的 lane 速率只能等于像素速率（888 Mbps），装不下包头和消隐包，
 * 帧尾会闪。用 burst + 1000 Mbps 留出余量。 */
#define LCD_DSI_NON_BURST      0
#if LCD_DSI_NON_BURST
#define LCD_LANE_BITRATE_MBPS  888
#else
#define LCD_LANE_BITRATE_MBPS  1000
#endif

/* 开发板 J3 的 TE（GPIO4）接到 6.9 寸转接板 J1-7（TE-IO4）。
 * 这根线是转接板 SY7200 的 EN，R1 10k 下拉，高电平才开背光。
 * 开发板自己的 GPIO20 只给板载背光升压，没有进这块屏的 LED。
 * LCD_RST（GPIO5，J1-6）经过 2SK3018 反相：高电平让屏进入复位，低电平释放。
 * MIPI PHY 的 2.5V 来自 ESP32-P4 的 LDO_VO3。 */
#define LCD_PIN_BK             GPIO_NUM_4
#define LCD_PIN_RST            GPIO_NUM_5
/* 开发板 J3：SDA=GPIO7，SCL=GPIO8，INT=GPIO21，RST=GPIO6。
 * 转接板上 TP_RST 经过 2SK3018 反相，GPIO 高电平才是触摸复位。 */
#define TOUCH_PIN_SDA          GPIO_NUM_7
#define TOUCH_PIN_SCL          GPIO_NUM_8
#define TOUCH_PIN_RST          GPIO_NUM_6
/* 转接板 Q4（2SK3018）：屏的 TP_INT 接栅极，ESP 的 GPIO21（J1-25 TP_INTo）是漏极，
 * R6 10k 上拉到 3.3V，源极接地，图上标注「注意逻辑反相」。
 * 芯片中断空闲为高，有坐标时拉低。反相后 GPIO21 空闲为 0，有数据时产生上升沿。 */
#define TOUCH_PIN_INT          GPIO_NUM_21
#define TOUCH_I2C_HZ           100000
#define MIPI_PHY_LDO_CHAN      3
#define MIPI_PHY_LDO_MV        2500
#define MIPI_PHY_SETTLE_MS     200

#define LVGL_DRAW_LINES        80

static esp_lcd_panel_handle_t s_panel;

static void phy_power_on(void)
{
    esp_ldo_channel_handle_t ldo = NULL;
    const esp_ldo_channel_config_t cfg = {
        .chan_id = MIPI_PHY_LDO_CHAN,
        .voltage_mv = MIPI_PHY_LDO_MV,
    };
    ESP_ERROR_CHECK(esp_ldo_acquire_channel(&cfg, &ldo));
    vTaskDelay(pdMS_TO_TICKS(MIPI_PHY_SETTLE_MS));
    ESP_LOGI(TAG, "MIPI DPHY 2.5V on");
}

static void panel_reset(void)
{
    const gpio_config_t rst = {
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = 1ULL << LCD_PIN_RST,
    };
    ESP_ERROR_CHECK(gpio_config(&rst));
    /* 反相后：GPIO 高 = 屏 RST 低 */
    gpio_set_level(LCD_PIN_RST, 1);
    vTaskDelay(pdMS_TO_TICKS(100));
    gpio_set_level(LCD_PIN_RST, 0);
    vTaskDelay(pdMS_TO_TICKS(200));
}

static void backlight_init(bool on)
{
    const gpio_config_t bk = {
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = 1ULL << LCD_PIN_BK,
    };
    ESP_ERROR_CHECK(gpio_config(&bk));
    gpio_set_level(LCD_PIN_BK, on ? 1 : 0);
}

static void lcd_init(void)
{
    backlight_init(true);
    phy_power_on();

    esp_lcd_dsi_bus_handle_t bus = NULL;
    const esp_lcd_dsi_bus_config_t bus_cfg = {
        .bus_id = 0,
        .num_data_lanes = LCD_LANE_NUM,
        .lane_bit_rate_mbps = LCD_LANE_BITRATE_MBPS,
    };
    ESP_ERROR_CHECK(esp_lcd_new_dsi_bus(&bus_cfg, &bus));

    esp_lcd_panel_io_handle_t io = NULL;
    const esp_lcd_dbi_io_config_t dbi_cfg = {
        .virtual_channel = 0,
        .lcd_cmd_bits = 8,
        .lcd_param_bits = 8,
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_dbi(bus, &dbi_cfg, &io));

    /* 总线进入 LP-11 后再释放复位，面板才能收到后续命令 */
    panel_reset();
    ESP_ERROR_CHECK(st7121p_send_init(io));

    /* 默认 PLL_F240M 只能整数分频，74 MHz 会变成 240/3 = 80 MHz（约 65 Hz）。
     * TDDI 按帧节拍在 LongV 里扫触摸，用 APLL 精确给 74 MHz，回到厂家的 60.4 Hz。 */
    uint32_t apll_hz = 0;
    ESP_ERROR_CHECK(esp_clk_tree_enable_src(SOC_MOD_CLK_APLL, true));
    ESP_ERROR_CHECK(esp_clk_tree_src_set_freq_hz(SOC_MOD_CLK_APLL, LCD_DPI_CLK_MHZ * 1000000, &apll_hz));
    ESP_LOGI(TAG, "DPI clock APLL %" PRIu32 " Hz", apll_hz);

    const esp_lcd_dpi_panel_config_t dpi_cfg = {
        .virtual_channel = 0,
        .dpi_clk_src = MIPI_DSI_DPI_CLK_SRC_APLL,
        .dpi_clock_freq_mhz = LCD_DPI_CLK_MHZ,
        .in_color_format = LCD_COLOR_FMT_RGB565,
        .out_color_format = LCD_COLOR_FMT_RGB888,
        .num_fbs = 1,
        .video_timing = {
            .h_size = LCD_H_RES,
            .v_size = LCD_V_RES,
            .hsync_pulse_width = LCD_HSYNC,
            .hsync_back_porch = LCD_HBP,
            .hsync_front_porch = LCD_HFP,
            .vsync_pulse_width = LCD_VSYNC,
            .vsync_back_porch = LCD_VBP,
            .vsync_front_porch = LCD_VFP,
        },
        /* TDDI 在 LongV 消隐里扫触摸，消隐期不能让链路掉进 LP */
        .flags.disable_lp = true,
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_dpi(bus, &dpi_cfg, &s_panel));
    /* esp_lcd 默认每帧结束做一次 BTA 要屏回 ACK，正好落在 TDDI 扫触摸的帧消隐里，
     * 触摸被打断、帧尾闪。SSD2828 不做这个，关掉。 */
    mipi_dsi_host_ll_dpi_enable_frame_ack(MIPI_DSI_LL_GET_HOST(0), false);
#if LCD_DSI_NON_BURST
    /* esp_lcd 写死 burst，视频模式要到 panel_init 才打开，这里先改掉 */
    mipi_dsi_host_ll_dpi_set_video_burst_type(MIPI_DSI_LL_GET_HOST(0), MIPI_DSI_LL_VIDEO_NON_BURST_WITH_SYNC_EVENTS);
    ESP_LOGI(TAG, "DSI non-burst sync events, %d Mbps/lane", LCD_LANE_BITRATE_MBPS);
#endif
    ESP_ERROR_CHECK(esp_lcd_panel_init(s_panel));
    ESP_LOGI(TAG, "LCD video mode on");
}

/* 转接板 TP_RST 经 2SK3018 反相：GPIO 高 = 触摸复位，GPIO 低 = 释放。
 * 驱动自带的复位只等 10 ms，ST7121P 从 Flash 起来会 NACK。这里自己复位并等够时间。 */
static void touch_hw_reset(bool inverted)
{
    const gpio_config_t rst = {
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = 1ULL << TOUCH_PIN_RST,
    };
    ESP_ERROR_CHECK(gpio_config(&rst));
    gpio_set_level(TOUCH_PIN_RST, inverted ? 1 : 0);
    vTaskDelay(pdMS_TO_TICKS(20));
    gpio_set_level(TOUCH_PIN_RST, inverted ? 0 : 1);
    vTaskDelay(pdMS_TO_TICKS(200));
}

static int touch_probe(i2c_master_bus_handle_t bus)
{
    /* 不能扫整条总线：0x28 是 ST7121P 的地址模式（ICP）入口，碰一下触摸 MCU 就停止扫描 */
    esp_err_t err = i2c_master_probe(bus, ESP_LCD_TOUCH_IO_I2C_ST7123_ADDRESS, 20);
    ESP_LOGI(TAG, "I2C 0x%02X probe: %s", ESP_LCD_TOUCH_IO_I2C_ST7123_ADDRESS, esp_err_to_name(err));
    if (err == ESP_ERR_TIMEOUT) {
        ESP_LOGW(TAG, "SDA=%d SCL=%d", gpio_get_level(TOUCH_PIN_SDA), gpio_get_level(TOUCH_PIN_SCL));
    }
    return err == ESP_OK ? ESP_LCD_TOUCH_IO_I2C_ST7123_ADDRESS : -1;
}

static uint16_t touch_sense_counter(i2c_master_dev_handle_t dev)
{
    const uint8_t reg[2] = {0x00, 0x0A};
    uint8_t cnt[2] = {0};
    if (i2c_master_transmit(dev, reg, sizeof(reg), 50) != ESP_OK) {
        return 0xFFFF;
    }
    esp_rom_delay_us(150);
    if (i2c_master_receive(dev, cnt, sizeof(cnt), 50) != ESP_OK) {
        return 0xFFFF;
    }
    return (cnt[0] << 8) | cnt[1];
}

static void touch_log_sensing(i2c_master_bus_handle_t bus)
{
    i2c_master_dev_handle_t dev = NULL;
    const i2c_device_config_t cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = ESP_LCD_TOUCH_IO_I2C_ST7123_ADDRESS,
        .scl_speed_hz = TOUCH_I2C_HZ,
    };
    if (i2c_master_bus_add_device(bus, &cfg, &dev) != ESP_OK) {
        return;
    }
    for (int i = 0; i < 4; i++) {
        ESP_LOGI(TAG, "sensing counter %04X", touch_sense_counter(dev));
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    i2c_master_bus_rm_device(dev);
}

static esp_lcd_touch_handle_t touch_init(void)
{
    const i2c_master_bus_config_t bus_cfg = {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = TOUCH_PIN_SDA,
        .scl_io_num = TOUCH_PIN_SCL,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .flags.enable_internal_pullup = true,
    };
    i2c_master_bus_handle_t bus = NULL;
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_cfg, &bus));

    int addr = touch_probe(bus);
    if (addr < 0) {
        ESP_LOGW(TAG, "触摸无应答，改试直接复位");
        touch_hw_reset(false);
        addr = touch_probe(bus);
    }
    if (addr < 0) {
        ESP_LOGE(TAG, "触摸 I2C 无应答");
        return NULL;
    }
    touch_log_sensing(bus);

    esp_lcd_panel_io_i2c_config_t io_cfg = ESP_LCD_TOUCH_IO_I2C_ST7123_CONFIG();
    io_cfg.dev_addr = addr;
    io_cfg.scl_speed_hz = TOUCH_I2C_HZ;
    esp_lcd_panel_io_handle_t io = NULL;
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_i2c(bus, &io_cfg, &io));

    /* 复位已经完成，不要再让驱动把脚拉一下只等 10 ms。
     * 先轮询读坐标，INT 只用来观察电平，确认芯片有没有在上报。 */
    const esp_lcd_touch_config_t tp_cfg = {
        .x_max = LCD_H_RES,
        .y_max = LCD_V_RES,
        .rst_gpio_num = GPIO_NUM_NC,
        .int_gpio_num = GPIO_NUM_NC,
    };
    esp_lcd_touch_handle_t tp = NULL;
    esp_err_t err = esp_lcd_touch_new_i2c_st7123(io, &tp_cfg, &tp);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "触摸初始化失败: %s", esp_err_to_name(err));
        return NULL;
    }
    const gpio_config_t int_cfg = {
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = 1ULL << TOUCH_PIN_INT,
    };
    gpio_config(&int_cfg);
    ESP_LOGI(TAG, "TP_INT level %d", gpio_get_level(TOUCH_PIN_INT));
    return tp;
}

static void tp_int_watch_task(void *arg)
{
    int prev = gpio_get_level(TOUCH_PIN_INT);
    uint32_t edges = 0;
    while (1) {
        int lvl = gpio_get_level(TOUCH_PIN_INT);
        if (lvl != prev) {
            edges++;
            if (edges <= 20 || (edges % 100) == 0) {
                ESP_LOGI(TAG, "TP_INT -> %d (edges %" PRIu32 ")", lvl, edges);
            }
            prev = lvl;
        }
        vTaskDelay(1);
    }
}

static void lvgl_init(esp_lcd_touch_handle_t tp)
{
    const lvgl_port_cfg_t lvgl_cfg = {
        .task_priority = 4,
        .task_stack = 16 * 1024,
        .task_affinity = -1,
        .task_max_sleep_ms = 500,
        .timer_period_ms = 5,
    };
    ESP_ERROR_CHECK(lvgl_port_init(&lvgl_cfg));

    const lvgl_port_display_cfg_t disp_cfg = {
        .panel_handle = s_panel,
        .buffer_size = LCD_H_RES * LVGL_DRAW_LINES,
        .double_buffer = true,
        .hres = LCD_H_RES,
        .vres = LCD_V_RES,
        .color_format = LV_COLOR_FORMAT_RGB565,
        .flags = {
            .buff_spiram = true,
            /* DPI 面板没有 swap_xy/mirror，交给 LVGL 软件旋转，避免启动时那两条报错 */
            .sw_rotate = true,
        },
    };
    const lvgl_port_display_dsi_cfg_t dsi_cfg = {0};
    lv_display_t *disp = lvgl_port_add_disp_dsi(&disp_cfg, &dsi_cfg);
    assert(disp);

    if (tp) {
        const lvgl_port_touch_cfg_t touch_cfg = {
            .disp = disp,
            .handle = tp,
        };
        assert(lvgl_port_add_touch(&touch_cfg));
    }

    if (lvgl_port_lock(0)) {
        lv_demo_widgets();
        lvgl_port_unlock();
    }
}

void app_main(void)
{
    /* TDDI 的触摸 MCU 要看到显示的 Sleep Out(0x11) 才开始扫描。
     * 必须先复位触摸、再初始化显示，否则复位后的触摸一直以为屏在睡眠。 */
    touch_hw_reset(true);
    lcd_init();
    esp_lcd_touch_handle_t tp = touch_init();
    lvgl_init(tp);
    xTaskCreate(tp_int_watch_task, "tp_int", 3072, NULL, 2, NULL);
    ESP_LOGI(TAG, "LVGL running");
}
