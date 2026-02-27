
#include "esp_lcd_st7305.h"
#include "soc/soc_caps.h"
#include "esp_check.h"
#include "esp_lcd_types.h"
#include <stdlib.h>
#include <sys/cdefs.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_lcd_panel_interface.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_commands.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_check.h"
// 定义面板操作函数

static esp_err_t panel_st7305_del(esp_lcd_panel_t *panel);
static esp_err_t panel_st7305_reset(esp_lcd_panel_t *panel);
static esp_err_t panel_st7305_init(esp_lcd_panel_t *panel);
static esp_err_t panel_st7305_draw_bitmap(esp_lcd_panel_t *panel, int x_start, int y_start, int x_end, int y_end, const void *color_data);
static esp_err_t panel_st7305_invert_color(esp_lcd_panel_t *panel, bool invert_color_data);
static esp_err_t panel_st7305_mirror(esp_lcd_panel_t *panel, bool mirror_x, bool mirror_y);
static esp_err_t panel_st7305_swap_xy(esp_lcd_panel_t *panel, bool swap_axes);
static esp_err_t panel_st7305_set_gap(esp_lcd_panel_t *panel, int x_gap, int y_gap);
static esp_err_t panel_st7305_disp_on_off(esp_lcd_panel_t *panel, bool off);
static inline void _swap_int(int *a, int *b) {
    int temp = *a;
    *a = *b;
    *b = temp;
}
const char *TAG = "st7305";
// 添加颜色定义
typedef union {
    uint16_t full;
} st7305_color_t;
typedef struct {
    esp_lcd_panel_t base;
    esp_lcd_panel_io_handle_t io;
    int reset_gpio_num;
    bool reset_level;
    int x_gap;
    int y_gap;
    int width;
    int height;
    uint8_t rotation;    // 当前旋转角度 (0, 1, 2, 3 对应 0°, 90°, 180°, 270°)

    uint8_t fb_bits_per_pixel;
    uint8_t madctl_val; // save current value of LCD_CMD_MADCTL register
    uint8_t colmod_val; // save current value of LCD_CMD_COLMOD register
    const st7305_lcd_init_cmd_t *init_cmds;
    uint16_t init_cmds_size;
} st7305_panel_t;

#ifdef CONFIG_ESP_LCD_ST7305_SCREEN_SIZE_2_9
static const st7305_lcd_init_cmd_t st7305_init_cmds_2_9_inch[] = {
    {0xD6, (uint8_t[]){0x13, 0x02}, 2, 0},           // NVM Load Control
    {0xD1, (uint8_t[]){0x01}, 1, 0},                 // Booster Enable
    {0xC0, (uint8_t[]){0x08, 0x06}, 2, 0},          // Gate Voltage Setting
    {0xC1, (uint8_t[]){0x3C, 0x3E, 0x3C, 0x3C}, 4, 0}, // VSHP Setting (4.8V)
    {0xC2, (uint8_t[]){0x23, 0x21, 0x23, 0x23}, 4, 0}, // VSLP Setting (0.98V)
    {0xC4, (uint8_t[]){0x5A, 0x5C, 0x5A, 0x5A}, 4, 0}, // VSHN Setting (-3.6V)
    {0xC5, (uint8_t[]){0x37, 0x35, 0x37, 0x37}, 4, 0}, // VSLN Setting (0.22V)
    {0xB2, (uint8_t[]){0x05}, 1, 0},                // Frame Rate Control
    {0xB3, (uint8_t[]){0xE5, 0xF6, 0x17, 0x77, 0x77, 0x77, 0x77, 0x77, 0x77, 0x71}, 10, 0}, // HPM
    {0xB4, (uint8_t[]){0x05, 0x46, 0x77, 0x77, 0x77, 0x77, 0x76, 0x45}, 8, 0}, // LPM
    {0x62, (uint8_t[]){0x32, 0x03, 0x1F}, 3, 0},    // Gate Timing Control
    {0xB7, (uint8_t[]){0x13}, 1, 0},                // Source EQ Enable
    {0xB0, (uint8_t[]){0x60}, 1, 0},                // Gate Line Setting: 384 line
    {0x11, NULL, 0, 100},                           // Sleep out
    {0xC9, (uint8_t[]){0x00}, 1, 0},                // Source Voltage Select
    {0x36, (uint8_t[]){0x00}, 1, 0},                // Memory Data Access Control
    {0x3A, (uint8_t[]){0x11}, 1, 0},                // Data Format Select
    {0xB9, (uint8_t[]){0x20}, 1, 0},                // Gamma Mode Setting
    {0xB8, (uint8_t[]){0x29}, 1, 0},                // Panel Setting
    {0x2A, (uint8_t[]){0x17, 0x24}, 2, 0},          // Column Address Setting
    {0x2B, (uint8_t[]){0x00, 0xBF}, 2, 0},          // Row Address Setting
    {0xD0, (uint8_t[]){0xFF}, 1, 0},                // Auto power down
    {0x39, NULL, 0, 0},                             // Frame rate mode
    {0x29, NULL, 0, 100},                           // Display on
};
static const st7305_lcd_init_cmd_t *st7305_init_cmds = st7305_init_cmds_2_9_inch;
static const size_t st7305_init_cmds_size = sizeof(st7305_init_cmds_2_9_inch) / sizeof(st7305_lcd_init_cmd_t);

#elif defined CONFIG_ESP_LCD_ST7305_SCREEN_SIZE_1_54
static const st7305_lcd_init_cmd_t st7305_init_cmds_1_54_inch[] = {
    {0xD6, (uint8_t[]){0x17, 0x02}, 2, 0},           // NVM Load Control
    {0xD1, (uint8_t[]){0x01}, 1, 0},                 // Booster Enable

    {0xC0, (uint8_t[]){0x12, 0x0a}, 2, 0},          // Gate Voltage Setting
    {0xC1, (uint8_t[]){0x73, 0x3E, 0x3C, 0x3C}, 4, 0}, // VSHP Setting (4.8V)
    {0xC2, (uint8_t[]){0x00, 0x21, 0x23, 0x23}, 4, 0}, // VSLP Setting (0.98V)
    {0xC4, (uint8_t[]){0x32, 0x5C, 0x5A, 0x5A}, 4, 0}, // VSHN Setting (-3.6V)
    {0xC5, (uint8_t[]){0x32, 0x35, 0x37, 0x37}, 4, 0}, // VSLN Setting (0.22V)

    {0xD8, (uint8_t[]){0x80, 0xE9}, 2, 0},           // OSC Setting: Enable OSC, HPM Frame Rate Max = 51Hz
    {0xB2, (uint8_t[]){0x12}, 1, 0},                 // Frame Rate Control (HPM=51hz)

    {0xB3, (uint8_t[]){0xE5, 0xF6, 0x17, 0x77, 0x77, 0x77, 0x77, 0x77, 0x77, 0x71}, 10, 0}, // Update Period Gate EQ Control in HPM
    {0xB4, (uint8_t[]){0x05, 0x46, 0x77, 0x77, 0x77, 0x77, 0x76, 0x45}, 8, 0}, // Update Period Gate EQ Control in LPM

    {0x62, (uint8_t[]){0x32, 0x03, 0x1F}, 3, 0},    // Gate Timing Control
    {0xB7, (uint8_t[]){0x13}, 1, 0},                // Source EQ Enable
    {0xB0, (uint8_t[]){0x32}, 1, 0},                // Gate Line Setting: 200 line (50 * 4)

    {0x11, NULL, 0, 120},                           // Sleep out (delay 120ms)

    {0xC9, (uint8_t[]){0x00}, 1, 0},                // Source Voltage Select
    {0x36, (uint8_t[]){0x48}, 1, 0},                // Memory Data Access Control (MX=1; DO=1)
    {0x3A, (uint8_t[]){0x11}, 1, 0},                // Data Format Select
    {0xB9, (uint8_t[]){0x20}, 1, 0},                // Gamma Mode Setting
    {0xB8, (uint8_t[]){0x29}, 1, 0},                // Panel Setting

    {0x2A, (uint8_t[]){0x16, 0x26}, 2, 0},          // Column Address Setting (0x16,0x26)
    {0x2B, (uint8_t[]){0x00, 0x63}, 2, 0},          // Row Address Setting (0x00,0x63)
    {0x35, (uint8_t[]){0x00}, 1, 0},                // TE

    {0xD0, (uint8_t[]){0xFF}, 1, 0},                // Auto power down ON
    {0x38, NULL, 0, 0},                             // HPM: High Power Mode ON

    {0x29, NULL, 0, 0},                             // Display ON
    {0x20, NULL, 0, 0},                             // Display Inversion Off

    {0xBB, (uint8_t[]){0x4F}, 1, 0},                // Enable Clear RAM, clear RAM to 0
};
static const st7305_lcd_init_cmd_t *st7305_init_cmds = st7305_init_cmds_1_54_inch;
static const size_t st7305_init_cmds_size = sizeof(st7305_init_cmds_1_54_inch) / sizeof(st7305_lcd_init_cmd_t);

#endif

esp_err_t esp_lcd_new_panel_st7305(const esp_lcd_panel_io_handle_t io, const esp_lcd_panel_dev_config_t *panel_dev_config,
                                   esp_lcd_panel_handle_t *ret_panel)
{
    esp_err_t ret = ESP_OK;
    st7305_panel_t *st7305 = NULL;
    gpio_config_t io_conf = { 0 };

    ESP_GOTO_ON_FALSE(io && panel_dev_config && ret_panel, ESP_ERR_INVALID_ARG, err, TAG, "invalid argument");
    st7305 = (st7305_panel_t *)calloc(1, sizeof(st7305_panel_t));
    ESP_GOTO_ON_FALSE(st7305, ESP_ERR_NO_MEM, err, TAG, "no mem for st7305 panel");

    if (panel_dev_config->reset_gpio_num >= 0) {
        io_conf.mode = GPIO_MODE_OUTPUT;
        io_conf.pin_bit_mask = 1ULL << panel_dev_config->reset_gpio_num;
        ESP_GOTO_ON_ERROR(gpio_config(&io_conf), err, TAG, "configure GPIO for RST line failed");
    }
    st7305->width = ST7305_WIDTH;
    st7305->height = ST7305_HEIGHT;
    st7305->madctl_val = 0;  // 初始化MADCTL值
    st7305->rotation = 0;  // 初始化MADCTL值

    st7305->io = io;
    st7305->reset_gpio_num = panel_dev_config->reset_gpio_num;
    st7305->reset_level = panel_dev_config->flags.reset_active_high;
    if (panel_dev_config->vendor_config) {
        st7305->init_cmds = ((st7305_vendor_config_t *)panel_dev_config->vendor_config)->init_cmds;
        st7305->init_cmds_size = ((st7305_vendor_config_t *)panel_dev_config->vendor_config)->init_cmds_size;
    }
    st7305->base.del = panel_st7305_del;
    st7305->base.reset = panel_st7305_reset;
    st7305->base.init = panel_st7305_init;
    st7305->base.draw_bitmap = panel_st7305_draw_bitmap;
    st7305->base.invert_color = panel_st7305_invert_color;
    st7305->base.set_gap = panel_st7305_set_gap;
    st7305->base.mirror = panel_st7305_mirror;
    st7305->base.swap_xy = panel_st7305_swap_xy;
#if ESP_IDF_VERSION < ESP_IDF_VERSION_VAL(5, 0, 0)
    st7305->base.disp_off = panel_st7305_disp_on_off;
#else
    st7305->base.disp_on_off = panel_st7305_disp_on_off;
#endif
    *ret_panel = &(st7305->base);
    ESP_LOGD(TAG, "new st7305 panel @%p", st7305);

    // ESP_LOGI(TAG, "LCD panel create success, version: %d.%d.%d", ESP_LCD_ILI9341_VER_MAJOR, ESP_LCD_ILI9341_VER_MINOR,
    //          ESP_LCD_ILI9341_VER_PATCH);

    return ESP_OK;

err:
    if (st7305) {
        if (panel_dev_config->reset_gpio_num >= 0) {
            gpio_reset_pin(panel_dev_config->reset_gpio_num);
        }
        free(st7305);
    }
    return ret;
}
// Panel删除函数
static esp_err_t panel_st7305_del(esp_lcd_panel_t *panel)
{
    st7305_panel_t *st7305 = __containerof(panel, st7305_panel_t, base);
    
    if (st7305->reset_gpio_num >= 0) {
        gpio_reset_pin(st7305->reset_gpio_num);
    }
    free(st7305);
    return ESP_OK;
}

// Panel复位函数
static esp_err_t panel_st7305_reset(esp_lcd_panel_t *panel)
{
    st7305_panel_t *st7305 = __containerof(panel, st7305_panel_t, base);
    
    // 执行硬件复位
    if (st7305->reset_gpio_num >= 0) {
        gpio_set_level(st7305->reset_gpio_num, st7305->reset_level);
        vTaskDelay(pdMS_TO_TICKS(10));
        gpio_set_level(st7305->reset_gpio_num, !st7305->reset_level);
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    
    return ESP_OK;
}

// Panel初始化函数
static esp_err_t panel_st7305_init(esp_lcd_panel_t *panel)
{
    st7305_panel_t *st7305 = __containerof(panel, st7305_panel_t, base);
    esp_lcd_panel_io_handle_t io = st7305->io;

    // 发送初始化命令序列
    for (size_t i = 0; i < st7305_init_cmds_size; i++) {
        if (st7305_init_cmds[i].data_bytes > 0) {
            esp_lcd_panel_io_tx_param(io, st7305_init_cmds[i].cmd,
                                    st7305_init_cmds[i].data,
                                    st7305_init_cmds[i].data_bytes);
        } else {
            esp_lcd_panel_io_tx_param(io, st7305_init_cmds[i].cmd, NULL, 0);
        }
        if (st7305_init_cmds[i].delay_ms > 0) {
            vTaskDelay(pdMS_TO_TICKS(st7305_init_cmds[i].delay_ms));
        }
    }

    return ESP_OK;
}

static esp_err_t panel_st7305_draw_bitmap(esp_lcd_panel_t *panel, int x_start, int y_start, int x_end, int y_end, const void *color_data)
{
    st7305_panel_t *st7305 = __containerof(panel, st7305_panel_t, base);
    esp_lcd_panel_io_handle_t io = st7305->io;

    // 创建主缓冲区和临时缓冲区
    uint16_t pages = (st7305->height) / 8;
    size_t lcd_buf_size = (size_t)st7305->width * pages;

#ifdef CONFIG_ESP_LCD_ST7305_SCREEN_SIZE_1_54
    lcd_buf_size += 100;
    // 200/4=50≈51(ST7305芯片3byte写一次，向上取整得51) 一行共51个byte的数据，上下两行共用一行的数据，所以总行数需要除2
    // 200/2=100 所以共100行，一行51个byte数据，共100*51=5100byte
#endif
    uint8_t *lcd_buffer = heap_caps_malloc(lcd_buf_size, MALLOC_CAP_DMA);
    uint8_t *temp_buffer = heap_caps_malloc(lcd_buf_size, MALLOC_CAP_DMA);
    if (!lcd_buffer || !temp_buffer) {
        if (lcd_buffer) free(lcd_buffer);
        if (temp_buffer) free(temp_buffer);
        return ESP_ERR_NO_MEM;
    }

    memset(lcd_buffer, 0, lcd_buf_size);
    
    // 判断是否需要交换XY
    bool is_xy_swapped = (st7305->madctl_val & ST7305_MADCTL_MV);
    
    // 复制并转换数据到lcd_buffer
    const uint8_t *src = (const uint8_t *)color_data;
    if (!is_xy_swapped) {
        // 正常模式
        memcpy(lcd_buffer, src, lcd_buf_size);
    } else {
        // XY交换模式 - 需要重新排列数据
        for (int y = 0; y < st7305->height; y++) {
            for (int x = 0; x < st7305->width; x++) {
                // 计算源数据中的位置
                uint16_t src_byte_idx = (y >> 3) * st7305->width + x;
                uint8_t src_bit_pos = y & 0x07;
                bool pixel = src[src_byte_idx] & (1 << src_bit_pos);
                
                // 计算目标位置（交换x和y）
                uint16_t dst_byte_idx = (x >> 3) * st7305->height + y;
                uint8_t dst_bit_pos = x & 0x07;
                
                if (pixel) {
                    lcd_buffer[dst_byte_idx] |= (1 << dst_bit_pos);
                }
            }
        }
    }

#ifdef CONFIG_ESP_LCD_ST7305_SCREEN_SIZE_2_9
    // 数据格式转换
    uint16_t k = 0;
    uint16_t width = is_xy_swapped ? st7305->height : st7305->width;
    uint16_t stride = is_xy_swapped ? st7305->height : st7305->width;

    for (uint16_t i = 0; i < width; i += 2) {
        for (uint16_t j = 0; j < pages; j += 3) {
            for (uint8_t y = 0; y < 3; y++) {
                if ((j + y) < pages) {
                    uint8_t b1 = lcd_buffer[(j + y) * stride + i];
                    uint8_t b2 = lcd_buffer[(j + y) * stride + i + 1];
                    
                    // 第一组4位
                    uint8_t mix = ((b1 & 0x01) << 7) | ((b2 & 0x01) << 6) |
                                 ((b1 & 0x02) << 4) | ((b2 & 0x02) << 3) |
                                 ((b1 & 0x04) << 1) | ((b2 & 0x04)) |
                                 ((b1 & 0x08) >> 2) | ((b2 & 0x08) >> 3);
                    temp_buffer[k++] = mix;
                    
                    // 第二组4位
                    b1 >>= 4;
                    b2 >>= 4;
                    mix = ((b1 & 0x01) << 7) | ((b2 & 0x01) << 6) |
                          ((b1 & 0x02) << 4) | ((b2 & 0x02) << 3) |
                          ((b1 & 0x04) << 1) | ((b2 & 0x04)) |
                          ((b1 & 0x08) >> 2) | ((b2 & 0x08) >> 3);
                    temp_buffer[k++] = mix;
                }
            }
        }
    }
    // 设置显示范围和发送数据
    uint8_t caset[] = {0x17, 0x17 + 14 - 1};
    uint8_t raset[] = {0x00, 0x00 + 192 - 1};
    
    esp_lcd_panel_io_tx_param(io, ST7305_CMD_CASET, caset, sizeof(caset));
    esp_lcd_panel_io_tx_param(io, ST7305_CMD_RASET, raset, sizeof(raset));
    esp_lcd_panel_io_tx_color(io, ST7305_CMD_RAMWR, temp_buffer, lcd_buf_size);
#elif defined(CONFIG_ESP_LCD_ST7305_SCREEN_SIZE_1_54)
    // 设置显示范围和发送数据
    uint8_t caset[] = {0x16, 0x16 + 17 - 1}; // 列像素200 → 列字节数=200/4=50 → 50向上取整为51（3的倍数）→ 51/3=17次写入
    uint8_t raset[] = {0x00, 0x00 + 100 - 1};
    
    esp_lcd_panel_io_tx_param(io, ST7305_CMD_CASET, caset, sizeof(caset));
    esp_lcd_panel_io_tx_param(io, ST7305_CMD_RASET, raset, sizeof(raset));
    esp_lcd_panel_io_tx_color(io, ST7305_CMD_RAMWR, lcd_buffer, lcd_buf_size);
#endif
    free(lcd_buffer);
    free(temp_buffer);

    return ESP_OK;
}
// 颜色反转函数
static esp_err_t panel_st7305_invert_color(esp_lcd_panel_t *panel, bool invert_color_data)
{
    st7305_panel_t *st7305 = __containerof(panel, st7305_panel_t, base);
    esp_lcd_panel_io_tx_param(st7305->io, invert_color_data ? 0x21 : 0x20, NULL, 0);
    return ESP_OK;
}

// 镜像函数
// 旋转接口实现
static esp_err_t panel_st7305_mirror(esp_lcd_panel_t *panel, bool mirror_x, bool mirror_y)
{
    st7305_panel_t *st7305 = __containerof(panel, st7305_panel_t, base);
    esp_lcd_panel_io_handle_t io = st7305->io;
    if (mirror_x) {
        st7305->madctl_val |= ST7305_MADCTL_MY;
    } else {
        st7305->madctl_val &= ~ST7305_MADCTL_MY;
    }
    if (mirror_y) {
        st7305->madctl_val |= ST7305_MADCTL_MX;
    } else {
        st7305->madctl_val &= ~ST7305_MADCTL_MX;
    }
    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, LCD_CMD_MADCTL, (uint8_t[]) {
        st7305->madctl_val
    }, 1), TAG, "send command failed");
    return ESP_OK;
}

static esp_err_t panel_st7305_swap_xy(esp_lcd_panel_t *panel, bool swap_axes)
{
    st7305_panel_t *st7305 = __containerof(panel, st7305_panel_t, base);
    esp_lcd_panel_io_handle_t io = st7305->io;
    if (swap_axes) {
        st7305->madctl_val |= ST7305_MADCTL_MV;
    } else {
        st7305->madctl_val &= ~ST7305_MADCTL_MV;
    }
    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, LCD_CMD_MADCTL, (uint8_t[]) {
        st7305->madctl_val
    }, 1), TAG, "send command failed");
    return ESP_OK;
}

// 设置偏移函数
static esp_err_t panel_st7305_set_gap(esp_lcd_panel_t *panel, int x_gap, int y_gap)
{
    st7305_panel_t *st7305 = __containerof(panel, st7305_panel_t, base);
    st7305->x_gap = x_gap;
    st7305->y_gap = y_gap;
    return ESP_OK;
}
// 添加显示开关函数
static esp_err_t panel_st7305_disp_on_off(esp_lcd_panel_t *panel, bool on)
{
    st7305_panel_t *st7305 = __containerof(panel, st7305_panel_t, base);
    esp_lcd_panel_io_tx_param(st7305->io, on ? ST7305_CMD_DISPON : ST7305_CMD_DISPOFF, NULL, 0);
    return ESP_OK;
}
