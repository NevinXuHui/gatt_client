#ifndef GPIO_BUTTON_H
#define GPIO_BUTTON_H

#include <stdbool.h>
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

// GPIO按键事件类型
typedef enum {
    GPIO_BUTTON_EVENT_PRESSED = 0,
    GPIO_BUTTON_EVENT_RELEASED,
    GPIO_BUTTON_EVENT_LONG_PRESSED
} gpio_button_event_t;

// GPIO按键事件数据
typedef struct {
    uint32_t gpio_num;
    gpio_button_event_t event;
    uint32_t press_duration_ms;
    uint32_t timestamp;
} gpio_button_event_data_t;

// GPIO按键事件回调函数类型
typedef void (*gpio_button_callback_t)(const gpio_button_event_data_t* event_data, void* user_data);

// GPIO按键配置结构体
typedef struct {
    uint32_t gpio_num;
    uint32_t debounce_time_ms;
    uint32_t long_press_time_ms;
    bool pull_up_enable;
    bool pull_down_enable;
    gpio_button_callback_t event_callback;
    void* user_data;
} gpio_button_config_t;

// ==================== 公共接口 ====================

/**
 * @brief 初始化GPIO按键模块
 * 
 * @param config GPIO按键配置
 * @return esp_err_t 
 */
esp_err_t gpio_button_init(const gpio_button_config_t* config);

/**
 * @brief 启动GPIO按键检测
 * 
 * @return esp_err_t 
 */
esp_err_t gpio_button_start(void);

/**
 * @brief 停止GPIO按键检测
 * 
 * @return esp_err_t 
 */
esp_err_t gpio_button_stop(void);

/**
 * @brief 检查GPIO按键是否已初始化
 * 
 * @return true 已初始化
 * @return false 未初始化
 */
bool gpio_button_is_initialized(void);

/**
 * @brief 获取按键当前状态
 * 
 * @return int 按键电平 (0或1)
 */
int gpio_button_get_level(void);

/**
 * @brief 模拟按键按下事件（用于测试）
 * 
 * @return esp_err_t 
 */
esp_err_t gpio_button_simulate_press(void);

/**
 * @brief 反初始化GPIO按键模块
 * 
 * @return esp_err_t 
 */
esp_err_t gpio_button_deinit(void);

/**
 * @brief 设置防抖动时间
 * 
 * @param debounce_time_ms 防抖动时间（毫秒）
 * @return esp_err_t 
 */
esp_err_t gpio_button_set_debounce_time(uint32_t debounce_time_ms);

/**
 * @brief 设置长按时间阈值
 * 
 * @param long_press_time_ms 长按时间阈值（毫秒）
 * @return esp_err_t 
 */
esp_err_t gpio_button_set_long_press_time(uint32_t long_press_time_ms);

/**
 * @brief 获取按键统计信息
 * 
 * @param total_presses 总按键次数
 * @param last_press_time 最后一次按键时间
 * @return esp_err_t 
 */
esp_err_t gpio_button_get_stats(uint32_t* total_presses, uint32_t* last_press_time);

#endif // GPIO_BUTTON_H
