#ifndef LED_INDICATOR_H
#define LED_INDICATOR_H

#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

// LED状态枚举
typedef enum {
    LED_STATE_OFF = 0,      // LED关闭
    LED_STATE_ON,           // LED常亮
    LED_STATE_BLINK_SLOW,   // LED慢闪（1Hz）
    LED_STATE_BLINK_FAST,   // LED快闪（2Hz）
    LED_STATE_BLINK_CUSTOM  // LED自定义闪烁
} led_state_t;

// LED指示器配置
typedef struct {
    int gpio_num;           // GPIO引脚号
    int on_level;           // 点亮电平
    int off_level;          // 熄灭电平
    int blink_period_ms;    // 默认闪烁周期
} led_config_t;

/**
 * @brief 初始化LED指示器
 * @param config LED配置
 * @return ESP_OK 成功，其他值失败
 */
esp_err_t led_indicator_init(const led_config_t* config);

/**
 * @brief 设置LED状态
 * @param state LED状态
 * @return ESP_OK 成功，其他值失败
 */
esp_err_t led_indicator_set_state(led_state_t state);

/**
 * @brief 设置自定义闪烁周期
 * @param period_ms 闪烁周期（毫秒）
 * @return ESP_OK 成功，其他值失败
 */
esp_err_t led_indicator_set_blink_period(int period_ms);

/**
 * @brief 获取当前LED状态
 * @return 当前LED状态
 */
led_state_t led_indicator_get_state(void);

/**
 * @brief 反初始化LED指示器
 */
void led_indicator_deinit(void);

// 便捷函数
#define led_indicator_off()         led_indicator_set_state(LED_STATE_OFF)
#define led_indicator_on()          led_indicator_set_state(LED_STATE_ON)
#define led_indicator_blink_slow()  led_indicator_set_state(LED_STATE_BLINK_SLOW)
#define led_indicator_blink_fast()  led_indicator_set_state(LED_STATE_BLINK_FAST)

#ifdef __cplusplus
}
#endif

#endif // LED_INDICATOR_H
