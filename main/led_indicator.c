#include "led_indicator.h"
#include "app_config.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "esp_log.h"
#include <string.h>

static const char* TAG = "LED_IND";

// LED管理器结构
typedef struct {
    bool initialized;
    led_config_t config;
    led_state_t current_state;
    TimerHandle_t blink_timer;
    bool led_on;
    int custom_period_ms;
} led_manager_t;

static led_manager_t led_mgr = {0};

// LED闪烁定时器回调
static void led_blink_timer_callback(TimerHandle_t xTimer)
{
    if (!led_mgr.initialized) {
        return;
    }

    // 切换LED状态
    led_mgr.led_on = !led_mgr.led_on;
    int level = led_mgr.led_on ? led_mgr.config.on_level : led_mgr.config.off_level;

    // 使用ESP_ERROR_CHECK确保GPIO操作成功（参考blink示例）
    ESP_ERROR_CHECK(gpio_set_level(led_mgr.config.gpio_num, level));
}

esp_err_t led_indicator_init(const led_config_t* config)
{
    if (!config) {
        ESP_LOGE(TAG, "Invalid config parameter");
        return ESP_ERR_INVALID_ARG;
    }

    if (led_mgr.initialized) {
        ESP_LOGW(TAG, "LED indicator already initialized");
        return ESP_OK;
    }

    // 保存配置
    led_mgr.config = *config;
    led_mgr.current_state = LED_STATE_OFF;
    led_mgr.led_on = false;
    led_mgr.custom_period_ms = config->blink_period_ms;

    // 配置GPIO（参考ESP-IDF blink示例的标准做法）
    ESP_LOGI(TAG, "Configuring GPIO %d as output", config->gpio_num);

    // 重置GPIO引脚
    gpio_reset_pin(config->gpio_num);

    // 设置GPIO方向为输出
    esp_err_t ret = gpio_set_direction(config->gpio_num, GPIO_MODE_OUTPUT);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set GPIO %d direction: %s", config->gpio_num, esp_err_to_name(ret));
        return ret;
    }

    // 初始状态设置为关闭
    ret = gpio_set_level(config->gpio_num, config->off_level);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set GPIO %d level: %s", config->gpio_num, esp_err_to_name(ret));
        return ret;
    }

    // 创建闪烁定时器
    led_mgr.blink_timer = xTimerCreate("led_blink", 
                                       pdMS_TO_TICKS(config->blink_period_ms),
                                       pdTRUE,  // 自动重载
                                       NULL,
                                       led_blink_timer_callback);
    
    if (!led_mgr.blink_timer) {
        ESP_LOGE(TAG, "Failed to create blink timer");
        return ESP_ERR_NO_MEM;
    }

    led_mgr.initialized = true;
    ESP_LOGI(TAG, "LED indicator initialized on GPIO %d", config->gpio_num);
    
    return ESP_OK;
}

esp_err_t led_indicator_set_state(led_state_t state)
{
    if (!led_mgr.initialized) {
        ESP_LOGE(TAG, "LED indicator not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (led_mgr.current_state == state) {
        return ESP_OK;  // 状态相同，无需改变
    }

    // 停止当前的闪烁定时器
    if (xTimerIsTimerActive(led_mgr.blink_timer)) {
        xTimerStop(led_mgr.blink_timer, 0);
    }

    led_mgr.current_state = state;

    switch (state) {
        case LED_STATE_OFF:
            ESP_ERROR_CHECK(gpio_set_level(led_mgr.config.gpio_num, led_mgr.config.off_level));
            led_mgr.led_on = false;
            ESP_LOGD(TAG, "LED set to OFF");
            break;

        case LED_STATE_ON:
            ESP_ERROR_CHECK(gpio_set_level(led_mgr.config.gpio_num, led_mgr.config.on_level));
            led_mgr.led_on = true;
            ESP_LOGD(TAG, "LED set to ON");
            break;

        case LED_STATE_BLINK_SLOW:
            // 慢闪：1Hz (1000ms周期)
            xTimerChangePeriod(led_mgr.blink_timer, pdMS_TO_TICKS(1000), 0);
            xTimerStart(led_mgr.blink_timer, 0);
            ESP_LOGD(TAG, "LED set to BLINK_SLOW");
            break;

        case LED_STATE_BLINK_FAST:
            // 快闪：2Hz (500ms周期)
            xTimerChangePeriod(led_mgr.blink_timer, pdMS_TO_TICKS(500), 0);
            xTimerStart(led_mgr.blink_timer, 0);
            ESP_LOGD(TAG, "LED set to BLINK_FAST");
            break;

        case LED_STATE_BLINK_CUSTOM:
            // 自定义闪烁周期
            xTimerChangePeriod(led_mgr.blink_timer, pdMS_TO_TICKS(led_mgr.custom_period_ms), 0);
            xTimerStart(led_mgr.blink_timer, 0);
            ESP_LOGD(TAG, "LED set to BLINK_CUSTOM (%d ms)", led_mgr.custom_period_ms);
            break;

        default:
            ESP_LOGE(TAG, "Invalid LED state: %d", state);
            return ESP_ERR_INVALID_ARG;
    }

    return ESP_OK;
}

esp_err_t led_indicator_set_blink_period(int period_ms)
{
    if (!led_mgr.initialized) {
        ESP_LOGE(TAG, "LED indicator not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (period_ms <= 0) {
        ESP_LOGE(TAG, "Invalid blink period: %d", period_ms);
        return ESP_ERR_INVALID_ARG;
    }

    led_mgr.custom_period_ms = period_ms;

    // 如果当前是自定义闪烁状态，更新定时器周期
    if (led_mgr.current_state == LED_STATE_BLINK_CUSTOM && 
        xTimerIsTimerActive(led_mgr.blink_timer)) {
        xTimerChangePeriod(led_mgr.blink_timer, pdMS_TO_TICKS(period_ms), 0);
    }

    return ESP_OK;
}

led_state_t led_indicator_get_state(void)
{
    return led_mgr.current_state;
}

void led_indicator_deinit(void)
{
    if (!led_mgr.initialized) {
        return;
    }

    // 停止并删除定时器
    if (led_mgr.blink_timer) {
        xTimerStop(led_mgr.blink_timer, 0);
        xTimerDelete(led_mgr.blink_timer, 0);
        led_mgr.blink_timer = NULL;
    }

    // 关闭LED
    gpio_set_level(led_mgr.config.gpio_num, led_mgr.config.off_level);

    // 重置状态
    memset(&led_mgr, 0, sizeof(led_mgr));

    ESP_LOGI(TAG, "LED indicator deinitialized");
}
