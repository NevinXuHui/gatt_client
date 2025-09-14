#include "gpio_button.h"
#include "app_config.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include <string.h>

static const char* TAG = "GPIO_BTN";

// GPIO按键模块状态
static struct {
    gpio_button_config_t config;
    QueueHandle_t event_queue;
    TaskHandle_t task_handle;
    bool initialized;
    bool started;
    uint32_t total_presses;
    uint32_t last_press_time;
    uint32_t press_start_time;
} gpio_btn = {0};

// 前向声明
static void gpio_button_isr_handler(void* arg);
static void gpio_button_task(void* arg);

esp_err_t gpio_button_init(const gpio_button_config_t* config)
{
    if (gpio_btn.initialized) {
        ESP_LOGW(TAG, "GPIO button already initialized");
        return ESP_OK;
    }

    if (!config) {
        ESP_LOGE(TAG, "GPIO button config is NULL");
        return ESP_ERR_INVALID_ARG;
    }

    // 复制配置
    memcpy(&gpio_btn.config, config, sizeof(gpio_button_config_t));
    gpio_btn.total_presses = 0;
    gpio_btn.last_press_time = 0;
    gpio_btn.press_start_time = 0;

    // 创建事件队列
    gpio_btn.event_queue = xQueueCreate(GPIO_EVENT_QUEUE_SIZE, sizeof(uint32_t));
    if (!gpio_btn.event_queue) {
        ESP_LOGE(TAG, "Failed to create GPIO event queue");
        return ESP_ERR_NO_MEM;
    }

    // 重置GPIO引脚
    gpio_reset_pin(gpio_btn.config.gpio_num);

    // 配置GPIO
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_ANYEDGE,  // 双边沿触发
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = (1ULL << gpio_btn.config.gpio_num),
        .pull_down_en = gpio_btn.config.pull_down_enable ? GPIO_PULLDOWN_ENABLE : GPIO_PULLDOWN_DISABLE,
        .pull_up_en = gpio_btn.config.pull_up_enable ? GPIO_PULLUP_ENABLE : GPIO_PULLUP_DISABLE,
    };

    esp_err_t ret = gpio_config(&io_conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure GPIO: %s", esp_err_to_name(ret));
        vQueueDelete(gpio_btn.event_queue);
        return ret;
    }

    // 安装GPIO中断服务
    ret = gpio_install_isr_service(ESP_INTR_FLAG_LEVEL1);
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "Failed to install GPIO ISR service: %s", esp_err_to_name(ret));
        vQueueDelete(gpio_btn.event_queue);
        return ret;
    }

    // 添加中断处理函数
    ret = gpio_isr_handler_add(gpio_btn.config.gpio_num, gpio_button_isr_handler, 
                              (void*) gpio_btn.config.gpio_num);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add GPIO ISR handler: %s", esp_err_to_name(ret));
        vQueueDelete(gpio_btn.event_queue);
        return ret;
    }

    gpio_btn.initialized = true;
    ESP_LOGI(TAG, "GPIO button initialized on pin %d", gpio_btn.config.gpio_num);

    return ESP_OK;
}

esp_err_t gpio_button_start(void)
{
    if (!gpio_btn.initialized) {
        ESP_LOGE(TAG, "GPIO button not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (gpio_btn.started) {
        ESP_LOGW(TAG, "GPIO button already started");
        return ESP_OK;
    }

    // 创建按键处理任务
    BaseType_t ret = xTaskCreate(gpio_button_task, "gpio_button", 
                                GPIO_TASK_STACK_SIZE, NULL, 
                                GPIO_TASK_PRIORITY, &gpio_btn.task_handle);
    if (ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create GPIO button task");
        return ESP_ERR_NO_MEM;
    }

    gpio_btn.started = true;
    ESP_LOGI(TAG, "GPIO button started");

    return ESP_OK;
}

esp_err_t gpio_button_stop(void)
{
    if (!gpio_btn.initialized || !gpio_btn.started) {
        return ESP_ERR_INVALID_STATE;
    }

    // 删除任务
    if (gpio_btn.task_handle) {
        vTaskDelete(gpio_btn.task_handle);
        gpio_btn.task_handle = NULL;
    }

    gpio_btn.started = false;
    ESP_LOGI(TAG, "GPIO button stopped");

    return ESP_OK;
}

bool gpio_button_is_initialized(void)
{
    return gpio_btn.initialized;
}

int gpio_button_get_level(void)
{
    if (!gpio_btn.initialized) {
        return -1;
    }

    return gpio_get_level(gpio_btn.config.gpio_num);
}

esp_err_t gpio_button_simulate_press(void)
{
    if (!gpio_btn.initialized || !gpio_btn.started) {
        return ESP_ERR_INVALID_STATE;
    }

    uint32_t gpio_num = gpio_btn.config.gpio_num;
    BaseType_t ret = xQueueSend(gpio_btn.event_queue, &gpio_num, pdMS_TO_TICKS(100));
    
    if (ret != pdTRUE) {
        ESP_LOGW(TAG, "Failed to send simulated button event");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Simulated button press");
    return ESP_OK;
}

esp_err_t gpio_button_set_debounce_time(uint32_t debounce_time_ms)
{
    gpio_btn.config.debounce_time_ms = debounce_time_ms;
    ESP_LOGI(TAG, "Debounce time set to %d ms", debounce_time_ms);
    return ESP_OK;
}

esp_err_t gpio_button_set_long_press_time(uint32_t long_press_time_ms)
{
    gpio_btn.config.long_press_time_ms = long_press_time_ms;
    ESP_LOGI(TAG, "Long press time set to %d ms", long_press_time_ms);
    return ESP_OK;
}

esp_err_t gpio_button_get_stats(uint32_t* total_presses, uint32_t* last_press_time)
{
    if (total_presses) {
        *total_presses = gpio_btn.total_presses;
    }
    if (last_press_time) {
        *last_press_time = gpio_btn.last_press_time;
    }
    return ESP_OK;
}

esp_err_t gpio_button_deinit(void)
{
    if (!gpio_btn.initialized) {
        return ESP_OK;
    }

    // 停止按键检测
    gpio_button_stop();

    // 移除中断处理函数
    gpio_isr_handler_remove(gpio_btn.config.gpio_num);

    // 清理资源
    if (gpio_btn.event_queue) {
        vQueueDelete(gpio_btn.event_queue);
        gpio_btn.event_queue = NULL;
    }

    gpio_btn.initialized = false;
    ESP_LOGI(TAG, "GPIO button deinitialized");

    return ESP_OK;
}

// 中断处理函数
static void IRAM_ATTR gpio_button_isr_handler(void* arg)
{
    if (gpio_btn.event_queue == NULL) {
        return;
    }

    uint32_t gpio_num = (uint32_t) arg;
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    xQueueSendFromISR(gpio_btn.event_queue, &gpio_num, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

// 按键处理任务
static void gpio_button_task(void* arg)
{
    uint32_t io_num;
    TickType_t last_event_time = 0;
    bool button_pressed = false;
    int last_gpio_level = 1; // 假设初始状态为高电平（未按下）

    ESP_LOGI(TAG, "GPIO button task started");

    // 读取初始GPIO状态
    last_gpio_level = gpio_get_level(gpio_btn.config.gpio_num);
    ESP_LOGI(TAG, "Initial GPIO level: %d", last_gpio_level);

    while (1) {
        bool event_received = false;
        TickType_t current_time = xTaskGetTickCount();

        // 检查是否有中断事件
        if (xQueueReceive(gpio_btn.event_queue, &io_num, pdMS_TO_TICKS(50))) {
            event_received = true;

            // 验证GPIO编号
            if (io_num != gpio_btn.config.gpio_num) {
                continue;
            }
        }

        // 读取当前GPIO状态
        int current_gpio_level = gpio_get_level(gpio_btn.config.gpio_num);

        // 检测状态变化（无论是否有中断事件）
        if (current_gpio_level != last_gpio_level || event_received) {

            // 防抖动检查
            if ((current_time - last_event_time) < pdMS_TO_TICKS(gpio_btn.config.debounce_time_ms)) {
                continue;
            }

            // 确认状态确实发生了变化
            if (current_gpio_level != last_gpio_level) {
                last_event_time = current_time;
                last_gpio_level = current_gpio_level;

                gpio_button_event_data_t event_data = {
                    .gpio_num = gpio_btn.config.gpio_num,
                    .timestamp = current_time * portTICK_PERIOD_MS
                };

                if (current_gpio_level == 0 && !button_pressed) {
                    // 按键按下（下降沿）
                    button_pressed = true;
                    gpio_btn.press_start_time = current_time;
                    event_data.event = GPIO_BUTTON_EVENT_PRESSED;
                    event_data.press_duration_ms = 0;

                    gpio_btn.total_presses++;
                    gpio_btn.last_press_time = event_data.timestamp;

                    ESP_LOGI(TAG, "Button pressed (level: %d)", current_gpio_level);

                    // 调用回调函数
                    if (gpio_btn.config.event_callback) {
                        gpio_btn.config.event_callback(&event_data, gpio_btn.config.user_data);
                    }

                } else if (current_gpio_level == 1 && button_pressed) {
                    // 按键释放（上升沿）
                    button_pressed = false;
                    uint32_t press_duration = (current_time - gpio_btn.press_start_time) * portTICK_PERIOD_MS;

                    if (press_duration >= gpio_btn.config.long_press_time_ms) {
                        event_data.event = GPIO_BUTTON_EVENT_LONG_PRESSED;
                        ESP_LOGI(TAG, "Button long pressed (%d ms, level: %d)", press_duration, current_gpio_level);
                    } else {
                        event_data.event = GPIO_BUTTON_EVENT_RELEASED;
                        ESP_LOGI(TAG, "Button released (%d ms, level: %d)", press_duration, current_gpio_level);
                    }

                    event_data.press_duration_ms = press_duration;

                    // 调用回调函数
                    if (gpio_btn.config.event_callback) {
                        gpio_btn.config.event_callback(&event_data, gpio_btn.config.user_data);
                    }
                }
            }
        }

        // 检查长按状态（即使没有释放）
        if (button_pressed) {
            uint32_t current_press_duration = (current_time - gpio_btn.press_start_time) * portTICK_PERIOD_MS;

            // 如果按键持续时间超过长按阈值，且还没有触发长按事件
            if (current_press_duration >= gpio_btn.config.long_press_time_ms &&
                current_gpio_level == 0) {

                // 检查是否已经触发过长按事件（避免重复触发）
                static TickType_t last_long_press_time = 0;
                if ((current_time - last_long_press_time) > pdMS_TO_TICKS(gpio_btn.config.long_press_time_ms)) {
                    last_long_press_time = current_time;

                    gpio_button_event_data_t event_data = {
                        .gpio_num = gpio_btn.config.gpio_num,
                        .timestamp = current_time * portTICK_PERIOD_MS,
                        .event = GPIO_BUTTON_EVENT_LONG_PRESSED,
                        .press_duration_ms = current_press_duration
                    };

                    ESP_LOGI(TAG, "Button long pressed (ongoing: %d ms)", current_press_duration);

                    // 调用回调函数
                    if (gpio_btn.config.event_callback) {
                        gpio_btn.config.event_callback(&event_data, gpio_btn.config.user_data);
                    }
                }
            }
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
