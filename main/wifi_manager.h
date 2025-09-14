#ifndef WIFI_MANAGER_H
#define WIFI_MANAGER_H

#include <stdbool.h>
#include "esp_err.h"
#include "esp_event.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"

// WiFi状态枚举
typedef enum {
    WIFI_STATE_DISCONNECTED = 0,
    WIFI_STATE_CONNECTING,
    WIFI_STATE_CONNECTED,
    WIFI_STATE_FAILED
} wifi_state_t;

// WiFi事件回调函数类型
typedef void (*wifi_event_callback_t)(wifi_state_t state, void* user_data);

// WiFi配置结构体
typedef struct {
    char ssid[32];
    char password[64];
    int max_retry;
    wifi_event_callback_t event_callback;
    void* user_data;
} wifi_manager_config_t;

// ==================== 公共接口 ====================

/**
 * @brief 初始化WiFi管理器
 * 
 * @param config WiFi配置
 * @return esp_err_t 
 */
esp_err_t wifi_manager_init(const wifi_manager_config_t* config);

/**
 * @brief 启动WiFi连接
 * 
 * @return esp_err_t 
 */
esp_err_t wifi_manager_start(void);

/**
 * @brief 停止WiFi连接
 * 
 * @return esp_err_t 
 */
esp_err_t wifi_manager_stop(void);

/**
 * @brief 获取WiFi连接状态
 * 
 * @return wifi_state_t 
 */
wifi_state_t wifi_manager_get_state(void);

/**
 * @brief 检查WiFi是否已连接
 * 
 * @return true 已连接
 * @return false 未连接
 */
bool wifi_manager_is_connected(void);

/**
 * @brief 获取IP地址字符串
 * 
 * @param ip_str 输出缓冲区
 * @param max_len 缓冲区最大长度
 * @return esp_err_t 
 */
esp_err_t wifi_manager_get_ip_string(char* ip_str, size_t max_len);

/**
 * @brief 重新连接WiFi
 * 
 * @return esp_err_t 
 */
esp_err_t wifi_manager_reconnect(void);

/**
 * @brief 获取WiFi连接信息
 *
 * @param info_str 输出缓冲区
 * @param max_len 缓冲区最大长度
 * @return esp_err_t
 */
esp_err_t wifi_manager_get_connection_info(char* info_str, size_t max_len);

/**
 * @brief 反初始化WiFi管理器
 *
 * @return esp_err_t
 */
esp_err_t wifi_manager_deinit(void);

#endif // WIFI_MANAGER_H
