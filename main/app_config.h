#ifndef APP_CONFIG_H
#define APP_CONFIG_H

#include <stdint.h>

// ==================== 应用程序配置 ====================

// 应用程序标签
#define APP_TAG "GATT_CLIENT"

// ==================== BLE GATT 配置 ====================

// GATT客户端配置
#define REMOTE_SERVICE_UUID        0x00FF
#define REMOTE_NOTIFY_CHAR_UUID    0xFF01
#define CUSTOM_CHAR_UUID_0013      0x0013
#define PROFILE_NUM                1
#define PROFILE_A_APP_ID           0
#define INVALID_HANDLE             0

// ==================== GPIO 配置 ====================

// GPIO按键配置
#define GPIO_BUTTON_PIN            18
#define GPIO_BUTTON_LEVEL          0
#define GPIO_DEBOUNCE_TIME_MS      200

// ==================== WiFi 配置 ====================

// WiFi连接配置
#define WIFI_SSID                  "K3_2.4g"
#define WIFI_PASS                  "xuhui1987"
#define WIFI_MAXIMUM_RETRY         5
#define WIFI_CONNECTED_BIT         BIT0
#define WIFI_FAIL_BIT              BIT1

// ==================== NTP 配置 ====================

// NTP时间同步配置
#define NTP_SERVER                 "pool.ntp.org"
#define NTP_SERVER_BACKUP          "time.nist.gov"
#define TIME_ZONE                  "CST-8"  // 中国标准时间 UTC+8
#define NTP_SYNC_TIMEOUT_MS        10000    // 10秒超时

// ==================== 任务配置 ====================

// 任务栈大小配置
#define GPIO_TASK_STACK_SIZE       3072
#define WIFI_TASK_STACK_SIZE       4096
#define NTP_TASK_STACK_SIZE        2048

// 任务优先级配置
#define GPIO_TASK_PRIORITY         5
#define WIFI_TASK_PRIORITY         6
#define NTP_TASK_PRIORITY          4

// ==================== 队列配置 ====================

// 队列大小配置
#define GPIO_EVENT_QUEUE_SIZE      10
#define WIFI_EVENT_QUEUE_SIZE      5

// ==================== 调试配置 ====================

// 日志级别配置
#define LOG_LEVEL_GPIO             ESP_LOG_INFO
#define LOG_LEVEL_WIFI             ESP_LOG_INFO
#define LOG_LEVEL_NTP              ESP_LOG_INFO
#define LOG_LEVEL_BLE              ESP_LOG_INFO

// 调试开关
#define ENABLE_GPIO_DEBUG          1
#define ENABLE_WIFI_DEBUG          1
#define ENABLE_NTP_DEBUG           1
#define ENABLE_BLE_DEBUG           1

// ==================== 数据格式配置 ====================

// 发送数据配置
#define DATA_PACKET_SIZE           20
#define DATA_HEADER_SIZE           4

// 时间格式配置
#define TIME_STRING_BUFFER_SIZE    64

#endif // APP_CONFIG_H
