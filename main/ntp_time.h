#ifndef NTP_TIME_H
#define NTP_TIME_H

#include <stdbool.h>
#include <time.h>
#include "esp_err.h"

// NTP同步状态枚举
typedef enum {
    NTP_STATE_NOT_SYNCED = 0,
    NTP_STATE_SYNCING,
    NTP_STATE_SYNCED,
    NTP_STATE_FAILED
} ntp_sync_state_t;

// NTP事件回调函数类型
typedef void (*ntp_sync_callback_t)(ntp_sync_state_t state, void* user_data);

// NTP配置结构体
typedef struct {
    char primary_server[64];
    char backup_server[64];
    char timezone[32];
    int sync_timeout_ms;
    ntp_sync_callback_t sync_callback;
    void* user_data;
} ntp_config_t;

// 时间信息结构体
typedef struct {
    int year;
    int month;
    int day;
    int hour;
    int minute;
    int second;
    int weekday;
    char formatted_time[64];
    char formatted_date[32];
    time_t timestamp;
} time_info_t;

// ==================== 公共接口 ====================

/**
 * @brief 初始化NTP时间模块
 * 
 * @param config NTP配置
 * @return esp_err_t 
 */
esp_err_t ntp_time_init(const ntp_config_t* config);

/**
 * @brief 启动NTP时间同步
 * 
 * @return esp_err_t 
 */
esp_err_t ntp_time_start_sync(void);

/**
 * @brief 停止NTP时间同步
 * 
 * @return esp_err_t 
 */
esp_err_t ntp_time_stop_sync(void);

/**
 * @brief 获取NTP同步状态
 * 
 * @return ntp_sync_state_t 
 */
ntp_sync_state_t ntp_time_get_sync_state(void);

/**
 * @brief 检查时间是否已同步
 * 
 * @return true 已同步
 * @return false 未同步
 */
bool ntp_time_is_synced(void);

/**
 * @brief 获取当前时间信息
 * 
 * @param time_info 输出时间信息
 * @return esp_err_t 
 */
esp_err_t ntp_time_get_current_time(time_info_t* time_info);

/**
 * @brief 获取格式化的时间字符串
 * 
 * @param time_str 输出缓冲区
 * @param max_len 缓冲区最大长度
 * @param format 时间格式 (NULL使用默认格式)
 * @return esp_err_t 
 */
esp_err_t ntp_time_get_formatted_time(char* time_str, size_t max_len, const char* format);

/**
 * @brief 获取时间戳
 * 
 * @return time_t 当前时间戳，失败返回0
 */
time_t ntp_time_get_timestamp(void);

/**
 * @brief 强制重新同步时间
 * 
 * @return esp_err_t 
 */
esp_err_t ntp_time_force_sync(void);

/**
 * @brief 反初始化NTP时间模块
 * 
 * @return esp_err_t 
 */
esp_err_t ntp_time_deinit(void);

/**
 * @brief 设置时区
 * 
 * @param timezone 时区字符串，如 "CST-8"
 * @return esp_err_t 
 */
esp_err_t ntp_time_set_timezone(const char* timezone);

/**
 * @brief 获取上次同步时间
 * 
 * @return time_t 上次同步的时间戳
 */
time_t ntp_time_get_last_sync_time(void);

#endif // NTP_TIME_H
