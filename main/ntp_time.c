#include "ntp_time.h"
#include "app_config.h"
#include "esp_sntp.h"
#include "esp_log.h"
#include <string.h>
#include <sys/time.h>

static const char* TAG = "NTP_TIME";

// NTP时间模块状态
static struct {
    ntp_config_t config;
    ntp_sync_state_t state;
    bool initialized;
    time_t last_sync_time;
} ntp_mgr = {0};

// 前向声明
static void sntp_sync_time_cb(struct timeval *tv);

esp_err_t ntp_time_init(const ntp_config_t* config)
{
    if (ntp_mgr.initialized) {
        ESP_LOGW(TAG, "NTP time module already initialized");
        return ESP_OK;
    }

    if (!config) {
        ESP_LOGE(TAG, "NTP config is NULL");
        return ESP_ERR_INVALID_ARG;
    }

    // 复制配置
    memcpy(&ntp_mgr.config, config, sizeof(ntp_config_t));
    ntp_mgr.state = NTP_STATE_NOT_SYNCED;
    ntp_mgr.last_sync_time = 0;

    // 设置时区
    setenv("TZ", ntp_mgr.config.timezone, 1);
    tzset();

    ntp_mgr.initialized = true;
    ESP_LOGI(TAG, "NTP time module initialized with timezone: %s", ntp_mgr.config.timezone);

    return ESP_OK;
}

esp_err_t ntp_time_start_sync(void)
{
    if (!ntp_mgr.initialized) {
        ESP_LOGE(TAG, "NTP time module not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    // 配置SNTP
    esp_sntp_setoperatingmode(SNTP_OPMODE_POLL);
    esp_sntp_setservername(0, ntp_mgr.config.primary_server);
    
    if (strlen(ntp_mgr.config.backup_server) > 0) {
        esp_sntp_setservername(1, ntp_mgr.config.backup_server);
    }

    // 设置同步回调
    sntp_set_time_sync_notification_cb(sntp_sync_time_cb);

    // 启动SNTP
    esp_sntp_init();

    ntp_mgr.state = NTP_STATE_SYNCING;
    ESP_LOGI(TAG, "NTP sync started with server: %s", ntp_mgr.config.primary_server);

    return ESP_OK;
}

esp_err_t ntp_time_stop_sync(void)
{
    if (!ntp_mgr.initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    esp_sntp_stop();
    ntp_mgr.state = NTP_STATE_NOT_SYNCED;
    ESP_LOGI(TAG, "NTP sync stopped");

    return ESP_OK;
}

ntp_sync_state_t ntp_time_get_sync_state(void)
{
    return ntp_mgr.state;
}

bool ntp_time_is_synced(void)
{
    return ntp_mgr.state == NTP_STATE_SYNCED;
}

esp_err_t ntp_time_get_current_time(time_info_t* time_info)
{
    if (!time_info) {
        return ESP_ERR_INVALID_ARG;
    }

    if (!ntp_time_is_synced()) {
        ESP_LOGW(TAG, "Time not synced yet");
        return ESP_ERR_INVALID_STATE;
    }

    time_t now;
    struct tm timeinfo;

    time(&now);
    localtime_r(&now, &timeinfo);

    // 填充时间信息
    time_info->year = timeinfo.tm_year + 1900;
    time_info->month = timeinfo.tm_mon + 1;
    time_info->day = timeinfo.tm_mday;
    time_info->hour = timeinfo.tm_hour;
    time_info->minute = timeinfo.tm_min;
    time_info->second = timeinfo.tm_sec;
    time_info->weekday = timeinfo.tm_wday;
    time_info->timestamp = now;

    // 格式化时间字符串
    strftime(time_info->formatted_time, sizeof(time_info->formatted_time), 
             "%Y-%m-%d %H:%M:%S", &timeinfo);
    strftime(time_info->formatted_date, sizeof(time_info->formatted_date), 
             "%Y-%m-%d", &timeinfo);

    return ESP_OK;
}

esp_err_t ntp_time_get_formatted_time(char* time_str, size_t max_len, const char* format)
{
    if (!time_str || max_len == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    if (!ntp_time_is_synced()) {
        strncpy(time_str, "Time not synced", max_len - 1);
        time_str[max_len - 1] = '\0';
        return ESP_ERR_INVALID_STATE;
    }

    time_t now;
    struct tm timeinfo;

    time(&now);
    localtime_r(&now, &timeinfo);

    const char* fmt = format ? format : "%Y-%m-%d %H:%M:%S";
    strftime(time_str, max_len, fmt, &timeinfo);

    return ESP_OK;
}

time_t ntp_time_get_timestamp(void)
{
    if (!ntp_time_is_synced()) {
        return 0;
    }

    time_t now;
    time(&now);
    return now;
}

esp_err_t ntp_time_force_sync(void)
{
    if (!ntp_mgr.initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    // 重启SNTP同步
    esp_sntp_stop();
    vTaskDelay(pdMS_TO_TICKS(100));
    
    ntp_mgr.state = NTP_STATE_SYNCING;
    esp_sntp_init();

    ESP_LOGI(TAG, "Force NTP sync started");
    return ESP_OK;
}

esp_err_t ntp_time_set_timezone(const char* timezone)
{
    if (!timezone) {
        return ESP_ERR_INVALID_ARG;
    }

    strncpy(ntp_mgr.config.timezone, timezone, sizeof(ntp_mgr.config.timezone) - 1);
    ntp_mgr.config.timezone[sizeof(ntp_mgr.config.timezone) - 1] = '\0';

    setenv("TZ", timezone, 1);
    tzset();

    ESP_LOGI(TAG, "Timezone set to: %s", timezone);
    return ESP_OK;
}

time_t ntp_time_get_last_sync_time(void)
{
    return ntp_mgr.last_sync_time;
}

esp_err_t ntp_time_deinit(void)
{
    if (!ntp_mgr.initialized) {
        return ESP_OK;
    }

    esp_sntp_stop();
    ntp_mgr.initialized = false;
    ESP_LOGI(TAG, "NTP time module deinitialized");

    return ESP_OK;
}

// SNTP同步回调函数
static void sntp_sync_time_cb(struct timeval *tv)
{
    ntp_mgr.state = NTP_STATE_SYNCED;
    ntp_mgr.last_sync_time = tv->tv_sec;

    ESP_LOGI(TAG, "Time synchronized successfully");

    // 调用用户回调
    if (ntp_mgr.config.sync_callback) {
        ntp_mgr.config.sync_callback(NTP_STATE_SYNCED, ntp_mgr.config.user_data);
    }
}
