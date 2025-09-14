#include "wifi_manager.h"
#include "app_config.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_log.h"
#include "lwip/err.h"
#include "lwip/sys.h"
#include <string.h>

static const char* TAG = "WIFI_MGR";

// WiFi管理器状态
static struct {
    wifi_manager_config_t config;
    wifi_state_t state;
    EventGroupHandle_t event_group;
    esp_netif_t* netif;
    int retry_count;
    bool initialized;
} wifi_mgr = {0};

// 前向声明
static void wifi_event_handler(void* arg, esp_event_base_t event_base,
                              int32_t event_id, void* event_data);
static void ip_event_handler(void* arg, esp_event_base_t event_base,
                            int32_t event_id, void* event_data);

esp_err_t wifi_manager_init(const wifi_manager_config_t* config)
{
    if (wifi_mgr.initialized) {
        ESP_LOGW(TAG, "WiFi manager already initialized");
        return ESP_OK;
    }

    if (!config) {
        ESP_LOGE(TAG, "WiFi config is NULL");
        return ESP_ERR_INVALID_ARG;
    }

    // 复制配置
    memcpy(&wifi_mgr.config, config, sizeof(wifi_manager_config_t));
    wifi_mgr.state = WIFI_STATE_DISCONNECTED;
    wifi_mgr.retry_count = 0;

    // 创建事件组
    wifi_mgr.event_group = xEventGroupCreate();
    if (!wifi_mgr.event_group) {
        ESP_LOGE(TAG, "Failed to create event group");
        return ESP_ERR_NO_MEM;
    }

    // 初始化网络接口
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    wifi_mgr.netif = esp_netif_create_default_wifi_sta();
    if (!wifi_mgr.netif) {
        ESP_LOGE(TAG, "Failed to create netif");
        vEventGroupDelete(wifi_mgr.event_group);
        return ESP_FAIL;
    }

    // 初始化WiFi
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    // 注册事件处理器
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                              &wifi_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP,
                                              &ip_event_handler, NULL));

    wifi_mgr.initialized = true;
    ESP_LOGI(TAG, "WiFi manager initialized");

    return ESP_OK;
}

esp_err_t wifi_manager_start(void)
{
    if (!wifi_mgr.initialized) {
        ESP_LOGE(TAG, "WiFi manager not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    // 配置WiFi
    wifi_config_t wifi_config = {
        .sta = {
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,
            .pmf_cfg = {
                .capable = true,
                .required = false
            },
        },
    };

    strncpy((char*)wifi_config.sta.ssid, wifi_mgr.config.ssid, sizeof(wifi_config.sta.ssid));
    strncpy((char*)wifi_config.sta.password, wifi_mgr.config.password, sizeof(wifi_config.sta.password));

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    wifi_mgr.state = WIFI_STATE_CONNECTING;
    ESP_LOGI(TAG, "WiFi connecting to SSID: %s", wifi_mgr.config.ssid);

    return ESP_OK;
}

esp_err_t wifi_manager_stop(void)
{
    if (!wifi_mgr.initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    ESP_ERROR_CHECK(esp_wifi_stop());
    wifi_mgr.state = WIFI_STATE_DISCONNECTED;
    wifi_mgr.retry_count = 0;

    ESP_LOGI(TAG, "WiFi stopped");
    return ESP_OK;
}

wifi_state_t wifi_manager_get_state(void)
{
    return wifi_mgr.state;
}

bool wifi_manager_is_connected(void)
{
    return wifi_mgr.state == WIFI_STATE_CONNECTED;
}

esp_err_t wifi_manager_get_ip_string(char* ip_str, size_t max_len)
{
    if (!ip_str || max_len == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    if (!wifi_manager_is_connected()) {
        strncpy(ip_str, "Not connected", max_len - 1);
        ip_str[max_len - 1] = '\0';
        return ESP_ERR_INVALID_STATE;
    }

    esp_netif_ip_info_t ip_info;
    ESP_ERROR_CHECK(esp_netif_get_ip_info(wifi_mgr.netif, &ip_info));

    snprintf(ip_str, max_len, IPSTR, IP2STR(&ip_info.ip));
    return ESP_OK;
}

esp_err_t wifi_manager_reconnect(void)
{
    if (!wifi_mgr.initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGI(TAG, "WiFi reconnect requested...");

    // 重置重试计数
    wifi_mgr.retry_count = 0;
    wifi_mgr.state = WIFI_STATE_CONNECTING;

    // 安全地断开连接
    esp_err_t ret = esp_wifi_disconnect();
    if (ret != ESP_OK && ret != ESP_ERR_WIFI_NOT_CONNECT) {
        ESP_LOGW(TAG, "WiFi disconnect failed: %s", esp_err_to_name(ret));
    }

    // 等待一小段时间确保断开完成
    vTaskDelay(pdMS_TO_TICKS(100));

    // 重新连接
    ret = esp_wifi_connect();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "WiFi connect failed: %s", esp_err_to_name(ret));
        wifi_mgr.state = WIFI_STATE_FAILED;
        return ret;
    }

    ESP_LOGI(TAG, "WiFi reconnecting...");
    return ESP_OK;
}

esp_err_t wifi_manager_deinit(void)
{
    if (!wifi_mgr.initialized) {
        return ESP_OK;
    }

    // 停止WiFi
    esp_wifi_stop();
    esp_wifi_deinit();

    // 注销事件处理器
    esp_event_handler_unregister(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler);
    esp_event_handler_unregister(IP_EVENT, IP_EVENT_STA_GOT_IP, &ip_event_handler);

    // 清理资源
    if (wifi_mgr.event_group) {
        vEventGroupDelete(wifi_mgr.event_group);
        wifi_mgr.event_group = NULL;
    }

    if (wifi_mgr.netif) {
        esp_netif_destroy_default_wifi(wifi_mgr.netif);
        wifi_mgr.netif = NULL;
    }

    wifi_mgr.initialized = false;
    ESP_LOGI(TAG, "WiFi manager deinitialized");

    return ESP_OK;
}

// 事件处理器实现
static void wifi_event_handler(void* arg, esp_event_base_t event_base,
                              int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (wifi_mgr.retry_count < wifi_mgr.config.max_retry) {
            esp_wifi_connect();
            wifi_mgr.retry_count++;
            ESP_LOGI(TAG, "Retry to connect to the AP (%d/%d)", 
                     wifi_mgr.retry_count, wifi_mgr.config.max_retry);
        } else {
            xEventGroupSetBits(wifi_mgr.event_group, WIFI_FAIL_BIT);
            wifi_mgr.state = WIFI_STATE_FAILED;
            ESP_LOGI(TAG, "Connect to the AP failed");
            
            // 调用回调函数
            if (wifi_mgr.config.event_callback) {
                wifi_mgr.config.event_callback(WIFI_STATE_FAILED, wifi_mgr.config.user_data);
            }
        }
    }
}

static void ip_event_handler(void* arg, esp_event_base_t event_base,
                            int32_t event_id, void* event_data)
{
    if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "Got IP:" IPSTR, IP2STR(&event->ip_info.ip));
        
        wifi_mgr.retry_count = 0;
        wifi_mgr.state = WIFI_STATE_CONNECTED;
        xEventGroupSetBits(wifi_mgr.event_group, WIFI_CONNECTED_BIT);
        
        // 调用回调函数
        if (wifi_mgr.config.event_callback) {
            wifi_mgr.config.event_callback(WIFI_STATE_CONNECTED, wifi_mgr.config.user_data);
        }
    }
}
