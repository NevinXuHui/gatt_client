/*
 * SPDX-FileCopyrightText: 2021-2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */

/****************************************************************************
*
* 模块化的BLE GATT客户端主程序
* 集成WiFi连接、NTP时间同步、GPIO按键中断和BLE通信功能
* 
* 架构特点：
* - 模块化设计，各功能独立
* - 统一的配置管理
* - 清晰的接口定义
* - 完善的错误处理
*
****************************************************************************/

#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include <stdio.h>
#include "nvs.h"
#include "nvs_flash.h"

// FreeRTOS相关头文件
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"

// 系统相关头文件
#include "esp_system.h"
#include "esp_log.h"
#include "esp_err.h"

// 自定义模块头文件
#include "app_config.h"
#include "wifi_manager.h"
#include "ntp_time.h"
#include "gpio_button.h"
#include "ble_gattc.h"

static const char* TAG = "MAIN_APP";

// 应用程序状态管理
typedef struct {
    bool wifi_connected;
    bool ntp_synced;
    bool gpio_initialized;
    bool ble_initialized;
    bool ble_connected;
    char current_time[64];
    char ip_address[32];
    char ble_device_name[32];
    uint32_t button_presses;
    uint32_t ble_send_count;
} app_state_t;

static app_state_t app_state = {0};

// 事件回调函数
static void wifi_event_callback(wifi_state_t state, void* user_data);
static void ntp_sync_callback(ntp_sync_state_t state, void* user_data);
static void gpio_button_callback(const gpio_button_event_data_t* event_data, void* user_data);
static void ble_gattc_callback(const ble_gattc_event_data_t* event_data, void* user_data);

// 应用程序任务
static void app_status_task(void* arg);
static void app_main_task(void* arg);

// 初始化函数
static esp_err_t app_nvs_init(void);
static esp_err_t app_wifi_init(void);
static esp_err_t app_ntp_init(void);
static esp_err_t app_gpio_init(void);
static esp_err_t app_ble_init(void);

void app_main(void)
{
    ESP_LOGI(TAG, "=== 模块化BLE GATT客户端启动 ===");
    ESP_LOGI(TAG, "版本: 1.0.0");
    ESP_LOGI(TAG, "编译时间: %s %s", __DATE__, __TIME__);

    // 初始化NVS
    esp_err_t ret = app_nvs_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "NVS初始化失败");
        return;
    }

    // 初始化WiFi
    ret = app_wifi_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "WiFi初始化失败");
        return;
    }

    // 初始化NTP
    ret = app_ntp_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "NTP初始化失败");
        return;
    }

    // 初始化GPIO按键
    ret = app_gpio_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "GPIO初始化失败");
        return;
    }

    // 初始化BLE GATT客户端
    ret = app_ble_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "BLE初始化失败");
        return;
    }

    // 创建应用程序任务
    xTaskCreate(app_status_task, "app_status", 2048, NULL, 3, NULL);
    xTaskCreate(app_main_task, "app_main", 4096, NULL, 5, NULL);

    ESP_LOGI(TAG, "=== 应用程序初始化完成 ===");
}

static esp_err_t app_nvs_init(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_LOGI(TAG, "NVS初始化成功");
    return ESP_OK;
}

static esp_err_t app_wifi_init(void)
{
    wifi_manager_config_t wifi_config = {
        .ssid = WIFI_SSID,
        .password = WIFI_PASS,
        .max_retry = WIFI_MAXIMUM_RETRY,
        .event_callback = wifi_event_callback,
        .user_data = &app_state
    };

    esp_err_t ret = wifi_manager_init(&wifi_config);
    if (ret != ESP_OK) {
        return ret;
    }

    ret = wifi_manager_start();
    if (ret != ESP_OK) {
        return ret;
    }

    ESP_LOGI(TAG, "WiFi初始化成功，正在连接到: %s", WIFI_SSID);
    return ESP_OK;
}

static esp_err_t app_ntp_init(void)
{
    ntp_config_t ntp_config = {
        .primary_server = NTP_SERVER,
        .backup_server = NTP_SERVER_BACKUP,
        .timezone = TIME_ZONE,
        .sync_timeout_ms = NTP_SYNC_TIMEOUT_MS,
        .sync_callback = ntp_sync_callback,
        .user_data = &app_state
    };

    esp_err_t ret = ntp_time_init(&ntp_config);
    if (ret != ESP_OK) {
        return ret;
    }

    ESP_LOGI(TAG, "NTP初始化成功，时区: %s", TIME_ZONE);
    return ESP_OK;
}

static esp_err_t app_gpio_init(void)
{
    gpio_button_config_t gpio_config = {
        .gpio_num = GPIO_BUTTON_PIN,
        .debounce_time_ms = GPIO_DEBOUNCE_TIME_MS,
        .long_press_time_ms = 1000,  // 1秒长按
        .pull_up_enable = true,
        .pull_down_enable = false,
        .event_callback = gpio_button_callback,
        .user_data = &app_state
    };

    esp_err_t ret = gpio_button_init(&gpio_config);
    if (ret != ESP_OK) {
        return ret;
    }

    ret = gpio_button_start();
    if (ret != ESP_OK) {
        return ret;
    }

    app_state.gpio_initialized = true;
    ESP_LOGI(TAG, "GPIO按键初始化成功，引脚: %d", GPIO_BUTTON_PIN);
    return ESP_OK;
}

static esp_err_t app_ble_init(void)
{
    ESP_LOGI(TAG, "初始化BLE GATT客户端...");

    // 配置BLE GATT客户端
    ble_gattc_config_t ble_config = {
        .target_service_uuid = REMOTE_SERVICE_UUID,
        .target_char_uuid = CUSTOM_CHAR_UUID_0013,
        .scan_duration = BLE_SCAN_DURATION,
        .auto_reconnect = BLE_AUTO_RECONNECT,
        .event_callback = ble_gattc_callback,
        .user_data = &app_state
    };

    // 复制目标设备名称
    strncpy(ble_config.target_device_name, BLE_TARGET_DEVICE_NAME, sizeof(ble_config.target_device_name) - 1);
    ble_config.target_device_name[sizeof(ble_config.target_device_name) - 1] = '\0';

    esp_err_t ret = ble_gattc_init(&ble_config);
    if (ret != ESP_OK) {
        return ret;
    }

    ret = ble_gattc_start();
    if (ret != ESP_OK) {
        return ret;
    }

    app_state.ble_initialized = true;
    ESP_LOGI(TAG, "BLE GATT客户端初始化成功");
    ESP_LOGI(TAG, "目标设备: %s", BLE_TARGET_DEVICE_NAME);
    ESP_LOGI(TAG, "目标服务: 0x%04X", REMOTE_SERVICE_UUID);
    ESP_LOGI(TAG, "目标特征: 0x%04X", CUSTOM_CHAR_UUID_0013);

    // 开始扫描
    ret = ble_gattc_start_scan();
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "BLE扫描启动失败: %s", esp_err_to_name(ret));
    }

    return ESP_OK;
}

// WiFi事件回调
static void wifi_event_callback(wifi_state_t state, void* user_data)
{
    app_state_t* app = (app_state_t*)user_data;
    
    switch (state) {
        case WIFI_STATE_CONNECTED:
            app->wifi_connected = true;
            wifi_manager_get_ip_string(app->ip_address, sizeof(app->ip_address));
            ESP_LOGI(TAG, "WiFi连接成功，IP地址: %s", app->ip_address);

            // WiFi连接成功后启动NTP同步
            esp_err_t ntp_ret = ntp_time_start_sync();
            if (ntp_ret != ESP_OK) {
                ESP_LOGE(TAG, "NTP同步启动失败: %s", esp_err_to_name(ntp_ret));
            } else {
                ESP_LOGI(TAG, "NTP同步已启动");
            }
            break;
            
        case WIFI_STATE_DISCONNECTED:
            app->wifi_connected = false;
            app->ntp_synced = false;
            ESP_LOGW(TAG, "WiFi连接断开");
            break;
            
        case WIFI_STATE_FAILED:
            app->wifi_connected = false;
            ESP_LOGE(TAG, "WiFi连接失败");
            break;
            
        default:
            break;
    }
}

// NTP同步回调
static void ntp_sync_callback(ntp_sync_state_t state, void* user_data)
{
    app_state_t* app = (app_state_t*)user_data;
    
    switch (state) {
        case NTP_STATE_SYNCED:
            app->ntp_synced = true;
            ntp_time_get_formatted_time(app->current_time, sizeof(app->current_time), NULL);
            ESP_LOGI(TAG, "NTP时间同步成功: %s", app->current_time);
            break;
            
        case NTP_STATE_FAILED:
            app->ntp_synced = false;
            ESP_LOGE(TAG, "NTP时间同步失败");
            break;
            
        default:
            break;
    }
}

// GPIO按键事件回调
static void gpio_button_callback(const gpio_button_event_data_t* event_data, void* user_data)
{
    app_state_t* app = (app_state_t*)user_data;
    
    switch (event_data->event) {
        case GPIO_BUTTON_EVENT_PRESSED:
            app->button_presses++;
            ESP_LOGI(TAG, "按键按下 (第%d次)", app->button_presses);

            // 按键按下时发送BLE数据
            if (app->ble_connected) {
                ESP_LOGI(TAG, "发送BLE按键数据...");
                esp_err_t ret = ble_gattc_send_button_data();
                if (ret == ESP_OK) {
                    app->ble_send_count++;
                    ESP_LOGI(TAG, "BLE数据发送成功 (第%d次)", app->ble_send_count);
                } else {
                    ESP_LOGE(TAG, "BLE数据发送失败: %s", esp_err_to_name(ret));
                }
            } else {
                ESP_LOGW(TAG, "BLE未连接，无法发送数据");
            }

            // 更新当前时间
            if (app->ntp_synced) {
                ntp_time_get_formatted_time(app->current_time, sizeof(app->current_time), NULL);
                ESP_LOGI(TAG, "当前时间: %s", app->current_time);
            } else {
                ESP_LOGW(TAG, "时间未同步");
            }
            break;
            
        case GPIO_BUTTON_EVENT_LONG_PRESSED:
            ESP_LOGI(TAG, "按键长按 (%d ms)", event_data->press_duration_ms);
            // 长按触发WiFi重连功能
            if (app->wifi_connected) {
                ESP_LOGI(TAG, "长按触发WiFi重连...");
                app->wifi_connected = false;  // 先标记为断开状态
                esp_err_t ret = wifi_manager_reconnect();
                if (ret != ESP_OK) {
                    ESP_LOGE(TAG, "WiFi重连失败: %s", esp_err_to_name(ret));
                }
            } else {
                ESP_LOGI(TAG, "WiFi未连接，尝试重新启动WiFi...");
                wifi_manager_start();
            }
            break;
            
        case GPIO_BUTTON_EVENT_RELEASED:
            ESP_LOGI(TAG, "按键释放 (%d ms)", event_data->press_duration_ms);
            break;
            
        default:
            break;
    }
}

// BLE GATT客户端事件回调
static void ble_gattc_callback(const ble_gattc_event_data_t* event_data, void* user_data)
{
    app_state_t* app = (app_state_t*)user_data;

    switch (event_data->event) {
        case BLE_GATTC_EVENT_SCAN_START:
            ESP_LOGI(TAG, "BLE扫描开始");
            break;

        case BLE_GATTC_EVENT_DEVICE_FOUND:
            ESP_LOGI(TAG, "发现目标设备: %s (RSSI: %d dBm)",
                     event_data->device_found.device.name,
                     event_data->device_found.device.rssi);
            strncpy(app->ble_device_name, event_data->device_found.device.name,
                    sizeof(app->ble_device_name) - 1);
            break;

        case BLE_GATTC_EVENT_CONNECTED:
            app->ble_connected = true;
            ESP_LOGI(TAG, "BLE设备连接成功: %s", event_data->connected.device.name);
            break;

        case BLE_GATTC_EVENT_DISCONNECTED:
            app->ble_connected = false;
            ESP_LOGI(TAG, "BLE设备断开连接，原因: %d", event_data->disconnected.reason);
            break;

        case BLE_GATTC_EVENT_SERVICE_DISCOVERED:
            ESP_LOGI(TAG, "BLE服务发现完成，发现 %d 个服务",
                     event_data->service_discovered.service_count);
            break;

        case BLE_GATTC_EVENT_READY:
            ESP_LOGI(TAG, "BLE GATT客户端就绪，可以发送数据");
            break;

        case BLE_GATTC_EVENT_DATA_SENT:
            if (event_data->data_sent.success) {
                ESP_LOGI(TAG, "BLE数据发送成功，句柄: %d", event_data->data_sent.char_handle);
            } else {
                ESP_LOGE(TAG, "BLE数据发送失败，句柄: %d", event_data->data_sent.char_handle);
            }
            break;

        case BLE_GATTC_EVENT_DATA_RECEIVED:
            ESP_LOGI(TAG, "收到BLE数据，长度: %d", event_data->data_received.data_len);
            ESP_LOG_BUFFER_HEX(TAG, event_data->data_received.data, event_data->data_received.data_len);
            break;

        case BLE_GATTC_EVENT_ERROR:
            ESP_LOGE(TAG, "BLE错误: %s (%s)",
                     event_data->error.description,
                     esp_err_to_name(event_data->error.error_code));
            break;

        default:
            break;
    }
}

// 应用程序状态监控任务
static void app_status_task(void* arg)
{
    ESP_LOGI(TAG, "状态监控任务启动");
    
    while (1) {
        // 每10秒打印一次状态
        vTaskDelay(pdMS_TO_TICKS(10000));
        
        ESP_LOGI(TAG, "=== 系统状态 ===");

        // 获取详细的WiFi连接信息
        char wifi_info[128];
        if (wifi_manager_get_connection_info(wifi_info, sizeof(wifi_info)) == ESP_OK) {
            ESP_LOGI(TAG, "%s", wifi_info);
        } else {
            ESP_LOGI(TAG, "WiFi: %s", app_state.wifi_connected ? "已连接" : "未连接");
            if (app_state.wifi_connected) {
                ESP_LOGI(TAG, "IP地址: %s", app_state.ip_address);
            }
        }
        ESP_LOGI(TAG, "NTP: %s", app_state.ntp_synced ? "已同步" : "未同步");
        if (app_state.ntp_synced) {
            ntp_time_get_formatted_time(app_state.current_time, sizeof(app_state.current_time), NULL);
            ESP_LOGI(TAG, "当前时间: %s", app_state.current_time);
        }
        ESP_LOGI(TAG, "GPIO: %s", app_state.gpio_initialized ? "已初始化" : "未初始化");
        ESP_LOGI(TAG, "按键次数: %d", app_state.button_presses);
        ESP_LOGI(TAG, "===============");
    }
}

// 应用程序主任务
static void app_main_task(void* arg)
{
    ESP_LOGI(TAG, "主任务启动");
    
    while (1) {
        // 主任务可以处理其他业务逻辑
        // 比如BLE通信、数据处理等
        
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
