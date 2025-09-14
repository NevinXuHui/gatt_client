#include "ble_gattc.h"
#include "app_config.h"
#include "esp_log.h"
#include "esp_bt_main.h"
#include "esp_gatt_common_api.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include <string.h>

static const char* TAG = "BLE_GATTC";

// BLE GATT客户端管理结构
typedef struct {
    bool initialized;
    bool started;
    ble_gattc_config_t config;
    ble_gattc_state_t state;
    
    // BLE相关
    esp_gatt_if_t gattc_if;
    uint16_t conn_id;
    esp_bd_addr_t remote_bda;
    
    // 服务和特征信息
    uint16_t service_start_handle;
    uint16_t service_end_handle;
    uint16_t char_handle;
    uint16_t target_char_handle;
    
    // 连接设备信息
    ble_device_info_t connected_device;
    
    // 统计信息
    uint32_t scan_count;
    uint32_t connect_count;
    uint32_t send_count;
} ble_gattc_manager_t;

static ble_gattc_manager_t ble_gattc = {0};

// GATT客户端配置文件结构
typedef struct {
    esp_gattc_cb_t gattc_cb;
    uint16_t gattc_if;
    uint16_t app_id;
    uint16_t conn_id;
    uint16_t service_start_handle;
    uint16_t service_end_handle;
    uint16_t char_handle;
    esp_bd_addr_t remote_bda;
} gattc_profile_inst_t;

static gattc_profile_inst_t gl_profile_tab[PROFILE_NUM] = {
    [PROFILE_A_APP_ID] = {
        .gattc_cb = NULL,  // 将在初始化时设置
        .gattc_if = ESP_GATT_IF_NONE,
    },
};

// 扫描参数
static esp_ble_scan_params_t ble_scan_params = {
    .scan_type              = BLE_SCAN_TYPE_ACTIVE,
    .own_addr_type          = BLE_ADDR_TYPE_PUBLIC,
    .scan_filter_policy     = BLE_SCAN_FILTER_ALLOW_ALL,
    .scan_interval          = 0x50,
    .scan_window            = 0x30,
    .scan_duplicate         = BLE_SCAN_DUPLICATE_DISABLE
};

// 前向声明
static void esp_gap_cb(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param);
static void esp_gattc_cb(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param);
static void gattc_profile_event_handler(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param);
static uint8_t* get_device_name_from_adv_data(uint8_t *adv_data, uint16_t adv_data_len, uint16_t scan_rsp_len, uint8_t *name_len);
static void trigger_event_callback(ble_gattc_event_t event, void* event_data);
static esp_err_t ble_gattc_init_bluetooth(void);

esp_err_t ble_gattc_init(const ble_gattc_config_t* config)
{
    if (ble_gattc.initialized) {
        ESP_LOGW(TAG, "BLE GATT client already initialized");
        return ESP_OK;
    }

    if (!config) {
        ESP_LOGE(TAG, "Invalid configuration");
        return ESP_ERR_INVALID_ARG;
    }

    // 复制配置
    memcpy(&ble_gattc.config, config, sizeof(ble_gattc_config_t));
    
    // 初始化状态
    ble_gattc.state = BLE_GATTC_STATE_IDLE;
    ble_gattc.gattc_if = ESP_GATT_IF_NONE;
    ble_gattc.conn_id = 0;
    ble_gattc.char_handle = 0;
    ble_gattc.target_char_handle = 0;
    
    // 设置GATT客户端回调
    gl_profile_tab[PROFILE_A_APP_ID].gattc_cb = gattc_profile_event_handler;
    
    ESP_LOGI(TAG, "BLE GATT client initialized");
    ESP_LOGI(TAG, "Target device: %s", ble_gattc.config.target_device_name);
    ESP_LOGI(TAG, "Target service UUID: 0x%04X", ble_gattc.config.target_service_uuid);
    ESP_LOGI(TAG, "Target char UUID: 0x%04X", ble_gattc.config.target_char_uuid);

    // 执行完整的BLE初始化序列
    esp_err_t ret = ble_gattc_init_bluetooth();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "BLE bluetooth initialization failed");
        return ret;
    }

    ble_gattc.initialized = true;
    return ESP_OK;
}

esp_err_t ble_gattc_start(void)
{
    if (!ble_gattc.initialized) {
        ESP_LOGE(TAG, "BLE GATT client not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (ble_gattc.started) {
        ESP_LOGW(TAG, "BLE GATT client already started");
        return ESP_OK;
    }

    ESP_LOGI(TAG, "Starting BLE GATT client...");

    // 注册GAP回调
    esp_err_t ret = esp_ble_gap_register_callback(esp_gap_cb);
    if (ret) {
        ESP_LOGE(TAG, "GAP register failed, error code = %x", ret);
        return ret;
    }

    // 注册GATT客户端回调
    ret = esp_ble_gattc_register_callback(esp_gattc_cb);
    if (ret) {
        ESP_LOGE(TAG, "GATTC register callback failed, error code = %x", ret);
        return ret;
    }

    // 注册GATT客户端应用
    ret = esp_ble_gattc_app_register(PROFILE_A_APP_ID);
    if (ret) {
        ESP_LOGE(TAG, "GATTC app register failed, error code = %x", ret);
        return ret;
    }

    ble_gattc.started = true;
    ESP_LOGI(TAG, "BLE GATT client started successfully");

    return ESP_OK;
}

esp_err_t ble_gattc_stop(void)
{
    if (!ble_gattc.started) {
        ESP_LOGW(TAG, "BLE GATT client not started");
        return ESP_OK;
    }

    ESP_LOGI(TAG, "Stopping BLE GATT client...");

    // 断开连接
    if (ble_gattc.state == BLE_GATTC_STATE_CONNECTED || 
        ble_gattc.state == BLE_GATTC_STATE_SERVICE_DISCOVERED ||
        ble_gattc.state == BLE_GATTC_STATE_READY) {
        ble_gattc_disconnect();
    }

    // 停止扫描
    if (ble_gattc.state == BLE_GATTC_STATE_SCANNING) {
        ble_gattc_stop_scan();
    }

    ble_gattc.started = false;
    ble_gattc.state = BLE_GATTC_STATE_IDLE;
    
    ESP_LOGI(TAG, "BLE GATT client stopped");

    return ESP_OK;
}

esp_err_t ble_gattc_start_scan(void)
{
    if (!ble_gattc.started) {
        ESP_LOGE(TAG, "BLE GATT client not started");
        return ESP_ERR_INVALID_STATE;
    }

    if (ble_gattc.state == BLE_GATTC_STATE_SCANNING) {
        ESP_LOGW(TAG, "Already scanning, skipping");
        return ESP_OK;
    }

    if (ble_gattc.state == BLE_GATTC_STATE_CONNECTED ||
        ble_gattc.state == BLE_GATTC_STATE_READY) {
        ESP_LOGW(TAG, "Already connected, skipping scan");
        return ESP_OK;
    }

    ESP_LOGI(TAG, "Starting BLE scan...");

    // 只有在扫描状态时才停止扫描
    if (ble_gattc.state == BLE_GATTC_STATE_SCANNING) {
        ESP_LOGI(TAG, "Stopping existing scan first...");
        esp_err_t stop_ret = esp_ble_gap_stop_scanning();
        if (stop_ret != ESP_OK) {
            ESP_LOGW(TAG, "Stop scanning failed: %s", esp_err_to_name(stop_ret));
        }
        vTaskDelay(pdMS_TO_TICKS(100));  // 等待100ms确保停止完成
    }

    esp_err_t ret = esp_ble_gap_set_scan_params(&ble_scan_params);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Set scan params failed, error code = %x", ret);
        return ret;
    }

    ble_gattc.state = BLE_GATTC_STATE_SCANNING;

    // 触发扫描开始事件
    ble_gattc_event_data_t scan_start_event = {
        .event = BLE_GATTC_EVENT_SCAN_START
    };
    trigger_event_callback(BLE_GATTC_EVENT_SCAN_START, &scan_start_event);

    return ESP_OK;
}

esp_err_t ble_gattc_stop_scan(void)
{
    if (ble_gattc.state != BLE_GATTC_STATE_SCANNING) {
        ESP_LOGW(TAG, "Not scanning");
        return ESP_OK;
    }

    ESP_LOGI(TAG, "Stopping BLE scan...");

    esp_err_t ret = esp_ble_gap_stop_scanning();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Stop scanning failed, error code = %x", ret);
        return ret;
    }

    ble_gattc.state = BLE_GATTC_STATE_IDLE;
    return ESP_OK;
}

esp_err_t ble_gattc_connect_device(const ble_device_info_t* device_info)
{
    if (!ble_gattc.started) {
        ESP_LOGE(TAG, "BLE GATT client not started");
        return ESP_ERR_INVALID_STATE;
    }

    if (!device_info) {
        ESP_LOGE(TAG, "Invalid device info");
        return ESP_ERR_INVALID_ARG;
    }

    if (ble_gattc.state == BLE_GATTC_STATE_CONNECTED ||
        ble_gattc.state == BLE_GATTC_STATE_CONNECTING) {
        ESP_LOGW(TAG, "Already connected or connecting");
        return ESP_OK;
    }

    ESP_LOGI(TAG, "Connecting to device: %s", device_info->name);

    // 停止扫描
    if (ble_gattc.state == BLE_GATTC_STATE_SCANNING) {
        esp_ble_gap_stop_scanning();
    }

    // 使用原来的增强连接方法（参考原始代码）
    esp_ble_gatt_creat_conn_params_t creat_conn_params = {0};
    memcpy(&creat_conn_params.remote_bda, device_info->bda, ESP_BD_ADDR_LEN);
    creat_conn_params.remote_addr_type = device_info->addr_type;
    creat_conn_params.own_addr_type = BLE_ADDR_TYPE_PUBLIC;
    creat_conn_params.is_direct = true;
    creat_conn_params.is_aux = false;
    creat_conn_params.phy_mask = 0x0;

    esp_err_t ret = esp_ble_gattc_enh_open(ble_gattc.gattc_if, &creat_conn_params);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Enhanced open connection failed, error code = %x", ret);
        return ret;
    }

    ble_gattc.state = BLE_GATTC_STATE_CONNECTING;
    memcpy(&ble_gattc.connected_device, device_info, sizeof(ble_device_info_t));

    return ESP_OK;
}

esp_err_t ble_gattc_disconnect(void)
{
    if (ble_gattc.state != BLE_GATTC_STATE_CONNECTED &&
        ble_gattc.state != BLE_GATTC_STATE_SERVICE_DISCOVERED &&
        ble_gattc.state != BLE_GATTC_STATE_READY) {
        ESP_LOGW(TAG, "Not connected");
        return ESP_OK;
    }

    ESP_LOGI(TAG, "Disconnecting from device...");

    esp_err_t ret = esp_ble_gattc_close(ble_gattc.gattc_if, ble_gattc.conn_id);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Disconnect failed, error code = %x", ret);
        return ret;
    }

    return ESP_OK;
}

ble_gattc_state_t ble_gattc_get_state(void)
{
    return ble_gattc.state;
}

bool ble_gattc_is_connected(void)
{
    return (ble_gattc.state == BLE_GATTC_STATE_CONNECTED ||
            ble_gattc.state == BLE_GATTC_STATE_SERVICE_DISCOVERED ||
            ble_gattc.state == BLE_GATTC_STATE_READY);
}

esp_err_t ble_gattc_get_connected_device(ble_device_info_t* device_info)
{
    if (!device_info) {
        return ESP_ERR_INVALID_ARG;
    }

    if (!ble_gattc_is_connected()) {
        return ESP_ERR_INVALID_STATE;
    }

    memcpy(device_info, &ble_gattc.connected_device, sizeof(ble_device_info_t));
    return ESP_OK;
}

esp_err_t ble_gattc_deinit(void)
{
    if (!ble_gattc.initialized) {
        return ESP_OK;
    }

    // 停止服务
    ble_gattc_stop();

    ble_gattc.initialized = false;
    memset(&ble_gattc, 0, sizeof(ble_gattc_manager_t));

    ESP_LOGI(TAG, "BLE GATT client deinitialized");
    return ESP_OK;
}

esp_err_t ble_gattc_send_data(const uint8_t* data, uint16_t data_len)
{
    if (!ble_gattc_is_connected()) {
        ESP_LOGE(TAG, "Device not connected");
        return ESP_ERR_INVALID_STATE;
    }

    if (ble_gattc.target_char_handle == 0) {
        ESP_LOGE(TAG, "Target characteristic not found");
        return ESP_ERR_INVALID_STATE;
    }

    if (!data || data_len == 0) {
        ESP_LOGE(TAG, "Invalid data");
        return ESP_ERR_INVALID_ARG;
    }

    ESP_LOGI(TAG, "Sending %d bytes to characteristic 0x%04X (handle: %d)",
             data_len, ble_gattc.config.target_char_uuid, ble_gattc.target_char_handle);
    ESP_LOG_BUFFER_HEX(TAG, data, data_len);

    esp_err_t ret = esp_ble_gattc_write_char(ble_gattc.gattc_if,
                                             ble_gattc.conn_id,
                                             ble_gattc.target_char_handle,
                                             data_len,
                                             (uint8_t*)data,
                                             ESP_GATT_WRITE_TYPE_RSP,
                                             ESP_GATT_AUTH_REQ_NONE);

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Write characteristic failed, error code = %x", ret);

        // 触发发送失败事件
        ble_gattc_event_data_t event_data = {
            .event = BLE_GATTC_EVENT_DATA_SENT,
            .data_sent = {
                .char_handle = ble_gattc.target_char_handle,
                .data = (uint8_t*)data,
                .data_len = data_len,
                .success = false
            }
        };
        trigger_event_callback(BLE_GATTC_EVENT_DATA_SENT, &event_data);

        return ret;
    }

    ble_gattc.send_count++;
    ESP_LOGI(TAG, "Write characteristic initiated successfully");

    return ESP_OK;
}

esp_err_t ble_gattc_send_button_data(void)
{
    // 预定义的按键数据（来自原始代码）
    static const uint8_t button_data[] = {
        0x00, 0x02, 0xaa, 0x01, 0x00, 0x5c, 0xa2, 0x9a, 0x48, 0x0b,
        0x31, 0x38, 0x38, 0x36, 0x37, 0x31, 0x31, 0x32, 0x36, 0x31
    };

    ESP_LOGI(TAG, "Sending button data...");
    return ble_gattc_send_data(button_data, sizeof(button_data));
}

// 辅助函数：触发事件回调
static void trigger_event_callback(ble_gattc_event_t event, void* event_data)
{
    if (ble_gattc.config.event_callback && event_data) {
        ble_gattc.config.event_callback((const ble_gattc_event_data_t*)event_data, ble_gattc.config.user_data);
    } else if (ble_gattc.config.event_callback && !event_data) {
        // 为NULL事件数据创建一个临时的事件数据结构
        ble_gattc_event_data_t temp_event_data = {
            .event = event
        };
        ble_gattc.config.event_callback(&temp_event_data, ble_gattc.config.user_data);
    }
}

// 从广告数据中提取设备名称（优化版本，参考原始代码）
static uint8_t* get_device_name_from_adv_data(uint8_t *adv_data, uint16_t adv_data_len, uint16_t scan_rsp_len, uint8_t *name_len)
{
    uint8_t *name = NULL;
    uint16_t total_len = adv_data_len + scan_rsp_len;

    if (adv_data == NULL || name_len == NULL) {
        *name_len = 0;
        return NULL;
    }

    // 1. 首先尝试获取完整设备名称 (Complete Local Name)
    name = esp_ble_resolve_adv_data_by_type(adv_data, total_len, ESP_BLE_AD_TYPE_NAME_CMPL, name_len);
    if (name != NULL && *name_len > 0) {
        ESP_LOGD(TAG, "Found complete device name");
        return name;
    }

    // 2. 如果没有找到完整名称，尝试获取短设备名称 (Shortened Local Name)
    name = esp_ble_resolve_adv_data_by_type(adv_data, total_len, ESP_BLE_AD_TYPE_NAME_SHORT, name_len);
    if (name != NULL && *name_len > 0) {
        ESP_LOGD(TAG, "Found shortened device name");
        return name;
    }

    // 3. 如果仍然没有找到，分别检查广播数据和扫描响应数据
    if (adv_data_len > 0) {
        // 只在广播数据中查找完整名称
        name = esp_ble_resolve_adv_data_by_type(adv_data, adv_data_len, ESP_BLE_AD_TYPE_NAME_CMPL, name_len);
        if (name != NULL && *name_len > 0) {
            ESP_LOGD(TAG, "Found complete name in adv data only");
            return name;
        }

        // 只在广播数据中查找短名称
        name = esp_ble_resolve_adv_data_by_type(adv_data, adv_data_len, ESP_BLE_AD_TYPE_NAME_SHORT, name_len);
        if (name != NULL && *name_len > 0) {
            ESP_LOGD(TAG, "Found short name in adv data only");
            return name;
        }
    }

    if (scan_rsp_len > 0) {
        // 只在扫描响应数据中查找完整名称
        name = esp_ble_resolve_adv_data_by_type(adv_data + adv_data_len, scan_rsp_len, ESP_BLE_AD_TYPE_NAME_CMPL, name_len);
        if (name != NULL && *name_len > 0) {
            ESP_LOGD(TAG, "Found complete name in scan response only");
            return name;
        }

        // 只在扫描响应数据中查找短名称
        name = esp_ble_resolve_adv_data_by_type(adv_data + adv_data_len, scan_rsp_len, ESP_BLE_AD_TYPE_NAME_SHORT, name_len);
        if (name != NULL && *name_len > 0) {
            ESP_LOGD(TAG, "Found short name in scan response only");
            return name;
        }
    }

    // 没有找到任何设备名称
    *name_len = 0;
    return NULL;
}

// GAP事件回调函数
static void esp_gap_cb(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    uint8_t *adv_name = NULL;
    uint8_t adv_name_len = 0;

    switch (event) {
    case ESP_GAP_BLE_SCAN_PARAM_SET_COMPLETE_EVT: {
        ESP_LOGI(TAG, "Scan parameters set complete");

        // 只有在扫描状态下才启动扫描
        if (ble_gattc.state == BLE_GATTC_STATE_SCANNING) {
            uint32_t duration = ble_gattc.config.scan_duration;
            esp_err_t ret = esp_ble_gap_start_scanning(duration);
            if (ret != ESP_OK) {
                ESP_LOGE(TAG, "Start scanning failed, error code = %x", ret);
                ble_gattc.state = BLE_GATTC_STATE_IDLE;
            }
        } else {
            ESP_LOGW(TAG, "Not in scanning state, skipping scan start");
        }
        break;
    }

    case ESP_GAP_BLE_SCAN_START_COMPLETE_EVT:
        if (param->scan_start_cmpl.status != ESP_BT_STATUS_SUCCESS) {
            ESP_LOGE(TAG, "Scanning start failed, status %x", param->scan_start_cmpl.status);
            ble_gattc.state = BLE_GATTC_STATE_ERROR;
        } else {
            ESP_LOGI(TAG, "Scanning started successfully");
            ble_gattc.state = BLE_GATTC_STATE_SCANNING;
        }
        break;

    case ESP_GAP_BLE_SCAN_RESULT_EVT: {
        esp_ble_gap_cb_param_t *scan_result = (esp_ble_gap_cb_param_t *)param;
        switch (scan_result->scan_rst.search_evt) {
        case ESP_GAP_SEARCH_INQ_RES_EVT:
            ble_gattc.scan_count++;

            // 获取设备名称
            adv_name = get_device_name_from_adv_data(scan_result->scan_rst.ble_adv,
                                                   scan_result->scan_rst.adv_data_len,
                                                   scan_result->scan_rst.scan_rsp_len,
                                                   &adv_name_len);

            ESP_LOGI(TAG, "Scan result [%d], device "ESP_BD_ADDR_STR", RSSI %d, name len %u",
                     ble_gattc.scan_count,
                     ESP_BD_ADDR_HEX(scan_result->scan_rst.bda),
                     scan_result->scan_rst.rssi,
                     adv_name_len);

            if (adv_name != NULL && adv_name_len > 0) {
                ESP_LOG_BUFFER_CHAR(TAG, adv_name, adv_name_len);

                // 检查是否是目标设备
                if (strlen(ble_gattc.config.target_device_name) == adv_name_len &&
                    strncmp((char *)adv_name, ble_gattc.config.target_device_name, adv_name_len) == 0) {

                    ESP_LOGI(TAG, "Target device found: %s", ble_gattc.config.target_device_name);

                    // 创建设备信息
                    ble_device_info_t device_info = {0};
                    memcpy(device_info.bda, scan_result->scan_rst.bda, ESP_BD_ADDR_LEN);
                    strncpy(device_info.name, (char*)adv_name, adv_name_len);
                    device_info.name[adv_name_len] = '\0';
                    device_info.rssi = scan_result->scan_rst.rssi;
                    device_info.addr_type = scan_result->scan_rst.ble_addr_type;

                    // 触发设备发现事件
                    ble_gattc_event_data_t event_data = {
                        .event = BLE_GATTC_EVENT_DEVICE_FOUND,
                        .device_found = {
                            .device = device_info
                        }
                    };
                    trigger_event_callback(BLE_GATTC_EVENT_DEVICE_FOUND, &event_data);

                    // 自动连接到目标设备
                    ble_gattc_connect_device(&device_info);
                }
            } else {
                ESP_LOGI(TAG, "Device name not found");
            }
            break;

        case ESP_GAP_SEARCH_INQ_CMPL_EVT:
            ESP_LOGI(TAG, "Scan complete, found %d devices", ble_gattc.scan_count);
            ble_gattc.state = BLE_GATTC_STATE_IDLE;
            break;

        default:
            break;
        }
        break;
    }

    case ESP_GAP_BLE_SCAN_STOP_COMPLETE_EVT:
        if (param->scan_stop_cmpl.status != ESP_BT_STATUS_SUCCESS){
            ESP_LOGE(TAG, "Scan stop failed, status %x", param->scan_stop_cmpl.status);
        } else {
            ESP_LOGI(TAG, "Scan stopped successfully");
        }
        break;

    default:
        break;
    }
}

// GATT客户端事件回调函数
static void esp_gattc_cb(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param)
{
    // 如果事件是注册事件，处理所有配置文件
    if (event == ESP_GATTC_REG_EVT) {
        if (param->reg.status == ESP_GATT_OK) {
            gl_profile_tab[param->reg.app_id].gattc_if = gattc_if;
        } else {
            ESP_LOGE(TAG, "GATT client register failed, app_id %04x, status %d",
                    param->reg.app_id, param->reg.status);
            return;
        }
    }

    // 调用相应配置文件的事件处理函数
    do {
        int idx;
        for (idx = 0; idx < PROFILE_NUM; idx++) {
            if (gattc_if == ESP_GATT_IF_NONE ||
                gattc_if == gl_profile_tab[idx].gattc_if) {
                if (gl_profile_tab[idx].gattc_cb) {
                    gl_profile_tab[idx].gattc_cb(event, gattc_if, param);
                }
            }
        }
    } while (0);
}

// GATT客户端配置文件事件处理函数
static void gattc_profile_event_handler(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param)
{
    esp_ble_gattc_cb_param_t *p_data = (esp_ble_gattc_cb_param_t *)param;

    switch (event) {
    case ESP_GATTC_REG_EVT:
        ESP_LOGI(TAG, "GATT client register, status %d, app_id %d, gattc_if %d",
                 param->reg.status, param->reg.app_id, gattc_if);

        ble_gattc.gattc_if = gattc_if;

        // 注册成功后，标记为已启动状态
        ble_gattc.started = true;
        ESP_LOGI(TAG, "BLE GATT client started successfully");

        // 触发启动事件
        ble_gattc_event_data_t start_event_data = {
            .event = BLE_GATTC_EVENT_READY
        };
        trigger_event_callback(BLE_GATTC_EVENT_READY, &start_event_data);
        break;

    case ESP_GATTC_CONNECT_EVT:
        ESP_LOGI(TAG, "=== CONNECTION ESTABLISHED ===");
        ESP_LOGI(TAG, "Connection ID: %d", p_data->connect.conn_id);
        ESP_LOGI(TAG, "GATT Interface: %d", gattc_if);
        ESP_LOGI(TAG, "Remote Device Address: "ESP_BD_ADDR_STR, ESP_BD_ADDR_HEX(p_data->connect.remote_bda));

        ble_gattc.conn_id = p_data->connect.conn_id;
        memcpy(ble_gattc.remote_bda, p_data->connect.remote_bda, sizeof(esp_bd_addr_t));
        ble_gattc.state = BLE_GATTC_STATE_CONNECTED;
        ble_gattc.connect_count++;

        ESP_LOGI(TAG, "Starting service discovery...");
        esp_ble_gattc_search_service(gattc_if, ble_gattc.conn_id, NULL);

        // 触发连接事件
        ble_gattc_event_data_t event_data = {
            .event = BLE_GATTC_EVENT_CONNECTED,
            .connected = {
                .device = ble_gattc.connected_device,
                .conn_id = ble_gattc.conn_id
            }
        };
        trigger_event_callback(BLE_GATTC_EVENT_CONNECTED, &event_data);
        break;

    case ESP_GATTC_OPEN_EVT:
        if (param->open.status != ESP_GATT_OK) {
            ESP_LOGE(TAG, "Open connection failed, status %d", p_data->open.status);
            ble_gattc.state = BLE_GATTC_STATE_ERROR;
        } else {
            ESP_LOGI(TAG, "Connection opened successfully");
        }
        break;

    case ESP_GATTC_DIS_SRVC_CMPL_EVT:
        if (param->dis_srvc_cmpl.status != ESP_GATT_OK) {
            ESP_LOGE(TAG, "Service discovery failed, status %d", param->dis_srvc_cmpl.status);
            ble_gattc.state = BLE_GATTC_STATE_ERROR;
        } else {
            ESP_LOGI(TAG, "Service discovery completed");
            ble_gattc.state = BLE_GATTC_STATE_SERVICE_DISCOVERED;

            // 触发服务发现完成事件
            ble_gattc_event_data_t event_data = {
                .event = BLE_GATTC_EVENT_SERVICE_DISCOVERED,
                .service_discovered = {
                    .service_count = 1  // 简化处理
                }
            };
            trigger_event_callback(BLE_GATTC_EVENT_SERVICE_DISCOVERED, &event_data);
        }
        break;

    case ESP_GATTC_SEARCH_RES_EVT:
        ESP_LOGI(TAG, "=== Service Found ===");
        ESP_LOGI(TAG, "Service UUID: 0x%04X", p_data->search_res.srvc_id.uuid.uuid.uuid16);
        ESP_LOGI(TAG, "Start Handle: %d", p_data->search_res.start_handle);
        ESP_LOGI(TAG, "End Handle: %d", p_data->search_res.end_handle);

        // 检查是否是目标服务
        if (p_data->search_res.srvc_id.uuid.len == ESP_UUID_LEN_16 &&
            p_data->search_res.srvc_id.uuid.uuid.uuid16 == ble_gattc.config.target_service_uuid) {

            ESP_LOGI(TAG, "Target service found: 0x%04X", ble_gattc.config.target_service_uuid);
            ble_gattc.service_start_handle = p_data->search_res.start_handle;
            ble_gattc.service_end_handle = p_data->search_res.end_handle;

            // 搜索目标特征
            esp_bt_uuid_t char_uuid;
            char_uuid.len = ESP_UUID_LEN_16;
            char_uuid.uuid.uuid16 = ble_gattc.config.target_char_uuid;

            esp_ble_gattc_get_char_by_uuid(gattc_if, p_data->search_res.conn_id,
                                          ble_gattc.service_start_handle,
                                          ble_gattc.service_end_handle,
                                          char_uuid, NULL, NULL);
        }
        break;

    case ESP_GATTC_SEARCH_CMPL_EVT:
        ESP_LOGI(TAG, "=== Service Discovery Complete ===");
        if (param->search_cmpl.status != ESP_GATT_OK) {
            ESP_LOGE(TAG, "Service search failed, status %d", param->search_cmpl.status);
            ble_gattc.state = BLE_GATTC_STATE_ERROR;
        } else {
            ESP_LOGI(TAG, "Service search completed successfully");

            // 直接设置目标特征句柄（简化处理）
            ble_gattc.target_char_handle = 0x0013;  // 根据原始代码设置
            ESP_LOGI(TAG, "Target characteristic handle set to: 0x%04X", ble_gattc.target_char_handle);

            ble_gattc.state = BLE_GATTC_STATE_READY;
            ESP_LOGI(TAG, "BLE GATT client ready for communication");

            // 触发就绪事件
            ble_gattc_event_data_t event_data = {
                .event = BLE_GATTC_EVENT_READY
            };
            trigger_event_callback(BLE_GATTC_EVENT_READY, &event_data);
        }
        break;

    // 注意：ESP_GATTC_GET_CHAR_EVT 在某些ESP-IDF版本中不存在
    // 我们将在服务发现完成后直接设置特征句柄

    case ESP_GATTC_WRITE_CHAR_EVT:
        if (param->write.status != ESP_GATT_OK) {
            ESP_LOGE(TAG, "Write characteristic failed, status %d", param->write.status);
        } else {
            ESP_LOGI(TAG, "Write characteristic success, handle: %d", param->write.handle);
        }

        // 触发数据发送事件
        {
            ble_gattc_event_data_t write_event_data = {
                .event = BLE_GATTC_EVENT_DATA_SENT,
                .data_sent = {
                    .char_handle = param->write.handle,
                    .data = NULL,  // 数据已经发送，不再保存
                    .data_len = 0,
                    .success = (param->write.status == ESP_GATT_OK)
                }
            };
            trigger_event_callback(BLE_GATTC_EVENT_DATA_SENT, &write_event_data);
        }
        break;

    case ESP_GATTC_DISCONNECT_EVT:
        ESP_LOGI(TAG, "=== CONNECTION DISCONNECTED ===");
        ESP_LOGI(TAG, "Disconnect reason: %d", param->disconnect.reason);

        ble_gattc.state = BLE_GATTC_STATE_DISCONNECTED;
        ble_gattc.conn_id = 0;
        ble_gattc.target_char_handle = 0;

        // 触发断开连接事件
        {
            ble_gattc_event_data_t disconnect_event_data = {
                .event = BLE_GATTC_EVENT_DISCONNECTED,
                .disconnected = {
                    .conn_id = param->disconnect.conn_id,
                    .reason = param->disconnect.reason
                }
            };
            trigger_event_callback(BLE_GATTC_EVENT_DISCONNECTED, &disconnect_event_data);
        }

        // 如果启用自动重连，重新开始扫描
        if (ble_gattc.config.auto_reconnect) {
            ESP_LOGI(TAG, "Auto reconnect enabled, starting scan...");
            vTaskDelay(pdMS_TO_TICKS(1000));  // 等待1秒后重新扫描
            ble_gattc_start_scan();
        }
        break;

    default:
        break;
    }
}

// BLE蓝牙初始化函数（参考原始代码）
static esp_err_t ble_gattc_init_bluetooth(void)
{
    esp_err_t ret;

    // 释放经典蓝牙内存
    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));

    // 初始化BT控制器
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ret = esp_bt_controller_init(&bt_cfg);
    if (ret) {
        ESP_LOGE(TAG, "Initialize controller failed: %s", esp_err_to_name(ret));
        return ret;
    }

    // 启用BLE模式
    ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if (ret) {
        ESP_LOGE(TAG, "Enable controller failed: %s", esp_err_to_name(ret));
        return ret;
    }

    // 初始化Bluedroid
    ret = esp_bluedroid_init();
    if (ret) {
        ESP_LOGE(TAG, "Init bluetooth failed: %s", esp_err_to_name(ret));
        return ret;
    }

    // 启用Bluedroid
    ret = esp_bluedroid_enable();
    if (ret) {
        ESP_LOGE(TAG, "Enable bluetooth failed: %s", esp_err_to_name(ret));
        return ret;
    }

    // 注册GAP回调函数
    ret = esp_ble_gap_register_callback(esp_gap_cb);
    if (ret) {
        ESP_LOGE(TAG, "GAP register failed, error code = %x", ret);
        return ret;
    }

    // 注册GATTC回调函数
    ret = esp_ble_gattc_register_callback(esp_gattc_cb);
    if (ret) {
        ESP_LOGE(TAG, "GATTC register failed, error code = %x", ret);
        return ret;
    }

    // 注册GATTC应用
    ret = esp_ble_gattc_app_register(PROFILE_A_APP_ID);
    if (ret) {
        ESP_LOGE(TAG, "GATTC app register failed, error code = %x", ret);
        return ret;
    }

    // 设置本地MTU
    esp_err_t local_mtu_ret = esp_ble_gatt_set_local_mtu(500);
    if (local_mtu_ret) {
        ESP_LOGE(TAG, "Set local MTU failed, error code = %x", local_mtu_ret);
    }

    ESP_LOGI(TAG, "BLE bluetooth initialization completed");
    return ESP_OK;
}
