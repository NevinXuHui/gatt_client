#ifndef BLE_GATTC_H
#define BLE_GATTC_H

#include <stdbool.h>
#include "esp_err.h"
#include "esp_bt.h"
#include "esp_gap_ble_api.h"
#include "esp_gattc_api.h"
#include "esp_gatt_defs.h"

// BLE GATT客户端状态枚举
typedef enum {
    BLE_GATTC_STATE_IDLE = 0,
    BLE_GATTC_STATE_SCANNING,
    BLE_GATTC_STATE_CONNECTING,
    BLE_GATTC_STATE_CONNECTED,
    BLE_GATTC_STATE_SERVICE_DISCOVERED,
    BLE_GATTC_STATE_READY,
    BLE_GATTC_STATE_DISCONNECTED,
    BLE_GATTC_STATE_ERROR
} ble_gattc_state_t;

// BLE GATT客户端事件枚举
typedef enum {
    BLE_GATTC_EVENT_SCAN_START = 0,
    BLE_GATTC_EVENT_DEVICE_FOUND,
    BLE_GATTC_EVENT_CONNECTED,
    BLE_GATTC_EVENT_DISCONNECTED,
    BLE_GATTC_EVENT_SERVICE_DISCOVERED,
    BLE_GATTC_EVENT_READY,
    BLE_GATTC_EVENT_DATA_SENT,
    BLE_GATTC_EVENT_DATA_RECEIVED,
    BLE_GATTC_EVENT_ERROR
} ble_gattc_event_t;

// BLE设备信息结构
typedef struct {
    esp_bd_addr_t bda;
    char name[ESP_BLE_ADV_NAME_LEN_MAX];
    int8_t rssi;
    uint8_t addr_type;
} ble_device_info_t;

// BLE GATT客户端事件数据
typedef struct {
    ble_gattc_event_t event;
    union {
        struct {
            ble_device_info_t device;
        } device_found;
        
        struct {
            ble_device_info_t device;
            uint16_t conn_id;
        } connected;
        
        struct {
            uint16_t conn_id;
            esp_gatt_conn_reason_t reason;
        } disconnected;
        
        struct {
            uint16_t service_count;
        } service_discovered;
        
        struct {
            uint16_t char_handle;
            uint8_t* data;
            uint16_t data_len;
            bool success;
        } data_sent;
        
        struct {
            uint16_t char_handle;
            uint8_t* data;
            uint16_t data_len;
        } data_received;
        
        struct {
            esp_err_t error_code;
            char description[64];
        } error;
    };
} ble_gattc_event_data_t;

// BLE GATT客户端事件回调函数类型
typedef void (*ble_gattc_event_callback_t)(const ble_gattc_event_data_t* event_data, void* user_data);

// BLE GATT客户端配置结构
typedef struct {
    char target_device_name[ESP_BLE_ADV_NAME_LEN_MAX];  // 目标设备名称
    uint16_t target_service_uuid;                       // 目标服务UUID
    uint16_t target_char_uuid;                          // 目标特征UUID
    uint32_t scan_duration;                             // 扫描持续时间（秒）
    bool auto_reconnect;                                // 是否自动重连
    ble_gattc_event_callback_t event_callback;          // 事件回调函数
    void* user_data;                                    // 用户数据
} ble_gattc_config_t;

/**
 * @brief 初始化BLE GATT客户端
 * 
 * @param config 配置参数
 * @return esp_err_t 
 */
esp_err_t ble_gattc_init(const ble_gattc_config_t* config);

/**
 * @brief 启动BLE GATT客户端
 * 
 * @return esp_err_t 
 */
esp_err_t ble_gattc_start(void);

/**
 * @brief 停止BLE GATT客户端
 * 
 * @return esp_err_t 
 */
esp_err_t ble_gattc_stop(void);

/**
 * @brief 开始扫描BLE设备
 * 
 * @return esp_err_t 
 */
esp_err_t ble_gattc_start_scan(void);

/**
 * @brief 停止扫描BLE设备
 * 
 * @return esp_err_t 
 */
esp_err_t ble_gattc_stop_scan(void);

/**
 * @brief 连接到指定设备
 * 
 * @param device_info 设备信息
 * @return esp_err_t 
 */
esp_err_t ble_gattc_connect_device(const ble_device_info_t* device_info);

/**
 * @brief 断开当前连接
 * 
 * @return esp_err_t 
 */
esp_err_t ble_gattc_disconnect(void);

/**
 * @brief 发送数据到目标特征
 * 
 * @param data 要发送的数据
 * @param data_len 数据长度
 * @return esp_err_t 
 */
esp_err_t ble_gattc_send_data(const uint8_t* data, uint16_t data_len);

/**
 * @brief 发送预定义的按键数据
 * 
 * @return esp_err_t 
 */
esp_err_t ble_gattc_send_button_data(void);

/**
 * @brief 获取当前状态
 * 
 * @return ble_gattc_state_t 
 */
ble_gattc_state_t ble_gattc_get_state(void);

/**
 * @brief 检查是否已连接
 * 
 * @return true 已连接
 * @return false 未连接
 */
bool ble_gattc_is_connected(void);

/**
 * @brief 获取连接的设备信息
 * 
 * @param device_info 输出设备信息
 * @return esp_err_t 
 */
esp_err_t ble_gattc_get_connected_device(ble_device_info_t* device_info);

/**
 * @brief 反初始化BLE GATT客户端
 * 
 * @return esp_err_t 
 */
esp_err_t ble_gattc_deinit(void);

#endif // BLE_GATTC_H
