/*
 * SPDX-FileCopyrightText: 2021-2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */



/****************************************************************************
*
* This demo showcases BLE GATT client. It can scan BLE devices and connect to one device.
* Run the gatt_server demo, the client demo will automatically connect to the gatt_server demo.
* Client demo will enable gatt_server's notify after connection. The two devices will then exchange
* data.
*
****************************************************************************/

#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include <stdio.h>
#include "nvs.h"
#include "nvs_flash.h"

#include "esp_bt.h"
#include "esp_gap_ble_api.h"
#include "esp_gattc_api.h"
#include "esp_gatt_defs.h"
#include "esp_bt_main.h"
#include "esp_gatt_common_api.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "soc/gpio_reg.h"
#include "soc/soc.h"
#include "esp_system.h"
#include "esp_intr_alloc.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "driver/gpio.h"

#define GATTC_TAG "GATTC_DEMO"
#define REMOTE_SERVICE_UUID        0x00FF
#define REMOTE_NOTIFY_CHAR_UUID    0xFF01
#define PROFILE_NUM      1
#define PROFILE_A_APP_ID 0
#define INVALID_HANDLE   0

// 自定义特征UUID
#define CUSTOM_CHAR_UUID_0013      0x0013

// GPIO按键配置
#define GPIO_BUTTON_PIN            18
#define GPIO_BUTTON_LEVEL          0
#define ESP_INTR_FLAG_DEFAULT      0

// GPIO中断相关寄存器
#include "soc/gpio_struct.h"
#include "hal/gpio_ll.h"
#include "soc/io_mux_reg.h"
#include "esp_intr_alloc.h"
#if CONFIG_EXAMPLE_INIT_DEINIT_LOOP
#define EXAMPLE_TEST_COUNT 50
#endif

static char remote_device_name[ESP_BLE_ADV_NAME_LEN_MAX] = "CMB2320647-1992";
static bool connect    = false;
static bool get_server = false;
static esp_gattc_char_elem_t *char_elem_result   = NULL;
static esp_gattc_descr_elem_t *descr_elem_result = NULL;

// 特征句柄相关变量
static uint16_t char_0013_handle = 0;

// GPIO中断相关变量
static QueueHandle_t gpio_evt_queue = NULL;
static intr_handle_t gpio_intr_handle = NULL;

/* Declare static functions */
static void esp_gap_cb(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param);
static void esp_gattc_cb(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param);
static void gattc_profile_event_handler(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param);
static uint8_t* get_device_name_from_adv_data(uint8_t *adv_data, uint16_t adv_data_len, uint16_t scan_rsp_len, uint8_t *name_len);
static void print_uuid(esp_bt_uuid_t *uuid);
static void print_char_properties(uint8_t properties);
static void discover_all_services(esp_gatt_if_t gattc_if, uint16_t conn_id);
static void discover_all_chars_in_service(esp_gatt_if_t gattc_if, uint16_t conn_id, uint16_t start_handle, uint16_t end_handle);
static void enumerate_all_chars_in_service(esp_gatt_if_t gattc_if, uint16_t conn_id, uint16_t start_handle, uint16_t end_handle);
static void discover_all_characteristics_after_service_discovery(esp_gatt_if_t gattc_if, uint16_t conn_id);
static void uuid_string_to_bytes(const char* uuid_str, uint8_t* uuid_bytes);
static void send_data_to_char_0013(void);
static void gpio_isr_handler(void* arg);
static void gpio_button_task(void* arg);
static void init_gpio_button(void);


static esp_bt_uuid_t remote_filter_service_uuid = {
    .len = ESP_UUID_LEN_16,
    .uuid = {.uuid16 = REMOTE_SERVICE_UUID,},
};

static esp_bt_uuid_t remote_filter_char_uuid = {
    .len = ESP_UUID_LEN_16,
    .uuid = {.uuid16 = REMOTE_NOTIFY_CHAR_UUID,},
};

static esp_bt_uuid_t notify_descr_uuid = {
    .len = ESP_UUID_LEN_16,
    .uuid = {.uuid16 = ESP_GATT_UUID_CHAR_CLIENT_CONFIG,},
};

static esp_ble_scan_params_t ble_scan_params = {
    .scan_type              = BLE_SCAN_TYPE_ACTIVE,
    .own_addr_type          = BLE_ADDR_TYPE_PUBLIC,
    .scan_filter_policy     = BLE_SCAN_FILTER_ALLOW_ALL,
    .scan_interval          = 0x50,
    .scan_window            = 0x30,
    .scan_duplicate         = BLE_SCAN_DUPLICATE_DISABLE
};

struct gattc_profile_inst {
    esp_gattc_cb_t gattc_cb;
    uint16_t gattc_if;
    uint16_t app_id;
    uint16_t conn_id;
    uint16_t service_start_handle;
    uint16_t service_end_handle;
    uint16_t char_handle;
    esp_bd_addr_t remote_bda;
};

/* One gatt-based profile one app_id and one gattc_if, this array will store the gattc_if returned by ESP_GATTS_REG_EVT */
static struct gattc_profile_inst gl_profile_tab[PROFILE_NUM] = {
    [PROFILE_A_APP_ID] = {
        .gattc_cb = gattc_profile_event_handler,
        .gattc_if = ESP_GATT_IF_NONE,       /* Not get the gatt_if, so initial is ESP_GATT_IF_NONE */
    },
};

static void gattc_profile_event_handler(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param)
{
    esp_ble_gattc_cb_param_t *p_data = (esp_ble_gattc_cb_param_t *)param;

    switch (event) {
    case ESP_GATTC_REG_EVT:
        ESP_LOGI(GATTC_TAG, "GATT client register, status %d, app_id %d, gattc_if %d", param->reg.status, param->reg.app_id, gattc_if);
        esp_err_t scan_ret = esp_ble_gap_set_scan_params(&ble_scan_params);
        if (scan_ret){
            ESP_LOGE(GATTC_TAG, "set scan params error, error code = %x", scan_ret);
        }
        break;
    case ESP_GATTC_CONNECT_EVT:{
        ESP_LOGI(GATTC_TAG, "=== CONNECTION ESTABLISHED ===");
        ESP_LOGI(GATTC_TAG, "Connection ID: %d", p_data->connect.conn_id);
        ESP_LOGI(GATTC_TAG, "GATT Interface: %d", gattc_if);
        ESP_LOGI(GATTC_TAG, "Connection established successfully");
        ESP_LOGI(GATTC_TAG, "Remote Device Address: "ESP_BD_ADDR_STR, ESP_BD_ADDR_HEX(p_data->connect.remote_bda));
        
        gl_profile_tab[PROFILE_A_APP_ID].conn_id = p_data->connect.conn_id;
        memcpy(gl_profile_tab[PROFILE_A_APP_ID].remote_bda, p_data->connect.remote_bda, sizeof(esp_bd_addr_t));
        
        ESP_LOGI(GATTC_TAG, "Requesting MTU exchange...");
        esp_err_t mtu_ret = esp_ble_gattc_send_mtu_req (gattc_if, p_data->connect.conn_id);
        if (mtu_ret){
            ESP_LOGE(GATTC_TAG, "Config MTU error, error code = %x", mtu_ret);
        }
        ESP_LOGI(GATTC_TAG, "==============================");
        break;
    }
    case ESP_GATTC_OPEN_EVT:
        if (param->open.status != ESP_GATT_OK){
            ESP_LOGE(GATTC_TAG, "Open failed, status %d", p_data->open.status);
            break;
        }
        ESP_LOGI(GATTC_TAG, "Open successfully, MTU %u", p_data->open.mtu);
        break;
    case ESP_GATTC_DIS_SRVC_CMPL_EVT:
        if (param->dis_srvc_cmpl.status != ESP_GATT_OK){
            ESP_LOGE(GATTC_TAG, "Service discover failed, status %d", param->dis_srvc_cmpl.status);
            break;
        }
        ESP_LOGI(GATTC_TAG, "Service discover complete, conn_id %d", param->dis_srvc_cmpl.conn_id);
        // 发现所有服务而不仅仅是特定的服务
        ESP_LOGI(GATTC_TAG, "Starting discovery of ALL services...");
        discover_all_services(gattc_if, param->dis_srvc_cmpl.conn_id);
        break;
    case ESP_GATTC_CFG_MTU_EVT:
        ESP_LOGI(GATTC_TAG, "MTU exchange, status %d, MTU %d", param->cfg_mtu.status, param->cfg_mtu.mtu);
        break;
    case ESP_GATTC_SEARCH_RES_EVT: {
        ESP_LOGI(GATTC_TAG, "=== Service Found ===");
        ESP_LOGI(GATTC_TAG, "Connection ID: %d", p_data->search_res.conn_id);
        ESP_LOGI(GATTC_TAG, "Primary Service: %s", p_data->search_res.is_primary ? "Yes" : "No");
        ESP_LOGI(GATTC_TAG, "Start Handle: %d", p_data->search_res.start_handle);
        ESP_LOGI(GATTC_TAG, "End Handle: %d", p_data->search_res.end_handle);
        ESP_LOGI(GATTC_TAG, "Instance ID: %d", p_data->search_res.srvc_id.inst_id);
        
        // 打印服务UUID
        ESP_LOGI(GATTC_TAG, "Service UUID:");
        print_uuid(&p_data->search_res.srvc_id.uuid);
        
        // 发现并打印此服务中的所有特征
        discover_all_chars_in_service(gattc_if, p_data->search_res.conn_id, 
                                    p_data->search_res.start_handle, 
                                    p_data->search_res.end_handle);
        
        // 同时尝试另一种枚举方法
        enumerate_all_chars_in_service(gattc_if, p_data->search_res.conn_id,
                                     p_data->search_res.start_handle,
                                     p_data->search_res.end_handle);
        
        // 检查是否是我们感兴趣的特定服务
        if (p_data->search_res.srvc_id.uuid.len == ESP_UUID_LEN_16 && 
            p_data->search_res.srvc_id.uuid.uuid.uuid16 == REMOTE_SERVICE_UUID) {
            ESP_LOGI(GATTC_TAG, "*** TARGET SERVICE FOUND ***");
            get_server = true;
            gl_profile_tab[PROFILE_A_APP_ID].service_start_handle = p_data->search_res.start_handle;
            gl_profile_tab[PROFILE_A_APP_ID].service_end_handle = p_data->search_res.end_handle;
        }
        
        ESP_LOGI(GATTC_TAG, "=====================");
        break;
    }
    case ESP_GATTC_SEARCH_CMPL_EVT:
        if (p_data->search_cmpl.status != ESP_GATT_OK){
            ESP_LOGE(GATTC_TAG, "Service search failed, status %x", p_data->search_cmpl.status);
            break;
        }
        if(p_data->search_cmpl.searched_service_source == ESP_GATT_SERVICE_FROM_REMOTE_DEVICE) {
            ESP_LOGI(GATTC_TAG, "Get service information from remote device");
        } else if (p_data->search_cmpl.searched_service_source == ESP_GATT_SERVICE_FROM_NVS_FLASH) {
            ESP_LOGI(GATTC_TAG, "Get service information from flash");
        } else {
            ESP_LOGI(GATTC_TAG, "Unknown service source");
        }
        ESP_LOGI(GATTC_TAG, "=== Service Discovery Complete ===");
        
        // 发现所有服务的特征
        ESP_LOGI(GATTC_TAG, "Starting comprehensive characteristic discovery...");
        discover_all_characteristics_after_service_discovery(gattc_if, p_data->search_cmpl.conn_id);
        
        if (get_server){
            uint16_t count = 0;
            esp_gatt_status_t status = esp_ble_gattc_get_attr_count( gattc_if,
                                                                     p_data->search_cmpl.conn_id,
                                                                     ESP_GATT_DB_CHARACTERISTIC,
                                                                     gl_profile_tab[PROFILE_A_APP_ID].service_start_handle,
                                                                     gl_profile_tab[PROFILE_A_APP_ID].service_end_handle,
                                                                     INVALID_HANDLE,
                                                                     &count);
            if (status != ESP_GATT_OK){
                ESP_LOGE(GATTC_TAG, "esp_ble_gattc_get_attr_count error");
                break;
            }

            if (count > 0){
                char_elem_result = (esp_gattc_char_elem_t *)malloc(sizeof(esp_gattc_char_elem_t) * count);
                if (!char_elem_result){
                    ESP_LOGE(GATTC_TAG, "gattc no mem");
                    break;
                }else{
                    status = esp_ble_gattc_get_char_by_uuid( gattc_if,
                                                             p_data->search_cmpl.conn_id,
                                                             gl_profile_tab[PROFILE_A_APP_ID].service_start_handle,
                                                             gl_profile_tab[PROFILE_A_APP_ID].service_end_handle,
                                                             remote_filter_char_uuid,
                                                             char_elem_result,
                                                             &count);
                    if (status != ESP_GATT_OK){
                        ESP_LOGE(GATTC_TAG, "esp_ble_gattc_get_char_by_uuid error");
                        free(char_elem_result);
                        char_elem_result = NULL;
                        break;
                    }

                    ESP_LOGI(GATTC_TAG, "=== Target Service Characteristics ===");
                    for (int i = 0; i < count; i++) {
                        ESP_LOGI(GATTC_TAG, "Characteristic %d:", i + 1);
                        ESP_LOGI(GATTC_TAG, "  Handle: %d", char_elem_result[i].char_handle);
                        print_uuid(&char_elem_result[i].uuid);
                        print_char_properties(char_elem_result[i].properties);
                        
                        // 检查是否是通知特征
                        if (char_elem_result[i].uuid.len == ESP_UUID_LEN_16 && 
                            char_elem_result[i].uuid.uuid.uuid16 == REMOTE_NOTIFY_CHAR_UUID) {
                            ESP_LOGI(GATTC_TAG, "  *** TARGET NOTIFICATION CHARACTERISTIC FOUND ***");
                            gl_profile_tab[PROFILE_A_APP_ID].char_handle = char_elem_result[i].char_handle;
                            if (char_elem_result[i].properties & ESP_GATT_CHAR_PROP_BIT_NOTIFY) {
                                ESP_LOGI(GATTC_TAG, "  Registering for notifications...");
                                esp_ble_gattc_register_for_notify(gattc_if, gl_profile_tab[PROFILE_A_APP_ID].remote_bda, 
                                                                 char_elem_result[i].char_handle);
                            }
                        }
                        ESP_LOGI(GATTC_TAG, "");
                    }
                }
                /* free char_elem_result */
                free(char_elem_result);
            }else{
                ESP_LOGE(GATTC_TAG, "no char found");
            }
        }
         break;
    case ESP_GATTC_REG_FOR_NOTIFY_EVT: {
        if (p_data->reg_for_notify.status != ESP_GATT_OK){
            ESP_LOGE(GATTC_TAG, "Notification register failed, status %d", p_data->reg_for_notify.status);
        }else{
            ESP_LOGI(GATTC_TAG, "Notification register successfully");
            uint16_t count = 0;
            uint16_t notify_en = 1;
            esp_gatt_status_t ret_status = esp_ble_gattc_get_attr_count( gattc_if,
                                                                         gl_profile_tab[PROFILE_A_APP_ID].conn_id,
                                                                         ESP_GATT_DB_DESCRIPTOR,
                                                                         gl_profile_tab[PROFILE_A_APP_ID].service_start_handle,
                                                                         gl_profile_tab[PROFILE_A_APP_ID].service_end_handle,
                                                                         gl_profile_tab[PROFILE_A_APP_ID].char_handle,
                                                                         &count);
            if (ret_status != ESP_GATT_OK){
                ESP_LOGE(GATTC_TAG, "esp_ble_gattc_get_attr_count error");
                break;
            }
            if (count > 0){
                descr_elem_result = malloc(sizeof(esp_gattc_descr_elem_t) * count);
                if (!descr_elem_result){
                    ESP_LOGE(GATTC_TAG, "malloc error, gattc no mem");
                    break;
                }else{
                    ret_status = esp_ble_gattc_get_descr_by_char_handle( gattc_if,
                                                                         gl_profile_tab[PROFILE_A_APP_ID].conn_id,
                                                                         p_data->reg_for_notify.handle,
                                                                         notify_descr_uuid,
                                                                         descr_elem_result,
                                                                         &count);
                    if (ret_status != ESP_GATT_OK){
                        ESP_LOGE(GATTC_TAG, "esp_ble_gattc_get_descr_by_char_handle error");
                        free(descr_elem_result);
                        descr_elem_result = NULL;
                        break;
                    }
                    /* Every char has only one descriptor in our 'ESP_GATTS_DEMO' demo, so we used first 'descr_elem_result' */
                    if (count > 0 && descr_elem_result[0].uuid.len == ESP_UUID_LEN_16 && descr_elem_result[0].uuid.uuid.uuid16 == ESP_GATT_UUID_CHAR_CLIENT_CONFIG){
                        ret_status = esp_ble_gattc_write_char_descr( gattc_if,
                                                                     gl_profile_tab[PROFILE_A_APP_ID].conn_id,
                                                                     descr_elem_result[0].handle,
                                                                     sizeof(notify_en),
                                                                     (uint8_t *)&notify_en,
                                                                     ESP_GATT_WRITE_TYPE_RSP,
                                                                     ESP_GATT_AUTH_REQ_NONE);
                    }

                    if (ret_status != ESP_GATT_OK){
                        ESP_LOGE(GATTC_TAG, "esp_ble_gattc_write_char_descr error");
                    }

                    /* free descr_elem_result */
                    free(descr_elem_result);
                }
            }
            else{
                ESP_LOGE(GATTC_TAG, "decsr not found");
            }

        }
        break;
    }
    case ESP_GATTC_NOTIFY_EVT:
        if (p_data->notify.is_notify){
            ESP_LOGI(GATTC_TAG, "Notification received");
        }else{
            ESP_LOGI(GATTC_TAG, "Indication received");
        }
        ESP_LOG_BUFFER_HEX(GATTC_TAG, p_data->notify.value, p_data->notify.value_len);
        break;
    case ESP_GATTC_WRITE_DESCR_EVT:
        if (p_data->write.status != ESP_GATT_OK){
            ESP_LOGE(GATTC_TAG, "Descriptor write failed, status %x", p_data->write.status);
            break;
        }
        ESP_LOGI(GATTC_TAG, "Descriptor write successfully");
        uint8_t write_char_data[35];
        for (int i = 0; i < sizeof(write_char_data); ++i)
        {
            write_char_data[i] = i % 256;
        }
        esp_ble_gattc_write_char( gattc_if,
                                  gl_profile_tab[PROFILE_A_APP_ID].conn_id,
                                  gl_profile_tab[PROFILE_A_APP_ID].char_handle,
                                  sizeof(write_char_data),
                                  write_char_data,
                                  ESP_GATT_WRITE_TYPE_RSP,
                                  ESP_GATT_AUTH_REQ_NONE);
        break;
    case ESP_GATTC_SRVC_CHG_EVT: {
        esp_bd_addr_t bda;
        memcpy(bda, p_data->srvc_chg.remote_bda, sizeof(esp_bd_addr_t));
        ESP_LOGI(GATTC_TAG, "Service change from "ESP_BD_ADDR_STR"", ESP_BD_ADDR_HEX(bda));
        break;
    }
    case ESP_GATTC_WRITE_CHAR_EVT:
        if (p_data->write.status != ESP_GATT_OK){
            ESP_LOGE(GATTC_TAG, "Characteristic write failed, status %x)", p_data->write.status);
            break;
        }
        ESP_LOGI(GATTC_TAG, "Characteristic write successfully");
        break;
    case ESP_GATTC_DISCONNECT_EVT:
        connect = false;
        get_server = false;
        ESP_LOGI(GATTC_TAG, "Disconnected, remote "ESP_BD_ADDR_STR", reason 0x%02x",
                 ESP_BD_ADDR_HEX(p_data->disconnect.remote_bda), p_data->disconnect.reason);
        break;
    default:
        break;
    }
}

static void esp_gap_cb(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    uint8_t *adv_name = NULL;
    uint8_t adv_name_len = 0;
    switch (event) {
    case ESP_GAP_BLE_SCAN_PARAM_SET_COMPLETE_EVT: {
        // The unit of duration is seconds.
        // If duration is set to 0, scanning will continue indefinitely
        // until esp_ble_gap_stop_scanning is explicitly called.
        uint32_t duration = 5;
        esp_ble_gap_start_scanning(duration);
        break;
    }
    case ESP_GAP_BLE_SCAN_START_COMPLETE_EVT:
        //scan start complete event to indicate scan start successfully or failed
        if (param->scan_start_cmpl.status != ESP_BT_STATUS_SUCCESS) {
            ESP_LOGE(GATTC_TAG, "Scanning start failed, status %x", param->scan_start_cmpl.status);
            break;
        }
        ESP_LOGI(GATTC_TAG, "Scanning start successfully");

        break;
    case ESP_GAP_BLE_SCAN_RESULT_EVT: {
        esp_ble_gap_cb_param_t *scan_result = (esp_ble_gap_cb_param_t *)param;
        switch (scan_result->scan_rst.search_evt) {
        case ESP_GAP_SEARCH_INQ_RES_EVT:
            // 使用我们的辅助函数来获取设备名称
            adv_name = get_device_name_from_adv_data(scan_result->scan_rst.ble_adv,
                                                   scan_result->scan_rst.adv_data_len,
                                                   scan_result->scan_rst.scan_rsp_len,
                                                   &adv_name_len);
            
            ESP_LOGI(GATTC_TAG, "Scan result, device "ESP_BD_ADDR_STR", RSSI %d, name len %u", 
                     ESP_BD_ADDR_HEX(scan_result->scan_rst.bda), 
                     scan_result->scan_rst.rssi, 
                     adv_name_len);
            
            if (adv_name != NULL && adv_name_len > 0) {
                ESP_LOG_BUFFER_CHAR(GATTC_TAG, adv_name, adv_name_len);
            } else {
                ESP_LOGI(GATTC_TAG, "Device name not found");
            }

#if CONFIG_EXAMPLE_DUMP_ADV_DATA_AND_SCAN_RESP
            if (scan_result->scan_rst.adv_data_len > 0) {
                ESP_LOGI(GATTC_TAG, "adv data:");
                ESP_LOG_BUFFER_HEX(GATTC_TAG, &scan_result->scan_rst.ble_adv[0], scan_result->scan_rst.adv_data_len);
            }
            if (scan_result->scan_rst.scan_rsp_len > 0) {
                ESP_LOGI(GATTC_TAG, "scan resp:");
                ESP_LOG_BUFFER_HEX(GATTC_TAG, &scan_result->scan_rst.ble_adv[scan_result->scan_rst.adv_data_len], scan_result->scan_rst.scan_rsp_len);
            }
#endif

            if (adv_name != NULL) {
                if (strlen(remote_device_name) == adv_name_len && strncmp((char *)adv_name, remote_device_name, adv_name_len) == 0) {
                    // Note: If there are multiple devices with the same device name, the device may connect to an unintended one.
                    // It is recommended to change the default device name to ensure it is unique.
                    ESP_LOGI(GATTC_TAG, "Device found %s", remote_device_name);
                    if (connect == false) {
                        connect = true;
                        ESP_LOGI(GATTC_TAG, "Connect to the remote device");
                        esp_ble_gap_stop_scanning();
                        esp_ble_gatt_creat_conn_params_t creat_conn_params = {0};
                        memcpy(&creat_conn_params.remote_bda, scan_result->scan_rst.bda, ESP_BD_ADDR_LEN);
                        creat_conn_params.remote_addr_type = scan_result->scan_rst.ble_addr_type;
                        creat_conn_params.own_addr_type = BLE_ADDR_TYPE_PUBLIC;
                        creat_conn_params.is_direct = true;
                        creat_conn_params.is_aux = false;
                        creat_conn_params.phy_mask = 0x0;
                        esp_ble_gattc_enh_open(gl_profile_tab[PROFILE_A_APP_ID].gattc_if,
                                            &creat_conn_params);
                    }
                }
            }
            break;
        case ESP_GAP_SEARCH_INQ_CMPL_EVT:
            break;
        default:
            break;
        }
        break;
    }

    case ESP_GAP_BLE_SCAN_STOP_COMPLETE_EVT:
        if (param->scan_stop_cmpl.status != ESP_BT_STATUS_SUCCESS){
            ESP_LOGE(GATTC_TAG, "Scanning stop failed, status %x", param->scan_stop_cmpl.status);
            break;
        }
        ESP_LOGI(GATTC_TAG, "Scanning stop successfully");
        break;

    case ESP_GAP_BLE_ADV_STOP_COMPLETE_EVT:
        if (param->adv_stop_cmpl.status != ESP_BT_STATUS_SUCCESS){
            ESP_LOGE(GATTC_TAG, "Advertising stop failed, status %x", param->adv_stop_cmpl.status);
            break;
        }
        ESP_LOGI(GATTC_TAG, "Advertising stop successfully");
        break;
    case ESP_GAP_BLE_UPDATE_CONN_PARAMS_EVT:
         ESP_LOGI(GATTC_TAG, "Connection params update, status %d, conn_int %d, latency %d, timeout %d",
                  param->update_conn_params.status,
                  param->update_conn_params.conn_int,
                  param->update_conn_params.latency,
                  param->update_conn_params.timeout);
        break;
    case ESP_GAP_BLE_SET_PKT_LENGTH_COMPLETE_EVT:
        ESP_LOGI(GATTC_TAG, "Packet length update, status %d, rx %d, tx %d",
                  param->pkt_data_length_cmpl.status,
                  param->pkt_data_length_cmpl.params.rx_len,
                  param->pkt_data_length_cmpl.params.tx_len);
        break;
    default:
        break;
    }
}

static void esp_gattc_cb(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param)
{
    /* If event is register event, store the gattc_if for each profile */
    if (event == ESP_GATTC_REG_EVT) {
        if (param->reg.status == ESP_GATT_OK) {
            gl_profile_tab[param->reg.app_id].gattc_if = gattc_if;
        } else {
            ESP_LOGI(GATTC_TAG, "reg app failed, app_id %04x, status %d",
                    param->reg.app_id,
                    param->reg.status);
            return;
        }
    }

    /* If the gattc_if equal to profile A, call profile A cb handler,
     * so here call each profile's callback */
    do {
        int idx;
        for (idx = 0; idx < PROFILE_NUM; idx++) {
            if (gattc_if == ESP_GATT_IF_NONE || /* ESP_GATT_IF_NONE, not specify a certain gatt_if, need to call every profile cb function */
                    gattc_if == gl_profile_tab[idx].gattc_if) {
                if (gl_profile_tab[idx].gattc_cb) {
                    gl_profile_tab[idx].gattc_cb(event, gattc_if, param);
                }
            }
        }
    } while (0);
}

void app_main(void)
{
    // Initialize NVS.
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK( ret );

    #if CONFIG_EXAMPLE_CI_PIPELINE_ID
    memcpy(remote_device_name, esp_bluedroid_get_example_name(), sizeof(remote_device_name));
    #endif

    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ret = esp_bt_controller_init(&bt_cfg);
    if (ret) {
        ESP_LOGE(GATTC_TAG, "%s initialize controller failed: %s", __func__, esp_err_to_name(ret));
        return;
    }

    ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if (ret) {
        ESP_LOGE(GATTC_TAG, "%s enable controller failed: %s", __func__, esp_err_to_name(ret));
        return;
    }

    ret = esp_bluedroid_init();
    if (ret) {
        ESP_LOGE(GATTC_TAG, "%s init bluetooth failed: %s", __func__, esp_err_to_name(ret));
        return;
    }

    ret = esp_bluedroid_enable();
    if (ret) {
        ESP_LOGE(GATTC_TAG, "%s enable bluetooth failed: %s", __func__, esp_err_to_name(ret));
        return;
    }

#if CONFIG_EXAMPLE_INIT_DEINIT_LOOP
    for(int i = 0; i < EXAMPLE_TEST_COUNT; i++) {
        ESP_ERROR_CHECK( esp_bluedroid_disable() );
        ESP_ERROR_CHECK( esp_bluedroid_deinit() );
        vTaskDelay(10/portTICK_PERIOD_MS);
        ESP_LOGI(GATTC_TAG, "Free memory: %" PRIu32 " bytes", esp_get_free_heap_size());
        ESP_ERROR_CHECK( esp_bluedroid_init() );
        ESP_ERROR_CHECK( esp_bluedroid_enable() );
        vTaskDelay(10/portTICK_PERIOD_MS);
    }
    return;
#endif

    // Note: Avoid performing time-consuming operations within callback functions.
    // Register the callback function to the gap module
    ret = esp_ble_gap_register_callback(esp_gap_cb);
    if (ret){
        ESP_LOGE(GATTC_TAG, "%s gap register failed, error code = %x", __func__, ret);
        return;
    }

    // Register the callback function to the gattc module
    ret = esp_ble_gattc_register_callback(esp_gattc_cb);
    if(ret){
        ESP_LOGE(GATTC_TAG, "%s gattc register failed, error code = %x", __func__, ret);
        return;
    }

    ret = esp_ble_gattc_app_register(PROFILE_A_APP_ID);
    if (ret){
        ESP_LOGE(GATTC_TAG, "%s gattc app register failed, error code = %x", __func__, ret);
    }
    esp_err_t local_mtu_ret = esp_ble_gatt_set_local_mtu(500);
    if (local_mtu_ret){
        ESP_LOGE(GATTC_TAG, "set local  MTU failed, error code = %x", local_mtu_ret);
    }

    // 初始化GPIO按键中断
    init_gpio_button();
    
    // 创建按键测试任务（每15秒模拟一次按键）
    // xTaskCreate(button_test_task, "button_test", 2048, NULL, 5, NULL);


    /*
    * This code is intended for debugging and prints all HCI data.
    * To enable it, turn on the "BT_HCI_LOG_DEBUG_EN" configuration option.
    * The output HCI data can be parsed using the script:
    * esp-idf/tools/bt/bt_hci_to_btsnoop.py.
    * For detailed instructions, refer to esp-idf/tools/bt/README.md.
    */

    /*
    while (1) {
        extern void bt_hci_log_hci_data_show(void);
        extern void bt_hci_log_hci_adv_show(void);
        bt_hci_log_hci_data_show();
        bt_hci_log_hci_adv_show();
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
    */

}

/**
 * @brief 从广播数据中获取设备名称
 * 
 * 此函数首先尝试获取完整设备名称，如果没有找到则尝试短设备名称。
 * 它会检查广播数据和扫描响应数据。
 * 
 * @param adv_data 广播数据缓冲区
 * @param adv_data_len 广播数据长度
 * @param scan_rsp_len 扫描响应数据长度
 * @param name_len 输出参数：找到的设备名称长度
 * @return 设备名称的指针，如果没有找到则返回NULL
 */
static uint8_t* get_device_name_from_adv_data(uint8_t *adv_data, uint16_t adv_data_len, uint16_t scan_rsp_len, uint8_t *name_len)
{
    uint8_t *name = NULL;
    uint16_t total_len = adv_data_len + scan_rsp_len;
    
    // 1. 首先尝试获取完整设备名称 (Complete Local Name)
    name = esp_ble_resolve_adv_data_by_type(adv_data, total_len, ESP_BLE_AD_TYPE_NAME_CMPL, name_len);
    if (name != NULL && *name_len > 0) {
        ESP_LOGD(GATTC_TAG, "Found complete device name");
        return name;
    }
    
    // 2. 如果没有找到完整名称，尝试获取短设备名称 (Shortened Local Name)
    name = esp_ble_resolve_adv_data_by_type(adv_data, total_len, ESP_BLE_AD_TYPE_NAME_SHORT, name_len);
    if (name != NULL && *name_len > 0) {
        ESP_LOGD(GATTC_TAG, "Found shortened device name");
        return name;
    }
    
    // 3. 如果仍然没有找到，分别检查广播数据和扫描响应数据
    if (adv_data_len > 0) {
        // 只在广播数据中查找完整名称
        name = esp_ble_resolve_adv_data_by_type(adv_data, adv_data_len, ESP_BLE_AD_TYPE_NAME_CMPL, name_len);
        if (name != NULL && *name_len > 0) {
            ESP_LOGD(GATTC_TAG, "Found complete name in adv data only");
            return name;
        }
        
        // 只在广播数据中查找短名称
        name = esp_ble_resolve_adv_data_by_type(adv_data, adv_data_len, ESP_BLE_AD_TYPE_NAME_SHORT, name_len);
        if (name != NULL && *name_len > 0) {
            ESP_LOGD(GATTC_TAG, "Found short name in adv data only");
            return name;
        }
    }
    
    if (scan_rsp_len > 0) {
        // 只在扫描响应数据中查找完整名称
        name = esp_ble_resolve_adv_data_by_type(adv_data + adv_data_len, scan_rsp_len, ESP_BLE_AD_TYPE_NAME_CMPL, name_len);
        if (name != NULL && *name_len > 0) {
            ESP_LOGD(GATTC_TAG, "Found complete name in scan response only");
            return name;
        }
        
        // 只在扫描响应数据中查找短名称
        name = esp_ble_resolve_adv_data_by_type(adv_data + adv_data_len, scan_rsp_len, ESP_BLE_AD_TYPE_NAME_SHORT, name_len);
        if (name != NULL && *name_len > 0) {
            ESP_LOGD(GATTC_TAG, "Found short name in scan response only");
            return name;
        }
    }
    
    // 没有找到任何设备名称
    *name_len = 0;
    return NULL;
}

/**
 * @brief 打印UUID信息
 */
static void print_uuid(esp_bt_uuid_t *uuid)
{
    switch (uuid->len) {
        case ESP_UUID_LEN_16:
            ESP_LOGI(GATTC_TAG, "UUID16: 0x%04x", uuid->uuid.uuid16);
            break;
        case ESP_UUID_LEN_32:
            ESP_LOGI(GATTC_TAG, "UUID32: 0x%08x", uuid->uuid.uuid32);
            break;
        case ESP_UUID_LEN_128:
            ESP_LOGI(GATTC_TAG, "UUID128: %02x%02x%02x%02x-%02x%02x-%02x%02x-%02x%02x-%02x%02x%02x%02x%02x%02x",
                     uuid->uuid.uuid128[15], uuid->uuid.uuid128[14], uuid->uuid.uuid128[13], uuid->uuid.uuid128[12],
                     uuid->uuid.uuid128[11], uuid->uuid.uuid128[10], uuid->uuid.uuid128[9], uuid->uuid.uuid128[8],
                     uuid->uuid.uuid128[7], uuid->uuid.uuid128[6], uuid->uuid.uuid128[5], uuid->uuid.uuid128[4],
                     uuid->uuid.uuid128[3], uuid->uuid.uuid128[2], uuid->uuid.uuid128[1], uuid->uuid.uuid128[0]);
            break;
        default:
            ESP_LOGI(GATTC_TAG, "UUID: Invalid length %d", uuid->len);
            break;
    }
}

/**
 * @brief 打印特征属性信息
 */
static void print_char_properties(uint8_t properties)
{
    ESP_LOGI(GATTC_TAG, "Characteristic properties: 0x%02x", properties);
    if (properties & ESP_GATT_CHAR_PROP_BIT_BROADCAST) {
        ESP_LOGI(GATTC_TAG, "  - Broadcast");
    }
    if (properties & ESP_GATT_CHAR_PROP_BIT_READ) {
        ESP_LOGI(GATTC_TAG, "  - Read");
    }
    if (properties & ESP_GATT_CHAR_PROP_BIT_WRITE_NR) {
        ESP_LOGI(GATTC_TAG, "  - Write No Response");
    }
    if (properties & ESP_GATT_CHAR_PROP_BIT_WRITE) {
        ESP_LOGI(GATTC_TAG, "  - Write");
    }
    if (properties & ESP_GATT_CHAR_PROP_BIT_NOTIFY) {
        ESP_LOGI(GATTC_TAG, "  - Notify");
    }
    if (properties & ESP_GATT_CHAR_PROP_BIT_INDICATE) {
        ESP_LOGI(GATTC_TAG, "  - Indicate");
    }
    if (properties & ESP_GATT_CHAR_PROP_BIT_AUTH) {
        ESP_LOGI(GATTC_TAG, "  - Authenticated Signed Writes");
    }
    if (properties & ESP_GATT_CHAR_PROP_BIT_EXT_PROP) {
        ESP_LOGI(GATTC_TAG, "  - Extended Properties");
    }
}

/**
 * @brief 发现所有服务
 */
static void discover_all_services(esp_gatt_if_t gattc_if, uint16_t conn_id)
{
    ESP_LOGI(GATTC_TAG, "=== Starting service discovery ===");
    
    // 发现所有主要服务
    esp_ble_gattc_search_service(gattc_if, conn_id, NULL);
}

/**
 * @brief 发现并打印指定服务中的所有特征
 */
static void discover_all_chars_in_service(esp_gatt_if_t gattc_if, uint16_t conn_id, uint16_t start_handle, uint16_t end_handle)
{
    uint16_t count = 0;
    
    ESP_LOGI(GATTC_TAG, "--- Discovering characteristics in service (handles %d-%d) ---", start_handle, end_handle);
    
    // 获取特征数量
    esp_gatt_status_t status = esp_ble_gattc_get_attr_count(gattc_if, conn_id, ESP_GATT_DB_CHARACTERISTIC,
                                                           start_handle, end_handle, INVALID_HANDLE, &count);
    if (status != ESP_GATT_OK) {
        ESP_LOGE(GATTC_TAG, "Failed to get characteristic count, status: %d", status);
        return;
    }
    
    ESP_LOGI(GATTC_TAG, "Attribute count reports %d characteristics in this service", count);
    
    if (count > 0) {
        ESP_LOGI(GATTC_TAG, "Attempting to discover characteristics using enhanced method...");
        
        // 尝试使用enhanced method来获取特征详细信息
        esp_gattc_char_elem_t *char_elem_result = (esp_gattc_char_elem_t *)malloc(sizeof(esp_gattc_char_elem_t) * count);
        if (char_elem_result) {
            uint16_t char_count = count;
            
            // 方法1: 尝试使用NULL UUID来获取所有特征
            esp_bt_uuid_t uuid_all;
            memset(&uuid_all, 0, sizeof(esp_bt_uuid_t));
            uuid_all.len = ESP_UUID_LEN_16;
            
            esp_gatt_status_t char_status = esp_ble_gattc_get_char_by_uuid(gattc_if, conn_id,
                                                                         start_handle, end_handle,
                                                                         uuid_all, char_elem_result, &char_count);
            
            if (char_status != ESP_GATT_OK || char_count == 0) {
                // 方法2: 尝试获取所有特征的不同方法
                ESP_LOGI(GATTC_TAG, "Trying alternative characteristic discovery...");
                char_count = count;
                
                // 方法3: 尝试通过句柄范围来枚举特征
                ESP_LOGI(GATTC_TAG, "Attempting handle-based characteristic enumeration...");
                
                // 简单的句柄遍历方法 - 这里我们使用一个启发式方法
                bool found_characteristics = false;
                
                // 尝试16位UUID扫描
                uint16_t common_uuids_16[] = {
                    0x2A00, 0x2A01, 0x2A04, 0x2A05, // GAP Service
                    0x2A05, 0x2A06, 0x2A07, 0x2A08, // GATT Service  
                    0xFF01, 0xFF02, 0xFF03, 0xFF04, // Custom characteristics
                    REMOTE_NOTIFY_CHAR_UUID, 0x2902, // Notification chars
                    0x2A19, 0x2A6E, 0x2A6F, 0x2A37, // Battery, temp, etc
                    0x2A29, 0x2A24, 0x2A25, 0x2A27, // Device info
                    0x2A56, 0x2A57, 0x2A58, 0x2A59, // More standard chars
                    0x0012, 0x0013, 0x0014, 0x0015  // User specified custom UUIDs
                };
                
                // 常见的128位UUID（按小端存储）
                uint8_t common_uuids_128[][16] = {
                    // Nordic UART Service: 6E400001-B5A3-F393-E0A9-E50E24DCCA9E
                    {0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0, 0x93, 0xF3, 0xA3, 0xB5, 0x01, 0x00, 0x40, 0x6E},
                    // Nordic UART TX: 6E400002-B5A3-F393-E0A9-E50E24DCCA9E
                    {0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0, 0x93, 0xF3, 0xA3, 0xB5, 0x02, 0x00, 0x40, 0x6E},
                    // Nordic UART RX: 6E400003-B5A3-F393-E0A9-E50E24DCCA9E
                    {0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0, 0x93, 0xF3, 0xA3, 0xB5, 0x03, 0x00, 0x40, 0x6E},
                    // ESP32 Custom Service: A0B20001-DEAD-BEEF-CAFE-123456789ABC
                    {0xBC, 0x9A, 0x78, 0x56, 0x34, 0x12, 0xFE, 0xCA, 0xEF, 0xBE, 0xAD, 0xDE, 0x01, 0x00, 0xB2, 0xA0},
                    // ESP32 Custom Char 1: A0B20002-DEAD-BEEF-CAFE-123456789ABC
                    {0xBC, 0x9A, 0x78, 0x56, 0x34, 0x12, 0xFE, 0xCA, 0xEF, 0xBE, 0xAD, 0xDE, 0x02, 0x00, 0xB2, 0xA0},
                    // ESP32 Custom Char 2: A0B20003-DEAD-BEEF-CAFE-123456789ABC
                    {0xBC, 0x9A, 0x78, 0x56, 0x34, 0x12, 0xFE, 0xCA, 0xEF, 0xBE, 0xAD, 0xDE, 0x03, 0x00, 0xB2, 0xA0},
                    // Generic 128-bit patterns (common base UUIDs)
                    // 12345678-1234-1234-1234-123456789ABC
                    {0xBC, 0x9A, 0x78, 0x56, 0x34, 0x12, 0x34, 0x12, 0x34, 0x12, 0x34, 0x12, 0x78, 0x56, 0x34, 0x12},
                    // ABCD1234-5678-9ABC-DEF0-123456789ABC
                    {0xBC, 0x9A, 0x78, 0x56, 0x34, 0x12, 0xF0, 0xDE, 0xBC, 0x9A, 0x78, 0x56, 0x34, 0x12, 0xCD, 0xAB}
                };
                
                int num_common_uuids_16 = sizeof(common_uuids_16) / sizeof(common_uuids_16[0]);
                int num_common_uuids_128 = sizeof(common_uuids_128) / sizeof(common_uuids_128[0]);
                
                // 首先尝试16位UUID
                ESP_LOGI(GATTC_TAG, "Scanning for 16-bit UUIDs...");
                for (int uuid_idx = 0; uuid_idx < num_common_uuids_16; uuid_idx++) {
                    esp_bt_uuid_t test_uuid;
                    test_uuid.len = ESP_UUID_LEN_16;
                    test_uuid.uuid.uuid16 = common_uuids_16[uuid_idx];
                    
                    uint16_t test_count = 1;
                    esp_gattc_char_elem_t test_char;
                    
                    esp_gatt_status_t test_status = esp_ble_gattc_get_char_by_uuid(gattc_if, conn_id,
                                                                                 start_handle, end_handle,
                                                                                 test_uuid, &test_char, &test_count);
                    
                    if (test_status == ESP_GATT_OK && test_count > 0) {
                        if (!found_characteristics) {
                            ESP_LOGI(GATTC_TAG, "Found characteristics by 16-bit UUID scanning:");
                            found_characteristics = true;
                        }
                        ESP_LOGI(GATTC_TAG, "  ★ Found 16-bit UUID Characteristic:");
                        ESP_LOGI(GATTC_TAG, "    Handle: %d", test_char.char_handle);
                        ESP_LOGI(GATTC_TAG, "    UUID:");
                        print_uuid(&test_char.uuid);
                        print_char_properties(test_char.properties);
                        
                        // 保存0x0013特征的句柄
                        if (test_char.uuid.len == ESP_UUID_LEN_16 && 
                            test_char.uuid.uuid.uuid16 == CUSTOM_CHAR_UUID_0013) {
                            char_0013_handle = test_char.char_handle;
                            ESP_LOGI(GATTC_TAG, "    *** CUSTOM 0x0013 CHARACTERISTIC FOUND ***");
                            ESP_LOGI(GATTC_TAG, "    Handle saved for GPIO button functionality");
                        }
                        
                        ESP_LOGI(GATTC_TAG, "    ---");
                    }
                }
                
                // 然后尝试128位UUID
                ESP_LOGI(GATTC_TAG, "Scanning for 128-bit UUIDs...");
                for (int uuid_idx = 0; uuid_idx < num_common_uuids_128; uuid_idx++) {
                    esp_bt_uuid_t test_uuid;
                    test_uuid.len = ESP_UUID_LEN_128;
                    memcpy(test_uuid.uuid.uuid128, common_uuids_128[uuid_idx], ESP_UUID_LEN_128);
                    
                    uint16_t test_count = 1;
                    esp_gattc_char_elem_t test_char;
                    
                    esp_gatt_status_t test_status = esp_ble_gattc_get_char_by_uuid(gattc_if, conn_id,
                                                                                 start_handle, end_handle,
                                                                                 test_uuid, &test_char, &test_count);
                    
                    if (test_status == ESP_GATT_OK && test_count > 0) {
                        if (!found_characteristics) {
                            ESP_LOGI(GATTC_TAG, "Found characteristics by 128-bit UUID scanning:");
                            found_characteristics = true;
                        }
                        ESP_LOGI(GATTC_TAG, "  ★ Found 128-bit UUID Characteristic:");
                        ESP_LOGI(GATTC_TAG, "    Handle: %d", test_char.char_handle);
                        ESP_LOGI(GATTC_TAG, "    UUID:");
                        print_uuid(&test_char.uuid);
                        print_char_properties(test_char.properties);
                        ESP_LOGI(GATTC_TAG, "    ---");
                    }
                }
                
                if (!found_characteristics) {
                    ESP_LOGI(GATTC_TAG, "No characteristics found with common 16-bit UUIDs");
                    ESP_LOGI(GATTC_TAG, "Service has %d characteristics - they may use 128-bit UUIDs", count);
                    ESP_LOGI(GATTC_TAG, "Attempting to discover characteristics through handle enumeration...");
                    
                    // 尝试通过句柄枚举的方法
                    ESP_LOGI(GATTC_TAG, "Handle range: %d to %d (span: %d)", start_handle, end_handle, end_handle - start_handle);
                    ESP_LOGI(GATTC_TAG, "Expected %d characteristics in this range", count);
                    ESP_LOGI(GATTC_TAG, "Note: To see actual characteristic UUIDs, you may need to:");
                    ESP_LOGI(GATTC_TAG, "  1. Use a generic BLE scanner app");
                    ESP_LOGI(GATTC_TAG, "  2. Check device documentation");
                    ESP_LOGI(GATTC_TAG, "  3. Enable more verbose GATT debugging");
                }
            } else {
                ESP_LOGI(GATTC_TAG, "Successfully found %d characteristics using UUID method:", char_count);
                for (int i = 0; i < char_count; i++) {
                    ESP_LOGI(GATTC_TAG, "  [Char %d] Handle: %d", i + 1, char_elem_result[i].char_handle);
                    ESP_LOGI(GATTC_TAG, "  [Char %d] UUID:", i + 1);
                    print_uuid(&char_elem_result[i].uuid);
                    print_char_properties(char_elem_result[i].properties);
                    ESP_LOGI(GATTC_TAG, "  ---");
                }
            }
            
            free(char_elem_result);
        } else {
            ESP_LOGE(GATTC_TAG, "Failed to allocate memory for characteristic discovery");
        }
    } else {
        ESP_LOGI(GATTC_TAG, "No characteristics found in this service");
    }
    
    ESP_LOGI(GATTC_TAG, "--- Basic characteristic discovery complete ---");
}

/**
 * @brief 简单的特征枚举方法
 */
static void enumerate_all_chars_in_service(esp_gatt_if_t gattc_if, uint16_t conn_id, uint16_t start_handle, uint16_t end_handle)
{
    ESP_LOGI(GATTC_TAG, ">> Enumerating characteristics in service handles %d-%d", start_handle, end_handle);
    
    // 使用更简单的方法 - 尝试不同的UUID来枚举特征
    uint16_t count = 1;  // 开始时假设有1个特征
    esp_gattc_char_elem_t *char_elem_result = (esp_gattc_char_elem_t*)malloc(sizeof(esp_gattc_char_elem_t) * 10);
    
    if (!char_elem_result) {
        ESP_LOGE(GATTC_TAG, "Failed to allocate memory for characteristic enumeration");
        return;
    }
    
    // 尝试获取所有特征 - 不指定特定UUID
    esp_gatt_status_t status = esp_ble_gattc_get_char_by_uuid(gattc_if, conn_id, start_handle, end_handle,
                                                             remote_filter_char_uuid, char_elem_result, &count);
    
    if (status == ESP_GATT_OK && count > 0) {
        ESP_LOGI(GATTC_TAG, ">> Found %d characteristics using specific UUID method", count);
    } else {
        ESP_LOGI(GATTC_TAG, ">> No characteristics found with specific UUID, trying generic discovery...");
        
        // 尝试使用get_all_char的不同方法
        count = 10;  // 最多查找10个特征
        memset(char_elem_result, 0, sizeof(esp_gattc_char_elem_t) * 10);
        
        // 可能需要使用其他API或方法
        ESP_LOGI(GATTC_TAG, ">> Service may contain characteristics that require different discovery method");
        ESP_LOGI(GATTC_TAG, ">> You may need to check the specific service documentation");
    }
    
    free(char_elem_result);
    ESP_LOGI(GATTC_TAG, ">> Characteristic enumeration complete");
}

/**
 * @brief 在服务发现完成后，发现所有服务的特征
 */
static void discover_all_characteristics_after_service_discovery(esp_gatt_if_t gattc_if, uint16_t conn_id)
{
    ESP_LOGI(GATTC_TAG, "=== COMPREHENSIVE CHARACTERISTIC DISCOVERY ===");
    
    // 获取所有服务的数量
    uint16_t service_count = 0;
    esp_gatt_status_t status = esp_ble_gattc_get_attr_count(gattc_if, conn_id, ESP_GATT_DB_PRIMARY_SERVICE,
                                                           1, 0xFFFF, INVALID_HANDLE, &service_count);
    
    if (status != ESP_GATT_OK) {
        ESP_LOGE(GATTC_TAG, "Failed to get service count, status: %d", status);
        return;
    }
    
    ESP_LOGI(GATTC_TAG, "Total services found: %d", service_count);
    
    ESP_LOGI(GATTC_TAG, "Note: Individual service characteristics were displayed during service discovery");
    ESP_LOGI(GATTC_TAG, "Each service's characteristics are shown when the service is found");
    
    // 这里可以添加额外的统计信息
    if (service_count > 0) {
        ESP_LOGI(GATTC_TAG, "Summary: %d services discovered during scan", service_count);
        ESP_LOGI(GATTC_TAG, "Look for '=== Service Found ===' entries above for detailed service and characteristic information");
    } else {
        ESP_LOGI(GATTC_TAG, "No services found");
    }
    
    ESP_LOGI(GATTC_TAG, "=== COMPREHENSIVE CHARACTERISTIC DISCOVERY COMPLETE ===");
}

/**
 * @brief 将UUID字符串转换为字节数组（小端格式）
 * @param uuid_str UUID字符串，格式为 "12345678-1234-1234-1234-123456789ABC"
 * @param uuid_bytes 输出的16字节数组
 */
static void uuid_string_to_bytes(const char* uuid_str, uint8_t* uuid_bytes)
{
    // 示例UUID: "12345678-1234-1234-1234-123456789ABC"
    // 转换为小端字节序: {0xBC, 0x9A, 0x78, 0x56, 0x34, 0x12, 0x34, 0x12, 0x34, 0x12, 0x34, 0x12, 0x78, 0x56, 0x34, 0x12}
    
    // 这里提供一个简单的示例实现
    // 实际使用时，您需要根据具体的UUID字符串格式来解析
    
    // 示例：解析Nordic UART Service UUID "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
    if (strstr(uuid_str, "6E400001") != NULL) {
        uint8_t nordic_uart_service[] = {0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0, 0x93, 0xF3, 0xA3, 0xB5, 0x01, 0x00, 0x40, 0x6E};
        memcpy(uuid_bytes, nordic_uart_service, 16);
    }
    // 可以继续添加更多UUID的解析...
}

/**
 * @brief 添加自定义128位UUID到扫描列表
 * 
 * 如果您知道目标设备的特定128位UUID，可以在这里添加：
 * 
 * 步骤1: 获取设备的128位UUID
 * - 使用手机BLE扫描器（如nRF Connect）
 * - 查看设备文档
 * - 使用其他BLE调试工具
 * 
 * 步骤2: 将UUID转换为小端字节数组
 * - UUID "12345678-1234-1234-1234-123456789ABC"
 * - 转换为 {0xBC, 0x9A, 0x78, 0x56, 0x34, 0x12, 0x34, 0x12, 0x34, 0x12, 0x34, 0x12, 0x78, 0x56, 0x34, 0x12}
 * 
 * 步骤3: 添加到代码中的common_uuids_128数组
 * 
 * 示例常见128位UUID:
 * 
 * 1. Nordic UART Service:
 *    - Service: 6E400001-B5A3-F393-E0A9-E50E24DCCA9E
 *    - TX Char: 6E400002-B5A3-F393-E0A9-E50E24DCCA9E  
 *    - RX Char: 6E400003-B5A3-F393-E0A9-E50E24DCCA9E
 * 
 * 2. ESP32 Custom UUIDs:
 *    - 通常以厂商特定的前缀开始
 *    - 例如: A0B2xxxx-DEAD-BEEF-CAFE-123456789ABC
 * 
 * 3. 其他常见模式:
 *    - 0000xxxx-0000-1000-8000-00805F9B34FB (标准蓝牙基础UUID)
 *    - xxxxxxxx-xxxx-xxxx-xxxx-xxxxxxxxxxxx (完全自定义)
 */

/**
 * @brief GPIO中断服务程序 - 安全版本
 *
 * 在中断上下文中执行，处理GPIO18的下降沿中断
 * 添加了安全检查，防止系统重启
 */
static void IRAM_ATTR gpio_isr_handler(void* arg)
{
    // 安全检查：确保队列已初始化
    if (gpio_evt_queue == NULL) {
        return;
    }

    uint32_t gpio_num = (uint32_t) arg;
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    // 尝试发送GPIO事件到队列，如果队列满了就丢弃
    xQueueSendFromISR(gpio_evt_queue, &gpio_num, &xHigherPriorityTaskWoken);

    // 安全的任务切换
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

/**
 * @brief GPIO按键处理任务 - 安全版本
 *
 * 等待中断事件并发送数据，添加了安全检查
 */
static void gpio_button_task(void* arg)
{
    TickType_t last_press_time = 0;
    const TickType_t debounce_delay = pdMS_TO_TICKS(200); // 200ms防抖动

    ESP_LOGI(GATTC_TAG, "GPIO button task started");

    uint32_t io_num;

    while(1) {
        // 安全检查：确保队列存在
        if (gpio_evt_queue == NULL) {
            ESP_LOGE(GATTC_TAG, "GPIO event queue is NULL, task exiting");
            break;
        }

        // 等待中断事件，使用超时避免永久阻塞
        if(xQueueReceive(gpio_evt_queue, &io_num, pdMS_TO_TICKS(1000))) {
            // 验证GPIO编号
            if (io_num != GPIO_BUTTON_PIN) {
                ESP_LOGW(GATTC_TAG, "Unexpected GPIO number: %d", io_num);
                continue;
            }

            TickType_t current_time = xTaskGetTickCount();

            // 防抖动检查
            if ((current_time - last_press_time) > debounce_delay) {
                last_press_time = current_time;

                ESP_LOGI(GATTC_TAG, "Button pressed! Sending data...");

                // 安全发送数据
                if (gl_profile_tab[PROFILE_A_APP_ID].char_handle != 0) {
                    send_data_to_char_0013();
                } else {
                    ESP_LOGW(GATTC_TAG, "Characteristic not ready, skipping send");
                }
            } else {
                ESP_LOGD(GATTC_TAG, "Button press ignored (debounce)");
            }
        }

        // 让出CPU时间
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    ESP_LOGI(GATTC_TAG, "GPIO button task ended");
    vTaskDelete(NULL);
}

/**
 * @brief 初始化GPIO按键中断 - 安全版本
 */
static void init_gpio_button(void)
{
    ESP_LOGI(GATTC_TAG, "Initializing GPIO18 button interrupt...");

    // 如果队列已存在，先清理
    if (gpio_evt_queue != NULL) {
        vQueueDelete(gpio_evt_queue);
        gpio_evt_queue = NULL;
    }

    // 创建GPIO事件队列
    gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));
    if (gpio_evt_queue == NULL) {
        ESP_LOGE(GATTC_TAG, "Failed to create GPIO event queue");
        return;
    }

    // 创建GPIO按键处理任务
    if (xTaskCreate(gpio_button_task, "gpio_button", 3072, NULL, 5, NULL) != pdPASS) {
        ESP_LOGE(GATTC_TAG, "Failed to create GPIO button task");
        vQueueDelete(gpio_evt_queue);
        gpio_evt_queue = NULL;
        return;
    }

    // 先重置GPIO配置
    gpio_reset_pin(GPIO_BUTTON_PIN);

    // 配置GPIO18
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_NEGEDGE,
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = (1ULL << GPIO_BUTTON_PIN),
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en = GPIO_PULLUP_ENABLE,
    };

    esp_err_t ret = gpio_config(&io_conf);
    if (ret != ESP_OK) {
        ESP_LOGE(GATTC_TAG, "Failed to configure GPIO: %s", esp_err_to_name(ret));
        vQueueDelete(gpio_evt_queue);
        gpio_evt_queue = NULL;
        return;
    }

    // 安装中断服务（如果已安装会返回错误，但不影响功能）
    ret = gpio_install_isr_service(ESP_INTR_FLAG_LEVEL1);
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(GATTC_TAG, "Failed to install GPIO ISR service: %s", esp_err_to_name(ret));
        vQueueDelete(gpio_evt_queue);
        gpio_evt_queue = NULL;
        return;
    }

    // 添加中断处理函数
    ret = gpio_isr_handler_add(GPIO_BUTTON_PIN, gpio_isr_handler, (void*) GPIO_BUTTON_PIN);
    if (ret != ESP_OK) {
        ESP_LOGE(GATTC_TAG, "Failed to add GPIO ISR handler: %s", esp_err_to_name(ret));
        vQueueDelete(gpio_evt_queue);
        gpio_evt_queue = NULL;
        return;
    }

    ESP_LOGI(GATTC_TAG, "GPIO18 button interrupt initialized successfully");
    ESP_LOGI(GATTC_TAG, "Connect button between GPIO18 and GND");
}

/**
 * @brief 向0x0013特征发送指定的字节流
 */
static void send_data_to_char_0013(void)
{
    if (char_0013_handle == 0) {
        ESP_LOGW(GATTC_TAG, "0x0013 characteristic handle not found, cannot send data");
        return;
    }
    
    if (!connect) {
        ESP_LOGW(GATTC_TAG, "Device not connected, cannot send data");
        return;
    }
    
    // 要发送的字节流
    uint8_t data_to_send[] = {
        0x00, 0x02, 0xaa, 0x01, 0x00, 0x5c, 0xa2, 0x9a, 0x48, 0x0b,
        0x31, 0x38, 0x38, 0x36, 0x37, 0x31, 0x31, 0x32, 0x36, 0x31
    };
    
    size_t data_len = sizeof(data_to_send);
    
    ESP_LOGI(GATTC_TAG, "Sending %d bytes to characteristic 0x0013 (handle: %d)", data_len, char_0013_handle);
    ESP_LOG_BUFFER_HEX(GATTC_TAG, data_to_send, data_len);
    
    // 发送数据到特征
    esp_err_t ret = esp_ble_gattc_write_char(gl_profile_tab[PROFILE_A_APP_ID].gattc_if,
                                             gl_profile_tab[PROFILE_A_APP_ID].conn_id,
                                             char_0013_handle,
                                             data_len,
                                             data_to_send,
                                             ESP_GATT_WRITE_TYPE_RSP,
                                             ESP_GATT_AUTH_REQ_NONE);
    
    if (ret != ESP_OK) {
        ESP_LOGE(GATTC_TAG, "Write characteristic failed, error code = %x", ret);
    } else {
        ESP_LOGI(GATTC_TAG, "Write characteristic success");
    }
}


