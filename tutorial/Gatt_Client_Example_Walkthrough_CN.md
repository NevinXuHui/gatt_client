# GATT客户端示例详解

## 简介

在本教程中，我们将回顾ESP32的GATT客户端示例代码。该代码实现了一个蓝牙低功耗（BLE）通用属性（GATT）客户端，用于扫描附近的外围服务器并连接到预定义的服务。客户端然后搜索可用的特征值，并订阅已知特征值以接收通知或指示。该示例可以注册一个应用配置文件并初始化一系列事件，这些事件可用于配置通用访问配置文件（GAP）参数以及处理扫描、连接外围设备、读写特征值等事件。

# 头文件包含

此示例位于ESP-IDF的示例文件夹中，路径为[bluetooth/bluedroid/ble/gatt_client/main](../main)。位于main文件夹中的[gattc_demo.c](../main/gattc_demo.c)文件包含了我们要回顾的所有功能。[gattc_demo.c](../main/gattc_demo.c)中包含的头文件有：

```c
#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include <stdio.h>
#include "nvs.h"
#include "nvs_flash.h"
#include "controller.h"

#include "bt.h"
#include "esp_gap_ble_api.h"
#include "esp_gattc_api.h"
#include "esp_gatt_defs.h"
#include "esp_bt_main.h"
#include "esp_gatt_common_api.h"
```

这些`includes`是FreeRTOS和底层系统组件运行所必需的，包括日志功能和在非易失性flash存储器中存储数据的库。我们感兴趣的是`"bt.h"`、`"esp_bt_main.h"`、`"esp_gap_ble_api.h"`和`"esp_gattc_api.h"`，它们提供了实现此示例所需的BLE API。

* `bt.h`：从主机端配置BT控制器和VHCI。
* `esp_bt_main.h`：初始化并启用Bluedroid协议栈。
* `esp_gap_ble_api.h`：实现GAP配置，如广播和连接参数。
* `esp_gattc_api.h`：实现GATT客户端配置，如连接外围设备和搜索服务。

## 主入口点

程序的入口点是app_main()函数：

```c
void app_main()
{
    // 初始化NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK( ret );

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ret = esp_bt_controller_init(&bt_cfg);
    if (ret) {
        ESP_LOGE(GATTC_TAG, "%s initialize controller failed, error code = %x", __func__, ret);
        return;
    }

    ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if (ret) {
        ESP_LOGE(GATTC_TAG, "%s enable controller failed, error code = %x", __func__, ret);
        return;
    }

    ret = esp_bluedroid_init();
    if (ret) {
        ESP_LOGE(GATTC_TAG, "%s init bluetooth failed, error code = %x", __func__, ret);
        return;
    }

    ret = esp_bluedroid_enable();
    if (ret) {
        ESP_LOGE(GATTC_TAG, "%s enable bluetooth failed, error code = %x", __func__, ret);
        return;
    }

    //向gap模块注册回调函数
    ret = esp_ble_gap_register_callback(esp_gap_cb);
    if (ret){
        ESP_LOGE(GATTC_TAG, "%s gap register failed, error code = %x", __func__, ret);
        return;
    }

    //向gattc模块注册回调函数
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

}
```

主函数首先初始化非易失性存储库。该库允许在flash存储器中保存键值对，被Wi-Fi库等一些组件用来保存SSID和密码：

```c
esp_err_t ret = nvs_flash_init();
if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_ERROR_CHECK(nvs_flash_erase());
    ret = nvs_flash_init();
}
ESP_ERROR_CHECK( ret );
```

## BT控制器和协议栈初始化

主函数还通过首先创建一个名为`esp_bt_controller_config_t`的BT控制器配置结构来初始化BT控制器，该结构使用`BT_CONTROLLER_INIT_CONFIG_DEFAULT()`宏生成默认设置。BT控制器在控制器端实现主机控制器接口（HCI）、链路层（LL）和物理层（PHY）。BT控制器对用户应用程序不可见，处理BLE协议栈的较低层。控制器配置包括设置BT控制器堆栈大小、优先级和HCI波特率。创建设置后，使用`esp_bt_controller_init()`函数初始化并启用BT控制器：

```c
esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
ret = esp_bt_controller_init(&bt_cfg);
```

接下来，在BLE模式下启用控制器。

```c
ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
```
>如果要使用双模式（BLE + BT），控制器应该在`ESP_BT_MODE_BTDM`模式下启用。

支持四种蓝牙模式：

1. `ESP_BT_MODE_IDLE`：蓝牙未运行
2. `ESP_BT_MODE_BLE`：BLE模式
3. `ESP_BT_MODE_CLASSIC_BT`：BT经典模式
4. `ESP_BT_MODE_BTDM`：双模式（BLE + BT经典）

在BT控制器初始化之后，Bluedroid协议栈（包括BT经典和BLE的通用定义和API）使用以下方式初始化并启用：

```c
ret = esp_bluedroid_init();
ret = esp_bluedroid_enable();
```
主函数最后注册GAP和GATT事件处理程序，以及应用配置文件并设置最大支持的MTU大小。

```c
    //向gap模块注册回调函数
    ret = esp_ble_gap_register_callback(esp_gap_cb);

    //向gattc模块注册回调函数
    ret = esp_ble_gattc_register_callback(esp_gattc_cb);

    ret = esp_ble_gattc_app_register(PROFILE_A_APP_ID);

    esp_err_t local_mtu_ret = esp_ble_gatt_set_local_mtu(500);
    if (local_mtu_ret){
        ESP_LOGE(GATTC_TAG, "set local  MTU failed, error code = %x", local_mtu_ret);
    }
```

GAP和GATT事件处理程序是用于捕获BLE协议栈生成的事件并执行函数来配置应用程序参数的函数。此外，事件处理程序还用于处理来自中心设备的读写事件。GAP事件处理程序负责扫描和连接服务器，GATT处理程序管理客户端连接到服务器后发生的事件，如搜索服务和读写数据。GAP和GATT事件处理程序通过以下方式注册：

```c
esp_ble_gap_register_callback();
esp_ble_gattc_register_callback();
```
函数`esp_gap_cb()`和`esp_gattc_cb()`处理BLE协议栈生成的所有事件。

## 应用配置文件

应用配置文件是一种将为一个或多个服务器应用程序设计的功能分组的方法。例如，您可以有一个连接到心率传感器的应用配置文件，另一个连接到温度传感器的应用配置文件。每个应用配置文件创建一个GATT接口来连接其他设备。代码中的应用配置文件是`gattc_profile_inst`结构的实例，定义如下：

```c
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
```

应用配置文件结构包含：

* `gattc_cb`：GATT客户端回调函数
* `gattc_if`：此配置文件的GATT客户端接口号
* `app_id`：应用配置文件ID号
* `conn_id`：连接ID
* `service_start_handle`：服务起始句柄
* `service_end_handle`：服务结束句柄
* `char_handle`：特征句柄
* `remote_bda`：连接到此客户端的远程设备地址

在此示例中有一个应用配置文件，其ID定义为：

```c
#define PROFILE_NUM 1
#define PROFILE_A_APP_ID 0
```
应用配置文件存储在`gl_profile_tab`数组中，初始化如下：

```c
/* 基于gatt的配置文件，一个app_id和一个gattc_if，此数组将存储ESP_GATTS_REG_EVT返回的gattc_if */
static struct gattc_profile_inst gl_profile_tab[PROFILE_NUM] = {
		[PROFILE_A_APP_ID] = {.gattc_cb = gattc_profile_event_handler,
								  .gattc_if = ESP_GATT_IF_NONE, /* 未获取gatt_if，所以初始值为ESP_GATT_IF_NONE */
    },
};
```

应用配置文件表数组的初始化包括为配置文件定义回调函数。它是`gattc_profile_event_handler()`。此外，GATT接口初始化为默认值`ESP_GATT_IF_NONE`。稍后，当注册应用配置文件时，BLE协议栈返回一个GATT接口实例以与该应用配置文件一起使用。

配置文件注册触发一个`ESP_GATTC_REG_EVT`事件，该事件由`esp_gattc_cb()`事件处理程序处理。处理程序获取事件返回的GATT接口并将其存储在配置文件表中：

```c
static void esp_gattc_cb(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param)
{
    ESP_LOGI(GATTC_TAG, "EVT %d, gattc if %d", event, gattc_if);

    /* 如果事件是注册事件，为每个配置文件存储gattc_if */
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
…
```

最后，回调函数为`gl_profile_tab`表中的每个配置文件调用相应的事件处理程序。

```c
…
/* 如果gattc_if等于配置文件A，调用配置文件A的回调处理程序，
     * 所以这里调用每个配置文件的回调 */
    do {
        int idx;
        for (idx = 0; idx < PROFILE_NUM; idx++) {
            if (gattc_if == ESP_GATT_IF_NONE || /* ESP_GATT_IF_NONE，不指定特定的gatt_if，需要调用每个配置文件的回调函数 */
                    gattc_if == gl_profile_tab[idx].gattc_if) {
                if (gl_profile_tab[idx].gattc_cb) {
                    gl_profile_tab[idx].gattc_cb(event, gattc_if, param);
                }
            }
        }
    } while (0);
}
```
## 设置扫描参数

GATT客户端通常扫描附近的服务器并尝试连接它们（如果感兴趣）。但是，为了执行扫描，首先需要设置配置参数。这在应用配置文件注册之后完成，因为注册一旦完成就会触发`ESP_GATTC_REG_EVT`事件。此事件第一次触发时，GATT事件处理程序捕获它并为配置文件A分配GATT接口，然后将事件转发给配置文件A的GATT事件处理程序。在此事件处理程序中，事件用于调用`esp_ble_gap_set_scan_params()`函数，该函数将`ble_scan_params`结构实例作为参数。该结构定义如下：

```c
/// BLE扫描参数
typedef struct {
    esp_ble_scan_type_t     scan_type;              /*!< 扫描类型 */
    esp_ble_addr_type_t     own_addr_type;          /*!< 自身地址类型 */
    esp_ble_scan_filter_t   scan_filter_policy;     /*!< 扫描过滤策略 */
    uint16_t                scan_interval;          /*!< 扫描间隔。定义为控制器从开始上次LE扫描到开始后续LE扫描的时间间隔。*/
    //范围：0x0004到0x4000
    //默认：0x0010（10毫秒）
    //时间 = N * 0.625毫秒
    //时间范围：2.5毫秒到10.24秒
    uint16_t                scan_window;            /*!< 扫描窗口。LE扫描的持续时间。LE_Scan_Window应小于或等于LE_Scan_Interval*/
    //范围：0x0004到0x4000                                                   	 //默认：0x0010（10毫秒）
    //时间 = N * 0.625毫秒
    //时间范围：2.5毫秒到10240毫秒
} esp_ble_scan_params_t;
```
它初始化为：

```c
static esp_ble_scan_params_t ble_scan_params = {
    .scan_type              = BLE_SCAN_TYPE_ACTIVE,
    .own_addr_type          = BLE_ADDR_TYPE_PUBLIC,
    .scan_filter_policy     = BLE_SCAN_FILTER_ALLOW_ALL,
    .scan_interval          = 0x50,
    .scan_window            = 0x30
};
```

BLE扫描参数配置为扫描类型为主动（包括读取扫描响应），为公共类型，允许读取任何广播设备，扫描间隔为100毫秒（1.25毫秒 * 0x50），扫描窗口为60毫秒（1.25毫秒 * 0x30）。

使用`esp_ble_gap_set_scan_params()`函数设置扫描值：

```c
case ESP_GATTC_REG_EVT:
        ESP_LOGI(GATTC_TAG, "REG_EVT");
        esp_err_t scan_ret = esp_ble_gap_set_scan_params(&ble_scan_params);
        if (scan_ret){
            ESP_LOGE(GATTC_TAG, "set scan params error, error code = %x", scan_ret);
        }
        break;
```

## 开始扫描

一旦设置了扫描参数，就会触发`ESP_GAP_BLE_SCAN_PARAM_SET_COMPLETE_EVT`事件，该事件由GAP事件处理程序`esp_gap_cb()`处理。此事件用于开始扫描附近的GATT服务器：

```c
    case ESP_GAP_BLE_SCAN_PARAM_SET_COMPLETE_EVT: {
        //持续时间的单位是秒
        uint32_t duration = 30;
        esp_ble_gap_start_scanning(duration);
        break;
        }
```

使用`esp_ble_gap_start_scanning()`函数开始扫描，该函数接受一个表示连续扫描持续时间（以秒为单位）的参数。扫描期结束后，触发`ESP_GAP_SEARCH_INQ_CMPL_EVT`事件。

## 获取扫描结果

扫描结果在到达时立即通过`ESP_GAP_BLE_SCAN_RESULT_EVT`事件显示，该事件包括以下参数：

```c
    /**
     * @brief ESP_GAP_BLE_SCAN_RESULT_EVT
     */
    struct ble_scan_result_evt_param {
        esp_gap_search_evt_t search_evt;            /*!< 搜索事件类型 */
        esp_bd_addr_t bda;                          /*!< 已搜索到的蓝牙设备地址 */
        esp_bt_dev_type_t dev_type;                 /*!< 设备类型 */
        esp_ble_addr_type_t ble_addr_type;          /*!< BLE设备地址类型 */
        esp_ble_evt_type_t ble_evt_type;            /*!< BLE扫描结果事件类型 */
        int rssi;                                   /*!< 搜索到的设备的RSSI */
        uint8_t  ble_adv[ESP_BLE_ADV_DATA_LEN_MAX + ESP_BLE_SCAN_RSP_DATA_LEN_MAX]; /*!< 接收到的EIR */
        int flag;                                   /*!< 广播数据标志位 */
        int num_resps;                              /*!< 扫描结果数量 */
        uint8_t adv_data_len;                       /*!< 广播数据长度 */
        uint8_t scan_rsp_len;                       /*!< 扫描响应长度 */
    } scan_rst;                                     /*!< ESP_GAP_BLE_SCAN_RESULT_EVT的事件参数 */
```

此事件还包括如下所示的子事件列表：

```c
/// ESP_GAP_BLE_SCAN_RESULT_EVT的子事件
typedef enum {
    ESP_GAP_SEARCH_INQ_RES_EVT             = 0,      /*!< 对等设备的查询结果。 */
    ESP_GAP_SEARCH_INQ_CMPL_EVT            = 1,      /*!< 查询完成。 */
    ESP_GAP_SEARCH_DISC_RES_EVT            = 2,      /*!< 对等设备的发现结果。 */
    ESP_GAP_SEARCH_DISC_BLE_RES_EVT        = 3,      /*!< 对等设备上基于BLE GATT的服务的发现结果。 */
    ESP_GAP_SEARCH_DISC_CMPL_EVT           = 4,      /*!< 发现完成。 */
    ESP_GAP_SEARCH_DI_DISC_CMPL_EVT        = 5,      /*!< 发现完成。 */
    ESP_GAP_SEARCH_SEARCH_CANCEL_CMPL_EVT  = 6,      /*!< 搜索已取消 */
} esp_gap_search_evt_t;
```
我们对`ESP_GAP_SEARCH_INQ_RES_EVT`事件感兴趣，该事件在每次找到新设备时调用。我们还对`ESP_GAP_SEARCH_INQ_CMPL_EVT`感兴趣，该事件在扫描持续时间完成时触发，可用于重新启动扫描过程：

```c
      case ESP_GAP_BLE_SCAN_RESULT_EVT: {
        esp_ble_gap_cb_param_t *scan_result = (esp_ble_gap_cb_param_t *)param;
        switch (scan_result->scan_rst.search_evt) {
	        case ESP_GAP_SEARCH_INQ_RES_EVT:
		        ESP_LOG_BUFFER_HEX(GATTC_TAG, scan_result->scan_rst.bda, 6);
		        ESP_LOGI(GATTC_TAG, "searched Adv Data Len %d, Scan Response Len %d", scan_result->scan_rst.adv_data_len, scan_result->scan_rst.scan_rsp_len);
		        adv_name = esp_ble_resolve_adv_data(scan_result->scan_rst.ble_adv, ESP_BLE_AD_TYPE_NAME_CMPL, &adv_name_len);
		        ESP_LOGI(GATTC_TAG, "searched Device Name Len %d", adv_name_len);
		        ESP_LOG_BUFFER_CHAR(GATTC_TAG, adv_name, adv_name_len);
		        ESP_LOGI(GATTC_TAG, " ");
		        if (adv_name != NULL) {
			        if (strlen(remote_device_name) == adv_name_len && strncmp((char *)adv_name, remote_device_name, adv_name_len) == 0) {
                    // 注意：如果有多个设备具有相同的设备名称，设备可能连接到非预期的设备。
                    // 建议更改默认设备名称以确保其唯一性。
                    ESP_LOGI(GATTC_TAG, "searched device %s", remote_device_name);
                    if (connect == false) {
                        connect = true;
                        ESP_LOGI(GATTC_TAG, "connect to the remote device.");
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
```

首先解析设备名称并与`remote_device_name`中定义的名称进行比较。如果它等于我们感兴趣的GATT服务器的设备名称，则停止扫描。

## 连接到GATT服务器

每次从`ESP_GAP_SEARCH_INQ_RES_EVT`事件接收到结果时，代码首先打印远程设备的地址：

```c
case ESP_GAP_SEARCH_INQ_RES_EVT:
     ESP_LOG_BUFFER_HEX(GATTC_TAG, scan_result->scan_rst.bda, 6);
```

然后客户端打印广播数据长度和扫描响应长度：

```c
ESP_LOGI(GATTC_TAG, "searched Adv Data Len %d, Scan Response Len %d", scan_result->scan_rst.adv_data_len, scan_result->scan_rst.scan_rsp_len);
```

为了获取设备名称，我们使用`esp_ble_resolve_adv_data()`函数，该函数接受存储在`scan_result->scan_rst.ble_adv`中的广播数据、广播数据类型和长度，以便从广播包帧中提取值。然后打印设备名称。

```c
adv_name = esp_ble_resolve_adv_data(scan_result->scan_rst.ble_adv, ESP_BLE_AD_TYPE_NAME_CMPL, &adv_name_len);
ESP_LOGI(GATTC_TAG, "searched Device Name Len %d", adv_name_len);
ESP_LOG_BUFFER_CHAR(GATTC_TAG, adv_name, adv_name_len);
```

最后，如果远程设备名称与我们上面定义的相同，本地设备停止扫描并尝试使用`esp_ble_gattc_enh_open()`函数打开与远程设备的连接。该函数接受应用配置文件GATT接口、远程服务器地址和布尔值作为参数。布尔值用于指示连接是直接完成还是在后台完成（自动连接），目前此布尔值必须设置为true才能建立连接。注意，客户端向服务器打开虚拟连接。虚拟连接返回连接ID。虚拟连接是应用配置文件与远程服务器之间的连接。由于许多应用配置文件可以在一个ESP32上运行，因此可能有许多虚拟连接打开到同一个远程服务器。还有物理连接，这是客户端和服务器之间的实际BLE链路。因此，如果使用`esp_ble_gap_disconnect()`函数断开物理连接，所有其他虚拟连接也会关闭。在此示例中，每个应用配置文件使用`esp_ble_gattc_enh_open()`函数创建与同一服务器的虚拟连接，因此当调用关闭函数时，只有来自应用配置文件的那个连接被关闭，而如果调用gap断开函数，两个连接都将被关闭。此外，连接事件传播到所有配置文件，因为它与物理连接相关，而打开事件仅传播到创建虚拟连接的配置文件。

## 配置MTU大小

ATT_MTU定义为客户端和服务器之间发送的任何数据包的最大大小。当客户端连接到服务器时，它通过交换MTU请求和响应协议数据单元（PDU）来通知服务器使用哪个MTU大小。这在打开连接后完成。打开连接后，触发`ESP_GATTC_CONNECT_EVT`事件：

```c
     case ESP_GATTC_CONNECT_EVT:
        //p_data->connect.status总是ESP_GATT_OK
        ESP_LOGI(GATTC_TAG, "ESP_GATTC_CONNECT_EVT conn_id %d, if %d, status %d", conn_id, gattc_if, p_data->connect.status);
        conn_id = p_data->connect.conn_id;
        gl_profile_tab[PROFILE_A_APP_ID].conn_id = p_data->connect.conn_id;
        memcpy(gl_profile_tab[PROFILE_A_APP_ID].remote_bda, p_data->connect.remote_bda, sizeof(esp_bd_addr_t));
        ESP_LOGI(GATTC_TAG, "REMOTE BDA:");
        ESP_LOG_BUFFER_HEX(GATTC_TAG, gl_profile_tab[PROFILE_A_APP_ID].remote_bda, sizeof(esp_bd_addr_t));
        esp_err_t mtu_ret = esp_ble_gattc_send_mtu_req (gattc_if, conn_id);
        if (mtu_ret){
            ESP_LOGE(GATTC_TAG, "config MTU error, error code = %x", mtu_ret);
        }
        break;
```

连接ID和远程设备（服务器）的地址存储在应用配置文件表中并打印到控制台：

```c
conn_id = p_data->connect.conn_id;
gl_profile_tab[PROFILE_A_APP_ID].conn_id = p_data->connect.conn_id;
memcpy(gl_profile_tab[PROFILE_A_APP_ID].remote_bda, p_data->connect.remote_bda,
		sizeof(esp_bd_addr_t));
ESP_LOGI(GATTC_TAG, "REMOTE BDA:");
ESP_LOG_BUFFER_HEX(GATTC_TAG, gl_profile_tab[PROFILE_A_APP_ID].remote_bda,
		sizeof(esp_bd_addr_t));
```

蓝牙4.0连接的典型MTU大小为23字节。客户端可以使用`esp_ble_gattc_send_mtu_req()`函数更改MTU大小，该函数接受GATT接口和连接ID。请求的MTU大小由`esp_ble_gatt_set_local_mtu()`定义。然后服务器可以接受或拒绝请求。ESP32支持高达517字节的MTU大小，这由`esp_gattc_api.h`中的`ESP_GATT_MAX_MTU_SIZE`定义。在此示例中，MTU大小设置为500字节。如果配置失败，返回的错误将被打印：

```c
esp_err_t mtu_ret = esp_ble_gattc_send_mtu_req (gattc_if, conn_id);
if (mtu_ret){
	ESP_LOGE(GATTC_TAG, "config MTU error, error code = %x", mtu_ret);
}
break;
```

连接打开还触发`ESP_GATTC_OPEN_EVT`，用于检查连接打开是否成功完成，否则打印错误并退出。

```c
case ESP_GATTC_OPEN_EVT:
        if (param->open.status != ESP_GATT_OK){
            ESP_LOGE(GATTC_TAG, "open failed, status %d", p_data->open.status);
            break;
        }
ESP_LOGI(GATTC_TAG, "open success");
```

当MTU交换时，触发`ESP_GATTC_CFG_MTU_EVT`，在此示例中用于打印新的MTU大小。

```c
case ESP_GATTC_CFG_MTU_EVT:
        if (param->cfg_mtu.status != ESP_GATT_OK){
            ESP_LOGE(GATTC_TAG,"config mtu failed, error status = %x", param->cfg_mtu.status);
        }
        ESP_LOGI(GATTC_TAG, "ESP_GATTC_CFG_MTU_EVT, Status %d, MTU %d, conn_id %d", param->cfg_mtu.status, param->cfg_mtu.mtu, param->cfg_mtu.conn_id);
…
```

## 发现服务

MTU配置事件还用于开始发现客户端刚刚连接到的服务器中可用的服务。要发现服务，使用`esp_ble_gattc_search_service()`函数。该函数的参数是GATT接口、应用配置文件连接ID和客户端感兴趣的服务应用程序的UUID。我们正在寻找的服务定义为：

```c
static esp_bt_uuid_t remote_filter_service_uuid = {
    .len = ESP_UUID_LEN_16,
    .uuid = {.uuid16 = REMOTE_SERVICE_UUID,},
};
```
其中，

```c
#define REMOTE_SERVICE_UUID        0x00FF
```
如果用户感兴趣的服务应用程序的UUID是128位，那么下面有一个与处理器架构的小端存储模式相关的用户注意事项。
UUID的结构定义为：

```c
typedef struct {
#define ESP_UUID_LEN_16     2
#define ESP_UUID_LEN_32     4
#define ESP_UUID_LEN_128    16
    uint16_t len;							/*!< UUID长度，16位、32位或128位 */
    union {
        uint16_t    uuid16;                 /*!< 16位UUID */
        uint32_t    uuid32;                 /*!< 32位UUID */
        uint8_t     uuid128[ESP_UUID_LEN_128]; /*!< 128位UUID */
    } uuid;									/*!< UUID */
} __attribute__((packed)) esp_bt_uuid_t;
```

注意：在小端存储模式下，如果是16位或32位UUID，您可以直接按正常顺序定义服务UUID，但如果服务UUID是128位，则有细微差异。例如，如果用户感兴趣的服务应用程序的UUID是12345678-a1b2-c3d4-e5f6-9fafd205e457，`REMOTE_SERVICE_UUID`应定义为{0x57,0xE4,0x05,0xD2,0xAF,0x9F,0xF6,0xE5,0xD4,0xC3,0xB2,0xA1,0x78,0x56,0x34,0x12}。

然后如下发现服务：

```c
esp_ble_gattc_search_service(gattc_if, param->cfg_mtu.conn_id, &remote_filter_service_uuid);
        break;
```

找到的结果服务（如果有的话）将从`ESP_GATTC_SEARCH_RES_EVT`返回。对于找到的每个服务，触发事件以打印有关发现的服务的信息，具体取决于UUID的大小：

```c
 case ESP_GATTC_SEARCH_RES_EVT: {
        esp_gatt_srvc_id_t *srvc_id = &p_data->search_res.srvc_id;
        conn_id = p_data->search_res.conn_id;
        if (srvc_id->id.uuid.len == ESP_UUID_LEN_16 && srvc_id->id.uuid.uuid.uuid16 ==
REMOTE_SERVICE_UUID) {
        get_server = true;
        gl_profile_tab[PROFILE_A_APP_ID].service_start_handle = p_data->search_res.start_handle;
        gl_profile_tab[PROFILE_A_APP_ID].service_end_handle = p_data->search_res.end_handle;
        ESP_LOGI(GATTC_TAG, "UUID16: %x", srvc_id->id.uuid.uuid.uuid16);
        }
        break;
```

如果客户端找到它正在寻找的服务，get_server标志设置为true，并保存起始句柄值和结束句柄值，这些值稍后将用于获取该服务的所有特征。在返回所有服务结果后，搜索完成并触发`ESP_GATTC_SEARCH_CMPL_EVT`事件。

## 获取特征

此示例实现从预定义服务获取特征数据。我们想要特征的服务具有UUID 0x00FF，我们感兴趣的特征具有UUID 0xFF01：

```c
#define REMOTE_NOTIFY_CHAR_UUID    0xFF01
```
服务使用`esp_gatt_srvc_id_t`结构定义为：

```c
/**
| * @brief Gatt id，包括uuid和实例id
| */
typedef struct {
    esp_bt_uuid_t   uuid;                   /*!< UUID */
    uint8_t         inst_id;                /*!< 实例id */
} __attribute__((packed)) esp_gatt_id_t;
```

在此示例中，我们将要获取特征的服务定义为：

```c
static esp_gatt_srvc_id_t remote_service_id = {
    .id = {
        .uuid = {
            .len = ESP_UUID_LEN_16,
            .uuid = {.uuid16 = REMOTE_SERVICE_UUID,},
        },
        .inst_id = 0,
    },
    .is_primary = true,
};
```

一旦定义，我们可以使用`esp_ble_gattc_get_char_by_uuid()`函数从该服务获取特征，该函数在搜索服务完成且客户端找到它正在寻找的服务后在`ESP_GATTC_SEARCH_CMPL_EVT`事件中调用。

```c
case ESP_GATTC_SEARCH_CMPL_EVT:
    if (p_data->search_cmpl.status != ESP_GATT_OK){
        ESP_LOGE(GATTC_TAG, "search service failed, error status = %x", p_data->search_cmpl.status);
        break;
    }
    conn_id = p_data->search_cmpl.conn_id;
    if (get_server){
        uint16_t count = 0;
        esp_gatt_status_t status = esp_ble_gattc_get_attr_count( gattc_if,
                          p_data->search_cmpl.conn_id,ESP_GATT_DB_CHARACTERISTIC,                                                                                                                 		                    gl_profile_tab[PROFILE_A_APP_ID].service_start_handle,                                                                   		                    gl_profile_tab[PROFILE_A_APP_ID].service_end_handle,
                                                                INVALID_HANDLE,
                                                                     &count);
        if (status != ESP_GATT_OK){
            ESP_LOGE(GATTC_TAG, "esp_ble_gattc_get_attr_count error");
        }

        if (count > 0){
            char_elem_result = (esp_gattc_char_elem_t*)malloc
                                          (sizeof(esp_gattc_char_elem_t) * count);
            if (!char_elem_result){
                ESP_LOGE(GATTC_TAG, "gattc no mem");
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
                }

                /*  在我们的'ESP_GATTS_DEMO'演示中，每个服务只有一个特征，
                    所以我们使用第一个'char_elem_result' */
                if (count > 0 && (char_elem_result[0].properties
                                 &ESP_GATT_CHAR_PROP_BIT_NOTIFY)){
                    gl_profile_tab[PROFILE_A_APP_ID].char_handle =
                    char_elem_result[0].char_handle;
                    esp_ble_gattc_register_for_notify (gattc_if,
                                   gl_profile_tab[PROFILE_A_APP_ID].remote_bda,
                                   char_elem_result[0].char_handle);
                }
            }
            /* 释放char_elem_result */
            free(char_elem_result);
        }else{
            ESP_LOGE(GATTC_TAG, "no char found");
        }        }
        break;
```

`esp_ble_gattc_get_attr_count()`在gattc缓存中获取给定服务或特征的属性计数。`esp_ble_gattc_get_attr_count()`函数的参数是GATT接口、连接ID、`esp_gatt_db_attr_type_t`中定义的属性类型、属性起始句柄、属性结束句柄、特征句柄（此参数仅在类型设置为`ESP_GATT_DB_DESCRIPTOR`时有效）并输出在gattc缓存中使用给定属性类型找到的属性数量。然后我们分配一个缓冲区来保存`esp_ble_gattc_get_char_by_uuid()`函数的特征信息。该函数在gattc缓存中找到具有给定特征UUID的特征。它只是从本地缓存获取特征，而不是从远程设备。在服务器中，可能有多个特征共享相同的UUID。但是，在我们的gatt_server演示中，每个特征都有唯一的UUID，这就是为什么我们只使用`char_elem_result`中的第一个特征的原因，它是指向服务特征的指针。Count最初存储客户端想要查找的特征数量，并将使用`esp_ble_gattc_get_char_by_uuid`在gattc缓存中实际找到的特征数量进行更新。

## 注册通知

客户端可以注册以在特征值发生变化时接收来自服务器的通知。在此示例中，我们要注册UUID为0xff01的特征的通知。获取所有特征后，我们检查接收到的特征的属性，然后使用`esp_ble_gattc_register_for_notify()`函数注册通知。函数参数是GATT接口、远程服务器的地址和我们要注册通知的句柄。

```c
…
/*  在我们的'ESP_GATTS_DEMO'演示中，每个服务只有一个特征，所以我们使用第一个'char_elem_result' */
                    if(count > 0 && (char_elem_result[0].properties & ESP_GATT_CHAR_PROP_BIT_NOTIFY)){
                        gl_profile_tab[PROFILE_A_APP_ID].char_handle = char_elem_result[0].char_handle;
                        esp_ble_gattc_register_for_notify (gattc_if, gl_profile_tab[PROFILE_A_APP_ID].remote_bda,
                        char_elem_result[0].char_handle);
                        }
…
```

此过程向BLE协议栈注册通知，并触发`ESP_GATTC_REG_FOR_NOTIFY_EVT`。此事件用于写入服务器客户端配置描述符：

```c
    case ESP_GATTC_REG_FOR_NOTIFY_EVT: {
        ESP_LOGI(GATTC_TAG, "ESP_GATTC_REG_FOR_NOTIFY_EVT");
        if (p_data->reg_for_notify.status != ESP_GATT_OK){
            ESP_LOGE(GATTC_TAG, "REG FOR NOTIFY failed: error status = %d", p_data->reg_for_notify.status);
        }else{
            uint16_t count = 0;
            uint16_t notify_en = 1;
            esp_gatt_status_t ret_status = esp_ble_gattc_get_attr_count( gattc_if, gl_profile_tab[PROFILE_A_APP_ID].conn_id,
											            ESP_GATT_DB_DESCRIPTOR,
											            gl_profile_tab[PROFILE_A_APP_ID].service_start_handle,
											            gl_profile_tab[PROFILE_A_APP_ID].service_end_handle,
											            gl_profile_tab[PROFILE_A_APP_ID].char_handle, &count);
            if (ret_status != ESP_GATT_OK){
                ESP_LOGE(GATTC_TAG, "esp_ble_gattc_get_attr_count error");
            }
            if (count > 0){
                descr_elem_result = malloc(sizeof(esp_gattc_descr_elem_t) * count);
                if (!descr_elem_result){
                    ESP_LOGE(GATTC_TAG, "malloc error, gattc no mem");
                }else{
                    ret_status = esp_ble_gattc_get_descr_by_char_handle(
                    gattc_if,
                    gl_profile_tab[PROFILE_A_APP_ID].conn_id,
                    p_data->reg_for_notify.handle,
                    notify_descr_uuid,
                    descr_elem_result,&count);

                    if (ret_status != ESP_GATT_OK){
                        ESP_LOGE(GATTC_TAG, "esp_ble_gattc_get_descr_by_char_handle
                                                                            error");
                    }

                    /* 在我们的'ESP_GATTS_DEMO'演示中，每个特征只有一个描述符，所以我们使用第一个'descr_elem_result' */
                    if (count > 0 && descr_elem_result[0].uuid.len == ESP_UUID_LEN_16 && descr_elem_result[0].uuid.uuid.uuid16 == ESP_GATT_UUID_CHAR_CLIENT_CONFIG){
                        ret_status = esp_ble_gattc_write_char_descr( gattc_if,
								                        gl_profile_tab[PROFILE_A_APP_ID].conn_id,
								                        descr_elem_result[0].handle,
								                        sizeof(notify_en),
								                        (Uint8 *)&notify_en,
								                        ESP_GATT_WRITE_TYPE_RSP,
								                        ESP_GATT_AUTH_REQ_NONE);
                    }

                    if (ret_status != ESP_GATT_OK){
                        ESP_LOGE(GATTC_TAG, "esp_ble_gattc_write_char_descr error");
                    }

                    /* 释放descr_elem_result */
                    free(descr_elem_result);
                }
            }
            else{
                ESP_LOGE(GATTC_TAG, "decsr not found");
            }

        }
        break;
    }
```

该事件用于首先打印通知注册状态以及刚刚注册的通知的服务和特征UUID。然后客户端使用`esp_ble_gattc_write_char_descr()`函数写入客户端配置描述符。蓝牙规范中定义了许多特征描述符。但是，在这种情况下，我们有兴趣写入处理启用通知的描述符，即客户端配置描述符。为了将此描述符作为参数传递，我们首先将其定义为：

```c
static esp_gatt_id_t notify_descr_id = {
    .uuid = {
        .len = ESP_UUID_LEN_16,
        .uuid = {.uuid16 = ESP_GATT_UUID_CHAR_CLIENT_CONFIG,},
    },
    .inst_id = 0,
};
```
其中`ESP_GATT_UUID_CHAR_CLIENT_CONFIG`使用UUID定义以标识特征客户端配置：

```c
#define ESP_GATT_UUID_CHAR_CLIENT_CONFIG            0x2902          /*  客户端特征配置 */
```
要写入的值是"1"以启用通知。我们还传递`ESP_GATT_WRITE_TYPE_RSP`以请求服务器响应启用通知的请求，以及`ESP_GATT_AUTH_REQ_NONE`以指示写入请求不需要授权。

## 结论

我们已经回顾了ESP32的GATT客户端示例代码。此示例扫描附近的设备并搜索感兴趣的服务器的服务和特征。当找到感兴趣的服务器时，与该服务器建立连接并执行服务搜索。最后，客户端在找到的服务中查找特定特征，如果找到，获取特征值并注册该特征的通知。这是通过注册一个应用配置文件并遵循一系列事件来配置所需的GAP和GATT参数来完成的。



