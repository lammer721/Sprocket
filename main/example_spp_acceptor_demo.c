#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include <stdio.h>
//#include "nvs.h"
#include "nvs_flash.h"
#include "esp_log.h"
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_gap_bt_api.h"
#include "esp_bt_device.h"
#include "esp_spp_api.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <hx711.h>
#include <esp_timer.h>
#include <esp_idf_lib_helpers.h>
#include <driver/gpio.h>

#define SPP_TAG "SPP_ACCEPTOR_DEMO"
#define SPP_SERVER_NAME "SPP_SERVER"
#define EXAMPLE_DEVICE_NAME "ESP_SPP_ACCEPTOR"

#define LINE_LENGTH 100
#define MESSAGE_LINES 5
#define MESSAGE_LENGTH LINE_LENGTH*MESSAGE_LINES

#define DOUT_GPIO1   21
#define PD_SCK_GPIO1 22
#define DOUT_GPIO2   23
#define PD_SCK_GPIO2 19
#define DOUT_GPIO3   18
#define PD_SCK_GPIO3  2
//#define DOUT_GPIO4   5
//#define PD_SCK_GPIO4 15
#define DOUT_GPIO5   27
#define PD_SCK_GPIO5 4
#define DOUT_GPIO6   17
#define PD_SCK_GPIO6 16
 
static bool bWriteAfterOpenEvt = true;
static bool bWriteAfterWriteEvt = false;
static bool bWriteAfterSvrOpenEvt = true;
static bool bWriteAfterDataReceived = true;
static bool bConnectionflag = false;

static const esp_spp_mode_t esp_spp_mode = ESP_SPP_MODE_CB;
static const esp_spp_sec_t sec_mask = ESP_SPP_SEC_AUTHENTICATE;
static const esp_spp_role_t role_slave = ESP_SPP_ROLE_SLAVE;

typedef struct {
	uint32_t handle;
	bool congested;
	bool connected;
} uartRxParam_t;

static uartRxParam_t uart_rx_task_params;
 
#define SPP_DATA_LEN 20
static uint8_t spp_data[SPP_DATA_LEN]; 
   
static void esp_spp_cb(esp_spp_cb_event_t event, esp_spp_cb_param_t *param){ 
    switch (event) {
    case ESP_SPP_INIT_EVT:
        ESP_LOGI(SPP_TAG, "ESP_SPP_INIT_EVT");
        esp_bt_dev_set_device_name(EXAMPLE_DEVICE_NAME);
        esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);
        esp_spp_start_srv(sec_mask,role_slave, 0, SPP_SERVER_NAME);
        break;
    case ESP_SPP_DISCOVERY_COMP_EVT:
        ESP_LOGI(SPP_TAG, "ESP_SPP_DISCOVERY_COMP_EVT");
        break;
    case ESP_SPP_OPEN_EVT:
        if (bWriteAfterOpenEvt){
            ESP_LOGI(SPP_TAG, "bWriteAfterOpenEvt = true");
            char * c = "Hello";
            uint8_t * u = (uint8_t *)c;
            esp_spp_write(param->srv_open.handle, 6, u);
        }
        else{
            ESP_LOGI(SPP_TAG, "bWriteAfterOpenEvt = false");
        }
        break;

    case ESP_SPP_CLOSE_EVT:
        ESP_LOGI(SPP_TAG, "ESP_SPP_CLOSE_EVT");
        break;

    case ESP_SPP_START_EVT:                                         //Short before connection is established
        ESP_LOGI(SPP_TAG, "ESP_SPP_START_EVT");
        break;

    case ESP_SPP_CL_INIT_EVT:
        ESP_LOGI(SPP_TAG, "ESP_SPP_CL_INIT_EVT");
        break;

    case ESP_SPP_DATA_IND_EVT:                                      //When SPP connection received data, the event comes, only for ESP_SPP_MODE_CB
        ESP_LOGI(SPP_TAG, "ESP_SPP_DATA_IND_EVT len=%d handle=%d", param->data_ind.len, param->data_ind.handle);
        esp_log_buffer_char("Received String Data",param->data_ind.data,param->data_ind.len);
        bConnectionflag = true;
        if (bWriteAfterDataReceived){
            ESP_LOGI(SPP_TAG, "bWriteAfterDataReceived = true");
           for (int i = 0; i < SPP_DATA_LEN; ++i) {
                spp_data[i] = i + 97;
            }
            char * c = "Received";
            uint8_t * u = (uint8_t *)c;
            esp_spp_write(param->srv_open.handle, 8, u);
        }
        else{
            ESP_LOGI(SPP_TAG, "bWriteAfterDataReceived = false");
        }
       break;

    case ESP_SPP_CONG_EVT:
        ESP_LOGI(SPP_TAG, "ESP_SPP_CONG_EVT");
        uart_rx_task_params.congested = param->write.cong;	// Update congestion status
        break;

    case ESP_SPP_WRITE_EVT:
        if (param->write.cong == 0) {
            ESP_LOGI(SPP_TAG, "param->write.cong == 0");
            if (bWriteAfterWriteEvt){
                esp_spp_write(param->write.handle, SPP_DATA_LEN, spp_data);
                ESP_LOGI(SPP_TAG, "ESP_SPP_WRITE_EVT len=%d cong=%d", param->write.len , param->write.cong);
            }
            else{
                ESP_LOGI(SPP_TAG, "bWriteAfterWriteEvt = false");
            }
        }
        else {
            ESP_LOGI(SPP_TAG, "param->write.cong <> 0");
        }
        break;

    case ESP_SPP_SRV_OPEN_EVT:                                      //After connection is established, short before data is received
        ESP_LOGI(SPP_TAG, "bWriteAfterOpenEvt: %d: ", bWriteAfterSvrOpenEvt);
        uart_rx_task_params.handle = param->srv_open.handle;
        uart_rx_task_params.connected = 1;	// Set connected status as true
        if (bWriteAfterSvrOpenEvt){
            char * c = "Hello";             
            uint8_t * u = (uint8_t *)c;
            ESP_LOGI(SPP_TAG, "Call esp_spp_write(param->srv_open.handle, 5, Hello)");
            esp_spp_write(param->srv_open.handle, 5, u);    //Works, but maybe it needs something like CR
        }
        else{
            ESP_LOGI(SPP_TAG, "bWriteAfterSvrOpenEvt = false");
        }
        break;

    default:
        break;
    }
}
 
void esp_bt_gap_cb(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param){
    switch (event) {
    case ESP_BT_GAP_AUTH_CMPL_EVT:{
        if (param->auth_cmpl.stat == ESP_BT_STATUS_SUCCESS) {
            ESP_LOGI(SPP_TAG, "authentication success: %s", param->auth_cmpl.device_name);
            esp_log_buffer_hex(SPP_TAG, param->auth_cmpl.bda, ESP_BD_ADDR_LEN);
        } else {
            ESP_LOGE(SPP_TAG, "authentication failed, status:%d", param->auth_cmpl.stat);
        }
        break;
    }
    case ESP_BT_GAP_PIN_REQ_EVT:{
        ESP_LOGI(SPP_TAG, "ESP_BT_GAP_PIN_REQ_EVT min_16_digit:%d", param->pin_req.min_16_digit);
        if (param->pin_req.min_16_digit) {
            ESP_LOGI(SPP_TAG, "Input pin code: 0000 0000 0000 0000");
            esp_bt_pin_code_t pin_code = {0};
            esp_bt_gap_pin_reply(param->pin_req.bda, true, 16, pin_code);
        } else {
            ESP_LOGI(SPP_TAG, "Input pin code: 1234");
            esp_bt_pin_code_t pin_code;
            pin_code[0] = '1';
            pin_code[1] = '2';
            pin_code[2] = '3';
            pin_code[3] = '4';
            esp_bt_gap_pin_reply(param->pin_req.bda, true, 4, pin_code);
        }
        break;
    }
 
    //Must be set in sdkconfig.h: CONFIG_BT_SSP_ENABLED == true
    //This enables the Secure Simple Pairing.

    case ESP_BT_GAP_CFM_REQ_EVT:
        ESP_LOGI(SPP_TAG, "ESP_BT_GAP_CFM_REQ_EVT Please compare the numeric value: %d", param->cfm_req.num_val);
        esp_bt_gap_ssp_confirm_reply(param->cfm_req.bda, true);
        break;
    case ESP_BT_GAP_KEY_NOTIF_EVT:
        ESP_LOGI(SPP_TAG, "ESP_BT_GAP_KEY_NOTIF_EVT passkey:%d", param->key_notif.passkey);
        break;
    case ESP_BT_GAP_KEY_REQ_EVT:
        ESP_LOGI(SPP_TAG, "ESP_BT_GAP_KEY_REQ_EVT Please enter passkey!");
        break;

    default: {
        ESP_LOGI(SPP_TAG, "event: %d", event);
        //  0 ESP_BT_GAP_DISC_RES_EVT
        //  1 ESP_BT_GAP_DISC_STATE_CHANGED_EVT
        //  2 ESP_BT_GAP_RMT_SRVCS_EVT
        //  3 ESP_BT_GAP_RMT_SRVC_REC_EVT
        //  4 ESP_BT_GAP_AUTH_CMPL_EVT
        //  5 ESP_BT_GAP_PIN_REQ_EVT
        //  6 ESP_BT_GAP_CFM_REQ_EVT
        //  7 ESP_BT_GAP_KEY_NOTIF_EVT
        //  8 ESP_BT_GAP_KEY_REQ_EVT
        //  9 ESP_BT_GAP_READ_RSSI_DELTA_EVT
        // 10 ESP_BT_GAP_CONFIG_EIR_DATA_EVT
        // 11 ESP_BT_GAP_EVT_MAX
        break;
    }
    }
    return;
}
 
void startClassicBtSpp(void){

    for (int i = 0; i < SPP_DATA_LEN; ++i) {
        spp_data[i] = i + 65;
    }

    esp_err_t ret = nvs_flash_init();   //Initialize the default NVS partition. Non-volatile storage (NVS) library is designed to store key-value pairs in flash.
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK( ret );
 
    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_BLE));        //release the controller memory as per the mode
 
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    if ((ret = esp_bt_controller_init(&bt_cfg)) != ESP_OK) {
        ESP_LOGE(SPP_TAG, "%s initialize controller failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }
    else{
        ESP_LOGI(SPP_TAG, "Initialize controller ok");
    }
   
    if ((ret = esp_bt_controller_enable(ESP_BT_MODE_CLASSIC_BT)) != ESP_OK) {
        ESP_LOGE(SPP_TAG, "%s enable controller failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }
    else{
        ESP_LOGI(SPP_TAG, "Enable controller ok");
    }
   
    if ((ret = esp_bluedroid_init()) != ESP_OK) {
        ESP_LOGE(SPP_TAG, "%s initialize bluedroid failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }
    else{
        ESP_LOGI(SPP_TAG, "Initialize bluedroid ok");
    }
   
    if ((ret = esp_bluedroid_enable()) != ESP_OK) {
        ESP_LOGE(SPP_TAG, "%s enable bluedroid failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }
    else{
        ESP_LOGI(SPP_TAG, "Enable bluedroid ok");
    }
   
    if ((ret = esp_bt_gap_register_callback(esp_bt_gap_cb)) != ESP_OK) {
        ESP_LOGE(SPP_TAG, "%s gap register failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }
    else{
        ESP_LOGI(SPP_TAG, "Gap register ok");
    }
   
    if ((ret = esp_spp_register_callback(esp_spp_cb)) != ESP_OK) {
        ESP_LOGE(SPP_TAG, "%s spp register failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }
    else{
        ESP_LOGI(SPP_TAG, "spp register ok");
    }
   
    if ((ret = esp_spp_init(esp_spp_mode)) != ESP_OK) {
        ESP_LOGE(SPP_TAG, "%s spp init failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }
    else{
        ESP_LOGI(SPP_TAG, "spp init ok");
    } 

    ESP_LOGI(SPP_TAG, "CONFIG_BT_SSP_ENABLED == true");    //Must be set in sdkconfig.h: CONFIG_BT_SSP_ENABLED == true. This enables the Secure Simple Pairing.
    ESP_LOGI(SPP_TAG, "Set default parameters for Secure Simple Pairing");  
    esp_bt_sp_param_t param_type = ESP_BT_SP_IOCAP_MODE;
    esp_bt_io_cap_t iocap = ESP_BT_IO_CAP_IO;
    esp_bt_gap_set_security_param(param_type, &iocap, sizeof(uint8_t));
      
    esp_bt_pin_type_t pin_type = ESP_BT_PIN_TYPE_VARIABLE;  //This enables the Secure Simple Pairing. Use variable pin, input pin code when pairing
    esp_bt_pin_code_t pin_code;
    esp_bt_gap_set_pin(pin_type, 0, pin_code);
}
 
void transmit_task(void * params){

    char message[MESSAGE_LENGTH];
    message[0] = '\0';
    UBaseType_t uxHighWaterMark;

    hx711_t dev1 = {
        .dout = DOUT_GPIO1,
        .pd_sck = PD_SCK_GPIO1,
        .gain = HX711_GAIN_A_64
    };

    hx711_t dev2 = {
        .dout = DOUT_GPIO2,
        .pd_sck = PD_SCK_GPIO2,
        .gain = HX711_GAIN_A_64
    };

    hx711_t dev3 = {
        .dout = DOUT_GPIO3,
        .pd_sck = PD_SCK_GPIO3,
        .gain = HX711_GAIN_A_64
    };

    hx711_t dev5 = {
        .dout = DOUT_GPIO5,
        .pd_sck = PD_SCK_GPIO5,
        .gain = HX711_GAIN_A_64
    };

    hx711_t dev6 = {
        .dout = DOUT_GPIO6,
        .pd_sck = PD_SCK_GPIO6,
        .gain = HX711_GAIN_A_64
    };

    int a = 0;
    int b = 0;
    int z = 0;
    int x = 0;
    int y = 0;
    int i = 0;
    

    while (1)
    {
        esp_err_t r1 = hx711_init(&dev1);
        if (r1 == ESP_OK)
            a = 1;
        ESP_LOGI(SPP_TAG, "Error Status Sensor 1: %d (%s)\n", r1, esp_err_to_name(r1));
        vTaskDelay(10 / portTICK_PERIOD_MS);
         
        esp_err_t r2 = hx711_init(&dev2);
        if (r2 == ESP_OK)
            b = 1;
        ESP_LOGI(SPP_TAG, "Error Status Sensor 2: %d (%s)\n", r2, esp_err_to_name(r2));
        vTaskDelay(10 / portTICK_PERIOD_MS);

        esp_err_t r3 = hx711_init(&dev3);
        if (r3 == ESP_OK)
            z = 1;
        ESP_LOGI(SPP_TAG, "Error Status Sensor 3: %d (%s)\n", r3, esp_err_to_name(r3));
        vTaskDelay(10 / portTICK_PERIOD_MS);

        esp_err_t r5 = hx711_init(&dev5);
        if (r5 == ESP_OK)
            x = 1;
        ESP_LOGI(SPP_TAG, "Error Status Sensor 5: %d (%s)\n", r5, esp_err_to_name(r5));
        vTaskDelay(10 / portTICK_PERIOD_MS);

        esp_err_t r6 = hx711_init(&dev6);
        if (r6 == ESP_OK)
            y = 1;
        ESP_LOGI(SPP_TAG, "Error Status Sensor 6: %d (%s)\n", r6, esp_err_to_name(r6));
        vTaskDelay(10 / portTICK_PERIOD_MS);

        a = a + b + z + x + y;

        if (a == 5){
            break;
        }
        a = 0;
        b = 0;
        z = 0;
        x = 0;
        y = 0;
    }

    // read from device
    while (1)
    {
        i = i + 1; 
        int32_t data1;
        int32_t time;
        char c[LINE_LENGTH];   //whole line buffer string. expected format  "time (10 = 1s), sensor1 (16 bit int), sens2, sens3" 
        char buf[12];
        // ind'l reading buffer string
        ESP_LOGI(SPP_TAG, "i = %d", i);

        TickType_t time_start = xTaskGetTickCount();
        time = pdTICKS_TO_MS(time_start);
        ESP_LOGI(SPP_TAG, " %d", time);
        
        sprintf(c, "%d", time);   //add current time to the line buffer
        strcat(c, ", ");          //append comma

        //SENSOR 1
        esp_err_t r1 = hx711_wait(&dev1, 500);
        if (r1 != ESP_OK)
        {
            ESP_LOGI(SPP_TAG, "Device #1 not found: %d (%s)\n", r1, esp_err_to_name(r1));
        }
        vTaskDelay(1 / portTICK_PERIOD_MS);

        r1 = hx711_read_data(&dev1, &data1);
        if (r1 != ESP_OK)
        {
            ESP_LOGI(SPP_TAG, "Could not read data: %d (%s)\n", r1, esp_err_to_name(r1));
        }

        sprintf(buf, "%d", data1);  //assign data1 to buffer string
        strcat(c, buf);      //append buf to c
        buf[0] = '\0';          // clear buffer for next reading (in case it's less digits)
        strcat(c, ", ");        // append comma
        
        vTaskDelay(1 / portTICK_PERIOD_MS);
        
        //SENSOR 2
        esp_err_t r2 = hx711_wait(&dev2, 500);
        if (r2 != ESP_OK)
        {
            ESP_LOGI(SPP_TAG, "Device #1 not found: %d (%s)\n", r2, esp_err_to_name(r2));
        }
        vTaskDelay(1 / portTICK_PERIOD_MS);

        r2 = hx711_read_data(&dev2, &data1);
        if (r2 != ESP_OK)
        {
            ESP_LOGI(SPP_TAG, "Could not read data: %d (%s)\n", r2, esp_err_to_name(r2));
        }
        
        sprintf(buf, "%d", data1);
        strcat(c, buf);
        buf[0] = '\0';
        strcat(c, ", ");

        vTaskDelay(1 / portTICK_PERIOD_MS);
        
        //SENSOR 3
        esp_err_t r3 = hx711_wait(&dev3, 500);
        if (r3 != ESP_OK)
        {
            ESP_LOGI(SPP_TAG, "Device #1 not found: %d (%s)\n", r3, esp_err_to_name(r3));
        }

        vTaskDelay(1 / portTICK_PERIOD_MS);

        r3 = hx711_read_data(&dev3, &data1);
        if (r3 != ESP_OK)
        {
            ESP_LOGI(SPP_TAG, "Could not read data: %d (%s)\n", r3, esp_err_to_name(r3));
        }

        sprintf(buf, "%d", data1);
        strcat(c, buf);
        buf[0] = '\0';
        strcat(c, ", ");

        vTaskDelay(1 / portTICK_PERIOD_MS);
        
        //SENSOR 5
        esp_err_t r5 = hx711_wait(&dev5, 500);
        if (r5 != ESP_OK)
        {
            ESP_LOGI(SPP_TAG, "Device #1 not found: %d (%s)\n", r5, esp_err_to_name(r5));
        }

        vTaskDelay(1 / portTICK_PERIOD_MS);

        r5 = hx711_read_data(&dev5, &data1);
        if (r5 != ESP_OK)
        {
            ESP_LOGI(SPP_TAG, "Could not read data: %d (%s)\n", r5, esp_err_to_name(r5));
        }

        sprintf(buf, "%d", data1);
        strcat(c, buf);
        buf[0] = '\0';
        strcat(c, ", ");

        vTaskDelay(1 / portTICK_PERIOD_MS);
        
        //SENSOR 6
        esp_err_t r6 = hx711_wait(&dev6, 500);
        if (r6 != ESP_OK)
        {
            ESP_LOGI(SPP_TAG, "Device #1 not found: %d (%s)\n", r6, esp_err_to_name(r6));
        }

        vTaskDelay(1 / portTICK_PERIOD_MS);

        r6 = hx711_read_data(&dev6, &data1);
        if (r6 != ESP_OK)
        {
            ESP_LOGI(SPP_TAG, "Could not read data: %d (%s)\n", r6, esp_err_to_name(r6));
        }       

        sprintf(buf, "%d", data1);
        strcat(c, buf);
        buf[0] = '\0';
        strcat(c, "\n");
        strcat(message, c);
        
        c[0] = '\0';        //clear line buffer for next reading. assign 0 to leading byte

        uxHighWaterMark = uxTaskGetStackHighWaterMark(NULL);

        if (i == MESSAGE_LINES){      
            i = 0;  
            ESP_LOGI(SPP_TAG, "Stack Remaining Size: %d\n", uxHighWaterMark);
            ESP_LOGI(SPP_TAG, "i = %d", i);
            ESP_LOGI(SPP_TAG, "Message: %s", message);
            
            if (bConnectionflag == true){     
                uint8_t * u = (uint8_t *)message;
                esp_spp_write(uart_rx_task_params.handle, strlen(message), u);  
                
            }
            memset(message, 0, sizeof message);     //clear message - assign 0 to every byte
        }
        vTaskDelay(90 / portTICK_PERIOD_MS);
        ESP_LOGI(SPP_TAG, "Message Size: %d\n", strlen(message));
        ESP_LOGI(SPP_TAG, "Line Size: %d\n", strlen(c));
    }
}

void app_main(void){

    ESP_LOGI(SPP_TAG, "void app_main(void) - Start");
    
    vTaskDelay(1000 / portTICK_PERIOD_MS);

    startClassicBtSpp();
    xTaskCreate(&transmit_task, "transmit data", 4048, "transmit_task", 2, NULL);

    ESP_LOGI(SPP_TAG, "void app_main(void) - End\n");
}
