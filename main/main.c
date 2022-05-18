#include <stdint.h>
#include <string.h>
#include <stdbool.h>
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
#include <stdio.h>
#include "esp_log.h"
#include "driver/i2c.h"
#include "sdkconfig.h"
#include <math.h>

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

#define DOUT_GPIO1   18
#define PD_SCK_GPIO1 19
#define DOUT_GPIO2   5
#define PD_SCK_GPIO2 17
#define DOUT_GPIO3   16
#define PD_SCK_GPIO3 4
#define DOUT_GPIO4   33
#define PD_SCK_GPIO4 25
#define DOUT_GPIO5   26
#define PD_SCK_GPIO5 27
#define DOUT_GPIO6   14
#define PD_SCK_GPIO6 12
 
#define PIN_SDA 21
#define PIN_CLK 22
#define I2C_ADDRESS 0x69 // I2C address of MPU6050
#define MPU6050_ACCEL_XOUT_H 0x3B
#define MPU6050_PWR_MGMT_1   0x6B
static char tag[] = "mpu6050"; //9250?

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
        .gain = HX711_GAIN_A_64  //change to 128
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

    hx711_t dev4 = {
        .dout = DOUT_GPIO4,
        .pd_sck = PD_SCK_GPIO4,
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

    int a = 0, b = 0, c = 0, z = 0, x = 0, y = 0, i = 0;
    
    uint8_t IMUdata[20];

	short accel_x;
	short accel_y;
	short accel_z;

	short temperature;

	short gyro_x;
	short gyro_y;
	short gyro_z;

    i2c_cmd_handle_t cmd;  //container for a command sequence 
	
	vTaskDelay(200/portTICK_PERIOD_MS); //not sure why the delay here

	cmd = i2c_cmd_link_create(); //creates/initializes i2c commands list. 
	ESP_ERROR_CHECK(i2c_master_start(cmd));    // ADD START BIT TO CONTAINER
	ESP_ERROR_CHECK(i2c_master_write_byte(cmd, (I2C_ADDRESS << 1) | I2C_MASTER_WRITE, 1)); //ADD THE DEVICE ADDRESS FOLLOWED BY WRITE BIT (0) AND DEMAND ACK
	 
	i2c_master_write_byte(cmd, MPU6050_ACCEL_XOUT_H, 1); //ADD THE REGISTER ADDRESS W/ ACK
	ESP_ERROR_CHECK(i2c_master_stop(cmd)); //ADD THE STOP SIGNAL. FYI THIS SENDS SDA HIGH DURING A HIGH CLK PULSE
	ESP_ERROR_CHECK(i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000/portTICK_PERIOD_MS)); //SEND THE COMMANDS, BLOCKS UNTIL ALL ARE SENT - "I2C APIS ARE NOT THREAD-SAFE"?.... ALSO SHOULD ERROR_CHECK THIS
	i2c_cmd_link_delete(cmd); //FREE THE CMD LISTS. NEEDS TO HAPPEN, REQUIRED BYTES DYNAMICALLY ALLOCATED

	cmd = i2c_cmd_link_create(); 
	ESP_ERROR_CHECK(i2c_master_start(cmd));
	ESP_ERROR_CHECK(i2c_master_write_byte(cmd, (I2C_ADDRESS << 1) | I2C_MASTER_WRITE, 1));
	i2c_master_write_byte(cmd, MPU6050_PWR_MGMT_1, 1); //AGAIN, SET REGISTER ADDRESS W/ ACK
	i2c_master_write_byte(cmd, 0, 1);  //SET THIS ADDRESS TO 0?
	ESP_ERROR_CHECK(i2c_master_stop(cmd));
	i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000/portTICK_PERIOD_MS);
	i2c_cmd_link_delete(cmd);

    //init HX711 devices - break when confirmed
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
        
        esp_err_t r4 = hx711_init(&dev4);
        if (r4 == ESP_OK)
            c = 1;
        ESP_LOGI(SPP_TAG, "Error Status Sensor 4: %d (%s)\n", r4, esp_err_to_name(r4));
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

        a = a + b + c + z + x + y; //Kinda dumb but quick way to make sure all are initialized

        if (a == 6){ //Break if all are good - otherwise try again
            break;
        }
        a = 0;
        b = 0;
        c = 0;
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
        char c[LINE_LENGTH];   //whole line buffer string. expected format  "time (10 = 1s), sensor1 (24 bit int), sens2, sens3..." 
        char buf[12];          //  buffer string for ind'l reading 
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

        //should probably make this a function
        sprintf(buf, "%d", data1);  //assign data1 to buffer string 
        strcat(c, buf);      //append buf to c
        buf[0] = '\0';          // clear buffer for next reading (in case it's less digits)
        strcat(c, ", ");        // append comma
        
        vTaskDelay(1 / portTICK_PERIOD_MS);
        
        //SENSOR 2
        esp_err_t r2 = hx711_wait(&dev2, 500);
        if (r2 != ESP_OK)
        {
            ESP_LOGI(SPP_TAG, "Device #2 not found: %d (%s)\n", r2, esp_err_to_name(r2));
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
            ESP_LOGI(SPP_TAG, "Device #3 not found: %d (%s)\n", r3, esp_err_to_name(r3));
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

        //SENSOR 4
        esp_err_t r4 = hx711_wait(&dev4, 500);
        if (r4 != ESP_OK)
        {
            ESP_LOGI(SPP_TAG, "Device #4 not found: %d (%s)\n", r4, esp_err_to_name(r4));
        }

        vTaskDelay(1 / portTICK_PERIOD_MS);

        r4 = hx711_read_data(&dev4, &data1);
        if (r4 != ESP_OK)
        {
            ESP_LOGI(SPP_TAG, "Could not read data: %d (%s)\n", r4, esp_err_to_name(r4));
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
            ESP_LOGI(SPP_TAG, "Device #5 not found: %d (%s)\n", r5, esp_err_to_name(r5));
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
            ESP_LOGI(SPP_TAG, "Device #6 not found: %d (%s)\n", r6, esp_err_to_name(r6));
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
        strcat(c, ",");

        cmd = i2c_cmd_link_create();
		ESP_ERROR_CHECK(i2c_master_start(cmd));
		ESP_ERROR_CHECK(i2c_master_write_byte(cmd, (I2C_ADDRESS << 1) | I2C_MASTER_WRITE, 1));
		ESP_ERROR_CHECK(i2c_master_write_byte(cmd, MPU6050_ACCEL_XOUT_H, 1));
		ESP_ERROR_CHECK(i2c_master_stop(cmd));
		ESP_ERROR_CHECK(i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000/portTICK_PERIOD_MS));
		i2c_cmd_link_delete(cmd);

		cmd = i2c_cmd_link_create();
		ESP_ERROR_CHECK(i2c_master_start(cmd));
		ESP_ERROR_CHECK(i2c_master_write_byte(cmd, (I2C_ADDRESS << 1) | I2C_MASTER_READ, 1));

        //why isn't this a loop too?
		ESP_ERROR_CHECK(i2c_master_read_byte(cmd, IMUdata,   0)); //accel responds to 'burst' or sequential reads. This one is ACCEL_XOUT_H 0X3B (59)
		ESP_ERROR_CHECK(i2c_master_read_byte(cmd, IMUdata+1, 0)); //read next register... and so on. This one is ACCEL_XOUT_L (60)
		ESP_ERROR_CHECK(i2c_master_read_byte(cmd, IMUdata+2, 0)); //ACCEL_YOUT_H (61)
		ESP_ERROR_CHECK(i2c_master_read_byte(cmd, IMUdata+3, 0)); // ACCEL_YOUT_L (62)
		ESP_ERROR_CHECK(i2c_master_read_byte(cmd, IMUdata+4, 0)); //ACCEL_ZOUT_H (63)
		ESP_ERROR_CHECK(i2c_master_read_byte(cmd, IMUdata+5, 0)); //ACCEL_ZOUT_L (64)
		ESP_ERROR_CHECK(i2c_master_read_byte(cmd, IMUdata+6, 0)); //TEMP_OUT_H (pre-formula) (65)
		ESP_ERROR_CHECK(i2c_master_read_byte(cmd, IMUdata+7, 0)); //TEMP_OUT_L (66)
		ESP_ERROR_CHECK(i2c_master_read_byte(cmd, IMUdata+8, 0)); //GYRO_XOUT_H (67)
		ESP_ERROR_CHECK(i2c_master_read_byte(cmd, IMUdata+9, 0)); //GYRO_XOUT_l (68)
		ESP_ERROR_CHECK(i2c_master_read_byte(cmd, IMUdata+10, 0)); //GYRO_YOUT_H (69)
		ESP_ERROR_CHECK(i2c_master_read_byte(cmd, IMUdata+11, 0)); //GYRO_YOUT_L (70)
		ESP_ERROR_CHECK(i2c_master_read_byte(cmd, IMUdata+12, 0)); //GYRO_ZOUT_H (71)
		ESP_ERROR_CHECK(i2c_master_read_byte(cmd, IMUdata+13, 1)); //GYRO_ZOUT_L (72)
		
		//i2c_master_read(cmd, data, sizeof(data), 1);
		ESP_ERROR_CHECK(i2c_master_stop(cmd));
		ESP_ERROR_CHECK(i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000/portTICK_PERIOD_MS));
		i2c_cmd_link_delete(cmd);

        //INSEAD OF ACCEL_X, ETC. just loop over IMUData[] 
        //OR JUST MAKE A FUNCTION THAT ACCEPTS DATA AND APPENDS TO C

		accel_x = (IMUdata[0] << 8) | IMUdata[1];

        sprintf(buf, "%d", accel_x);
        strcat(c, buf);
        buf[0] = '\0';
        strcat(c, ",");

		accel_y = (IMUdata[2] << 8) | IMUdata[3];

        sprintf(buf, "%d", accel_y);
        strcat(c, buf);
        buf[0] = '\0';
        strcat(c, ",");

		accel_z = (IMUdata[4] << 8) | IMUdata[5];

        sprintf(buf, "%d", accel_z);
        strcat(c, buf);
        buf[0] = '\0';
        strcat(c, ",");

		temperature = (IMUdata[6] << 8) | IMUdata[7];
 
        sprintf(buf, "%d", temperature);
        strcat(c, buf);
        buf[0] = '\0';
        strcat(c, ",");

		gyro_x = (IMUdata[8] << 8) |  IMUdata[9];

        sprintf(buf, "%d", gyro_x);
        strcat(c, buf);
        buf[0] = '\0';
        strcat(c, ",");
		gyro_y = (IMUdata[10] << 8) | IMUdata[11];

        sprintf(buf, "%d", gyro_y);
        strcat(c, buf);
        buf[0] = '\0';
        strcat(c, ",");

		gyro_z = (IMUdata[12] << 8) | IMUdata[13];
        
        sprintf(buf, "%d", gyro_z);
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

    ESP_LOGI(tag, ">> mpu6050");
	i2c_config_t conf = {0};
	conf.mode = I2C_MODE_MASTER;
	conf.sda_io_num = PIN_SDA;
	conf.scl_io_num = PIN_CLK;
	conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
	conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
	conf.master.clk_speed = 400000;
	ESP_ERROR_CHECK(i2c_param_config(I2C_NUM_0, &conf)); //I2C_NUM_0 IS THE PORT (1 OF 2?), CONF IS OBVI POINTER TO CONFIG STRUCTURE
	ESP_ERROR_CHECK(i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0)); //PORT, SLAVE/MASTER MODE, SOME SLAVE BUFFER STUFF THAT'S IGNORED AS MASTER REGARDLESS (X2), AND LAST IS A # OF INTERRUPT FLAGS
    ESP_LOGI(SPP_TAG, "I2C Configured\n");

    xTaskCreate(&transmit_task, "transmit data", 4048, "transmit_task", 2, NULL);

    ESP_LOGI(SPP_TAG, "void app_main(void) - End\n");
}
