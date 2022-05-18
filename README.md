
Sprocket Firmware
======================

Reads data sequentially from HX711 modules and transmits over bluetooth. 

FreeRTOS: Data task on core1, BT transfer occurs on core0 (task started within esp_bt.c)

Further data processing upstream. 
