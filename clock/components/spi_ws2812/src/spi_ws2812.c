

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "spi_ws2812.h"
#include "esp_event.h"


#define DMA_CHAN    2

#define PIN_NUM_MOSI 22

#define TH           0xF8 //11111000
#define TL          0XE0 //11100000

static spi_device_handle_t spi;

//发送数据
void send_data(const uint8_t *data, int len)
{
    esp_err_t ret;
    spi_transaction_t t;
    if (len==0) return;             //no need to send anything
    memset(&t, 0, sizeof(t));       //Zero out the transaction
    t.length=len*8;                 //Len is in bytes, transaction length is in bits.
    t.tx_buffer=data;               //Data
    t.user=(void*)1;                //D/C needs to be set to 1
    ret=spi_device_polling_transmit(spi, &t);  //Transmit!
    assert(ret==ESP_OK);            //Should have had no issues.
}

//将输入的RGB格式的颜色转化为ws2812灯带的GRB格式
uint32_t changToGRB(uint32_t rgb){
    return (rgb & 0x000000FF) |
    ((rgb << 8) & 0x00FF0000) |
    ((rgb >> 8) & 0x0000FF00) ;
}

//通过颜色数组设置颜色
void setLedColorByArr( uint32_t len , uint32_t *colorArr ){
    for(int i = 0 ; i<len; i++){
        uint32_t cr = changToGRB(colorArr[i]);
        uint32_t mask = 0x800000;
        uint8_t data[24];
        for(int j = 0 ; j<24; j++){
            data[j] = (mask & cr) ? TH : TL;
            mask>>=1;
        }
        send_data(data,24);
    }
}

void spi_ws2812_init()
{
    esp_err_t ret;
    spi_bus_config_t buscfg={
        .miso_io_num=-1,
        .mosi_io_num=PIN_NUM_MOSI,
        .sclk_io_num=-1,
        .quadwp_io_num=-1,
        .quadhd_io_num=-1,
        .max_transfer_sz=10000
    };
    spi_device_interface_config_t devcfg={
        .clock_speed_hz=8*1000*1000,           //Clock out at 8 MHz
        .mode=0,                                //SPI mode 0
        .spics_io_num=-1,               //CS pin
        .queue_size=7,                          //We want to be able to queue 7 transactions at a time

    };

    ret=spi_bus_initialize(HSPI_HOST, &buscfg, DMA_CHAN);
    ESP_ERROR_CHECK(ret);

    ret=spi_bus_add_device(HSPI_HOST, &devcfg, &spi);
    ESP_ERROR_CHECK(ret);
}