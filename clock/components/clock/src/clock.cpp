#include "clock.h"
#include "spi_ws2812.h"
#include <iostream>
#include <stdio.h>
#include <stdint.h>
#include "sdkconfig.h"
#include "esp_system.h"
#include "esp_err.h"
#include "driver/i2c.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "DS1307.h"

#define DS1307_I2C_NUM I2C_NUM_1
#define DS1307_I2C_RATE 100000
#define DS1307_SCL_GPIO GPIO_NUM_13
#define DS1307_SDA_GPIO GPIO_NUM_14


int8_t
DS1307_Platform_Init(void)
{
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = DS1307_SDA_GPIO;
    conf.sda_pullup_en = GPIO_PULLUP_DISABLE;
    conf.scl_io_num = DS1307_SCL_GPIO;
    conf.scl_pullup_en = GPIO_PULLUP_DISABLE;
    conf.master.clk_speed = DS1307_I2C_RATE;
    if (i2c_param_config(DS1307_I2C_NUM, &conf) != ESP_OK)
        return -1;
    if (i2c_driver_install(DS1307_I2C_NUM, conf.mode, 0, 0, 0) != ESP_OK)
        return -1;
    return 0;
}

int8_t
DS1307_Platform_DeInit(void)
{
    i2c_driver_delete(DS1307_I2C_NUM);
    gpio_reset_pin(DS1307_SDA_GPIO);
    gpio_reset_pin(DS1307_SCL_GPIO);
    return 0;
}

int8_t
DS1307_Platform_Send(uint8_t Address, uint8_t *Data, uint8_t DataLen)
{
    i2c_cmd_handle_t DS1307_i2c_cmd_handle = 0;
    Address <<= 1;
    Address &= 0xFE;

    DS1307_i2c_cmd_handle = i2c_cmd_link_create();
    i2c_master_start(DS1307_i2c_cmd_handle);
    i2c_master_write(DS1307_i2c_cmd_handle, &Address, 1, 1);
    i2c_master_write(DS1307_i2c_cmd_handle, Data, DataLen, 1);
    i2c_master_stop(DS1307_i2c_cmd_handle);
    if (i2c_master_cmd_begin(DS1307_I2C_NUM, DS1307_i2c_cmd_handle, 1000 / portTICK_RATE_MS) != ESP_OK)
    {
        i2c_cmd_link_delete(DS1307_i2c_cmd_handle);
        return -1;
    }
    i2c_cmd_link_delete(DS1307_i2c_cmd_handle);
    return 0;
}

int8_t
DS1307_Platform_Receive(uint8_t Address, uint8_t *Data, uint8_t DataLen)
{
    i2c_cmd_handle_t DS1307_i2c_cmd_handle = 0;
    Address <<= 1;
    Address |= 0x01;

    DS1307_i2c_cmd_handle = i2c_cmd_link_create();
    i2c_master_start(DS1307_i2c_cmd_handle);
    i2c_master_write(DS1307_i2c_cmd_handle, &Address, 1, 1);
    i2c_master_read(DS1307_i2c_cmd_handle, Data, DataLen, I2C_MASTER_LAST_NACK);
    i2c_master_stop(DS1307_i2c_cmd_handle);
    if (i2c_master_cmd_begin(DS1307_I2C_NUM, DS1307_i2c_cmd_handle, 1000 / portTICK_RATE_MS) != ESP_OK)
    {
        i2c_cmd_link_delete(DS1307_i2c_cmd_handle);
        return -1;
    }
    i2c_cmd_link_delete(DS1307_i2c_cmd_handle);
    return 0;
}

void TaskShowTime(DS1307_DateTime_t *DateTime0 = NULL)
{
    DS1307_Handler_t Handler;

    Handler.PlatformInit = DS1307_Platform_Init;
    Handler.PlatformDeInit = DS1307_Platform_DeInit;
    Handler.PlatformSend = DS1307_Platform_Send;
    Handler.PlatformReceive = DS1307_Platform_Receive;
    DS1307_Init(&Handler);
    DS1307_DateTime_t DateTime_begin;
    //校时
    if (DateTime0 != NULL)//联网校时
    {
        DateTime_begin = *DateTime0;
    }
    else//无网络，使用eerom34c32中时间
    {
        u_int8_t TIME_read[7];
        DS1307_ReadRAM(&Handler, 0, TIME_read, 7);
        DateTime_begin =
            {
                .Second = TIME_read[6],
                .Minute = TIME_read[5],
                .Hour = TIME_read[4],
                .WeekDay = TIME_read[3],
                .Day = TIME_read[2],
                .Month = TIME_read[1],
                .Year = TIME_read[0]};
    }
    DS1307_SetDateTime(&Handler, &DateTime_begin);
    DS1307_SetOutWave(&Handler, DS1307_OutWave_1Hz);
    for (;;)
    {
        DS1307_DateTime_t DateTime_now;
        DS1307_GetDateTime(&Handler, &DateTime_now);
        if(TIME_is_equal(DateTime_now,DateTime_begin))
        {

        }

    }
}