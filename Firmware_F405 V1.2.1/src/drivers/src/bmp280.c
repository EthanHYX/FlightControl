#include <math.h>
#include "stdbool.h"
#include "delay.h"
#include "config.h"
#include "bmp280.h"
#include "i2cdev.h"

/********************************************************************************	 
 * 本程序只供学习使用，未经作者许可，不得用于其它任何用途
 * ATKflight飞控固件
 * BMP280驱动代码	
 * 正点原子@ALIENTEK
 * 技术论坛:www.openedv.com
 * 创建日期:2018/5/2
 * 版本：V1.0
 * 版权所有，盗版必究。
 * Copyright(C) 广州市星翼电子科技有限公司 2014-2024
 * All rights reserved
********************************************************************************/

/*配置bmp280气压和温度过采样 工作模式*/
#define BMP280_PRESSURE_OSR         (BMP280_OVERSAMP_16X)
#define BMP280_TEMPERATURE_OSR      (BMP280_OVERSAMP_4X)
#define BMP280_MODE                 (BMP280_PRESSURE_OSR << 2 | BMP280_TEMPERATURE_OSR << 5 | BMP280_NORMAL_MODE) //

/*配置bmp280气压IIR滤波器*/
#define BMP280_FILTER               (4 << 2)	// BMP280_FILTER_COEFF_16

#define BMP280_DATA_FRAME_SIZE      (6)

typedef struct 
{
    u16 dig_T1;	/* calibration T1 data */
    s16 dig_T2; /* calibration T2 data */
    s16 dig_T3; /* calibration T3 data */
    u16 dig_P1;	/* calibration P1 data */
    s16 dig_P2; /* calibration P2 data */
    s16 dig_P3; /* calibration P3 data */
    s16 dig_P4; /* calibration P4 data */
    s16 dig_P5; /* calibration P5 data */
    s16 dig_P6; /* calibration P6 data */
    s16 dig_P7; /* calibration P7 data */
    s16 dig_P8; /* calibration P8 data */
    s16 dig_P9; /* calibration P9 data */
    s32 t_fine; /* calibration t_fine data */
} bmp280Calib;


bmp280Calib  bmp280Cal;
static bool isInit = false;
static s32 bmp280RawPressure = 0;
static s32 bmp280RawTemperature = 0;

static void bmp280GetPressure(void);

bool bmp280Init(void)
{	
	if (isInit) return true;

	i2c1Init();
	
	u8 id = 0x00;
	for (int i=0; i<10; i++)
	{
		i2cdevReadByte(I2C1_DEV, BMP280_I2C_ADDR, BMP280_CHIP_ID, &id);//读取ID
		
		if (id == BMP280_DEFAULT_CHIP_ID)
		{
			break;
		}
		delay_ms(10);
	}
	
	if (id == BMP280_DEFAULT_CHIP_ID)//读取正常
	{
		i2cdevRead(I2C1_DEV, BMP280_I2C_ADDR, BMP280_TEMPERATURE_CALIB_DIG_T1_LSB_REG, 24, (u8*)&bmp280Cal);//读取校准数据
		i2cdevWriteByte(I2C1_DEV, BMP280_I2C_ADDR, BMP280_CTRL_MEAS_REG, BMP280_MODE);//设置过采样率和工作模式
		i2cdevWriteByte(I2C1_DEV, BMP280_I2C_ADDR, BMP280_CONFIG_REG, BMP280_FILTER);//配置IIR滤波
		
		isInit = true;
		
//		printf("BMP280 I2C connection [OK].\n");
//		printf("BMP280 Calibrate Registor Are: \r\n");
//		for(i=0;i<24;i++)
//			printf("Registor %2d: 0x%X\n",i,p[i]);
	}
    else
	{
//		printf("BMP280 I2C connection [FAIL].\n");
	}

    return isInit;
}

static void bmp280GetPressure(void)
{
    u8 data[BMP280_DATA_FRAME_SIZE];
//	u8 status;
//	i2cdevReadByte(I2C1_DEV, BMP280_I2C_ADDR, BMP280_STAT_REG, &status);
//	if(status & 0x08)
	{	
		i2cdevRead(I2C1_DEV, BMP280_I2C_ADDR, BMP280_PRESSURE_MSB_REG, BMP280_DATA_FRAME_SIZE, data);
		bmp280RawPressure = (s32)((((uint32_t)(data[0])) << 12) | (((uint32_t)(data[1])) << 4) | ((uint32_t)data[2] >> 4));
		bmp280RawTemperature = (s32)((((uint32_t)(data[3])) << 12) | (((uint32_t)(data[4])) << 4) | ((uint32_t)data[5] >> 4));
	}
}

// Returns temperature in DegC, resolution is 0.01 DegC. Output value of "5123" equals 51.23 DegC
// t_fine carries fine temperature as global value
u32 bmp280CompensateT(s32 adcT)
{
    s32 var1, var2, T;

    var1 = ((((adcT >> 3) - ((s32)bmp280Cal.dig_T1 << 1))) * ((s32)bmp280Cal.dig_T2)) >> 11;
    var2  = (((((adcT >> 4) - ((s32)bmp280Cal.dig_T1)) * ((adcT >> 4) - ((s32)bmp280Cal.dig_T1))) >> 12) * ((s32)bmp280Cal.dig_T3)) >> 14;
    bmp280Cal.t_fine = var1 + var2;
    T = (bmp280Cal.t_fine * 5 + 128) >> 8;

    return T;
}

// Returns pressure in Pa as unsigned 32 bit integer in Q24.8 format (24 integer bits and 8 fractional bits).
// Output value of "24674867" represents 24674867/256 = 96386.2 Pa = 963.862 hPa
u32 bmp280CompensateP(s32 adcP)
{
    int64_t var1, var2, p;
    var1 = ((int64_t)bmp280Cal.t_fine) - 128000;
    var2 = var1 * var1 * (int64_t)bmp280Cal.dig_P6;
    var2 = var2 + ((var1*(int64_t)bmp280Cal.dig_P5) << 17);
    var2 = var2 + (((int64_t)bmp280Cal.dig_P4) << 35);
    var1 = ((var1 * var1 * (int64_t)bmp280Cal.dig_P3) >> 8) + ((var1 * (int64_t)bmp280Cal.dig_P2) << 12);
    var1 = (((((int64_t)1) << 47) + var1)) * ((int64_t)bmp280Cal.dig_P1) >> 33;
    if (var1 == 0)
        return 0;
    p = 1048576 - adcP;
    p = (((p << 31) - var2) * 3125) / var1;
    var1 = (((int64_t)bmp280Cal.dig_P9) * (p >> 13) * (p >> 13)) >> 25;
    var2 = (((int64_t)bmp280Cal.dig_P8) * p) >> 19;
    p = ((p + var1 + var2) >> 8) + (((int64_t)bmp280Cal.dig_P7) << 4);
    return (uint32_t)p;
}

void bmp280GetData(float* pressure, float* temperature)
{
	bmp280GetPressure();
	
	*temperature = bmp280CompensateT(bmp280RawTemperature)/100.0f;	/*单位度*/	
	*pressure = bmp280CompensateP(bmp280RawPressure)/256.0f;		/*单位Pa*/	
	
//	*asl=bmp280PressureToAltitude(pressure);	/*转换成海拔*/	
}

#define CONST_PF 0.1902630958	//(1/5.25588f) Pressure factor
#define FIX_TEMP 25				// Fixed Temperature. ASL is a function of pressure and temperature, but as the temperature changes so much (blow a little towards the flie and watch it drop 5 degrees) it corrupts the ASL estimates.
								// TLDR: Adjusting for temp changes does more harm than good.
/**
 * Converts pressure to altitude above sea level (ASL) in meters
 */
float bmp280PressureToAltitude(float* pressure/*, float* groundPressure, float* groundTemp*/)
{
    if (*pressure > 0)
    {
        return ((pow((1015.7f / *pressure), CONST_PF) - 1.0f) * (FIX_TEMP + 273.15f)) / 0.0065f;
    }
    else
    {
        return 0;
    }
}
