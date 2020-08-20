
最新资料下载地址:
	http://www.openedv.com/thread-105197-1-1.html
	
硬件资源:
	1,MCU: STM32F405RGT6 (FLAH:1024K, RAM:196K, 系统运行时钟频率:168MHz)
	2,IMU: MPU6000(通信方式:SPI1, SCK:PA5 \ MISO:PA6 \ MOSI:PA7 \ CS:PC2) 
	3,气压计: BMP280(通讯方式:IIC1, SCL:PB6 \ SDA:PB7)
	4,磁力计接口: HMC5883L(通讯方式:IIC1, SCL:PB6 \ SDA:PB7)
	5,电调信号: PWM或Oneshot125(S1:PC6 \ S2:PC7 \ S3:PC8 \ S4:PC9)
	6,接收机: PPM信号(PA3)
	7,蓝色LED: 运行指示灯(PB9)
	8,红色LED: 未使用(PA14)
	9,电压采集: (ADC1 IN15 PC5)
	10,电流采集: (ADC1 IN14 PC4)
	11,蜂鸣器: (PC13)
	12,WS2812B灯带: (PA15)
	13,USB_SLAVE接口(USB_DM:PA11, USB_DP:PA12)

使用本飞控步骤:
    1,校准加速度计
	2,校准磁力计（如有）
	3,确认接收机遥杆信号是否反向及中点值是否是1500
	4,确认遥控器辅助通道是否正常工作（固件使用AUX2打开关闭蜂鸣器、AUX3切换飞行模式(自稳定高定点三挡)、AUX4切换（有头无头））
	5,电调校准（如之前未校准）
	6,在匿名上位机检查参数是否是默认值
	7,在空旷无人的地方解锁试飞

注意事项:
	1,定点功能还未实现，目前定点就是定高
	2,解锁不成功原因：加速度计磁力计未校准、陀螺仪未放置静止校准通过、飞机倾斜25度以上。

固件更新记录:
	Firmware_F405 V1.2    Release(硬件版本:ATK-F405 V1.2, DATE:2018-09-15)
	Firmware_F405 V1.2.1  Release(硬件版本:ATK-F405 V1.2, DATE:2018-11-24)
	


					正点原子@ALIENTEK
					2018-9-15
					广州市星翼电子科技有限公司
					电话：020-38271790
					传真：020-36773971
					购买：http://shop62103354.taobao.com
					http://shop62057469.taobao.com
					公司网站：www.alientek.com
					技术论坛：www.openedv.com
					
					
					
					
					
					
					
					
					
					
					
					
					
					
					
					
					
					
















