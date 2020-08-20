#include <stdbool.h>
#include "spi1.h"

#include "FreeRTOS.h"
#include "semphr.h"

#define DUMMY_BYTE         0xA5

static bool isInit = false;

static SemaphoreHandle_t txComplete;
static SemaphoreHandle_t rxComplete;
//static SemaphoreHandle_t spiMutex;


static const uint16_t divisorMap[] = { 
    SPI_BaudRatePrescaler_256,    // SPI_CLOCK_INITIALIZATON      281.25 KBits/s
    SPI_BaudRatePrescaler_128,    // SPI_CLOCK_SLOW               562.5 KBits/s
    SPI_BaudRatePrescaler_8,      // SPI_CLOCK_STANDARD           9.0 MBits/s
    SPI_BaudRatePrescaler_4,      // SPI_CLOCK_FAST               18.0 MBits/s
    SPI_BaudRatePrescaler_4       // SPI_CLOCK_ULTRAFAST          18.0 MBits/s
};

static void spi1DMAInit(void)
{
	DMA_InitTypeDef  DMA_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);
	
	//配置DMA初始化结构体
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable ;
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull ;
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single ;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
	DMA_InitStructure.DMA_PeripheralBaseAddr =(uint32_t) (&(SPI1->DR)) ;
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;
	DMA_InitStructure.DMA_BufferSize = 0; // set later
	DMA_InitStructure.DMA_Memory0BaseAddr = 0; // set later

	//初始化TX DMA
	DMA_InitStructure.DMA_Channel = DMA_Channel_3;
	DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;
	DMA_Cmd(DMA2_Stream5, DISABLE);
	DMA_Init(DMA2_Stream5, &DMA_InitStructure);

	//初始化RX DMA
	DMA_InitStructure.DMA_Channel = DMA_Channel_3;
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
	DMA_Cmd(DMA2_Stream0,DISABLE);
	DMA_Init(DMA2_Stream0, &DMA_InitStructure);

	//DMA中断配置
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 7;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;

	NVIC_InitStructure.NVIC_IRQChannel = DMA2_Stream0_IRQn;
	NVIC_Init(&NVIC_InitStructure);

	NVIC_InitStructure.NVIC_IRQChannel = DMA2_Stream5_IRQn;
	NVIC_Init(&NVIC_InitStructure);
}

void spi1Init(void)
{
	if (isInit) return;
	
	GPIO_InitTypeDef GPIO_InitStructure;
	SPI_InitTypeDef SPI_InitStructure;
	
	txComplete = xSemaphoreCreateBinary();
	rxComplete = xSemaphoreCreateBinary();
//	spiMutex = xSemaphoreCreateMutex();
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);
	
	//配置SPI1的SCK(PA5),MISO(PA6),MOSI(PA7)引脚
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource5, GPIO_AF_SPI1);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource6, GPIO_AF_SPI1);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource7, GPIO_AF_SPI1);
	
	//初始化SPI1 TX DMA RX DMA
	spi1DMAInit();
	
	//初始化SPI1
	SPI_I2S_DeInit(SPI1);
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
	SPI_InitStructure.SPI_CRCPolynomial = 7;
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_128;
	SPI_Init(SPI1, &SPI_InitStructure);
	
	isInit = true;
	
//	SPI_Cmd(SPI1, ENABLE);
}

bool spi1Transfer(uint8_t *out, const uint8_t *in, int length)
{
	DMA2_Stream0->M0AR = (uint32_t)in;
	DMA2_Stream0->NDTR = length;

	DMA2_Stream5->M0AR = (uint32_t)out;
	DMA2_Stream5->NDTR = length;
	
	//使能DMA传输完成中断
	DMA_ITConfig(DMA2_Stream0, DMA_IT_TC, ENABLE);
	DMA_ITConfig(DMA2_Stream5, DMA_IT_TC, ENABLE);

	//清楚DMA标志位
	DMA_ClearFlag(DMA2_Stream0, DMA_FLAG_FEIF5|DMA_FLAG_DMEIF5|DMA_FLAG_TEIF5|DMA_FLAG_HTIF5|DMA_FLAG_TCIF5);
	DMA_ClearFlag(DMA2_Stream5, DMA_FLAG_FEIF0|DMA_FLAG_DMEIF0|DMA_FLAG_TEIF0|DMA_FLAG_HTIF0|DMA_FLAG_TCIF0);

	//使能DMA数据流
	DMA_Cmd(DMA2_Stream0,ENABLE);
	DMA_Cmd(DMA2_Stream5,ENABLE);

	//使能SPI1 DMA请求
	SPI_I2S_DMACmd(SPI1, SPI_I2S_DMAReq_Tx, ENABLE);
	SPI_I2S_DMACmd(SPI1, SPI_I2S_DMAReq_Rx, ENABLE);

	//使能SPI1
	SPI_Cmd(SPI1, ENABLE);

	//等待传输完成
	bool result = (xSemaphoreTake(txComplete, portMAX_DELAY) == pdTRUE)
			 && (xSemaphoreTake(rxComplete, portMAX_DELAY) == pdTRUE);

	//失能SPI1
	SPI_Cmd(SPI1, DISABLE);
	return result;
}

void spi1SetSpeed(SPIClockSpeed_e speed)
{
#define BR_CLEAR_MASK 0xFFC7
    SPI_Cmd(SPI1, DISABLE);

    uint16_t tempRegister = SPI1->CR1;
    tempRegister &= BR_CLEAR_MASK;
    tempRegister |= divisorMap[speed];
    SPI1->CR1 = tempRegister;
    SPI_Cmd(SPI1, ENABLE);
}

void DMA2_Stream5_IRQHandler(void)
{
	portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;

	//停止DMA数据流
	DMA_ITConfig(DMA2_Stream5, DMA_IT_TC, DISABLE);
	DMA_ClearITPendingBit(DMA2_Stream5, DMA_IT_TCIF5);

	//清除标志位
	DMA_ClearFlag(DMA2_Stream5,DMA_IT_TCIF5);

	//禁止SPI1 DMA请求
	SPI_I2S_DMACmd(SPI1, SPI_I2S_DMAReq_Tx, DISABLE);

	//禁止DMA数据流
	DMA_Cmd(DMA2_Stream5,DISABLE);

	//释放传输完成信号量
	xSemaphoreGiveFromISR(txComplete, &xHigherPriorityTaskWoken);

	if (xHigherPriorityTaskWoken)
	{
		portYIELD();
	}
}

void DMA2_Stream0_IRQHandler(void)
{
	portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;

	//停止DMA数据流
	DMA_ITConfig(DMA2_Stream0, DMA_IT_TC, DISABLE);
	DMA_ClearITPendingBit(DMA2_Stream0, DMA_IT_TCIF0);

	//清除标志位
	DMA_ClearFlag(DMA2_Stream0,DMA_IT_TCIF0);

	//禁止SPI1 DMA请求
	SPI_I2S_DMACmd(SPI1, SPI_I2S_DMAReq_Rx, DISABLE);

	//禁止DMA数据流
	DMA_Cmd(DMA2_Stream0,DISABLE);

	//释放传输完成信号量
	xSemaphoreGiveFromISR(rxComplete, &xHigherPriorityTaskWoken);

	if (xHigherPriorityTaskWoken)
	{
		portYIELD();
	}
}

u8 spiTransferByte(u8 data)
{
    uint16_t spiTimeout = 1000;

    while(SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET)
	{
		if((spiTimeout--)==0)
			return false;
	}  
    SPI_SendData(SPI1, data);

	spiTimeout = 1000;
    while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET)
	{
		if ((spiTimeout--) == 0)
            return false;
	}
    return ((u8)SPI_ReceiveData(SPI1));
}

bool spiTransfer(uint8_t *out, const uint8_t *in, int len)
{
	uint16_t spiTimeout = 1000;

	SPI1->DR;
	while (len--) 
	{
		uint8_t b = in ? *(in++) : 0xFF;
		while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET) {
			if ((spiTimeout--) == 0)
				return false;;
		}
		SPI_SendData(SPI1, b);

		spiTimeout = 1000;
		while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET) {
			if ((spiTimeout--) == 0)
				return false;
		}
		b = SPI_ReceiveData(SPI1);
		if (out)
			*(out++) = b;
	}
	return true;
}
