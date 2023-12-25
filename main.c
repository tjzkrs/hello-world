/*!
    \file    main.c
    \brief   USART transmit and receive interrupt

    \version 2023-08-17, V1.3.1, demo for GD32E50x
*/

/*
    Copyright (c) 2023, GigaDevice Semiconductor Inc.

    Redistribution and use in source and binary forms, with or without modification, 
are permitted provided that the following conditions are met:

    1. Redistributions of source code must retain the above copyright notice, this 
       list of conditions and the following disclaimer.
    2. Redistributions in binary form must reproduce the above copyright notice, 
       this list of conditions and the following disclaimer in the documentation 
       and/or other materials provided with the distribution.
    3. Neither the name of the copyright holder nor the names of its contributors 
       may be used to endorse or promote products derived from this software without 
       specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED 
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. 
IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, 
INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT 
NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR 
PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, 
WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY 
OF SUCH DAMAGE.
*/

#include "gd32e50x.h"
#include "gd32e503v_eval.h"
#include "systick.h"
#include <stdio.h>
#include "LiBAT.h"
#include "gd25qxx.h"

#define DEV_ADDRESS_ID	0x01
#define DEV_CMD_READ		0x03
#define DEV_CMD_WRITE		0x06
#define SOC_COM	RS485
#define BAT_COM	RS485_2
#define SYS_COM	RS485_4
#define SOC_STARTADDRESS	0x1000
struct LiBAT LiBATPack;

/* transmit buffer and receive buffer */
uint8_t tx_buffer_Bat[20] = {0x7E, 0x32, 0x30, 0x30, 0x30, 0x34, 0x36, 0x34, 0x32, 0x45, 0x30, 0x30, 0x32, 0x30, 0x30, 0x46, 0x44, 0x33, 0x37, 0x0D};
uint8_t tx_buffer_Sys[20] = {0x2A, 0x53, 0x30, 0x7C, 0x00, 0x7C, 0x7C, 0x00, 0x08, 0x7B, 0x01, 0x03, 0x00, 0x00, 0x00, 0x12, 0xC5, 0xC7, 0x7D, 0x23};
uint8_t rx_buffer[COMn][BUFFER_SIZE];
uint8_t timerun[COMn] = {0}, timeout[COMn] = {0}, resetframe[COMn] = {0};
/* counter of transmit buffer and receive buffer */
__IO uint8_t  rx_count[COMn];
/* size of transmit buffer and receive buffer */
uint32_t  rx_buffer_size[COMn] = {0};
uint32_t  tx_buffer_size = BUFFER_SIZE;
uint8_t holdingregister[30]={0};
uint8_t flash_write_buffer[10] = {0};
uint8_t Libat_flag;
/* result of the transfer */
__IO ErrStatus transfer_status = ERROR; 

void nvic_config(void);
ErrStatus memory_compare(uint8_t* src, uint8_t* dst, uint16_t length);
uint8_t	timer_update_flag, timer_sample_flag, init_flag = 0;
void AnalyzePro(com_typedef_enum com_num, uint8_t start_num);
void AnalyzeCV(com_typedef_enum com_num, uint8_t start_num);

/**
    \brief      configure the TIMER peripheral
    \param[in]  none
    \param[out] none
    \retval     none
  */
void Timer2_Init(uint16_t arr,uint16_t psc)
{
	timer_parameter_struct timer2_parameter;
	//时钟使能
	rcu_periph_clock_enable(RCU_TIMER2);
	//定时器配置
	timer_deinit(TIMER2);
	timer2_parameter.period = arr - 1;
	timer2_parameter.prescaler = psc - 1;
	timer2_parameter.clockdivision = TIMER_CKDIV_DIV1;
	timer2_parameter.alignedmode = TIMER_COUNTER_EDGE;
	timer2_parameter.counterdirection = TIMER_COUNTER_UP;
	timer2_parameter.repetitioncounter = 0;
	timer_init(TIMER2,&timer2_parameter);
	//中断配置
	nvic_irq_enable(TIMER2_IRQn,1,1);
	timer_interrupt_flag_clear(TIMER2,TIMER_INT_FLAG_UP);
	timer_interrupt_enable(TIMER2,TIMER_INT_UP);
	timer_enable(TIMER2);
}

void Timer3_Init(uint16_t arr,uint16_t psc)
{
	timer_parameter_struct timer3_parameter;
	//时钟使能
	rcu_periph_clock_enable(RCU_TIMER3);
	//定时器配置
	timer_deinit(TIMER3);
	timer3_parameter.period = arr - 1;
	timer3_parameter.prescaler = psc - 1;
	timer3_parameter.clockdivision = TIMER_CKDIV_DIV1;
	timer3_parameter.alignedmode = TIMER_COUNTER_EDGE;
	timer3_parameter.counterdirection = TIMER_COUNTER_UP;
	timer3_parameter.repetitioncounter = 0;
	timer_init(TIMER3,&timer3_parameter);
	//中断配置
	nvic_irq_enable(TIMER3_IRQn,1,1);
	timer_interrupt_flag_clear(TIMER3,TIMER_INT_FLAG_UP);
	timer_interrupt_enable(TIMER3,TIMER_INT_UP);
	//timer_enable(TIMER3);
}

void Timer4_Init(uint16_t arr,uint16_t psc)
{
	timer_parameter_struct timer4_parameter;
	//时钟使能
	rcu_periph_clock_enable(RCU_TIMER4);
	//定时器配置
	timer_deinit(TIMER4);
	timer4_parameter.period = arr - 1;
	timer4_parameter.prescaler = psc - 1;
	timer4_parameter.clockdivision = TIMER_CKDIV_DIV1;
	timer4_parameter.alignedmode = TIMER_COUNTER_EDGE;
	timer4_parameter.counterdirection = TIMER_COUNTER_UP;
	timer4_parameter.repetitioncounter = 0;
	timer_init(TIMER4,&timer4_parameter);
	//中断配置
	nvic_irq_enable(TIMER4_IRQn,1,1);
	timer_interrupt_flag_clear(TIMER4,TIMER_INT_FLAG_UP);
	timer_interrupt_enable(TIMER4,TIMER_INT_UP);
	timer_enable(TIMER4);
}

uint16_t mbcrc16(uint8_t *buf, uint8_t len)
{
		unsigned int crc = 0xFFFF;
		for (int pos = 0; pos < len; pos++)
		{
				crc ^= (unsigned int)buf[pos]; // XOR byte into least sig. byte of crc
				for (int i = 8; i != 0; i--)   // Loop over each bit
				{
						if ((crc & 0x0001) != 0)   // If the LSB is set
						{
								crc >>= 1; // Shift right and XOR 0xA001
								crc ^= 0xA001;
						}
						else // LSB is not set
						{
								crc >>= 1;    // Just shift right
						}
				}
		}
		return crc;
}

/*!
    \brief      main function
    \param[in]  none
    \param[out] none
    \retval     none
*/
int main(void)
{
		uint8_t i, j, byte_read, count_start[COMn];
		uint8_t soc_rx_buffer[8];
		uint8_t soc_tx_buffer[25];
		uint16_t crc_recv, crc_calc, soc_start_address;
		uint16_t timer_flash_flag;

    systick_config();
	
		timer_sample_flag = 0;
		timer_update_flag = 0;
		Libat_flag = 1;//是锂电池还是铅酸电池
    Timer2_Init(16800, 10000);
    Timer3_Init(16800, 10000);
    Timer4_Init(16800, 10);
		timer_flash_flag = 0;

    rcu_periph_clock_enable(RCU_AF);		
		gd_eval_com_init(RS485,				9600,			0,1);		//rs485		
		gd_eval_com_init(RS485_2,			9600,			0,2);		//rs485		
		//gd_eval_com_init(WIFI_BLE,		115200,		0,3);		//wifi&ble
    gd_eval_com_init(RS485_4,			9600,			0,4);		//rs485		
		//gd_eval_com_init(METER,				9600,			0,5);		//meter
		//gd_eval_com_init(G4,					115200,		0,6);		//4G
	
    /* configure SPI GPIO and parameter */
		spi_flash_init();
		
    while(1)
		{
				if(resetframe[SOC_COM] == 1)
				{
						count_start[SOC_COM] = (rx_count[SOC_COM] + BUFFER_SIZE - 8) & 0xffff;
						if(rx_buffer_size[SOC_COM] == 8 && rx_buffer[SOC_COM][count_start[SOC_COM]] == DEV_ADDRESS_ID)
						{
								for(i = 0; i < 8; i++)
								{
										soc_rx_buffer[i] = rx_buffer[SOC_COM][count_start[SOC_COM]++];
								}
								crc_recv = mbcrc16(soc_rx_buffer, 6);
								crc_calc = ((uint16_t)soc_rx_buffer[7] << 8) | soc_rx_buffer[6];
								if(crc_recv == crc_calc)
								{
										soc_start_address = SOC_STARTADDRESS;
										if(soc_rx_buffer[1] == DEV_CMD_READ && soc_rx_buffer[2] == (uint8_t)(soc_start_address >> 8) && soc_rx_buffer[3] + soc_rx_buffer[5] <= 15)//modbus读
										{
												soc_tx_buffer[0] = DEV_ADDRESS_ID;
												soc_tx_buffer[1] = DEV_CMD_READ;
												j = soc_rx_buffer[3] << 1;
												byte_read = soc_rx_buffer[5] << 1;
												soc_tx_buffer[2] = byte_read;
												for(i = 3; i < byte_read + 3; i = i + 2)
												{
														soc_tx_buffer[i] = holdingregister[j++];
														soc_tx_buffer[i+1] = holdingregister[j++];											
												}
												crc_calc = mbcrc16(soc_tx_buffer, byte_read + 3);
												soc_tx_buffer[byte_read + 3] = (uint8_t)(crc_calc & 0xff);
												soc_tx_buffer[byte_read + 4] = (uint8_t)(crc_calc >> 8);
												tx_buffer_size = byte_read + 5;
												Com_Send(RS485, soc_tx_buffer, tx_buffer_size);
										}
										else if(soc_rx_buffer[1] == DEV_CMD_WRITE && soc_rx_buffer[2] == (uint8_t)(soc_start_address >> 8) && soc_rx_buffer[3] <= 0x0c && soc_rx_buffer[3] >= 0x0a)//modbus写
										{
												j = soc_rx_buffer[3] << 1;
												holdingregister[j++] = soc_rx_buffer[4];
												holdingregister[j++] = soc_rx_buffer[5];
												if(soc_rx_buffer[3] == 0x0a)
												{
														LiBATPack.Qfull = (soc_rx_buffer[4] << 8) | soc_rx_buffer[5];
												}
												else if(soc_rx_buffer[3] == 0x0b)
												{
														LiBATPack.Vfull = (soc_rx_buffer[4] << 8) | soc_rx_buffer[5];
												}
												else
												{
														LiBATPack.Cfull = (soc_rx_buffer[4] << 8) | soc_rx_buffer[5];
												}
												for(i = 0; i < 8; i++)
												{
														soc_tx_buffer[i] = soc_rx_buffer[i];
												}
												/* erases the specified flash sector */
												spi_flash_sector_erase(FLASH_WRITE_ADDRESS);
												/* write tx_buffer data to the flash */ 
												spi_flash_buffer_write(holdingregister + 20, FLASH_WRITE_ADDRESS, 6);

												delay_1ms(10);
												tx_buffer_size = 8;
												Com_Send(RS485, soc_tx_buffer, tx_buffer_size);
										}
								}
						}
						rx_buffer_size[SOC_COM] = 0;
						resetframe[SOC_COM] = 0;
				}
				if(resetframe[BAT_COM] == 1)
				{
						if(rx_buffer_size[BAT_COM] == 136)//双登电池协议返回136个字节
						{
								count_start[BAT_COM] = (rx_count[BAT_COM] + BUFFER_SIZE - 136) & 0xffff;
								AnalyzePro(BAT_COM, count_start[BAT_COM]);
						}
						rx_buffer_size[BAT_COM] = 0;
						resetframe[BAT_COM] = 0;
				}
				if(resetframe[SYS_COM] == 1)
				{
						if(rx_buffer_size[SYS_COM] == 41)//系统控制器返回41个字节
						{						
								count_start[SYS_COM] = (rx_count[SYS_COM] + BUFFER_SIZE - 41) & 0xffff;
								AnalyzeCV(SYS_COM, count_start[SYS_COM]);
								if(init_flag == 0)
								{
										LiBAT_Init();
										init_flag = 1;
								}
								else
								{
										LiBAT_I_Integral(1);
								}
						}
						rx_buffer_size[SYS_COM] = 0;
						resetframe[SYS_COM] = 0;
				}
				if(timer_update_flag == 1)
				{
						if(Libat_flag == 1)
						{
								Com_Send(RS485_2,tx_buffer_Bat,20);
								delay_1ms(50);
								if(rx_buffer_size[BAT_COM] == 0)
								{
										Libat_flag = 0;
								}
						}
						Com_Send(RS485_4,tx_buffer_Sys,20);
						timer_update_flag = 0;
						delay_1ms(5);
						timer_flash_flag++;
						if(timer_flash_flag == 600)
						{
								timer_flash_flag = 0;
								spi_flash_sector_erase(FLASH_WRITE_ADDRESS);
								for(i = 0; i < 6; i++)
								{
										flash_write_buffer[i] = holdingregister[20 + i];
								}
								flash_write_buffer[6] = (uint8_t)(LiBATPack.SOC >> 8);
								flash_write_buffer[7] = (uint8_t)(LiBATPack.SOC & 0x00ff);								
								flash_write_buffer[8] = LiBATPack.Vmea[0];
								flash_write_buffer[9] = LiBATPack.Vmea[1];
								spi_flash_buffer_write(flash_write_buffer, FLASH_WRITE_ADDRESS, 10);								
						}
				}
				if(timer_sample_flag == 1)
				{
						timer_sample_flag = 0;
				}
		}
}

void AnalyzePro(com_typedef_enum com_num, uint8_t start_num)
{
		uint8_t i, j, k[96];
		j = start_num + 19;
		for(i = 0; i < 96; i++)
		{
				k[i] = rx_buffer[com_num][j] > 0x39 ? rx_buffer[com_num][j] - 0x37 : rx_buffer[com_num][j] - 0x30;
				j++;
		}
		j = 0;
		for(i = 0;i < 8; i++)
		{
				LiBATPack.Voltage[i] = ((uint16_t)k[j] << 12) | ((uint16_t)k[j + 1] << 8) | (k[j + 2] << 4) | k[j + 3];
				j += 4;
		}
		j += 22;
		LiBATPack.Temp = (((uint16_t)k[j] << 12) | ((uint16_t)k[j + 1] << 8) | (k[j + 2] << 4) | k[j + 3]) - 2731;
		j += 4;
		LiBATPack.Current_read = ((uint16_t)k[j] << 12) | ((uint16_t)k[j + 1] << 8) | (k[j + 2] << 4) | k[j + 3];
		j += 4;
		LiBATPack.Vmax = ((uint16_t)k[j] << 12) | ((uint16_t)k[j + 1] << 8) | (k[j + 2] << 4) | k[j + 3];
		j += 4;
		LiBATPack.Qleft_read = ((uint16_t)k[j] << 12) | ((uint16_t)k[j + 1] << 8) | (k[j + 2] << 4) | k[j + 3];
		j += 6;
		LiBATPack.QBAT = ((uint16_t)k[j] << 12) | ((uint16_t)k[j + 1] << 8) | (k[j + 2] << 4) | k[j + 3];
		j += 4;
		LiBATPack.SOC_read = ((uint16_t)k[j] << 12) | ((uint16_t)k[j + 1] << 8) | (k[j + 2] << 4) | k[j + 3];
		j += 4;
		LiBATPack.Qmax = ((uint16_t)k[j] << 12) | ((uint16_t)k[j + 1] << 8) | (k[j + 2] << 4) | k[j + 3];
		j += 4;
		LiBATPack.Cycletimes = ((uint16_t)k[j] << 12) | ((uint16_t)k[j + 1] << 8) | (k[j + 2] << 4) | k[j + 3];
		j += 4;
		LiBATPack.SOH = ((uint16_t)k[j] << 12) | ((uint16_t)k[j + 1] << 8) | (k[j + 2] << 4) | k[j + 3];
		j += 4;
		LiBATPack.Vport = ((uint16_t)k[j] << 12) | ((uint16_t)k[j + 1] << 8) | (k[j + 2] << 4) | k[j + 3];

		holdingregister[8] = (uint8_t)(LiBATPack.Temp >> 8);
		holdingregister[9] = (uint8_t)LiBATPack.Temp;
		holdingregister[10] = (uint8_t)(LiBATPack.QBAT >> 8);
		holdingregister[11] = (uint8_t)LiBATPack.QBAT;
		holdingregister[14] = (uint8_t)(LiBATPack.Qleft_read >> 8);
		holdingregister[15] = (uint8_t)LiBATPack.Qleft_read;
		holdingregister[18] = (uint8_t)(LiBATPack.SOC_read >> 8);
		holdingregister[19] = (uint8_t)LiBATPack.SOC_read;
		holdingregister[28] = (uint8_t)(LiBATPack.Vport >> 8);
		holdingregister[29] = (uint8_t)LiBATPack.Vport;
}

void AnalyzeCV(com_typedef_enum com_num, uint8_t start_num)
{		
		uint8_t i, j;
		j = start_num + 3;
		LiBATPack.Vmea[0] = rx_buffer[com_num][j++];//头16个16进制数表示8组小数点前3位数
		LiBATPack.Vmea[1] = rx_buffer[com_num][j++];
		LiBATPack.Ccharge[0] = rx_buffer[com_num][j++];
		LiBATPack.Ccharge[1] = rx_buffer[com_num][j++];
		LiBATPack.Cdischarge[0] = rx_buffer[com_num][j++];
		LiBATPack.Cdischarge[1] = rx_buffer[com_num][j++];
		j = j + 14;		
		LiBATPack.Vmea[2] = rx_buffer[com_num][j++];//然后是4位00，后16位16进制数表示8组小数点后2位数
		LiBATPack.Vmea[3] = rx_buffer[com_num][j++];
		LiBATPack.Ccharge[2] = rx_buffer[com_num][j++];
		LiBATPack.Ccharge[3] = rx_buffer[com_num][j++];
		LiBATPack.Cdischarge[2] = rx_buffer[com_num][j++];
		LiBATPack.Cdischarge[3] = rx_buffer[com_num][j++];

		LiBATPack.Cchargevalue = ((int16_t)(LiBATPack.Ccharge[0] << 8) | LiBATPack.Ccharge[1]) * 100 + ((int16_t)(LiBATPack.Ccharge[2] << 8) | LiBATPack.Ccharge[3]);
	  LiBATPack.Cdischargevalue = (((int16_t)LiBATPack.Cdischarge[0] << 8) | LiBATPack.Cdischarge[1]) * 100 + (((int16_t)LiBATPack.Cdischarge[2] << 8) | LiBATPack.Cdischarge[3]);
		if(LiBATPack.Cchargevalue < -6000 || LiBATPack.Cchargevalue > 6000)
		{
				LiBATPack.Cchargevalue = 0;
		}
		if(LiBATPack.Cdischargevalue < -6000 || LiBATPack.Cdischargevalue > 6000)//放电电流或充电电流读取值大于60A认为异常
		{
				LiBATPack.Cdischargevalue = 0;
		}
		LiBATPack.Cvalue = LiBATPack.Cchargevalue - LiBATPack.Cdischargevalue;
		LiBATPack.Vvalue = (((int16_t)LiBATPack.Vmea[0] << 8) | LiBATPack.Vmea[1]) / 10;
		LiBATPack.Current = LiBATPack.Cvalue / 100.0;
		holdingregister[0] = (int8_t)(LiBATPack.Cchargevalue >> 8);
		holdingregister[1] = (int8_t)LiBATPack.Cchargevalue;
		holdingregister[2] = (int8_t)(LiBATPack.Cdischargevalue >> 8);
		holdingregister[3] = (int8_t)LiBATPack.Cdischargevalue;
		holdingregister[4] = (int8_t)(LiBATPack.Cvalue >> 8);
		holdingregister[5] = (int8_t)LiBATPack.Cvalue;
		holdingregister[6] = (int8_t)(LiBATPack.Vvalue >> 8);
		holdingregister[7] = (int8_t)LiBATPack.Vvalue;
		if(LiBATPack.Vvalue >= LiBATPack.Vfull && LiBATPack.Vfull != 0 && LiBATPack.Cvalue >= 10 && LiBATPack.Cvalue <= LiBATPack.Cfull && LiBATPack.Cfull != 0 && LiBATPack.work_mode == CHARGE_MODE)//判断电量满，在充电状态电流小于设定值，且大于0.1A
		{
				holdingregister[27] = 1;
				LiBATPack.Qleft = LiBATPack.Qfull;
		}
		else
		{
				holdingregister[27] = 0;
		}
		if(LiBATPack.Cvalue > 30)
		{
				LiBATPack.work_mode = CHARGE_MODE;
		}
		else if(LiBATPack.Cvalue < -30)
		{
				LiBATPack.work_mode = DISCHARGE_MODE;
		}
		else
		{
				LiBATPack.work_mode = WAIT_MODE;
				if(LiBATPack.Cchargevalue > -10 && LiBATPack.Cchargevalue < 10 && LiBATPack.Cdischargevalue > -10 && LiBATPack.Cdischargevalue < 10)
				{
						LiBATPack.Cvalue = 0;						
						LiBATPack.Current = 0;
				}
		}
}