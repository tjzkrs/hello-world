/*!
    \file    gd32e50x_it.c
    \brief   interrupt service routines

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

#include "gd32e50x_it.h"
#include "gd32e503v_eval.h"
#include "systick.h"
#include "LiBAT.h"

extern uint8_t  tx_buffer[];
extern uint8_t  rx_buffer[COMn][BUFFER_SIZE];
extern uint32_t rx_buffer_size[COMn], tx_buffer_size;
extern uint8_t	rx_count[COMn];
extern uint8_t	timer_update_flag, timer_sample_flag;
extern uint8_t	timerun[COMn], timeout[COMn], resetframe[COMn];

/*!
    \brief      this function handles NMI exception
    \param[in]  none
    \param[out] none
    \retval     none
*/
void NMI_Handler(void)
{
}

/*!
    \brief      this function handles HardFault exception
    \param[in]  none
    \param[out] none
    \retval     none
*/
void HardFault_Handler(void)
{
    /* if Hard Fault exception occurs, go to infinite loop */
    while(1){
    }
}

/*!
    \brief      this function handles SVC exception
    \param[in]  none
    \param[out] none
    \retval     none
*/
void SVC_Handler(void)
{
}

/*!
    \brief      this function handles PendSV exception
    \param[in]  none
    \param[out] none
    \retval     none
*/
void PendSV_Handler(void)
{
}

/*!
    \brief      this function handles SysTick exception
    \param[in]  none
    \param[out] none
    \retval     none
*/
void SysTick_Handler(void)
{
    delay_decrement();
}

/*!
    \brief      this function handles USART RBNE interrupt request and TBE interrupt request
    \param[in]  none
    \param[out] none
    \retval     none
*/
void USART0_IRQHandler(void)
{
		unsigned char ch = RS485;
    if(RESET != usart_interrupt_flag_get(USART0, USART_INT_FLAG_RBNE))
		{
        /* receive data */
        rx_buffer[ch][rx_count[ch]++] = usart_data_receive(USART0);
				//rx_count[ch] = rx_count[ch]%BUFFER_SIZE;     
				rx_buffer_size[ch]++;
				timeout[ch] = 0;
				if(rx_buffer_size[ch] >= 1)
				{
						timerun[ch] = 1;
						timeout[ch] = 0;
				}
				usart_interrupt_flag_clear(USART0,USART_INT_FLAG_RBNE);			
    }
		
		if(RESET != usart_interrupt_flag_get(USART0, USART_INT_FLAG_RBNE_ORERR))	
		{			
				rx_buffer[ch][rx_count[ch]++] = usart_data_receive(USART0);
				//rx_count[ch] = rx_count[ch]%BUFFER_SIZE;     
				rx_buffer_size[ch]++;
				timeout[ch] = 0;
				if(rx_buffer_size[ch] >= 1)
				{
						timerun[ch] = 1;
						timeout[ch] = 0;
				}
				usart_interrupt_flag_clear(USART0, USART_INT_FLAG_RBNE);									
    } 
}

void USART1_IRQHandler(void)		//COM1  RS485_2
{		
		unsigned char ch = RS485_2;
	  if(RESET != usart_interrupt_flag_get(USART1, USART_INT_FLAG_RBNE))
		{			
				rx_buffer[ch][rx_count[ch]++] = usart_data_receive(USART1);
				//rx_count[ch] = rx_count[ch]%BUFFER_SIZE;     
				rx_buffer_size[ch]++;	
				timeout[ch] = 0;
				if(rx_buffer_size[ch] >= 1)
				{
						timerun[ch] = 1;
						timeout[ch] = 0;
				}
				usart_interrupt_flag_clear(USART1,USART_INT_FLAG_RBNE);
    }
		
		if(RESET != usart_interrupt_flag_get(USART1, USART_INT_FLAG_RBNE_ORERR))	
		{		
				rx_buffer[ch][rx_count[ch]++] = usart_data_receive(USART1);
				//rx_count[ch] = rx_count[ch]%BUFFER_SIZE;     
				rx_buffer_size[ch]++;			
				timeout[ch] = 0;
				if(rx_buffer_size[ch] >= 1)
				{
						timerun[ch] = 1;
						timeout[ch] = 0;
				}
				usart_interrupt_flag_clear(USART1, USART_INT_FLAG_RBNE_ORERR);									
    } 		
}


void USART2_IRQHandler(void)		//COM2  WIFI
{
		unsigned char ch = WIFI_BLE;
	  if(RESET != usart_interrupt_flag_get(USART2, USART_INT_FLAG_RBNE))
		{
				rx_buffer[ch][rx_count[ch]++] = usart_data_receive(USART2);
				//rx_count[ch] = rx_count[ch]%BUFFER_SIZE;     
				rx_buffer_size[ch]++;
				if(rx_buffer_size[ch] >= 1)
				{
						timerun[ch] = 1;
						timeout[ch] = 0;
				}
				usart_interrupt_flag_clear(USART2,USART_INT_FLAG_RBNE);
    }   
		
		if(RESET != usart_interrupt_flag_get(USART2, USART_INT_FLAG_RBNE_ORERR))	
		{			
				rx_buffer[ch][rx_count[ch]++] = usart_data_receive(USART2);
				//rx_count[ch] = rx_count[ch]%BUFFER_SIZE;     
				rx_buffer_size[ch]++;
				if(rx_buffer_size[ch] >= 1)
				{
						timerun[ch] = 1;
						timeout[ch] = 0;
				}							
				usart_interrupt_flag_clear(USART2, USART_INT_FLAG_RBNE_ORERR);									
    } 
}



void UART3_IRQHandler(void)		//COM3   RS485_4
{		
		unsigned char ch = RS485_4;	
	  if(RESET != usart_interrupt_flag_get(UART3, USART_INT_FLAG_RBNE))
		{			
				rx_buffer[ch][rx_count[ch]++] = usart_data_receive(UART3);
				//rx_count[ch] = rx_count[ch]%BUFFER_SIZE;     
				rx_buffer_size[ch]++;
				if(rx_buffer_size[ch] >= 1)
				{
						timerun[ch] = 1;
						timeout[ch] = 0;
				}			
				usart_interrupt_flag_clear(UART3,USART_INT_FLAG_RBNE);
    } 		
		
		if(RESET != usart_interrupt_flag_get(UART3, USART_INT_FLAG_RBNE_ORERR))	
		{		
				rx_buffer[ch][rx_count[ch]++] = usart_data_receive(UART3);
				//rx_count[ch] = rx_count[ch]%BUFFER_SIZE;     
				rx_buffer_size[ch]++;	
				if(rx_buffer_size[ch] >= 1)
				{
						timerun[ch] = 1;
						timeout[ch] = 0;
				}						
				usart_interrupt_flag_clear(UART3, USART_INT_FLAG_RBNE_ORERR);									
				//usart_data_transmit(USART0, (uint8_t)(g_ch[COM_PORT2]+2));				//调试打印			
    }
}
void UART4_IRQHandler(void)		//COM4  Meter
{
		unsigned char ch = METER;		
	  if(RESET != usart_interrupt_flag_get(UART4, USART_INT_FLAG_RBNE))
		{			
				rx_buffer[ch][rx_count[ch]++] = usart_data_receive(UART4);
				//rx_count[ch] = rx_count[ch]%BUFFER_SIZE;     
				rx_buffer_size[ch]++;
				if(rx_buffer_size[ch] >= 1)
				{
						timerun[ch] = 1;
						timeout[ch] = 0;
				}		
				usart_interrupt_flag_clear(UART4,USART_INT_FLAG_RBNE);
    }  
		
		if(RESET != usart_interrupt_flag_get(UART4, USART_INT_FLAG_RBNE_ORERR))	
		{			
				rx_buffer[ch][rx_count[ch]++] = usart_data_receive(UART4);
				//rx_count[ch] = rx_count[ch]%BUFFER_SIZE;     
				rx_buffer_size[ch]++;
				if(rx_buffer_size[ch] >= 1)
				{
						timerun[ch] = 1;
						timeout[ch] = 0;
				}									
				usart_interrupt_flag_clear(UART4, USART_INT_FLAG_RBNE_ORERR);									
    }   
}

void USART5_IRQHandler(void)	//COM5  4G
{
		unsigned char ch = G4;		
	  if(RESET != usart5_interrupt_flag_get(USART5, USART5_INT_FLAG_RBNE))
		{
				rx_buffer[ch][rx_count[ch]++] = usart_data_receive(USART5);
				//rx_count[ch] = rx_count[ch]%BUFFER_SIZE;     
				rx_buffer_size[ch]++;
				if(rx_buffer_size[ch] >= 1)
				{
						timerun[ch] = 1;
						timeout[ch] = 0;
				}
				usart5_interrupt_flag_clear(USART5, USART5_INT_FLAG_RBNE);				
    }   
		
		if(RESET != usart5_interrupt_flag_get(USART5, USART5_INT_FLAG_RBNE_ORERR))
		{		
				rx_buffer[ch][rx_count[ch]++] = usart_data_receive(USART5);
				//rx_count[ch] = rx_count[ch]%BUFFER_SIZE;     
				rx_buffer_size[ch]++;
				if(rx_buffer_size[ch] >= 1)
				{
						timerun[ch] = 1;
						timeout[ch] = 0;
				}      			
				usart5_interrupt_flag_clear(USART5, USART5_INT_FLAG_RBNE_ORERR);									
    }   
}

void TIMER2_IRQHandler(void)
{
		if(timer_interrupt_flag_get(TIMER2,TIMER_INT_FLAG_UP) != RESET)
		{
				timer_update_flag = 1;				
				timer_interrupt_flag_clear(TIMER2,TIMER_INT_FLAG_UP);
		}
}

void TIMER3_IRQHandler(void)
{
		if(timer_interrupt_flag_get(TIMER3,TIMER_INT_FLAG_UP) != RESET)
		{
				timer_sample_flag = 1;
				timer_interrupt_flag_clear(TIMER3,TIMER_INT_FLAG_UP);
		}
}

void TIMER4_IRQHandler(void)
{
		uint8_t i;
		if(timer_interrupt_flag_get(TIMER4,TIMER_INT_FLAG_UP) != RESET)
		{
				for(i = 0; i < COMn; i++)
				{
						if(timerun[i] != 0)
						{
								timeout[i]++;
								if(timeout[i] >= 8)
								{
										timerun[i] = 0;
										resetframe[i] = 1;
								}
						}
				}
				timer_interrupt_flag_clear(TIMER4,TIMER_INT_FLAG_UP);
		}
}