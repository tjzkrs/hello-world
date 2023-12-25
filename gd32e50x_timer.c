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
	timer_enable(TIMER3);
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