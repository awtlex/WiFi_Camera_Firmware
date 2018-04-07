/*
***************************************************************************************************
*                            Motor Control Task
*
* File   : motor_task.c
* Author : Douglas Xie
* Date   : 2017.10.27
***************************************************************************************************
* Copyright (C) 2017 Douglas Xie.  All rights reserved.
***************************************************************************************************
*/

#ifndef USE_DEMO_VERSION

/* Include Head Files ---------------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "cmsis_os.h"
#include "string.h"

#include "global_config.h"
#include "client_task.h"
#include "wifi_task.h"
#include "motor_task.h"
#include "debug_task.h"
#include "memory.h"

/* Macro Define ---------------------------------------------------------------------------------*/



/* Global Variable ------------------------------------------------------------------------------*/
extern TIM_HandleTypeDef htim6;
extern RTC_HandleTypeDef hrtc;
extern RTC_TimeTypeDef   sTime;
extern RTC_DateTypeDef   sDate;
Motor_Group_t motor_group;


/* Private Function Declaration -----------------------------------------------------------------*/
void Motor_SetTimerPeriod(uint16_t period);
void Motor_SetDefault(void);


/* Public Function ------------------------------------------------------------------------------*/

/*******************************************************************************
* @Brief   Motor Control Task
* @Param
* @Note
* @Return
*******************************************************************************/
void Motor_ControlTask(void * argument)
{
    Motor_State_t motor_state = MOTOR_IDLE;
    uint8_t schedule_index = 0;
    uint8_t schedule_today = 0;

    /* Default setting */
    Motor_SetTimerPeriod(0);
    Motor_SetDefault();

    DBG_SendMessage(DBG_MSG_TASK_STATE, "Motor Task Start\r\n");

    for (;;)
    {
        HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
        HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN);

        /* restart schedule for new day */
        if (sDate.Date != schedule_today)
        {
            schedule_today = sDate.Date;
            schedule_index = 0;
        }

        /* motor schedule analyze */
        if (schedule_index < app_config.sch_count)
        {
            if ((app_config.schedule[schedule_index].t_hour == sTime.Hours) &&
                    (app_config.schedule[schedule_index].t_minute == sTime.Minutes))
            {
                motor_group.motor1.direction = app_config.motor_cfg.m_dir[0];
                motor_group.motor2.direction = app_config.motor_cfg.m_dir[1];
                motor_group.motor3.direction = app_config.motor_cfg.m_dir[2];
                motor_group.motor4.direction = app_config.motor_cfg.m_dir[3];
                motor_group.motor5.direction = app_config.motor_cfg.m_dir[4];

                motor_group.motor1.period = (uint16_t)(MOTOR_TIMER_FREQ / (app_config.motor_cfg.m_freq[0] * 2));
                motor_group.motor2.period = (uint16_t)(MOTOR_TIMER_FREQ / (app_config.motor_cfg.m_freq[1] * 2));
                motor_group.motor3.period = (uint16_t)(MOTOR_TIMER_FREQ / (app_config.motor_cfg.m_freq[2] * 2));
                motor_group.motor4.period = (uint16_t)(MOTOR_TIMER_FREQ / (app_config.motor_cfg.m_freq[3] * 2));
                motor_group.motor5.period = (uint16_t)(MOTOR_TIMER_FREQ / (app_config.motor_cfg.m_freq[4] * 2));

                motor_group.motor1.step += app_config.schedule[schedule_index].feed_m1 * (app_config.motor_cfg.m_step[0] << 1);
                motor_group.motor2.step += app_config.schedule[schedule_index].feed_m2 * (app_config.motor_cfg.m_step[1] << 1);
                motor_group.motor3.step += app_config.schedule[schedule_index].feed_m3 * (app_config.motor_cfg.m_step[2] << 1);
                motor_group.motor4.step += app_config.schedule[schedule_index].feed_m4 * (app_config.motor_cfg.m_step[3] << 1);
                motor_group.motor5.step += app_config.schedule[schedule_index].feed_m5 * (app_config.motor_cfg.m_step[4] << 1);

                schedule_index++;
            }
        }

        /* motor activation process */
        switch (motor_state)
        {
        case MOTOR_RUN_CH1:
            if (motor_group.motor1.step == 0) // TODO: add motor error check and error handler here, reference IRQ handler
            {
                /* No action for motor1, swift to next channel */
                motor_state = MOTOR_RUN_CH2;
            }
            else
            {
                /* Update timer if channel timer period is different with current setting */
                if ( motor_group.timer_period != motor_group.motor1.period )
                {
                    Motor_SetTimerPeriod(motor_group.motor1.period);
                    Motor_OutputLow(MOTOR_STEP_CH1);
                }

                /* Start control motor1 */
                if ( motor_group.motor1.enable == 0 )
                {
                    if (motor_group.motor1.direction == 'L')
                    {
                        Motor_TurnLeft(MOTOR_DIR_CH1);
                    }
                    else
                    {
                        Motor_TurnRight(MOTOR_DIR_CH1);
                    }
                    Motor_OutputLow(MOTOR_STEP_CH1);
                    Motor_TurnEnable(MOTOR_EN_CH1);
                    motor_group.motor1.enable = 1;

                    DBG_SendMessage(DBG_MSG_MOTOR, "Motor: Channel1 Start Turning\r\n");
                }
            }
            break;

        case MOTOR_RUN_CH2:
            if (motor_group.motor2.step == 0)   // TODO: add motor error check and error handler here, reference IRQ handler
            {
                /* No action for motor2, swift to next channel */
                Motor_OutputLow(MOTOR_STEP_CH2);
                Motor_TurnDisable(MOTOR_EN_CH2);
                motor_group.motor2.enable = 0;
                motor_state = MOTOR_RUN_CH3;
            }
            else
            {
                /* Update timer if channel timer period is different with current setting */
                if ( motor_group.timer_period != motor_group.motor2.period )
                {
                    Motor_SetTimerPeriod(motor_group.motor2.period);
                    Motor_OutputLow(MOTOR_STEP_CH2);
                }

                /* Start control motor2 */
                if ( motor_group.motor2.enable == 0 )
                {
                    if (motor_group.motor2.direction == 'L')
                    {
                        Motor_TurnLeft(MOTOR_DIR_CH3);
                    }
                    else
                    {
                        Motor_TurnRight(MOTOR_DIR_CH2);
                    }
                    Motor_OutputLow(MOTOR_STEP_CH2);
                    Motor_TurnEnable(MOTOR_EN_CH2);
                    motor_group.motor2.enable = 1;

                    DBG_SendMessage(DBG_MSG_MOTOR, "Motor: Channel2 Start Turning\r\n");
                }
            }
            break;

        case MOTOR_RUN_CH3:
            if (motor_group.motor3.step == 0)   // TODO: add motor error check and error handler here, reference IRQ handler
            {
                /* No action for motor3, swift to next channel */
                Motor_OutputLow(MOTOR_STEP_CH3);
                Motor_TurnDisable(MOTOR_EN_CH3);
                motor_group.motor3.enable = 0;
                motor_state = MOTOR_RUN_CH4;
            }
            else
            {
                /* Update timer if channel timer period is different with current setting */
                if ( motor_group.timer_period != motor_group.motor3.period )
                {
                    Motor_SetTimerPeriod(motor_group.motor3.period);
                    Motor_OutputLow(MOTOR_STEP_CH3);
                }

                /* Start control motor3 */
                if ( motor_group.motor3.enable == 0 )
                {
                    if (motor_group.motor3.direction == 'L')
                    {
                        Motor_TurnLeft(MOTOR_DIR_CH3);
                    }
                    else
                    {
                        Motor_TurnRight(MOTOR_DIR_CH3);
                    }
                    Motor_OutputLow(MOTOR_STEP_CH3);
                    Motor_TurnEnable(MOTOR_EN_CH3);
                    motor_group.motor3.enable = 1;

                    DBG_SendMessage(DBG_MSG_MOTOR, "Motor: Channel3 Start Turning\r\n");
                }
            }
            break;

        case MOTOR_RUN_CH4:
            if (motor_group.motor4.step == 0)   // TODO: add motor error check and error handler here, reference IRQ handler
            {
                /* No action for motor4, swift to next channel */
                Motor_OutputLow(MOTOR_STEP_CH4);
                Motor_TurnDisable(MOTOR_EN_CH4);
                motor_group.motor4.enable = 0;
                motor_state = MOTOR_RUN_CH5;
            }
            else
            {
                /* Update timer if channel timer period is different with current setting */
                if ( motor_group.timer_period != motor_group.motor4.period )
                {
                    Motor_SetTimerPeriod(motor_group.motor4.period);
                    Motor_OutputLow(MOTOR_STEP_CH4);
                }

                /* Start control motor4 */
                if ( motor_group.motor4.enable == 0 )
                {
                    if (motor_group.motor4.direction == 'L')
                    {
                        Motor_TurnLeft(MOTOR_DIR_CH4);
                    }
                    else
                    {
                        Motor_TurnRight(MOTOR_DIR_CH4);
                    }
                    Motor_OutputLow(MOTOR_STEP_CH4);
                    Motor_TurnEnable(MOTOR_EN_CH4);
                    motor_group.motor4.enable = 1;

                    DBG_SendMessage(DBG_MSG_MOTOR, "Motor: Channel4 Start Turning\r\n");
                }
            }
            break;

        case MOTOR_RUN_CH5:
            if (motor_group.motor5.step == 0)   // TODO: add motor error check and error handler here, reference IRQ handler
            {
                /* No action for motor5, swift to next channel */
                Motor_OutputLow(MOTOR_STEP_CH5);
                Motor_TurnDisable(MOTOR_EN_CH5);
                motor_group.motor5.enable = 0;
                motor_state = MOTOR_IDLE;
            }
            else
            {
                /* Update timer if channel timer period is different with current setting */
                if ( motor_group.timer_period != motor_group.motor5.period )
                {
                    Motor_SetTimerPeriod(motor_group.motor5.period);
                    Motor_OutputLow(MOTOR_STEP_CH5);
                }

                /* Start control motor5 */
                if ( motor_group.motor5.enable == 0 )
                {
                    if (motor_group.motor5.direction == 'L')
                    {
                        Motor_TurnLeft(MOTOR_DIR_CH5);
                    }
                    else
                    {
                        Motor_TurnRight(MOTOR_DIR_CH5);
                    }
                    Motor_OutputLow(MOTOR_STEP_CH5);
                    Motor_TurnEnable(MOTOR_EN_CH5);
                    motor_group.motor5.enable = 1;

                    DBG_SendMessage(DBG_MSG_MOTOR, "Motor: Channel5 Start Turning\r\n");
                }
            }
            break;

        case MOTOR_IDLE:
            if ( (motor_group.motor1.step == 0) &&
                    (motor_group.motor2.step == 0) &&
                    (motor_group.motor3.step == 0) &&
                    (motor_group.motor4.step == 0) &&
                    (motor_group.motor5.step == 0))
            {
                /* Stop pwm timer to save power when there is no motor running */
                if (motor_group.timer_period != 0)
                {
                    /* Disable pwm timer by set period to 0 */
                    Motor_SetTimerPeriod(0);

                    DBG_SendMessage(DBG_MSG_MOTOR, "Motor: Waiting for Event\r\n");
                }
            }
            else
            {
                motor_state = MOTOR_RUN_CH1;
            }
            break;

        default:
            break;
        }

        vTaskDelay(MOTOR_TASK_PERIOD);
    }
}

/*******************************************************************************
* @Brief   Motor Step Control
* @Param
* @Note    motor step control signal generation in timer IRQ handler
* @Return
*******************************************************************************/
void Motor_StepControl_IRQ(void)
{
    if (motor_group.motor1.enable == 1)
    {
        if (motor_group.motor1.step != 0)
        {
            motor_group.motor1.step--;
            Motor_OutputToggle( MOTOR_STEP_CH1 );
        }
        else if (motor_group.motor1.enable == 1)
        {
            Motor_OutputLow(MOTOR_STEP_CH1);
            Motor_TurnDisable(MOTOR_EN_CH1);
            motor_group.motor1.enable = 0;
        }
    }
    else if (motor_group.motor2.enable == 1)
    {
        if (motor_group.motor2.step != 0)
        {
            motor_group.motor2.step--;
            Motor_OutputToggle( MOTOR_STEP_CH2 );
        }
        else if (motor_group.motor2.enable == 1)
        {
            Motor_OutputLow(MOTOR_STEP_CH2);
            Motor_TurnDisable(MOTOR_EN_CH2);
            motor_group.motor2.enable = 0;
        }
    }
    else if (motor_group.motor3.enable == 1)
    {
        if (motor_group.motor3.step != 0)
        {
            motor_group.motor3.step--;
            Motor_OutputToggle( MOTOR_STEP_CH3 );
        }
        else if (motor_group.motor3.enable == 1)
        {
            Motor_OutputLow(MOTOR_STEP_CH3);
            Motor_TurnDisable(MOTOR_EN_CH3);
            motor_group.motor3.enable = 0;
        }
    }
    else if (motor_group.motor4.enable == 1)
    {
        if (motor_group.motor4.step != 0)
        {
            motor_group.motor4.step--;
            Motor_OutputToggle( MOTOR_STEP_CH4 );
        }
        else if (motor_group.motor4.enable == 1)
        {
            Motor_OutputLow(MOTOR_STEP_CH4);
            Motor_TurnDisable(MOTOR_EN_CH4);
            motor_group.motor4.enable = 0;
        }
    }
    else if (motor_group.motor5.enable == 1)
    {
        if (motor_group.motor5.step != 0)
        {
            motor_group.motor5.step--;
            Motor_OutputToggle( MOTOR_STEP_CH5 );
        }
        else if (motor_group.motor5.enable == 1)
        {
            Motor_OutputLow(MOTOR_STEP_CH5);
            Motor_TurnDisable(MOTOR_EN_CH5);
            motor_group.motor5.enable = 0;
        }

    }
}


/* Private Function -----------------------------------------------------------------------------*/

/*******************************************************************************
* @Brief   Set Motor Timer Period
* @Param
* @Note    set pwm timer period to update speed
* @Return
*******************************************************************************/
void Motor_SetTimerPeriod(uint16_t period)
{
    TIM_MasterConfigTypeDef sMasterConfig;

    /* Reset timer setting */
    HAL_TIM_Base_DeInit(&hmotor_timer);

    if (period != 0)
    {
        /* Update new setting */
        hmotor_timer.Instance = MOTOR_TIMER;
        hmotor_timer.Init.Prescaler = MOTOR_TIMER_PRES;
        hmotor_timer.Init.CounterMode = TIM_COUNTERMODE_UP;
        hmotor_timer.Init.Period = period;
        if (HAL_TIM_Base_Init(&hmotor_timer) != HAL_OK)
        {
            Error_Handler();
        }

        sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
        sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
        if (HAL_TIMEx_MasterConfigSynchronization(&hmotor_timer, &sMasterConfig) != HAL_OK)
        {
            Error_Handler();
        }

        HAL_TIM_Base_Start_IT(&hmotor_timer);
    }

    /* Update current period to global variable */
    motor_group.timer_period = period;
}


/*******************************************************************************
* @Brief   Set Motor Parameter to Default
* @Note
* @Return
*******************************************************************************/
void Motor_SetDefault(void)
{
    motor_group.motor1.direction = MOTOR_DEFAULT_DIR;
    motor_group.motor1.period = MOTOR_DEFAULT_PERIOD;
    motor_group.motor1.step = 0;
    motor_group.motor1.enable = 0;

    motor_group.motor2.direction = MOTOR_DEFAULT_DIR;
    motor_group.motor2.period = MOTOR_DEFAULT_PERIOD;
    motor_group.motor2.step = 0;
    motor_group.motor2.enable = 0;

    motor_group.motor3.direction = MOTOR_DEFAULT_DIR;
    motor_group.motor3.period = MOTOR_DEFAULT_PERIOD;
    motor_group.motor3.step = 0;
    motor_group.motor3.enable = 0;

    motor_group.motor4.direction = MOTOR_DEFAULT_DIR;
    motor_group.motor4.period = MOTOR_DEFAULT_PERIOD;
    motor_group.motor4.step = 0;
    motor_group.motor4.enable = 0;

    motor_group.motor5.direction = MOTOR_DEFAULT_DIR;
    motor_group.motor5.period = MOTOR_DEFAULT_PERIOD;
    motor_group.motor5.step = 0;
    motor_group.motor5.enable = 0;
}


#endif /* USE_DEMO_VERSION */