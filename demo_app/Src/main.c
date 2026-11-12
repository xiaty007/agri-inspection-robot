/**
 ******************************************************************************
 * File Name          : main.c
 * Description        : Main program body
 ******************************************************************************
 *
 * COPYRIGHT(c) 2017 STMicroelectronics
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright notice,
 *      this list of conditions and the following disclaimer in the documentation
 *      and/or other materials provided with the distribution.
 *   3. Neither the name of STMicroelectronics nor the names of its contributors
 *      may be used to endorse or promote products derived from this software
 *      without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <math.h>

#include "board_config.h"
#include "stm32f4xx_it.h"

#include "app_def.h"
#include "mm_loop.h"
#include "mm_types.h"
#include "mm_util.h"
#include "hal_i2c.h"

#include "oled_i2c.h"
#include "util.h"


__no_init uint8_t app_reset_reason;


/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
extern uint32_t irq_sets;

UART_HandleTypeDef uart_handles;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

void Error_Handler(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
#ifdef __GNUC__
/* With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf
   set to 'Yes') calls __io_putchar() */
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE * f)
#endif /* __GNUC__ */


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    switch (GPIO_Pin)
    {
        case GPIO_PIN_8:
            SET_BITS(irq_sets, IRQ_EVT_SA2);
            break;
    }
}


void gpio_b15_Callback(void)
{
    __HAL_PWR_CLEAR_FLAG(PWR_FLAG_WU);
    led_blink(2);
}


void SetDebugMode(uint8_t u8enable)
{
    u8enable = !!u8enable;

//    __HAL_RCC_DBGMCU_CLK_ENABLE();
//    DBGMCU->CR     = u8enable > 0 ? 0x7 : 0x0;
//    DBGMCU->APB1FZ = u8enable > 0 ? (0x7 << 10) : 0x0;
//    DBGMCU->APB2FZ = 0;
//    __HAL_RCC_DBGMCU_CLK_DISABLE();
}


int hal_uart_init(void)
{

    /*##-1- Configure the UART peripheral ######################################*/
    /* Put the USART peripheral in the Asynchronous mode (UART Mode) */
    /* UART1 configured as follow:
        - Word Length = 8 Bits
        - Stop Bit = One Stop bit
        - Parity = ODD parity
        - BaudRate = 9600 baud
        - Hardware flow control disabled (RTS and CTS signals) */

#if TARGET_BOARD == BOARD_PNI_13860
    uart_handles.Instance = USART6;                        //USARTx;
    uart_handles.Init.BaudRate = 115200;
    uart_handles.Init.WordLength = UART_WORDLENGTH_8B;
    uart_handles.Init.StopBits = UART_STOPBITS_1;
    uart_handles.Init.Parity = UART_PARITY_NONE;
    uart_handles.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    uart_handles.Init.Mode = UART_MODE_TX_RX;
    uart_handles.Init.OverSampling = UART_OVERSAMPLING_16; //UART_OVERSAMPLING_16;
#elif TARGET_BOARD == BOARD_STM32_NUCELO_F411
    GPIO_InitTypeDef GPIO_InitStruct;

    USARTx_TX_GPIO_CLK_ENABLE();
    USARTx_RX_GPIO_CLK_ENABLE();

    //
    //  /* Enable USARTx clock */
    USARTx_CLK_ENABLE();

    GPIO_InitStruct.Pin = USARTx_TX_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FAST;
    GPIO_InitStruct.Alternate = USARTx_TX_AF;

    HAL_GPIO_Init(USARTx_TX_GPIO_PORT, &GPIO_InitStruct);

    /* UART RX GPIO pin configuration  */
    GPIO_InitStruct.Pin = USARTx_RX_PIN;
    GPIO_InitStruct.Alternate = USARTx_RX_AF;

    HAL_GPIO_Init(USARTx_RX_GPIO_PORT, &GPIO_InitStruct);

    uart_handles.Instance = USART2;                        //USARTx;
    uart_handles.Init.BaudRate = 115200;
    uart_handles.Init.WordLength = UART_WORDLENGTH_8B;
    uart_handles.Init.StopBits = UART_STOPBITS_1;
    uart_handles.Init.Parity = UART_PARITY_NONE;
    uart_handles.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    uart_handles.Init.Mode = UART_MODE_TX_RX;
    uart_handles.Init.OverSampling = UART_OVERSAMPLING_16; //UART_OVERSAMPLING_16;
#endif

    if (HAL_UART_Init(&uart_handles) != HAL_OK)
    {
        /* Initialization Error */
        Error_Handler();
    }

    return 0;
}


int main(void)
{
    uint32_t u32val = 0;
    uint32_t u32csr = 0;

    int ret = HAL_OK;
    uint16_t u16appreset = 0;
    uint8_t u8reset_in_fsm = 0;

    stc_app_t *pApp = NULL;

    /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
    HAL_Init();
    SystemClock_Config();
    SetDebugMode(FALSE);

    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    hal_i2c_init();
    hal_uart_init();

    pApp = get_app();

    led_blink(3);

    
    DBG_LED_OFF();

    printf("HOST amber-%d  STUDY  svn 149----------------------------\n", SENTRAL_TYPE_USING);

    pApp->runtime.UserAction = ACTION_NONE;
    pApp->pRAM->bdocalibration = 0;
    pApp->runtime.bUseCalDat = 0;
    pApp->pRAM->bClearWarmStartEEPROM = 0x0;


    u32csr = RCC->CSR;
    RCC->CSR |= 1 << 23;




    u16appreset = pApp->pRAM->app_reset_reason;  //en_app_resetreason
    u8reset_in_fsm = pApp->pRAM->app_reset_in_fsm;


    APP_DBG("cz:%08X, app_rst=%X, fsm=%d tot_rbt=%d\n", u32csr, u16appreset, u8reset_in_fsm, u32val);

    //clear ram
    pApp->pRAM->app_reset_reason = 0xFFFF;
    pApp->pRAM->app_reset_in_fsm = 0xFF;

    Clock_Wait(0);


    ret = mm2_init();
    if (HAL_OK != ret)
    {
        APP_E("mm init failed: %d !\n", ret);
        goto end;
    }

    hal_delay(0);

    APP_DBG("Host init all done\n\n");


    mm2_loop();

 end:
    return HAL_ERROR;

}


/** System Clock Configuration
 */
void SystemClock_Config(void)
{

    RCC_ClkInitTypeDef RCC_ClkInitStruct;
    RCC_OscInitTypeDef RCC_OscInitStruct;

    /* Enable Power Control clock */
    __HAL_RCC_PWR_CLK_ENABLE();

    /* The voltage scaling allows optimizing the power consumption when the device is
       clocked below the maximum system frequency, to update the voltage scaling value
       regarding system frequency refer to product datasheet.  */
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

    /* Enable HSI Oscillator and activate PLL with HSI as source */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = 0x10;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
    RCC_OscInitStruct.PLL.PLLM = 16;
    RCC_OscInitStruct.PLL.PLLN = 400;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
    RCC_OscInitStruct.PLL.PLLQ = 7;

    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }

    /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2
       clocks dividers */
    RCC_ClkInitStruct.ClockType =
        (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
    {
        Error_Handler();
    }

    HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq() / 1000);

    /**Configure the Systick
     */
    HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);
    HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* USER CODE BEGIN 4 */
PUTCHAR_PROTOTYPE
{
    /* Place your implementation of fputc here */

    HAL_UART_Transmit(&uart_handles, (uint8_t *)&ch, 1, 0xFFFF);

    return ch;
}

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @param  None
 * @retval None
 */
void Error_Handler(void)
{
    /* USER CODE BEGIN Error_Handler */
    APP_DBG("[ERROR] Error Handler\r\n");

    /* User can add his own implementation to report the HAL error return state */
    while (1)
    {
    }

    /* USER CODE END Error_Handler */
}


void WWDG_IRQHandler(void)
{
    //*(uint16_t *)RAM_APP_RESET_REASON = APP_RESET_WDT;
}


#ifdef USE_FULL_ASSERT

/**
 * @brief Reports the name of the source file and the source line number
 * where the assert_param error has occurred.
 * @param file: pointer to the source file name
 * @param line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line)
{
    /* USER CODE BEGIN 6 */
    /* User can add his own implementation to report the file name and line number,
       ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
    /* USER CODE END 6 */

}

#endif


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
