/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "tim.h"
#include "gpio.h"
#include "fsmc.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "LCD.h"
#include "lvgl.h"

#include "lv_port_disp_template.h"
//#include "lv_demo_widgets.h"
#include "touch.h"
#include "demo.h"
#include "lv_port_indev_template.h"
#include "KEY.h"
#define LED_GPIO_PORT GPIOC
#define LED_GPIO_PIN GPIO_PIN_4
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
extern lv_indev_t * indev_keypad;
extern lv_indev_t * indev_touch;

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_FSMC_Init();
  MX_TIM3_Init();
	MX_GPIO_Init();
  /* USER CODE BEGIN 2 */
     // ʹ��GPIOCʱ��
    __HAL_RCC_GPIOC_CLK_ENABLE();

    // ����PC4����Ϊ���ģʽ
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = LED_GPIO_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(LED_GPIO_PORT, &GPIO_InitStruct);
		    // ����PC4���ų�ʼ��ƽΪ�ߵ�ƽ
    HAL_GPIO_WritePin(LED_GPIO_PORT, LED_GPIO_PIN, GPIO_PIN_SET);
		
lv_init();

lv_port_disp_init();
HAL_TIM_Base_Start_IT(&htim3); 

lv_port_indev_init();
lv_group_t *g = lv_group_create();//������
lv_group_set_default(g);//����ΪĬ�� group ��
lv_indev_set_group(indev_keypad,g);//���������豸�� group ��

//�����ú����ٴ����ؼ�
lv_obj_t *slider = lv_slider_create(lv_scr_act());
lv_obj_center(slider);
lv_group_add_obj(g,slider);//���ָ��������group���У�����Ĭ�����еĲ���ʹ��
 
//���� btn1
lv_obj_t *btn1 = lv_btn_create(lv_scr_act());
lv_obj_align(btn1,LV_ALIGN_CENTER,0,-80);
lv_obj_set_size(btn1,200,80);
lv_obj_t *label1 = lv_label_create(btn1);
lv_label_set_text(label1,"01");
lv_obj_align_to(label1,btn1,LV_ALIGN_CENTER,0,0);

//���� btn2
lv_obj_t *btn2 = lv_btn_create(lv_scr_act());
lv_obj_align_to(btn2,btn1,LV_ALIGN_OUT_BOTTOM_LEFT,0,80);
lv_obj_set_size(btn2,200,80);
lv_obj_t *label2 = lv_label_create(btn2);
lv_label_set_text(label2,"02");
lv_obj_align_to(label2,btn2,LV_ALIGN_CENTER,0,0);

lv_group_add_obj(g,btn1);
lv_group_add_obj(g,btn2);
// 
//lv_obj_t * container = lv_obj_create(lv_scr_act());
//lv_obj_set_size(container, 320, 240);
//lv_obj_center(container);

//lv_obj_t * button = lv_btn_create(container);
//lv_obj_set_size(button, 60, 35);
//// lv_obj_center(button);
//lv_obj_set_pos(button, 50, 125);
//lv_obj_add_event_cb(button, btn_event_cb_1, LV_EVENT_ALL, NULL);

//lv_group_add_obj(g, button);
 
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
//		if(HAL_GPIO_ReadPin (KEY4_GPIO_Port, KEY4_Pin)==0)
//		{
//			//HAL_Delay(500);
//		HAL_GPIO_WritePin(LED_GPIO_PORT, LED_GPIO_PIN, GPIO_PIN_RESET);}
//		else
//		{HAL_GPIO_WritePin(LED_GPIO_PORT, LED_GPIO_PIN, GPIO_PIN_SET);
//		}
		HAL_Delay(5);
		lv_task_handler();
		 
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
   void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance==TIM3)
	{
		lv_tick_inc(1);
	}}

static uint32_t g_fac_us=0; /*us��ʱ������*/ 
void delay_us(uint32_t nus) 
{ 
	uint32_t ticks; 
	uint32_t told, tnow, tent=0; 
	uint32_t reload=SysTick->LOAD; /*LOAD��ֵ */ 
	ticks= nus* g_fac_us; /*��Ҫ�Ľ����� */ 
	told=SysTick->VAL; /*�ս���ʱ�ļ�����ֵ */ 
	while(1) 
	{ 
		tnow=SysTick->VAL; 
		if(tnow!= told) 
		{ 
				if(tnow< told) 
				{ 
				tent+= told-tnow; 
						/*����ע��һ��SYSTICK��һ���ݼ��ļ������Ϳ����� */ 
					} 
				else 
					{ 
				tent+= reload-tnow+ told; 
						} 
			told = tnow; 
				if(tent>= ticks) 
			{ 
					break; /*ʱ�䳬��/����Ҫ�ӳٵ�ʱ�䣬���˳� */ 
			} 
}} }
void load_draw_dialog(void) 
{ 
 lcd_clear(WHITE); /*���� */ 
 lcd_show_string(lcddev.width-24,0,200,16,16, "RST",BLUE);/* ��ʾ�������� */
} 
/** 
*@ brief ���败�������Ժ��� 
*@ param �� 
*@ retval �� 
*/ 
void rtp_test(void) 
{
		while(1) 
		{ 
			tp_dev.scan(0); 
if(tp_dev.sta& TP_PRES_DOWN) /* ������������ */ 
{ 
if(tp_dev.x[0] < lcddev.width && tp_dev.y[0] < lcddev.height) 
{ 
if(tp_dev.x[0] > (lcddev.width -24) && tp_dev.y[0]<16) 
{ 
load_draw_dialog();/* ��� */ 
} 
else 
{ 
tp_draw_big_point(tp_dev.x[0], tp_dev.y[0],RED); /* ���� */ 
} 
} 
} 
else 
{ 
HAL_Delay(10); /*û�а������µ�ʱ�� */ 
} 
} 
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
