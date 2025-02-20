/* USER CODE BEGIN Header */
/**
  **************************
  * @file           : main.c
  * @brief          : Main program body
  **************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  **************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define mainDELAY_LOOP_COUNT            ( 0xfffff )
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
osThreadId defaultTaskHandle;
TIM_HandleTypeDef htim2;
/* USER CODE BEGIN PV */
uint32_t x = 0;
uint32_t y = 0;

volatile uint32_t gtimestamp1 = 0;
volatile uint32_t gtimestamp2 = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
void StartDefaultTask(void const * argument);

/* USER CODE BEGIN PFP */
void vTask_Re( void *pvParameters );
void vTask1(void *pvParameters);
void vTask2(void *pvParameters);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
struct prv_obj{

	int task_id;

  int hw_led;
        //fill the hw details
  int task_period;
   //fill the periodi of a taski
  int task_offset;
  //fill the offseti of a taski
   //add more elements
};
SemaphoreHandle_t Mutex_Handle_t = NULL;

struct prv_obj obj1;
struct prv_obj obj2;
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */


	 unsigned int ret = 0;

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  //BSP_LED_Init(GPIO_PIN_0);
 // BSP_LED_Init(GPIO_PIN_1);
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  //osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  //defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  	  	obj1.task_id = 1;
  	    obj1.hw_led = GPIO_PIN_0;
  	    obj1.task_period = 10015;
  	    obj1.task_offset = 100;
  ret = xTaskCreate(    vTask_Re,         /* Pointer to the function that implements the task. */
 	                                    "Task 1",       /* Text name for the task.  This is to facilitate debugging only. */
 	                                    240,            /* Stack depth in words. */
 	                                    (void *)&obj1
 	                                     ,           /* We are not using the task parameter. */
 	                                    1,                      /* This task will run at priority 1. */
 	                                    NULL );         /* We are not using the task handle. */
 	    configASSERT(ret==pdPASS);


 	   obj2.task_id = 2;
 	   obj2.hw_led = GPIO_PIN_1;
 	   obj2.task_period = 20030;
 	   obj2.task_offset = 100;

 	   	    ret = xTaskCreate(    vTask_Re,         /* Pointer to the function that implements the task. */
 	   	                                    "Task 2",       /* Text name for the task.  This is to facilitate debugging only. */
 	   	                                    240,            /* Stack depth in words. */
 	   	                                    (void *)&obj2
 	   	                                     ,           /* We are not using the task parameter. */
 	   	                                    1,                      /* This task will run at priority 1. */
 	   	                                    NULL );         /* We are not using the task handle. */
 	   	    configASSERT(ret==pdPASS);
  Mutex_Handle_t = xSemaphoreCreateMutex();
//  ret = xTaskCreate(vTask1,"Task1",240,NULL,1,NULL);
//  configASSERT(ret==pdPASS);
//  ret = xTaskCreate(vTask2,"Task2",240,NULL,1,NULL);
//  configASSERT(ret==pdPASS);
  if(HAL_TIM_Base_Start(&htim2) != HAL_OK)
    	  {
    	    /* Starting Error */
    	    Error_Handler();
    	  }
  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  vTaskStartScheduler();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  for( ;; );  //a form of crash ??

  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  return 0;
  /* USER CODE END 3 */

}
void vTask_Re( void *pvParameters )
{
const char *pcTaskName = "Task 1 is running\n";
//volatile unsigned long ul;
struct prv_obj *obj = (struct prv_obj *) pvParameters;
TickType_t xLastWakeTime;
volatile uint32_t timestamp1 = 0;
volatile uint32_t timestamp2 = 0;
volatile uint32_t x=0;
xLastWakeTime = xTaskGetTickCount();
        /* As per most tasks, this task is implemented in an infinite loop. */
        for( ;; )
        {

        	xSemaphoreTake(Mutex_Handle_t, portMAX_DELAY);
        	timestamp1 = __HAL_TIM_GET_COUNTER(&htim2);

        	printf("iteration is %u...Task is %u ...vTaskDelayUntil periodic release-time is .. %u\n",x,obj->task_id, timestamp1);
        	xSemaphoreGive(Mutex_Handle_t);

      	    HAL_GPIO_TogglePin(GPIOB, obj->hw_led);
        		/* This task should execute every 250 milliseconds deterministically .
        		xLastWakeTime is automatically updated within vTaskDelayUntil(), so is not
        		explicitly updated by the task. */
       		vTaskDelayUntil(&xLastWakeTime,obj->task_period);
			    //BSP_LED_Toggle(obj->led);
#if 0
                //lecture - a dummy loop - just for demo ??
                /* Delay for a period. */
                for( ul = 0; ul < mainDELAY_LOOP_COUNT; ul++ )
                {
                        /* This loop is just a very crude delay implementation.  There is
                        nothing to do in here.  Later exercises will replace this crude
                        loop with a proper delay/sleep function. */
                }
#endif
                x++;
        }
}
void vTask1( void *pvParameters )
{
const char *pcTaskName = "Task 1 is running\n";
volatile unsigned long ul;

//timestamp1 = __HAL_TIM_GET_COUNTER(&htim5);
//vTaskDelay(50);
//timestamp2 = __HAL_TIM_GET_COUNTER(&htim5);

//printf("vTaskDelay latency is .... %u\n", (timestamp2-timestamp1));





        /* As per most tasks, this task is implemented in an infinite loop. */
        for( ;; )
        {

//        		BSP_LED_Toggle(LED4);
//        	 vTaskDelay(500);
        	    HAL_GPIO_TogglePin(GPIOB, obj1.hw_led );


                //lecture - a dummy loop - just for demo ??
                /* Delay for a period. */
                for( ul = 0; ul < mainDELAY_LOOP_COUNT; ul++ )
                {
                        /* This loop is just a very crude delay implementation.  There is
                        nothing to do in here.  Later exercises will replace this crude
                        loop with a proper delay/sleep function. */
                }
                //these variables can be used to check the execution of the task

                x++;
     //       timestamp1 = __HAL_TIM_GET_COUNTER(&htim5);
    //            printf("task1....%d\n %u\n",x, timestamp1);
     //  timestamp1 = __HAL_TIM_GET_COUNTER(&htim5);
     //  vTaskDelay(50);
     //  timestamp2 = __HAL_TIM_GET_COUNTER(&htim5);

     //  printf("vTaskDelay latency is .... %u\n", (timestamp2-timestamp1));




        }
}

//lecture - just a duplicate of the first task's code

void vTask2( void *pvParameters )
{
const char *pcTaskName = "Task 2 is running\n";
volatile unsigned long ul;

        /* As per most tasks, this task is implemented in an infinite loop. */
        for( ;; )
        {
        		//BSP_LED_Toggle(LED3);

        	HAL_GPIO_TogglePin(GPIOD, obj2.hw_led);
                /* Delay for a period. */
                for( ul = 0; ul < mainDELAY_LOOP_COUNT; ul++ )
                {
                        /* This loop is just a very crude delay implementation.  There is
                        nothing to do in here.  Later exercises will replace this crude
                        loop with a proper delay/sleep function. */
                }
                //these variables can be used to check the execution of the task
                y++;
        }
}
void vApplicationMallocFailedHook( void )
{
        /* This function will only be called if an API call to create a task, queue
        or semaphore fails because there is too little heap RAM remaining. */
        for( ;; );
}
void vApplicationStackOverflowHook( xTaskHandle *pxTask, signed char *pcTaskName )
{
        /* This function will only be called if a task overflows its stack.  Note
        that stack overflow checking does slow down the context switch
        implementation. */
        for( ;; );
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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}
/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_SlaveConfigTypeDef sSlaveConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_DISABLE;
  sSlaveConfig.InputTrigger = TIM_TS_ITR2;
  if (HAL_TIM_SlaveConfigSynchro(&htim2, &sSlaveConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1|GPIO_PIN_2, GPIO_PIN_SET);

  /*Configure GPIO pins : PB1 PB2 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END 5 */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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
