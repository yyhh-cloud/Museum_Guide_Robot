/* USER CODE BEGIN Header */
/**
  * 博物馆智能导览机器人 - 主控逻辑 (最终完整版)
  * 功能：按键接单 -> 视觉识别路标 -> 智能变道 -> 定点停车
  * 包含：完整的中断回调、时钟配置、串口重定向
  */
/* USER CODE END Header */
#include "main.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include <stdio.h>
#include "k210.h"      
#include "bsp_oled.h" 

/* USER CODE BEGIN PV */
float ranger_dis=0;       // 超声波距离
float temperature=0;      // MPU6050温度
uint8_t k210_buffer[30];  // K210接收缓冲
uint8_t k210_active_cnt = 0;
uint8_t k210_active_flag=0;

C10B_Sendmsg c10b_send = { 0 }; // 发送给底盘的结构体

// === 🚩 任务目标 ===
// 0: 巡逻 (Patrol), 1: 蒙娜丽莎 (Mona Lisa), 2: 星空 (Starry Night)
uint8_t TARGET_ID = 0; 

// === 🛡️ 转向冷却锁 (防止路口鬼打墙) ===
static uint32_t turn_cooldown_timestamp = 0; 
/* USER CODE END PV */

/* Private function prototypes */
void SystemClock_Config(void);

// === 🕹️ 按键扫描函数 (带消抖) ===
void Scan_Keys(void)
{
    // 如果是 PA0 按键 (根据实际板子调整)
    if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == GPIO_PIN_SET) 
    {
        HAL_Delay(20); // 软件消抖
        if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == GPIO_PIN_SET)
        {
            TARGET_ID++; 
            if(TARGET_ID > 2) TARGET_ID = 0; // 循环切换: 0->1->2->0
            
            // 切换任务时，重置一下冷却锁，立刻响应
            turn_cooldown_timestamp = 0;
            
            // 死等松手，防止一次按键变多次
            while(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == GPIO_PIN_SET); 
        }
    }
}

int main(void)
{
  // 1. 硬件初始化
  HAL_Init();
  SystemClock_Config(); // 配置系统时钟 (72MHz)
  MX_GPIO_Init(); 
  MX_DMA_Init(); 
  MX_UART5_Init(); 
  MX_USART3_UART_Init();
  MX_TIM2_Init(); 
  MX_TIM7_Init(); 
  MX_USART1_UART_Init();
  
  // 2. 模块初始化
  DWT_Init(); 
  OLED_Init();
  HAL_TIM_IC_Start_IT(&htim2,TIM_CHANNEL_2); // 开启超声波定时器
  MPU6050_initialize(); 
  DMP_Init(); // 初始化陀螺仪DMP
  HAL_UARTEx_ReceiveToIdle_DMA(&huart3,k210_buffer,30); // 开启K210接收DMA

  while (1)
  {
      // 3. 扫描按键
      Scan_Keys();

      // 4. OLED 状态显示
      OLED_ShowString(0,0,"Mode:");
      if (TARGET_ID == 0)      OLED_ShowString(40,0,"Patrol   ");
      else if (TARGET_ID == 1) OLED_ShowString(40,0,"Go->Lisa ");
      else if (TARGET_ID == 2) OLED_ShowString(40,0,"Go->Star ");

      // 5. 显示视觉数据
      if (k210.y < 100) // 识别到 Tag ID (因为Tag ID通常小于100，巡线Y坐标通常>100)
      {
          OLED_ShowString(0,20,"Tag ID:");
          OLED_ShowNumber(60,20,k210.y,2,12);
          OLED_ShowString(0,30,"Dist:");
          OLED_ShowNumber(40,30,k210.z,3,12);
          
          if(k210.y == TARGET_ID) OLED_ShowString(0,50,"Target Found!");
          else if(k210.y == 0)    OLED_ShowString(0,50,"Turning...   ");
      }
      else // 正在巡线 (Tag ID位置显示的是 Y坐标)
      {
          OLED_ShowString(0,20,"Line Follow  ");
          OLED_ShowString(0,30,"             ");
          OLED_ShowString(0,50,"             ");
      }

      OLED_Refresh_Gram();
      HAL_Delay(20);
  }
}

// === ⚙️ 系统时钟配置 (标准 STM32F103 72MHz) ===
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

// === 🖨️ printf 重定向 (用于调试) ===
int fputc(int ch, FILE *f)
{
  HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 0xffff);
  return ch;
}

// === 🧠 核心中断逻辑 (MPU6050 引脚触发) ===
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    static uint8_t ranger_cnt = 0;
    static uint32_t temperature_cnt = 0;
    static uint8_t c50c_send_cnt = 0;
    uint8_t tempbuffer[2];
    
    // MPU6050 INT引脚触发中断 (200Hz)
    if( GPIO_Pin == MPU6050_INT_Pin )
    {
        // 1. 读取温度 (每500次读取一次)
        temperature_cnt++;
        if(temperature_cnt >= 500) {
            temperature_cnt = 0;
            temperature = MPU_Get_Temperature(); // 读取温度
        }
        
        // 2. 读取陀螺仪DMP数据 (核心!)
        Read_DMP();
        
        // 3. 触发超声波测距 (每5次触发一次)
        ranger_cnt++;
        if(ranger_cnt >= 5) {
            ranger_cnt = 0;
            // 触发超声波 Trig 引脚 (PA1) - 产生高电平脉冲
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);
            DWT_Delay_us(20);
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);
        }
        
        // 4. K210 心跳检测
        k210_active_cnt++;
        if( k210_active_cnt>=100 ) k210_active_flag = 0;
        
        // 5. 发送控制指令给底盘 (200Hz / 5 = 40Hz)
        c50c_send_cnt++;
        if( c50c_send_cnt>=5 )
        {
            c50c_send_cnt = 0;
            c10b_send.Head1 = C10B_HEAD1;
            c10b_send.Head2 = C10B_HEAD2;
            
            uint8_t allow_move = k210_active_flag;
            int16_t final_cx = k210.x; 
            
            // ============================================
            // 🛑 核心导航逻辑 (这里就是我们修改的地方) 🛑
            // ============================================
            if (TARGET_ID != 0) 
            {
                // --- 情况 A: 遇到路口 (Tag 0) ---
                if (k210.y == 0) 
                {
                    // 检查冷却锁
                    if (HAL_GetTick() > turn_cooldown_timestamp)
                    {
                        if (TARGET_ID == 1)      final_cx = -150; // 左转
                        else if (TARGET_ID == 2) final_cx = 150;  // 右转
                        turn_cooldown_timestamp = HAL_GetTick() + 3000; // 冷却3秒
                    }
                }
                
                // --- 情况 B: 遇到终点 (Tag 1 或 2) ---
                else if (k210.y == TARGET_ID) 
                {
                    // 距离检测 (放宽到 45cm)
                    if (k210.z <= 45) 
                    {
                        allow_move = 0; // 停车
                    }
                }
            }
            // ============================================

            c10b_send.k210_alive = allow_move;
            c10b_send.k210_cx = final_cx; 
            c10b_send.k210_size = k210.z; 
            
            // 发送数据
            c10b_send.distance = ranger_dis*1000;
            c10b_send.temperature = temperature*100;
            uint8_t* sendptr = (uint8_t*)&c10b_send;
            c10b_send.BCCcheck = calculateBCC(sendptr,sizeof(c10b_send)-1);
            HAL_UART_Transmit(&huart5,sendptr,sizeof(c10b_send),500);
        }
    }
}

// 错误处理函数
void Error_Handler(void)
{
  __disable_irq();
  while (1) { }
}