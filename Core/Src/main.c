/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
//Programa que tiene el codigo de la tarjeta de procesamiento central
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "string.h"
#include <stdio.h>
#include "ssd1306.h"
#include "fonts.h"
#include "test.h"
#include "math.h"
#include "Mensajes"


//bilbiotecas de cinematica inversa




/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SPI_MASTER hspi1 //comunicación spi para tarjetas de control (envía)
#define SPI_SLAVE hspi2 //comunicación spi para tarjetas de control (recibe)

#define OLED hi2c1 //comunicación i2c para oled

#define BT huart1 //comunicación uart para el módulo BT
#define HMI huart2 // comunicación uart para la HMI
#define USB huart3 // comunicación uart para el módulo USB
#define WIFI huart6 //comunicación uart para el modulo WiFi



/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim10;
TIM_HandleTypeDef htim14;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
UART_HandleTypeDef huart6;

/* USER CODE BEGIN PV */
//Direcciones de memoria
uint16_t tc1_addr=(0x11<<1);
uint16_t tc2_addr=(0x22<<1);
uint16_t tc3_addr=(0x33<<1);
uint16_t tc4_addr=(0x44<<1);
uint16_t tc5_addr=(0x55<<1);
uint16_t tc6_addr=(0x66<<1);

//Variables globales
uint8_t msj[4];//vector en el que fuardamos el mensaje
uint8_t msj_rcv[200];//vector en el que recibimos el mensaje
//Mensajes de hmi
uint8_t msj_hmi[8];//vector en el que guardamos el mensaje
uint8_t msj_hmi_rcv[9];//vector en el que recibimos el mensaje de la hmi

//Mensajes de computadora
uint8_t msj_pc_rcv[4];//vector en el que se recibe el mensaje de la computadora
uint8_t msj_pc_send[4];//vector para enviar mensaje a la computadora
uint8_t pc_flag=0;//bandera para mensajes de la computadora
//Modo diagnóstico
uint8_t active_gdl=0;
//Modo manual
uint8_t paso=0;//para paso x1
//Monitoreo
uint32_t num=1200;
uint8_t RHI,RHD,TCI,TCD,SUM;//Lecturas del sensor DHT11
uint32_t pMillis, cMillis;
float tCelsius=0.0;
float RH=0.0;
float q[6];//joint angle
float q_points[6];
float x=000.00;
float y=000.00;
float z=000.00;
float r=000.00;
float p=000.00;
float yaw=000.00;
float gpi_param[4];
float points[1000];
float sensor[1200];
float current[1200];
float error[1200];

//Efector final
uint32_t apperture=0;
/*//Cinematica inversa
double roll,pitch,yaw;
const double weights[6]={.25,.25,.25,1,1,1};
double Q1_data[1],Q2_data[1],Q3_data[1],Q4_data[1],Q5_data[1],Q6_data[1];
int Q1_size[2],Q2_size[2],Q3_size[2],Q4_size[2],Q5_size[2],Q6_size[2];*/

//Creación de trayectorias
float qd[1200];//vector que guarda os valores de la variable de junta deseadas
float qpd[1200];//vector que guarda los valores de la velocidad de junta deseadas
float qppd[1200];//vector que guarda las aceleraciones deseadas de la junta
float time_res=0.005;

//maquina de estados
uint8_t estado=0;//variable para la maquina de estados

//Banderas
int8_t timeout=0;//cuando sea 1 es que ha pasado el tiempo destinado para espera de un mensaje
uint8_t spc_flag=0;//bandera para detección de SPC
uint8_t msj_rcv_flag=0;
uint8_t hmi_flag=0;//bandera para mensajes de la HMI
//Pruebas
float derivada[2000];
float temperatura=12.34;
char temp_char[5];

uint32_t i;

UART_HandleTypeDef UART_com;
uint8_t estado_cerrado=0;//para maquina de estados del modo cerrado

int tarjeta=1;
float angle=180.0;
float speed=0.5;
float q0;

uint8_t msjprueba[1000];
float pwm1,pwm2;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USART6_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM10_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM14_Init(void);
static void MX_SPI2_Init(void);
/* USER CODE BEGIN PFP */
//Funciones de modos de operación
void Modo_Diagnostico(void);//Función que ejecuta el modo diagnostico
	void HMI_com_init(void);//Funcion que corrobora la comunicación con la HMI
	void TC_com_init(void);//Función que corrobora la comunicaicón con las tarjetas de control
	void OLED_diagnostico();//Función que inicializa y prueba la pantalla oled
	uint8_t GDL_connection(void);//Función que corrobora las conexiones de los GDL
	void SPC_diagnostico(void);//FUnción que corrobora el funcionamiento del SPC
void menu_hmi(void);//Función para el menú
void Modo_Cerrado(void);//Funcion que maneja el modo cerrado
	void PC_connection(void);//Funcion que establece comunicacion con la computadora
	void decode_function(void);//Función que decodifica el tipo de función que se envía de la computadora
void Modo_Abierto(void);//Función que maneja el modo abierto
void Modo_Manual(void);//Función que maneja el modo manual
//Funciones de comunicación
void send_msj_tc(uint8_t tarjeta, uint8_t size_send);//Función que manda mensaje a la tarjeta de control (spi)
void send_msj_tc_control(uint8_t index,uint8_t tarjeta, uint8_t size_send);
void rcv_msj_tc(uint8_t size_rcv);
//Funciones de monitoreo de parametros
void get_vref_spc(void);//Función de actualiza el valor de vref1 y vref2 del SPC
void load_sensor_data(void);
//Funciones de movimiento de modo cerrado
void send_control(int tarjeta);
void control_ready(int tarjeta);
void set_joint_angle(void);
void home_offset(void);
void get_joint_angle(int tarjeta);
void get_position(void);
void get_orientation(void);
void get_temperature(void);
void get_humidity(void);
void get_active_joints(void);
void get_gpi_param(void);
void get_tool_aperture(void);
void load_points(void);
void direct_kine(void);
//Funciones de monitoreo de modo cerrado
void get_joint_angle(int tarjeta);
//Funcions de generación de trayectorias
uint32_t trapezoidal(float tf, float q0, float qf, float v);//calcula la trayectoria trapezoidal
void reset_trayectoria(void);
//Funciones set
void set_tool(void);
void update_hmi_th(void);
void update_hmi(void);
//Funciones auxiliares para debuggear
void send_rcv_float(void);//funcion que muestra como pasar un float a uint32 para poder mandarlo y reconstruirlo
void caracterizar_motor(void);//funcion para recibir de la tc posiciones y mandarlas a la computadora
void delay_us(uint16_t us);
//Funciones DHT11
uint8_t DHT11_Start (void);
uint8_t DHT11_Read (void);
void microDelay(uint16_t delay);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
//Funciones para lectura de DHT11

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
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  MX_USART3_UART_Init();
  MX_USART6_UART_Init();
  MX_TIM1_Init();
  MX_TIM4_Init();
  MX_TIM10_Init();
  MX_TIM2_Init();
  MX_TIM14_Init();
  MX_SPI2_Init();
  /* USER CODE BEGIN 2 */
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  estado=0;
  while (1)
  {
	  switch(estado){
	  case 0://setup
		  //SET para pines de CS de SPI Master
		    HAL_GPIO_WritePin(TC_1_GPIO_Port, TC_1_Pin, SET);
		    HAL_GPIO_WritePin(TC_2_GPIO_Port, TC_2_Pin, SET);
		    HAL_GPIO_WritePin(TC_3_GPIO_Port, TC_3_Pin, SET);
		    HAL_GPIO_WritePin(TC_4_GPIO_Port, TC_4_Pin, SET);
		    HAL_GPIO_WritePin(TC_5_GPIO_Port, TC_5_Pin, SET);
		    HAL_GPIO_WritePin(TC_6_GPIO_Port, TC_6_Pin, SET);
		    //SET para pin de CS de SPI Slave
		    HAL_GPIO_WritePin(SPI_SLAVE_EN_GPIO_Port, SPI_SLAVE_EN_Pin, SET);
		    HAL_TIM_Base_Start(&htim4);//Timer de DHT11
		    HAL_TIM_Base_Start(&htim2);//Timer de funcion delay_us
		    HAL_TIM_PWM_Start(&htim14, TIM_CHANNEL_1); //PWM gripper
		    HAL_TIM_Base_Start_IT(&htim10);//Se inicializa el timer para leer DHT11
		    HAL_GPIO_WritePin(CONMUTADOR_GPIO_Port, CONMUTADOR_Pin, RESET);//Pin del conmutador
		    estado=1;
		  break;
	  case 1://Modo diagnóstico
		  Modo_Diagnostico();
		  estado=2;
		  break;
	  case 2://Menú
		  HAL_UART_Transmit(&HMI, &menu, 7, HAL_MAX_DELAY);
		  HAL_Delay(1);
		  HAL_UART_Receive(&HMI, &msj_hmi_rcv, 9,HAL_MAX_DELAY);//Espera de mensaje de siguiente de hmi
		  if(msj_hmi_rcv[4]==menu_cerrado[0] && msj_hmi_rcv[5]==menu_cerrado[1]){
			  HAL_UART_Transmit(&HMI, &modo_cerrado, 7, HAL_MAX_DELAY);
			  estado=3;
		  }
		  else if(msj_hmi_rcv[4]==menu_abierto[0]&&msj_hmi_rcv[5]==menu_abierto[1]){
			  HAL_UART_Transmit(&HMI, &modo_abierto, 7, HAL_MAX_DELAY);
			  estado=4;
		  }
		  else{
		  	HAL_UART_Transmit(&HMI, &modo_manual, 7, HAL_MAX_DELAY);
		  	estado=5;
		  }
		  break;
	  case 3://Modo cerrado
		  Modo_Cerrado();
		  estado=2;
		  break;
	  case 4://Modo abierto
		  Modo_Abierto();
		  estado=2;
		  break;
	  case 5://Modo manual
		  Modo_Manual();
		  estado=2;
		  break;
	  case 6:
		  HAL_UART_Transmit(&HMI, &modo_cerrado, 7, HAL_MAX_DELAY);
		  update_hmi_th();
		  break;
	  }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	//snprintf(&temp_char,5,"%f",temperatura);
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 96;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 8;
  RCC_OscInitStruct.PLL.PLLR = 2;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_SLAVE;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES_RXONLY;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi2.Init.NSS = SPI_NSS_HARD_INPUT;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 96-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 10000;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 90-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
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
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 96-1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief TIM10 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM10_Init(void)
{

  /* USER CODE BEGIN TIM10_Init 0 */

  /* USER CODE END TIM10_Init 0 */

  /* USER CODE BEGIN TIM10_Init 1 */

  /* USER CODE END TIM10_Init 1 */
  htim10.Instance = TIM10;
  htim10.Init.Prescaler = 19200-1;
  htim10.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim10.Init.Period = 50000;
  htim10.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim10.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim10) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM10_Init 2 */

  /* USER CODE END TIM10_Init 2 */

}

/**
  * @brief TIM14 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM14_Init(void)
{

  /* USER CODE BEGIN TIM14_Init 0 */

  /* USER CODE END TIM14_Init 0 */

  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM14_Init 1 */

  /* USER CODE END TIM14_Init 1 */
  htim14.Instance = TIM14;
  htim14.Init.Prescaler = 96-1;
  htim14.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim14.Init.Period = 20000;
  htim14.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim14.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim14) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim14) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim14, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM14_Init 2 */

  /* USER CODE END TIM14_Init 2 */
  HAL_TIM_MspPostInit(&htim14);

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 9600;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief USART6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 115200;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */

  /* USER CODE END USART6_Init 2 */

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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CONMUTADOR_GPIO_Port, CONMUTADOR_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LD1_Pin|SPI_SLAVE_EN_Pin|LD3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOF, TC_1_Pin|TC_2_Pin|TC_3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOG, TC_4_Pin|TC_5_Pin|USB_PowerSwitchOn_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, TC_6_Pin|TC1_ready_Pin|TC2_ready_Pin|TC3_ready_Pin
                          |TC4_ready_Pin|TC5_ready_Pin|TC6_ready_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(DHT11_GPIO_Port, DHT11_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : TC1_CS_Pin TC5_CS_Pin TC2_CS_Pin TC3_CS_Pin
                           TC4_CS_Pin */
  GPIO_InitStruct.Pin = TC1_CS_Pin|TC5_CS_Pin|TC2_CS_Pin|TC3_CS_Pin
                          |TC4_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : USER_Btn_Pin */
  GPIO_InitStruct.Pin = USER_Btn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USER_Btn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : TC6_CS_Pin */
  GPIO_InitStruct.Pin = TC6_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(TC6_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : CONMUTADOR_Pin */
  GPIO_InitStruct.Pin = CONMUTADOR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(CONMUTADOR_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD1_Pin SPI_SLAVE_EN_Pin LD3_Pin */
  GPIO_InitStruct.Pin = LD1_Pin|SPI_SLAVE_EN_Pin|LD3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : SPC_Pin */
  GPIO_InitStruct.Pin = SPC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(SPC_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : TC_1_Pin TC_2_Pin TC_3_Pin */
  GPIO_InitStruct.Pin = TC_1_Pin|TC_2_Pin|TC_3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pins : TC_4_Pin TC_5_Pin USB_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = TC_4_Pin|TC_5_Pin|USB_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pins : TC_6_Pin TC1_ready_Pin TC2_ready_Pin TC3_ready_Pin
                           TC4_ready_Pin TC5_ready_Pin TC6_ready_Pin */
  GPIO_InitStruct.Pin = TC_6_Pin|TC1_ready_Pin|TC2_ready_Pin|TC3_ready_Pin
                          |TC4_ready_Pin|TC5_ready_Pin|TC6_ready_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : DMS_Pin */
  GPIO_InitStruct.Pin = DMS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(DMS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PARO_Pin */
  GPIO_InitStruct.Pin = PARO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(PARO_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_OverCurrent_Pin */
  GPIO_InitStruct.Pin = USB_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : USB_ID_Pin USB_DP_Pin */
  GPIO_InitStruct.Pin = USB_ID_Pin|USB_DP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF10_OTG_FS;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : DHT11_Pin */
  GPIO_InitStruct.Pin = DHT11_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(DHT11_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
//Aquí se describen todas los prototipos de funciones especificados
void Modo_Diagnostico(void)
{
	OLED_diagnostico();
	HMI_com_init();
	TC_com_init();
	active_gdl=GDL_connection();
	//SPC_diagnostico();
}

void Modo_Cerrado(void){
	msj[0]=3;//Mensajes para avisar a las tarjetas que se trabaja en modo cerrado
	msj[1]=2;
	send_msj_tc(1, 2);
	send_msj_tc(2, 2);
	send_msj_tc(3, 2);
	send_msj_tc(4, 2);
	send_msj_tc(5, 2);
	send_msj_tc(6, 2);
	for(i=1;i<=active_gdl;i++){
		get_joint_angle(i);
	}
	direct_kine();
	update_hmi();
	update_hmi_th();
	PC_connection();//Definir el tipo de conexión con la computadora
	hmi_flag=0;
	do{
		HAL_UART_Receive(&UART_com, &msj_pc_rcv, 2, HAL_MAX_DELAY);
		decode_function();
	}while(hmi_flag!=1);
	msj[0]=4;//Mensajes para avisar a las tarjetas esperen
	msj[1]=4;
	send_msj_tc(1, 2);
	send_msj_tc(2, 2);
	send_msj_tc(3, 2);
	send_msj_tc(4, 2);
	send_msj_tc(5, 2);
	send_msj_tc(6, 2);
}
void Modo_Abierto(void){
	HAL_GPIO_WritePin(CONMUTADOR_GPIO_Port, CONMUTADOR_Pin, SET);
	HAL_UART_Receive(&HMI, &msj_hmi_rcv, 9, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(CONMUTADOR_GPIO_Port, CONMUTADOR_Pin, RESET);
}
void Modo_Manual(void){
	msj[0]=0x03;//Mensajes para avisar a las tarjetas que se trabaja en modo manual
	msj[1]=0x03;
	send_msj_tc(1, 2);
	send_msj_tc(2, 2);
	send_msj_tc(3, 2);
	send_msj_tc(4, 2);
	send_msj_tc(5, 2);
	send_msj_tc(6, 2);
	for(i=1;i<=active_gdl;i++){
		msj[0]=0x04;
		msj[1]=0x04;
		send_msj_tc(i, 2);
		HAL_Delay(1);
		get_joint_angle(i);
	}
	direct_kine();
	update_hmi();
	hmi_flag=0;
	do{
		HAL_UART_Receive(&HMI, &msj_hmi_rcv, 9, HAL_MAX_DELAY);
		do{}while(HAL_GPIO_ReadPin(DMS_GPIO_Port, DMS_Pin)!=RESET);
		if(msj_hmi_rcv[4]==pasox1[0]&&msj_hmi_rcv[5]==pasox1[1]){
			paso=0;//para paso x1
		}
		else if(msj_hmi_rcv[4]==pasox10[0]&&msj_hmi_rcv[5]==pasox10[1]){
			paso=1;//para paso x10
		}
		else if(msj_hmi_rcv[4]==pasox20[0]&&msj_hmi_rcv[5]==pasox20[1]){
			paso=2;//para paso x20
		}
		else if(msj_hmi_rcv[4]==manual_guardar[0]&&msj_hmi_rcv[5]==manual_guardar[1]){
			 for(i=1;i<=active_gdl;i++){
				 q_points[i-1]=q[i-1];
			 }
		}
		else if(msj_hmi_rcv[4]==return_menu[0]&&msj_hmi_rcv[5]==return_menu[1]){
			hmi_flag=1;
		}
		else{
			msj[0]=paso;
			switch(msj_hmi_rcv[4]){
			case 0://junta 1 mas
				msj[1]=1;
				send_msj_tc(1, 2);
				HAL_Delay(250);
				get_joint_angle(1);
				break;
			case 1://junta 1 menos
				msj[1]=0;
				send_msj_tc(1, 2);
				HAL_Delay(250);
				get_joint_angle(1);
				break;
			case 2://junta 2 mas
				msj[1]=1;
				send_msj_tc(2, 2);
				HAL_Delay(250);
				get_joint_angle(2);
				break;
			case 3://junta 2 menos
				msj[1]=0;
				send_msj_tc(2, 2);
				HAL_Delay(250);
				get_joint_angle(2);
				break;
			case 4://junta 3 mas
				msj[1]=1;
				send_msj_tc(3, 2);
				HAL_Delay(250);
				get_joint_angle(3);
				break;
			case 5://junta 3 menos
				msj[1]=0;
				send_msj_tc(3, 2);
				HAL_Delay(250);
				get_joint_angle(3);
				break;
			case 6://junta 4 mas
				msj[1]=1;
				send_msj_tc(4, 2);
				HAL_Delay(250);
				get_joint_angle(4);
				break;
			case 7://junta 4 menos
				msj[1]=0;
				send_msj_tc(4, 2);
				HAL_Delay(250);
				get_joint_angle(4);
				break;
			case 8://junta 5 mas
				msj[1]=1;
				send_msj_tc(5, 2);
				HAL_Delay(250);
				get_joint_angle(5);
				break;
			case 9://junta 5 menos
				msj[1]=0;
				send_msj_tc(5, 2);
				HAL_Delay(250);
				get_joint_angle(5);
				break;
			case 10://junta 6 mas
				msj[1]=1;
				send_msj_tc(6, 2);
				HAL_Delay(250);
				get_joint_angle(6);
				break;
			case 11://junta 6 menos
				msj[1]=0;
				send_msj_tc(6, 2);
				HAL_Delay(250);
				get_joint_angle(6);
				break;
			}
			direct_kine();
			update_hmi();
		}
	}while(hmi_flag!=1);
	msj[0]=3;//Mensajes para avisar a las tarjetas esperen otro mensaje
	msj[1]=4;
	send_msj_tc(1, 2);
	send_msj_tc(2, 2);
	send_msj_tc(3, 2);
	send_msj_tc(4, 2);
	send_msj_tc(5, 2);
	send_msj_tc(6, 2);
}
//Funciones modo diagnóstico
void OLED_diagnostico(void)
{
	SSD1306_Init();
	SSD1306_GotoXY (10,10);
	SSD1306_Puts ("Inicializando...", &Font_7x10, 1);
	SSD1306_UpdateScreen();
	HAL_Delay(2000);
	SSD1306_Clear();
	SSD1306_GotoXY (10,10);
	SSD1306_Puts ("Sistema robotico", &Font_7x10, 1);
	SSD1306_GotoXY (10,30);
	SSD1306_Puts ("didactico", &Font_7x10, 1);
	SSD1306_GotoXY (10,50);
	SSD1306_Puts ("UPIITA-IPN", &Font_7x10, 1);
	SSD1306_UpdateScreen();
	HAL_Delay(2000);
}
void HMI_com_init(void)
{
	//uint8_t hmi_init[9];
	HAL_UART_Receive(&HMI, &msj_hmi_rcv, 9, HAL_MAX_DELAY);
	HAL_Delay(500);
	if(msj_hmi_rcv[4]==button_con_HMI[0]&&msj_hmi_rcv[5]==button_con_HMI[1])
	{
		HAL_Delay(500);
		HAL_UART_Transmit(&HMI, &suc_con_HMI, 7, HAL_MAX_DELAY);
	}
	else
	{
		SSD1306_Clear();//128x64
		SSD1306_GotoXY (10,10);
		SSD1306_Puts ("ERROR HMI:", &Font_7x10, 1);
		SSD1306_GotoXY (10, 20);
		SSD1306_Puts ("No detectada", &Font_7x10, 1);
		SSD1306_UpdateScreen();
	}
}
void TC_com_init(void)
{
	HAL_UART_Receive(&HMI, &msj_hmi_rcv, 9,HAL_MAX_DELAY);
	for(int i=1;i<=5;i++)
	{
		msj[0]=i;//Cargar mensaje para la tarjeta
		send_msj_tc(i, 1);//Manda a la tarjeta i un dato de 1 byte
		rcv_msj_tc(1);//Recibe un mensaje de 1 byte
		switch(msj_rcv[0])
		{
			case 1:
				HAL_UART_Transmit(&HMI, &suc_con_tc1, 7, HAL_MAX_DELAY);
				HAL_Delay(1000);
				break;
			case 2:
				HAL_UART_Transmit(&HMI, &suc_con_tc2, 7, HAL_MAX_DELAY);
				HAL_Delay(1000);
				break;
			case 3:
				HAL_UART_Transmit(&HMI, &suc_con_tc3, 7, HAL_MAX_DELAY);
				HAL_Delay(1000);
				break;
			case 4:
				HAL_UART_Transmit(&HMI, &suc_con_tc4, 7, HAL_MAX_DELAY);
				HAL_Delay(1000);
				break;
			case 5:
				HAL_UART_Transmit(&HMI, &suc_con_tc5, 7, HAL_MAX_DELAY);
				HAL_Delay(1000);
				break;
			case 6:
				HAL_UART_Transmit(&HMI, &suc_con_tc6, 7, HAL_MAX_DELAY);
				HAL_Delay(1000);
				break;
		}
	}
}
uint8_t GDL_connection()
{
	uint8_t gdl_connected=0;
	uint8_t gdl_connected_flag=0;
	HAL_UART_Receive(&HMI, &msj_hmi_rcv, 9,HAL_MAX_DELAY);
	do{
		gdl_connected++;
		HAL_UART_Receive(&HMI, &msj_hmi_rcv, 9,HAL_MAX_DELAY);//HOME
		msj[0]=gdl_connected;
		send_msj_tc(gdl_connected, 1);
		control_ready(gdl_connected);
		HAL_Delay(10);
		HAL_UART_Transmit(&HMI, &junta_lista, 27, HAL_MAX_DELAY);
		HAL_UART_Receive(&HMI, &msj_hmi_rcv, 9,HAL_MAX_DELAY);//SIGUIENTE GDL
		if(msj_hmi_rcv[4]==next[0]&&msj_hmi_rcv[5]==next[1]){
			switch(gdl_connected){
			case 1:
				HAL_UART_Transmit(&HMI, &suc_con_gdl2, 7, HAL_MAX_DELAY);
				HAL_UART_Transmit(&HMI, &junta_null, 27, HAL_MAX_DELAY);
				break;
			case 2:
				HAL_UART_Transmit(&HMI, &suc_con_gdl3, 7, HAL_MAX_DELAY);
				HAL_UART_Transmit(&HMI, &junta_null, 27, HAL_MAX_DELAY);
				break;
			case 3:
				HAL_UART_Transmit(&HMI, &suc_con_gdl4, 7, HAL_MAX_DELAY);
				HAL_UART_Transmit(&HMI, &junta_null, 27, HAL_MAX_DELAY);
				break;
			case 4:
				HAL_UART_Transmit(&HMI, &suc_con_gdl5, 7, HAL_MAX_DELAY);
				HAL_UART_Transmit(&HMI, &junta_null, 27, HAL_MAX_DELAY);
				break;
			}
		}
		else{
			gdl_connected_flag=1;
		}
	}while(gdl_connected_flag==0);
	return gdl_connected;
}
void SPC_diagnostico(void)
{
	for (int i=2;i<=active_gdl;i++)
	{
		do{}while(HAL_GPIO_ReadPin(SPC_GPIO_Port, SPC_Pin)==RESET);
		switch(i)
		{
		case 2:
			HAL_UART_Transmit(&HMI, &spc2, 9, HAL_MAX_DELAY);
			HAL_UART_Receive(&HMI, &msj_hmi_rcv, 9, HAL_MAX_DELAY);
			break;
		case 3:
			HAL_UART_Transmit(&HMI, &spc3, 9, HAL_MAX_DELAY);
			HAL_UART_Receive(&HMI, &msj_hmi_rcv, 9,HAL_MAX_DELAY);
			break;
		case 4:
			HAL_UART_Transmit(&HMI, &spc4, 9, HAL_MAX_DELAY);
			HAL_UART_Receive(&HMI, &msj_hmi_rcv, 9,HAL_MAX_DELAY);
			break;
		case 5:
			HAL_UART_Transmit(&HMI, &spc5, 9, HAL_MAX_DELAY);
			HAL_UART_Receive(&HMI, &msj_hmi_rcv, 9,HAL_MAX_DELAY);
			break;
		}
	}
	HAL_UART_Transmit(&HMI, &spc_done, 9, HAL_MAX_DELAY);
}
//Funciones modo cerrado
void PC_connection(void){
	HAL_UART_Receive(&HMI, &msj_hmi_rcv, 9, HAL_MAX_DELAY);
	if(msj_hmi_rcv[7]==conexion_usb[0]&&msj_hmi_rcv[8]==conexion_usb[1]){
		UART_com=USB;
	}
	else if(msj_hmi_rcv[7]==conexion_wifi[0]&&msj_hmi_rcv[8]==conexion_wifi[1]){
		UART_com=WIFI;
	}
	else{
		UART_com=BT;
	}
}
void decode_function(void){
	switch(msj_pc_rcv[0]){
	case 1://Funciones de movimiento
		switch(msj_pc_rcv[1]){
		case 1://set_joint_angle(int joint, float angle, float speed):
			set_joint_angle();
			update_hmi();
			break;
		case 2://set_position(float xf, float yf, float zf, float phi/roll, float theta/pitch, float psi/yaw, int frame, float speed, float aux_x, float aux_y, float aux_z):
			//set_position();
			break;
		case 3://move_X(float x, int frame, float speed)
			//move_X();
			break;
		case 4://move_Y(float y, int frame, float speed)
			//move_Y();
			break;
		case 5://move_Z(float z, int frame, float speed)
			//move_Z();
			break;
		case 6://home()
			home_offset();
			update_hmi();
			break;
		}
		break;
	case 2://Funciones de monitoreo
		switch(msj_pc_rcv[1]){
		case 1://get_joint_angle(int joint)
			HAL_UART_Receive(&UART_com, &msj_pc_rcv, 1, HAL_MAX_DELAY);
			tarjeta=msj_pc_rcv[0];
			get_joint_angle(tarjeta);
			HAL_Delay(1);
			HAL_UART_Transmit(&UART_com,&msj_rcv, 4,HAL_MAX_DELAY);//Enviar dato a la computadora
			break;
		case 2://get_position(int frame):
			get_position();
			break;
		case 3://get_orientation(int frame)
			get_orientation();
			break;
		case 4://get_temperature()
			get_temperature();
			HAL_Delay(1);
			HAL_UART_Transmit(&UART_com, &msj_pc_send, 4, HAL_MAX_DELAY);
			break;
		case 5://get_humidity()
			get_humidity();
			HAL_Delay(1);
			HAL_UART_Transmit(&UART_com, &msj_pc_send, 4, HAL_MAX_DELAY);
			break;
		case 6://get_active_joints()
			get_active_joints();
			break;
		case 7://get_GPI_param(int joint)
			get_gpi_param();
			break;
		case 8://get_tool_apperture()
			get_tool_apperture();
			break;
		case 9://load_points(int i);
			//load_points(int i);
			break;
		case 10://load_sensor_data(int sens, int i);
			load_sensor_data();
			break;
		}
		break;
	case 3://Funciones set
		switch(msj_pc_rcv[1]){
		case 1://set_frame_position(int i)
			//set_frame_position();
			break;
		case 2://set_frame_orientation (int i, vector3 Dir_x, vector3 Dir_y, vector3 Dir_z)
			//set_frame_orientation();
			break;
		case 3://config(int Euler/RPY, int joint/linnear/circular, int tool, int SPC)
			//config();
			break;
		case 4://set_tool(float aperture)
			set_tool();
			break;
		case 5://clear_frames()
			//clear_frames();
			break;
		case 6://clear_points()
			//clear_points();
			break;
		}
		break;
	case 4:
		hmi_flag=1;
		break;
	}
}
//Funciones generales
void send_msj_tc(uint8_t tarjeta, uint8_t size_send)
{
	switch(tarjeta)
	{
	case 1:
			HAL_GPIO_WritePin(TC_1_GPIO_Port, TC_1_Pin, RESET);//CS yes
			delay_us(100);
			HAL_SPI_Transmit(&SPI_MASTER, &msj, size_send,HAL_MAX_DELAY);//send data
			HAL_GPIO_WritePin(TC_1_GPIO_Port, TC_1_Pin, SET);//CS no
			break;
	case 2:
			HAL_GPIO_WritePin(TC_2_GPIO_Port, TC_2_Pin, RESET);//CS yes
			delay_us(100);
			HAL_SPI_Transmit(&SPI_MASTER, &msj, size_send, HAL_MAX_DELAY);//send data
			HAL_GPIO_WritePin(TC_2_GPIO_Port, TC_2_Pin, SET);//CS no
			break;
	case 3:
			HAL_GPIO_WritePin(TC_3_GPIO_Port, TC_3_Pin, RESET);//CS yes
			delay_us(100);
			HAL_SPI_Transmit(&SPI_MASTER, &msj, size_send, HAL_MAX_DELAY);//send data
			HAL_GPIO_WritePin(TC_3_GPIO_Port, TC_3_Pin, SET);//CS no
			break;
	case 4:
			HAL_GPIO_WritePin(TC_4_GPIO_Port, TC_4_Pin, RESET);//CS yes
			delay_us(100);
			HAL_SPI_Transmit(&SPI_MASTER, &msj, size_send, HAL_MAX_DELAY);//send data
			HAL_GPIO_WritePin(TC_4_GPIO_Port, TC_4_Pin, SET);//CS no
			break;
	case 5:
			HAL_GPIO_WritePin(TC_5_GPIO_Port, TC_5_Pin, RESET);//CS yes
			delay_us(100);
			HAL_SPI_Transmit(&SPI_MASTER, &msj, size_send, HAL_MAX_DELAY);//send data
			HAL_GPIO_WritePin(TC_5_GPIO_Port, TC_5_Pin, SET);//CS no
			break;
	case 6:
			HAL_GPIO_WritePin(TC_6_GPIO_Port, TC_6_Pin, RESET);//CS yes
			delay_us(100);
			HAL_SPI_Transmit(&SPI_MASTER, &msj, size_send, HAL_MAX_DELAY);//send data
			HAL_GPIO_WritePin(TC_6_GPIO_Port, TC_6_Pin, SET);//CS no
			break;
	}
}
void send_msj_tc_control(uint8_t index,uint8_t tarjeta, uint8_t size_send)
{
	switch(tarjeta)
	{
	case 1:
			HAL_GPIO_WritePin(TC_1_GPIO_Port, TC_1_Pin, RESET);//CS yes
			HAL_Delay(1);
			HAL_SPI_Transmit(&SPI_MASTER, &msj[index], size_send,HAL_MAX_DELAY);//send data
			HAL_Delay(1);
			HAL_GPIO_WritePin(TC_1_GPIO_Port, TC_1_Pin, SET);//CS no
			break;
	case 2:
			HAL_GPIO_WritePin(TC_2_GPIO_Port, TC_2_Pin, RESET);//CS yes
			HAL_Delay(1);
			HAL_SPI_Transmit(&SPI_MASTER, &msj[index], size_send, HAL_MAX_DELAY);//send data
			HAL_Delay(1);
			HAL_GPIO_WritePin(TC_2_GPIO_Port, TC_2_Pin, SET);//CS no
			break;
	case 3:
			HAL_GPIO_WritePin(TC_3_GPIO_Port, TC_3_Pin, RESET);//CS yes
			HAL_Delay(1);
			HAL_SPI_Transmit(&SPI_MASTER, &msj[index], size_send, HAL_MAX_DELAY);//send data
			HAL_Delay(1);
			HAL_GPIO_WritePin(TC_3_GPIO_Port, TC_3_Pin, SET);//CS no
			break;
	case 4:
			HAL_GPIO_WritePin(TC_4_GPIO_Port, TC_4_Pin, RESET);//CS yes
			HAL_Delay(1);
			HAL_SPI_Transmit(&SPI_MASTER, &msj[index], size_send, HAL_MAX_DELAY);//send data
			HAL_Delay(1);
			HAL_GPIO_WritePin(TC_4_GPIO_Port, TC_4_Pin, SET);//CS no
			break;
	case 5:
			HAL_GPIO_WritePin(TC_5_GPIO_Port, TC_5_Pin, RESET);//CS yes
			HAL_Delay(1);
			HAL_SPI_Transmit(&SPI_MASTER, &msj[index], size_send, HAL_MAX_DELAY);//send data
			HAL_Delay(1);
			HAL_GPIO_WritePin(TC_5_GPIO_Port, TC_5_Pin, SET);//CS no
			break;
	case 6:
			HAL_GPIO_WritePin(TC_6_GPIO_Port, TC_6_Pin, RESET);//CS yes
			HAL_Delay(1);
			HAL_SPI_Transmit(&SPI_MASTER, &msj[index], size_send, HAL_MAX_DELAY);//send data
			HAL_Delay(1);
			HAL_GPIO_WritePin(TC_6_GPIO_Port, TC_6_Pin, SET);//CS no
			break;
	}
}
void rcv_msj_tc(uint8_t size_rcv)
{
	msj_rcv_flag=0;
	do{
		if((HAL_GPIO_ReadPin(TC1_CS_GPIO_Port, TC1_CS_Pin)==RESET)||
		  (HAL_GPIO_ReadPin(TC2_CS_GPIO_Port, TC2_CS_Pin)==RESET)||
		(HAL_GPIO_ReadPin(TC3_CS_GPIO_Port, TC3_CS_Pin)==RESET)||
		(HAL_GPIO_ReadPin(TC4_CS_GPIO_Port, TC4_CS_Pin)==RESET)||
		(HAL_GPIO_ReadPin(TC5_CS_GPIO_Port, TC5_CS_Pin)==RESET)){
		//if((HAL_GPIO_ReadPin(TC2_CS_GPIO_Port, TC2_CS_Pin)==RESET)){

			msj_rcv_flag=1;
		}
	}while(msj_rcv_flag==0);
	HAL_GPIO_WritePin(SPI_SLAVE_EN_GPIO_Port, SPI_SLAVE_EN_Pin, RESET);//Este pin también va a un pin de ready de la tarjeta de control
	HAL_SPI_Receive(&SPI_SLAVE, &msj_rcv, size_rcv, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(SPI_SLAVE_EN_GPIO_Port, SPI_SLAVE_EN_Pin, SET);
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	//Casos para mensajes de hmi del modo cerrado
	if(huart->Instance==USART2){
		if(msj_hmi_rcv[4]==return_menu[0]&&msj_hmi_rcv[5]==return_menu[1]){
			hmi_flag=1;
		}
		else{
			HAL_UART_Receive_IT(&HMI, msj_hmi_rcv, 9);
		}
	}
	else{
		pc_flag=1;
	}

}
void HAL_GPIO_EXTI_Callback (uint16_t GPIO_Pin)
{
	if (GPIO_Pin==SPC_Pin)
	{
		spc_flag=1;
	}
}
void delay_us(uint16_t us){
	__HAL_TIM_SET_COUNTER(&htim2,0);
	while(__HAL_TIM_GET_COUNTER(&htim2)<us);
}
uint32_t trapezoidal(float tf, float q0, float qf, float v)
{
	float vmin,vmax,tb,a0,a1,a2,b0,b1,c0,c1,c2;
	vmin=(qf-q0)/tf;
	vmax=2*(qf-q0)/tf;
	v=(vmax-vmin)*v+vmin;
	tb=(q0-qf+v*tf)/v;
	a0=q0;
	a1=0;
	a2=v/(2*tb);
	b0=(q0+qf-v*tf)/2;
	b1=v;
	c0=qf-tf*tf*(v/(2*tb));
	c1=v/tb*tf;
	c2=-(v/tb)/2;
	uint32_t tf_m=tf*1/time_res;//cantidad de milisegundos entre cero y tf
	uint32_t tb_m=tb*1/time_res;//cantidad de milisegundos entre 0 y tb
	float t;
	uint32_t i;
	for(i=0;i<tb_m;i++)
	{
		t=(float)i*time_res;
		qd[i]=a0+a1*t+a2*t*t;
		qpd[i]=a1+2*a2*t;
		qppd[i]=2*a2;
	}
	for(i=tb_m;i<tf_m-tb_m;i++)
	{
		t=(float)i*time_res;
		qd[i]=b0+b1*t;
		qpd[i]=b1;
		qppd[i]=0;
	}
	for(i=tf_m-tb_m;i<tf_m;i++)
	{
		t=(float)i*time_res;
		qd[i]=c0+c1*t+c2*t*t;
		qpd[i]=c1+2*c2*t;
		qppd[i]=2*c2;
	}
	for(i=tf_m-1;i<num;i++){
		qd[i]=qf;
		qpd[i]=0;
		qppd[i]=0;
	}
	return i;
}
void reset_trayectoria(void)
{
	for(i=0;i<num;i++)
	{
		qd[i]=0;
		qpd[i]=0;
		qppd[i]=0;
	}
}
void send_control(int tarjeta){
	uint32_t mensaje;
	msj[0]=0x01;
	msj[1]=0x01;
	send_msj_tc(tarjeta, 2);
	for(i=0;i<num;i++){
		memcpy(msj,&qd[i],4);
		send_msj_tc(tarjeta, 4);
		delay_us(200);
	}
	for(i=0;i<num;i++){
		memcpy(msj,&qpd[i],4);
		send_msj_tc(tarjeta, 4);
		delay_us(200);
	}
	for(i=0;i<num;i++){
		memcpy(msj,&qppd[i],4);
		send_msj_tc(tarjeta, 4);
		delay_us(200);
	}
}
void control_ready(int tarjeta){
	switch(tarjeta){
		case 1:
			HAL_GPIO_WritePin(TC1_ready_GPIO_Port, TC1_ready_Pin, SET);
			do{}while(HAL_GPIO_ReadPin(TC1_CS_GPIO_Port, TC1_CS_Pin)==SET);
			HAL_GPIO_WritePin(TC1_ready_GPIO_Port, TC1_ready_Pin, RESET);
			break;
		case 2:
			HAL_GPIO_WritePin(TC2_ready_GPIO_Port, TC2_ready_Pin, SET);
			do{}while(HAL_GPIO_ReadPin(TC2_CS_GPIO_Port, TC2_CS_Pin)==SET);
			HAL_GPIO_WritePin(TC2_ready_GPIO_Port, TC2_ready_Pin, RESET);
			break;
		case 3:
			HAL_GPIO_WritePin(TC3_ready_GPIO_Port, TC3_ready_Pin, SET);
			do{}while(HAL_GPIO_ReadPin(TC3_CS_GPIO_Port, TC3_CS_Pin)==SET);
			HAL_GPIO_WritePin(TC3_ready_GPIO_Port, TC3_ready_Pin, RESET);
			break;
		case 4:
			HAL_GPIO_WritePin(TC4_ready_GPIO_Port, TC4_ready_Pin, SET);
			do{}while(HAL_GPIO_ReadPin(TC4_CS_GPIO_Port, TC4_CS_Pin)==SET);
			HAL_GPIO_WritePin(TC4_ready_GPIO_Port, TC4_ready_Pin, RESET);
			break;
		case 5:
			HAL_GPIO_WritePin(TC5_ready_GPIO_Port, TC5_ready_Pin, SET);
			do{}while(HAL_GPIO_ReadPin(TC5_CS_GPIO_Port, TC5_CS_Pin)==SET);
			HAL_GPIO_WritePin(TC5_ready_GPIO_Port, TC5_ready_Pin, RESET);
			break;
	}
}
void send_rcv_float(void)
{
	float pi,pi_reconstruido;
	uint32_t pi_entero,reconstruido;
	uint8_t mensaje[4];
	pi=atof("3.1416");//obtenemos el flotante a partir del string
	pi_entero=*(int*)&pi;//obtenemos el valor entero de ese flotante
	memcpy(mensaje,&pi_entero,sizeof(pi_entero));//lo separamos en 4 bytes
	reconstruido=mensaje[0]|(mensaje[1]<<8)|(mensaje[2]<<16)|(mensaje[3]<<24);//reconstruimos el valor
	//pi_reconstruido=*(float*)&reconstruido;//obtenemos de nuevo el entero
	pi_reconstruido=*(float*)&mensaje;//obtenemos de nuevo el entero
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){//Timer de actualización de datos de temperatura y humedad
	if(htim->Instance==TIM10){
		update_hmi_th();
	}
}

void caracterizar_motor(void)
{
	HAL_Delay(1500);
	for(i=0;i<200;i++)//recibimos las 400 posiciones registradas desde la tarjeta de control
	{
		rcv_msj_tc(4);
		derivada[i]=*(float*)&msj_rcv;
	}

	for(i=0;i<200;i++)//mandamos las 400 posiciones registradas a la computadora por usb
	{
		HAL_UART_Transmit(&USB, &derivada[i], 4, HAL_MAX_DELAY);
	}
}

//Funciones de movimiento modo cerrado
void set_joint_angle(void){
	float q0,angle,speed;
	int tarjeta;
	HAL_UART_Receive(&UART_com, &msj_pc_rcv, 1,HAL_MAX_DELAY);//Tarjeta
	tarjeta=msj_pc_rcv[0];
	HAL_UART_Receive(&UART_com, &msj_pc_rcv, 4,HAL_MAX_DELAY);//Angulo
	angle=*(float*)&msj_pc_rcv;
	HAL_UART_Receive(&UART_com, &msj_pc_rcv, 4,HAL_MAX_DELAY);//Velocidad
	speed=*(float*)&msj_pc_rcv;
	get_joint_angle(tarjeta);
	q0=q[tarjeta-1];
	trapezoidal(4, q0 , angle, speed);
	send_control(tarjeta);
	control_ready(tarjeta);//Poner afuera de la máquina de estados
	HAL_Delay(1);
	get_joint_angle(tarjeta);
	direct_kine();
}
void home_offset(void){
	uint8_t tarjeta=0;
	HAL_UART_Receive(&UART_com, &msj_pc_rcv, 1, HAL_MAX_DELAY);
	tarjeta=msj_pc_rcv[0];
	msj[0]=1;
	msj[1]=6;
	send_msj_tc(tarjeta, 2);
	control_ready(tarjeta);
}
//Funciones de monitoreo modo cerrado
void get_joint_angle(int tarjeta){
	msj[0]=0x02;
	msj[1]=0x01;
	send_msj_tc(tarjeta, 2);
	rcv_msj_tc(4);
	q[tarjeta-1]=*(float*)&msj_rcv;
}
void get_position(void){
	for (i=1;i<active_gdl;i++){
		get_joint_angle(i);
	}
	direct_kine();//Guarda en variables globales x,y,z y los ángulos
	uint32_t mensaje;
	mensaje=*(int*)&x;
	memcpy(msj_pc_send,&mensaje,4);
	HAL_Delay(1);
	HAL_UART_Transmit(&UART_com, &msj_pc_send, 4, HAL_MAX_DELAY);
	mensaje=*(int*)&y;
	memcpy(msj_pc_send,&mensaje,4);
	HAL_Delay(1);
	HAL_UART_Transmit(&UART_com, &msj_pc_send, 4, HAL_MAX_DELAY);
	mensaje=*(int*)&z;
	memcpy(msj_pc_send,&mensaje,4);
	HAL_Delay(1);
	HAL_UART_Transmit(&UART_com, &msj_pc_send, 4, HAL_MAX_DELAY);
}
void get_orientation(void){
	for (i=1;i<active_gdl;i++){
		get_joint_angle(i);
	}
	direct_kine();//Guarda en variables globales x,y,z y los ángulos
	uint32_t mensaje;
	mensaje=*(int*)&r;
	memcpy(msj_pc_send,&mensaje,4);
	HAL_Delay(1);
	HAL_UART_Transmit(&UART_com, &msj_pc_send, 4, HAL_MAX_DELAY);
	mensaje=*(int*)&p;
	memcpy(msj_pc_send,&mensaje,4);
	HAL_Delay(1);
	HAL_UART_Transmit(&UART_com, &msj_pc_send, 4, HAL_MAX_DELAY);
	mensaje=*(int*)&y;
	memcpy(msj_pc_send,&mensaje,4);
	HAL_Delay(1);
	HAL_UART_Transmit(&UART_com, &msj_pc_send, 4, HAL_MAX_DELAY);
}
void get_temperature(void){
	if(DHT11_Start())
	{
		RHI = DHT11_Read(); // Relative humidity integral
	    RHD = DHT11_Read(); // Relative humidity decimal
	    TCI = DHT11_Read(); // Celsius integral
	    TCD = DHT11_Read(); // Celsius decimal
	    SUM = DHT11_Read(); // Check sum
	    if (RHI + RHD + TCI + TCD == SUM){
	    	tCelsius = (float)TCI + (float)(TCD/10.0);
	    }
	}
	uint32_t mensaje;
	mensaje=*(int*)&tCelsius;
	memcpy(msj_pc_send,&mensaje,4);
}
void get_humidity(void){
	if(DHT11_Start())
	{
		RHI = DHT11_Read(); // Relative humidity integral
	    RHD = DHT11_Read(); // Relative humidity decimal
	    TCI = DHT11_Read(); // Celsius integral
	    TCD = DHT11_Read(); // Celsius decimal
	    SUM = DHT11_Read(); // Check sum
	    if (RHI + RHD + TCI + TCD == SUM){
	        RH = (float)RHI + (float)(RHD/10.0);
	    }
	}
	uint32_t mensaje;
	mensaje=*(int*)&RH;
	memcpy(msj_pc_send,&mensaje,4);
}
void get_active_joints(void){
	msj_pc_send[0]=active_gdl;
	HAL_Delay(1);
	HAL_UART_Transmit(&UART_com, &msj_pc_send, 1, HAL_MAX_DELAY);
}
void get_gpi_param(void){
	uint8_t tarjeta;
	uint32_t mensaje;
	HAL_UART_Receive(&UART_com, &msj_pc_rcv, 1,HAL_MAX_DELAY);//Tarjeta
	tarjeta=msj_pc_rcv[0];
	msj[0]=0x02;
	msj[1]=0x07;
	send_msj_tc(tarjeta, 2);
	for(i=0;i<4;i++){
		rcv_msj_tc(4);
		gpi_param[i]=*(float*)&msj_rcv;
	}
	for(i=0;i<4;i++){
		mensaje=*(int*)&gpi_param[i];
		memcpy(msj_pc_send,&mensaje,4);
		HAL_Delay(1);
		HAL_UART_Transmit(&UART_com, msj_pc_send, 4, HAL_MAX_DELAY);
	}
}
void get_tool_apperture(void){
	uint32_t mensaje;
	mensaje=*(int*)&apperture;
	memcpy(msj_pc_send,&mensaje,4);
	HAL_Delay(1);
	HAL_UART_Transmit(&UART_com, msj_pc_send, 4, HAL_MAX_DELAY);
}
void load_points(void){
}
void load_sensor_data(void){
	uint8_t tarjeta=0;
	uint8_t tipo=0;
	uint16_t indice=0;
	//UART_com=USB;
	msj[0]=0x02;
	msj[1]=0x0A;
	HAL_UART_Transmit(&UART_com, &msj, 2, HAL_MAX_DELAY);
	HAL_UART_Receive(&UART_com, &msj_pc_rcv, 4, HAL_MAX_DELAY);
	tarjeta=*(uint8_t*)&msj_pc_rcv[0];
	tipo=*(uint8_t*)&msj_pc_rcv[1];
	memcpy(&indice,&msj_pc_rcv[2],2);
	msj[0]=0x02;
	msj[1]=0x0A;
	send_msj_tc(tarjeta, 2);
	delay_us(100);
	msj[0]=tipo;//0 posicion, 1 error, 2 corriente
	send_msj_tc(tarjeta, 1);
	delay_us(100);
	memcpy(&msj,&indice,2);
	send_msj_tc(tarjeta, 2);
	rcv_msj_tc(4);
	sensor[indice]=*(float*)&msj_rcv;
	HAL_UART_Transmit(&UART_com, &sensor[indice], 4, HAL_MAX_DELAY);
	/*for(i=0;i<24;i++){//posicion
		j=i*200;
		rcv_msj_tc(200);
		memcpy(&vec[j],&msj_rcv[0],200);
	}
	for(i=0;i<1200;i++){
		j=i*4;
		memcpy(vec_float,&vec[0],4);
		position[i]=*(float*)&vec_float;
	}
	for(i=0;i<24;i++){//error
		j=i*200;
		rcv_msj_tc(200);
		memcpy(&vec[j],&msj_rcv[0],200);
	}
	for(i=0;i<1200;i++){
		j=i*4;
		memcpy(vec_float,&vec[0],4);
		error[i]=*(float*)&vec_float;
	}
	for(i=0;i<24;i++){//corriente
		j=i*200;
		rcv_msj_tc(200);
		memcpy(&vec[j],&msj_rcv[0],200);
	}
	for(i=0;i<1200;i++){
		j=i*4;
		memcpy(vec_float,&vec[0],4);
		current[i]=*(float*)&vec_float;
	}
	for(i=1;i<600;i++){
		if(fabs(position[i]-position[i-1])>10){
			position[i]=position[i-1];
		}
		if(fabs(error[i]-error[i-1])>10){
			error[i]=error[i-1];
		}
		if(fabs(current[i]-current[i-1])>10){
			current[i]=current[i-1];
		}
	}
	//Enviar a MATLAB
	for(i=0;i<600;i++)//posición
	{
		HAL_UART_Transmit(&UART_com, &position[i], 4, HAL_MAX_DELAY);
	}
	for(i=0;i<600;i++)//error
	{
		HAL_UART_Transmit(&UART_com, &error[i], 4, HAL_MAX_DELAY);
	}
	for(i=0;i<600;i++)//current
	{
		HAL_UART_Transmit(&UART_com, &current[i], 4, HAL_MAX_DELAY);
	}*/
}
void direct_kine(void){
	//Declaraciones
	float l1=144.81;
	float l2=67.3;
	float l3=112.51;
	float l4=67.3;
	float l5=112.51;
	float l6=122.81;
	float l7=122.81;
	float l8=110.83;
	//Offsets
	float q1,q2,q3,q4,q5;
	float pi=3.1416;
	q1=(q[0]*pi)/180+pi;
	q2=pi/2+(q[1]*pi)/180;
	q3=pi/2+(q[2]*pi)/180;
	q4=(q[3]*pi)/180;
	q5=(q[4]*pi)/180;

	//Cálculo
	float r11,r12,r13,r21,r22,r23,r31,r32,r33;
	x=l7*(cos(q1)*cos(q2)*sin(q3) - cos(q1)*cos(q3)*sin(q2)) + l2*sin(q1) - l6*sin(q1) + l8*(cos(q4)*sin(q1) + sin(q4)*(cos(q1)*cos(q2)*cos(q3) + cos(q1)*sin(q2)*sin(q3))) + l3*cos(q1)*cos(q2);
	y=l7*(cos(q2)*sin(q1)*sin(q3) - cos(q3)*sin(q1)*sin(q2)) - l2*cos(q1) + l6*cos(q1) - l8*(cos(q1)*cos(q4) - sin(q4)*(sin(q1)*sin(q2)*sin(q3) + cos(q2)*cos(q3)*sin(q1))) + l3*cos(q2)*sin(q1);
	z=l1 + l3*sin(q2) + l7*cos(q2 - q3) + l8*sin(q2 - q3)*sin(q4);
}
void update_hmi(void){
	hmi_tool[6]=0x31;//Efector final activo
	HAL_UART_Transmit(&HMI, &hmi_tool, 7, HAL_MAX_DELAY);
	HAL_UART_Transmit(&HMI, &hmi_reference, 7, HAL_MAX_DELAY);
	snprintf(&hmi_x[6],8,"%f",x);
	HAL_UART_Transmit(&HMI, &hmi_x[0], 13, HAL_MAX_DELAY);
	snprintf(&hmi_y[6],8,"%f",y);
	HAL_UART_Transmit(&HMI, &hmi_y[0], 13, HAL_MAX_DELAY);
	snprintf(&hmi_z[6],8,"%f",z);
	HAL_UART_Transmit(&HMI, &hmi_z[0], 13, HAL_MAX_DELAY);
	snprintf(&hmi_j1[6],8,"%f",q[0]);
	HAL_UART_Transmit(&HMI, &hmi_j1[0], 13, HAL_MAX_DELAY);
	snprintf(&hmi_j2[6],8,"%f",q[1]);
	HAL_UART_Transmit(&HMI, &hmi_j2[0], 13, HAL_MAX_DELAY);
	snprintf(&hmi_j3[6],8,"%f",q[2]);
	HAL_UART_Transmit(&HMI, &hmi_j3[0], 13, HAL_MAX_DELAY);
	snprintf(&hmi_j4[6],8,"%f",q[3]);
	HAL_UART_Transmit(&HMI, &hmi_j4[0], 13, HAL_MAX_DELAY);
	snprintf(&hmi_j5[6],8,"%f",q[4]);
	HAL_UART_Transmit(&HMI, &hmi_j5[0], 13, HAL_MAX_DELAY);
}
void update_hmi_th(void){
	get_temperature();
	get_humidity();
	snprintf(&hmi_temp[6],6,"%f",tCelsius);
	hmi_temp[11]=0xA7;
	hmi_temp[12]=0x43;
	snprintf(&hmi_hum[6],3,"%f",RH);
	hmi_hum[8]=0x25;
	HAL_UART_Transmit(&HMI, &hmi_temp, 13, HAL_MAX_DELAY);
	HAL_UART_Transmit(&HMI, &hmi_hum, 9, HAL_MAX_DELAY);
}
//Funciones set
void set_tool(void){
	float g_angle;
	HAL_UART_Receive(&UART_com, &msj_pc_rcv, 4, HAL_MAX_DELAY);
	apperture=*(float*)&msj_pc_rcv;
	g_angle=(98.445-(apperture+2))/0.0849;
	htim14.Instance->CCR1=g_angle;
}
//Funciones DHT11
void microDelay(uint16_t delay){
	__HAL_TIM_SET_COUNTER(&htim4, 0);
	  while (__HAL_TIM_GET_COUNTER(&htim4) < delay);
}
uint8_t DHT11_Start (void)
{
  uint8_t Response = 0;
  GPIO_InitTypeDef GPIO_InitStructPrivate = {0};
  GPIO_InitStructPrivate.Pin = DHT11_Pin;
  GPIO_InitStructPrivate.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStructPrivate.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStructPrivate.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(DHT11_GPIO_Port, &GPIO_InitStructPrivate); // set the pin as output
  HAL_GPIO_WritePin (DHT11_GPIO_Port, DHT11_Pin, 0);   // pull the pin low
  HAL_Delay(20);   // wait for 20ms
  HAL_GPIO_WritePin (DHT11_GPIO_Port, DHT11_Pin, 1);   // pull the pin high
  microDelay (30);   // wait for 30us
  GPIO_InitStructPrivate.Mode = GPIO_MODE_INPUT;
  GPIO_InitStructPrivate.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(DHT11_GPIO_Port, &GPIO_InitStructPrivate); // set the pin as input
  microDelay (40);
  if (!(HAL_GPIO_ReadPin (DHT11_GPIO_Port, DHT11_Pin)))
  {
    microDelay (80);
    if ((HAL_GPIO_ReadPin (DHT11_GPIO_Port, DHT11_Pin))) Response = 1;
  }
  pMillis = HAL_GetTick();
  cMillis = HAL_GetTick();
  while ((HAL_GPIO_ReadPin (DHT11_GPIO_Port, DHT11_Pin)) && pMillis + 2 > cMillis)
  {
    cMillis = HAL_GetTick();
  }
  return Response;
}
uint8_t DHT11_Read (void)
{
  uint8_t a,b;
  for (a=0;a<8;a++)
  {
    pMillis = HAL_GetTick();
    cMillis = HAL_GetTick();
    while (!(HAL_GPIO_ReadPin (DHT11_GPIO_Port, DHT11_Pin)) && pMillis + 2 > cMillis)
    {  // wait for the pin to go high
      cMillis = HAL_GetTick();
    }
    microDelay (40);   // wait for 40 us
    if (!(HAL_GPIO_ReadPin (DHT11_GPIO_Port, DHT11_Pin)))   // if the pin is low
      b&= ~(1<<(7-a));
    else
      b|= (1<<(7-a));
    pMillis = HAL_GetTick();
    cMillis = HAL_GetTick();
    while ((HAL_GPIO_ReadPin (DHT11_GPIO_Port, DHT11_Pin)) && pMillis + 2 > cMillis)
    {  // wait for the pin to go low
      cMillis = HAL_GetTick();
    }
  }
  return b;
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
