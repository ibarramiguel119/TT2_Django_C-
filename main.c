/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <math.h> // Incluir librería matemática
#include <stdio.h> // Incluir librería estándar para sprintf
#include <stdlib.h> // Incluir librería estándar para itoa
#include <string.h> // Incluir librería estándar para strlen

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

typedef struct {
    float Kp;           // Proportional gain
    float Ki;           // Integral gain
    float Kd;           // Derivative gain
    float setpoint;     // Desired or target value
    float integral;     // Integral term accumulation
    float prevError;    // Previous error value (for derivative calculation)
    float prevTime;     // Previous time stamp (for derivative calculation)
} PID_Controller;


/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MAX_OUTPUT 50
#define BUFFER_SIZE 1000  // Ajusta este tamaño según tus necesidades
#define bufersize 1
#define TOLERANCE 0.001
#define VALOR_0 65
#define VALOR_PI 295
#define ENABLE_PIN_q1 GPIO_PIN_1
#define MS0_PIN_q1 GPIO_PIN_9
#define MS1_PIN_q1 GPIO_PIN_7
#define MS2_PIN_q1 GPIO_PIN_5
#define STEP_q1 GPIO_PIN_3
#define DIR_q1 GPIO_PIN_6
#define ENABLE_PIN_q2 GPIO_PIN_4
#define MS0_PIN_q2 GPIO_PIN_2
#define MS1_PIN_q2 GPIO_PIN_0
#define MS2_PIN_q2 GPIO_PIN_11
#define STEP_q2 GPIO_PIN_15
#define DIR_q2 GPIO_PIN_11
#define ENABLE_PIN_q3 GPIO_PIN_9
#define MS0_PIN_q3 GPIO_PIN_9
#define MS1_PIN_q3 GPIO_PIN_7
#define MS2_PIN_q3 GPIO_PIN_15
#define STEP_q3 GPIO_PIN_13
#define DIR_q3 GPIO_PIN_11
#define VELOCIDAD 0.5
// Máscaras para los eventos
#define TASK1_DONE (1 << 0)  // 00000001
#define TASK2_DONE (1 << 1)  // 00000010
#define TASK3_DONE (1 << 2)  // 00000100
#define TASK4_DONE (1 << 3)  // 00001000
#define TASK5_DONE (1 << 4)  // 00010000
#define TASK6_DONE (1 << 5)  // 00100000
#define TASK7_DONE (1 << 6)  // 01000000
#define TASK8_DONE (1 << 7)  // 10000000

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */



/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;
DMA_HandleTypeDef hdma_adc1;
DMA_HandleTypeDef hdma_adc2;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart3;

/* Definitions for Mover_Motor1 */
osThreadId_t Mover_Motor1Handle;
const osThreadAttr_t Mover_Motor1_attributes = {
  .name = "Mover_Motor1",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for Mover_Motor2 */
osThreadId_t Mover_Motor2Handle;
const osThreadAttr_t Mover_Motor2_attributes = {
  .name = "Mover_Motor2",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for Mover_Motor3 */
osThreadId_t Mover_Motor3Handle;
const osThreadAttr_t Mover_Motor3_attributes = {
  .name = "Mover_Motor3",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for Mover_Motor4 */
osThreadId_t Mover_Motor4Handle;
const osThreadAttr_t Mover_Motor4_attributes = {
  .name = "Mover_Motor4",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for Mover_Motor5 */
osThreadId_t Mover_Motor5Handle;
const osThreadAttr_t Mover_Motor5_attributes = {
  .name = "Mover_Motor5",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for Envar_datos_Uar */
osThreadId_t Envar_datos_UarHandle;
const osThreadAttr_t Envar_datos_Uar_attributes = {
  .name = "Envar_datos_Uar",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for Recibir_datos_u */
osThreadId_t Recibir_datos_uHandle;
const osThreadAttr_t Recibir_datos_u_attributes = {
  .name = "Recibir_datos_u",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for Tarea_Maestra */
osThreadId_t Tarea_MaestraHandle;
const osThreadAttr_t Tarea_Maestra_attributes = {
  .name = "Tarea_Maestra",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for Boton_system_in */
osThreadId_t Boton_system_inHandle;
const osThreadAttr_t Boton_system_in_attributes = {
  .name = "Boton_system_in",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* USER CODE BEGIN PV */

uint8_t byte;
uint8_t bufferOverflowFlag = 0; // Bandera para indicar desbordamiento
uint8_t buffer[BUFFER_SIZE];
uint16_t GPIO_PIN_LED = GPIO_PIN_3;     // Pin GPIO que controla el LED
uint16_t bufferIndex = 0;
GPIO_TypeDef* GPIO_PORT_LED = GPIOE;   // Puerto GPIO donde está conectado el LED

PID_Controller pid_px;
PID_Controller pid_py;
PID_Controller pid_pz;
//Control de las articulacciones
PID_Controller pid_q1;
PID_Controller pid_q2;
PID_Controller pid_q3;
PID_Controller pid_q4;
PID_Controller pid_q5;

osEventFlagsId_t eventGroup;


volatile uint8_t FC_Home_q2 = 1;// Variable to control motor state
volatile uint8_t FC_Home_q3 = 1;
volatile uint8_t Paro_emergencia = 1;
volatile uint8_t Contador = 0;
volatile int pasos_retroceso = 0;
volatile int bandera = 0;

int paso_actual_q1 = 0;
int paso_actual_q2 = 5350;
int paso_actual_q3 = 0;
int index_px = 0;
int index_py = 0;
int index_pz = 0;
int index_q1 = 0;
int index_q2 = 0;
int index_q3 = 0;
int index_q4 = 0;
int index_q5 = 0;
int q2_int;
int q3_int;

float q1_float;
float q4_float;
float q5_float;
float errors_px[BUFFER_SIZE];
float times_px[BUFFER_SIZE];
float errors_py[BUFFER_SIZE];
float times_py[BUFFER_SIZE];
float errors_pz[BUFFER_SIZE];
float times_pz[BUFFER_SIZE];
float errors_q1[BUFFER_SIZE];
float times_q1[BUFFER_SIZE];
float errors_q2[BUFFER_SIZE];
float times_q2[BUFFER_SIZE];
float errors_q3[BUFFER_SIZE];
float times_q3[BUFFER_SIZE];
float errors_q4[BUFFER_SIZE];
float times_q4[BUFFER_SIZE];
float errors_q5[BUFFER_SIZE];
float times_q5[BUFFER_SIZE];
float control_output_px;
float control_output_py;
float control_output_pz;
float control_output_q1;
float control_output_q2;
float control_output_q3;
float control_output_q4;
float control_output_q5;
float error_x;
float error_y;
float error_z;
float error_q1;
float error_q2;
float error_q3;
float error_q4;
float error_q5;
float q1_float=0;
float q2_float=0;
float q3_float=0;
float q4_float=0;
float q5_float=0;

char buffer_1[50];
char buffer_output_px[50];
char buffer_measirement[50];
char buffer_corrected_length_px[50];
char buffer_corrected_length_py[50];
char buffer_corrected_length_pz[50];
char buffer_error[50];
char buffer_q1[50];
char buffer_q2[50];
char buffer_q3[50];
char q1[BUFFER_SIZE] = {0};
char q2[BUFFER_SIZE] = {'1','1','0'};
char q3[BUFFER_SIZE] = {0};
char q4[BUFFER_SIZE] = {'1','.','5','7','0','7'};
char q5[BUFFER_SIZE] = {'1','.','5','7','0','7'};
char buffer_error[50];




/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC2_Init(void);
static void MX_USART3_UART_Init(void);
void Mover_Motor_1(void *argument);
void Mover_Motor_2(void *argument);
void Mover_Motor_3(void *argument);
void Mover_Motor_4(void *argument);
void Mover_Motor_5(void *argument);
void Enviar_datos_confirmaccion(void *argument);
void Recibir_datos_app(void *argument);
void Tarea_administradora(void *argument);
void Boton_interrupt(void *argument);
void A4988_q1();
void A4988_q2();
void A4988_q3();
void Home (void);
void Home_q2(void);
void Home_q3(void);
void mover_motorq1_rad(float radianes);
void mover_motorq2_mm(float milimetros);
void mover_motorq3_mm(float milimetros);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

///////////////////////////////////////////Inicializacion de funciones////////////////////////////////////////////////////////////////////////////////////////////

void PID_Init(PID_Controller *pid, float Kp, float Ki, float Kd, float setpoint) {
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    pid->setpoint = setpoint;
    pid->integral = 0;
    pid->prevError = 0;
    pid->prevTime = HAL_GetTick();
}

float clamp(float value, float min, float max) {
    if (value < min) {
        return min;
    } else if (value > max) {
        return max;
    } else {
        return value;
    }
}




float PID_Update(PID_Controller *pid,float setpoint , float measurement,const char* identifier,float *errors, float *times, int *index,float *output, float *error) {
    float currentTime = HAL_GetTick();
    float dt = (currentTime - pid->prevTime) / 1000; // Convertir a segundos
    *error = setpoint - measurement;

    // Proporcional
    float P_out = pid->Kp * (*error);

    // Integral
    pid->integral += (*error) * dt;
    float I_out = pid->Ki * pid->integral;

    // Derivativo
    float derivative = ((*error) - pid->prevError) / dt;
    float D_out = pid->Kd * derivative;

    // Total Output
    *output = P_out + I_out + D_out;
    *output = clamp(*output, -MAX_OUTPUT, MAX_OUTPUT); // Ajusta MAX_OUTPUT según tu sistema

    // Guardar el error y el tiempo actuales para la próxima iteración
    pid->prevError = (*error)-0.001;
    pid->prevTime = currentTime;



    // Almacenar el error y el tiempo
    if (*index < BUFFER_SIZE) {
        errors[*index] = *error;
        times[*index] = currentTime / 1000.0; // Convertir a segundos
        (*index)++;
    }


    // Convertir error y tiempo a cadena y transmitir por UART
    char buffer_error[20];
    char buffer_time[20];
    float_to_string(*error, buffer_error, 3);
    float_to_string(currentTime / 1000.0, buffer_time, 3);
    sprintf(buffer_1, "Medida error %s: %s, Tiempo: %s\r\n", identifier, buffer_error, buffer_time);

     //Transmite la cadena anterior
    HAL_UART_Transmit(&huart1, (uint8_t*)buffer_1, strlen(buffer_1), HAL_MAX_DELAY);
}

uint32_t radianes_a_valor(float radianes) {
    // Ajusta los radianes negativos a su equivalente positivo en el rango de 0 a 2PI
    if (radianes < 0) {
        radianes += M_PI;
    }

    // Normaliza el valor de radianes en el rango de 0 a PI
    if (radianes > M_PI) {
        radianes = M_PI;
    }

    return VALOR_0 + (uint32_t)((VALOR_PI - VALOR_0) * (radianes / M_PI));
}

uint32_t milimetros_a_pasos(float milimetros) {
    // Calcular el número de pasos necesarios para mover la distancia en milímetros
    float pasos_por_mm = 200.0 / 8.0; // 200 pasos por 8 mm
    return (uint32_t)(fabs(milimetros) * pasos_por_mm);
}

void A4988_q1(){
	HAL_GPIO_WritePin(GPIOE, ENABLE_PIN_q1, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, MS0_PIN_q1, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB, MS1_PIN_q1, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, MS2_PIN_q1, GPIO_PIN_RESET);
}

void A4988_q2(){
	HAL_GPIO_WritePin(GPIOD, ENABLE_PIN_q2, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOD, MS0_PIN_q2, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOD, MS1_PIN_q2, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOC, MS2_PIN_q2, GPIO_PIN_RESET);
}

void A4988_q3(){
	HAL_GPIO_WritePin(GPIOA, ENABLE_PIN_q3, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOC, MS0_PIN_q3, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOC, MS1_PIN_q3, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOD, MS2_PIN_q3, GPIO_PIN_RESET);
}

void Home (void){
	//Home_q2();
	//Home_q3();
	//TIM2->CCR2 = radianes_a_valor(M_PI/2); //q5
	//TIM2->CCR4 = radianes_a_valor(M_PI/2); //q4
}

void Home_q2(void){
	while(FC_Home_q2){
		HAL_GPIO_WritePin(GPIOA, DIR_q2, GPIO_PIN_RESET);  //Retroceso
		for (int i = 0; i < 100000 && FC_Home_q2; i++) {
			HAL_GPIO_WritePin(GPIOA, STEP_q2, GPIO_PIN_SET);
			HAL_Delay(VELOCIDAD);
			HAL_GPIO_WritePin(GPIOA, STEP_q2, GPIO_PIN_RESET);
			HAL_Delay(VELOCIDAD);
		}
		if (!FC_Home_q2) break;
		HAL_Delay(500);
	}

	HAL_GPIO_WritePin(GPIOA, DIR_q2, GPIO_PIN_SET); //Avance
	for (int i = 0; i < 2500; i++) {
		HAL_GPIO_WritePin(GPIOA, STEP_q2, GPIO_PIN_SET);
		HAL_Delay(VELOCIDAD);
		HAL_GPIO_WritePin(GPIOA, STEP_q2, GPIO_PIN_RESET);
		HAL_Delay(VELOCIDAD);
		paso_actual_q2--;
	}
	HAL_Delay(500);
	FC_Home_q2 = 1;
}

void Home_q3(void){
	while(FC_Home_q3){
		HAL_GPIO_WritePin(GPIOD, DIR_q3, GPIO_PIN_RESET);  //Abajo
		for (int i = 0; i < 100000 && FC_Home_q3; i++) {
			HAL_GPIO_WritePin(GPIOD, STEP_q3, GPIO_PIN_SET);
			HAL_Delay(VELOCIDAD);
			HAL_GPIO_WritePin(GPIOD, STEP_q3, GPIO_PIN_RESET);
			HAL_Delay(VELOCIDAD);
		}
		if (!FC_Home_q3) break;
		HAL_Delay(500);
	}

	HAL_GPIO_WritePin(GPIOD, DIR_q3, GPIO_PIN_SET); //Arriba
	for (int i = 0; i < 80; i++) {
		HAL_GPIO_WritePin(GPIOD, STEP_q3, GPIO_PIN_SET);
		HAL_Delay(VELOCIDAD);
		HAL_GPIO_WritePin(GPIOD, STEP_q3, GPIO_PIN_RESET);
		HAL_Delay(VELOCIDAD);
	}
	HAL_Delay(500);
	FC_Home_q3 = 1;
}

void mover_motorq1_rad(float radianes){

    int pasos = (int)((radianes / (2 * M_PI)) * 400);
    int nuevo_paso = pasos;
    int diferencia_pasos = nuevo_paso - paso_actual_q1;

    if (diferencia_pasos > 0) {
        // Movimiento hacia adelante
    	HAL_GPIO_WritePin(GPIOD, DIR_q1, GPIO_PIN_RESET); //Antihorario
    	for (int i = 0; i < diferencia_pasos; i++) {
    		HAL_GPIO_WritePin(GPIOB, STEP_q1, GPIO_PIN_SET);
    		HAL_Delay(VELOCIDAD);
    		HAL_GPIO_WritePin(GPIOB, STEP_q1, GPIO_PIN_RESET);
    		HAL_Delay(VELOCIDAD);
    	}
    }

//    if(radianes == (2*M_PI))
//    {
//    	radianes = 0;
//    }

    else if (diferencia_pasos < 0) {
        // Movimiento hacia atrás
    	HAL_GPIO_WritePin(GPIOD, DIR_q1, GPIO_PIN_SET); //Horario
    	diferencia_pasos = -diferencia_pasos;
    	for (int i = 0; i < diferencia_pasos ; i++) {
    		HAL_GPIO_WritePin(GPIOB, STEP_q1, GPIO_PIN_SET);
    		HAL_Delay(VELOCIDAD);
    		HAL_GPIO_WritePin(GPIOB, STEP_q1, GPIO_PIN_RESET);
    		HAL_Delay(VELOCIDAD);
    	}
    }

    paso_actual_q1 = nuevo_paso;
    HAL_Delay(500);
}

void mover_motorq2_mm(float milimetros){

	//milimetros = milimetros - 500;

    if (milimetros < 0) {
        milimetros = 0;
    }
    else if (milimetros > 210) {
        milimetros = 210;
    }

    uint32_t pasos = milimetros_a_pasos(milimetros);
    int diferencia_pasos = pasos - paso_actual_q2;

    if (diferencia_pasos != 0) {
        if (diferencia_pasos > 0) {
        	 HAL_GPIO_WritePin(GPIOA, DIR_q2, GPIO_PIN_RESET); //Retroceso
        }
        else {
        	HAL_GPIO_WritePin(GPIOA, DIR_q2, GPIO_PIN_SET); //Avance
            diferencia_pasos = -diferencia_pasos; // Hacer positiva la diferencia para el bucle
        }

        for (int i = 0; i < diferencia_pasos; i++) {
        	HAL_GPIO_WritePin(GPIOA, STEP_q2, GPIO_PIN_SET);
        	HAL_Delay(VELOCIDAD);
        	HAL_GPIO_WritePin(GPIOA, STEP_q2, GPIO_PIN_RESET);
        	HAL_Delay(VELOCIDAD);
        }

        paso_actual_q2 = pasos;
    }

    HAL_Delay(500);
}

void mover_motorq3_mm(float milimetros){

	if (milimetros < 0) {
		milimetros = 0;
	}
	else if (milimetros > 215) {
		milimetros = 215 ;
	}

    uint32_t pasos = milimetros_a_pasos(milimetros);
    int nuevo_paso = pasos;
    int diferencia_pasos = nuevo_paso - paso_actual_q3;

    if (diferencia_pasos > 0) {
    	HAL_GPIO_WritePin(GPIOD, DIR_q3, GPIO_PIN_SET); //Arriba
    	for (int i = 0; i < diferencia_pasos; i++) {
    		HAL_GPIO_WritePin(GPIOD, STEP_q3, GPIO_PIN_SET);
    		HAL_Delay(VELOCIDAD);
    		HAL_GPIO_WritePin(GPIOD, STEP_q3, GPIO_PIN_RESET);
    		HAL_Delay(VELOCIDAD);
    	}
    }

    else if (diferencia_pasos < 0) {
    	HAL_GPIO_WritePin(GPIOD, DIR_q3, GPIO_PIN_RESET);  //Abajo
    	diferencia_pasos = -diferencia_pasos; // Hacer positiva la diferencia para el bucle
    	for (int i = 0; i < diferencia_pasos; i++) {
    		HAL_GPIO_WritePin(GPIOD, STEP_q3, GPIO_PIN_SET);
    		HAL_Delay(VELOCIDAD);
    		HAL_GPIO_WritePin(GPIOD, STEP_q3, GPIO_PIN_RESET);
    		HAL_Delay(VELOCIDAD);
    	}
    }

    paso_actual_q3 = nuevo_paso;
    HAL_Delay(500);
}



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

/* Configure the peripherals common clocks */
  PeriphCommonClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_TIM2_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
  A4988_q1();
  A4988_q2();
  A4988_q3();
  Home();

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

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
  /* creation of Mover_Motor1 */

  /* creation of Tarea_Maestra */
  Tarea_MaestraHandle = osThreadNew(Tarea_administradora, NULL, &Tarea_Maestra_attributes);

  /* creation of Boton_system_in */
  //Boton_system_inHandle = osThreadNew(Boton_interrupt, NULL, &Boton_system_in_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  eventGroup = osEventFlagsNew(NULL);
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Macro to configure the PLL clock source
  */
  __HAL_RCC_PLL_PLLSOURCE_CONFIG(RCC_PLLSOURCE_HSE);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = 64;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV8;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInitStruct.PLL2.PLL2M = 2;
  PeriphClkInitStruct.PLL2.PLL2N = 15;
  PeriphClkInitStruct.PLL2.PLL2P = 5;
  PeriphClkInitStruct.PLL2.PLL2Q = 2;
  PeriphClkInitStruct.PLL2.PLL2R = 2;
  PeriphClkInitStruct.PLL2.PLL2RGE = RCC_PLL2VCIRANGE_3;
  PeriphClkInitStruct.PLL2.PLL2VCOSEL = RCC_PLL2VCOWIDE;
  PeriphClkInitStruct.PLL2.PLL2FRACN = 2950;
  PeriphClkInitStruct.AdcClockSelection = RCC_ADCCLKSOURCE_PLL2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_16B;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DR;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.LeftBitShift = ADC_LEFTBITSHIFT_NONE;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_7;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  sConfig.OffsetSignedSaturation = DISABLE;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */

  /** Common config
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc2.Init.Resolution = ADC_RESOLUTION_16B;
  hadc2.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc2.Init.LowPowerAutoWait = DISABLE;
  hadc2.Init.ContinuousConvMode = ENABLE;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DR;
  hadc2.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc2.Init.LeftBitShift = ADC_LEFTBITSHIFT_NONE;
  hadc2.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  sConfig.OffsetSignedSaturation = DISABLE;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 64;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 2499;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

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
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart3, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart3, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
  /* DMA1_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3|GPIO_PIN_1, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3|GPIO_PIN_7|GPIO_PIN_9|GPIO_PIN_11, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_11|GPIO_PIN_13|GPIO_PIN_15|GPIO_PIN_0
                          |GPIO_PIN_2|GPIO_PIN_4|GPIO_PIN_6, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9|GPIO_PIN_11|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3|GPIO_PIN_5|GPIO_PIN_7|GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pins : PE3 PE1 */
  GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : PC3 PC7 PC9 PC11 */
  GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_7|GPIO_PIN_9|GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PE7 PE9 PE11 PE13 */
  GPIO_InitStruct.Pin = GPIO_PIN_7|GPIO_PIN_9|GPIO_PIN_11|GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : PD11 PD13 PD15 PD0
                           PD2 PD4 PD6 */
  GPIO_InitStruct.Pin = GPIO_PIN_11|GPIO_PIN_13|GPIO_PIN_15|GPIO_PIN_0
                          |GPIO_PIN_2|GPIO_PIN_4|GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : PA9 PA11 PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_9|GPIO_PIN_11|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB3 PB5 PB7 PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_5|GPIO_PIN_7|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_Mover_Motor_1 */
/**
  * @brief  Function implementing the Mover_Motor1 thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_Mover_Motor_1 */
void Mover_Motor_1(void *argument)
{
  /* USER CODE BEGIN 5 */
  char msg[] = "Motor 1 is running\r\n";  // Message specific to Motor 1
  char msg_1[] = "Process runing....\r\n";  // Message specific to Motor 1
  /* Infinite loop */
  //for(;;)
  //{
	HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
	mover_motorq1_rad(2.582);
    osDelay(1000);
    osEventFlagsSet(eventGroup, TASK1_DONE); // Señalar que Task1 terminó
    osThreadExit();
  //}
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_Mover_Motor_2 */
/**
* @brief Function implementing the Mover_Motor2 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Mover_Motor_2 */
void Mover_Motor_2(void *argument)
{
  /* USER CODE BEGIN Mover_Motor_2 */
  char msg[] = "Motor 2 is running\r\n";  // Message specific to Motor 1
  /* Infinite loop */
  //for(;;)
  //{
	HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
	mover_motorq2_mm(300);
	osDelay(1000);
	osEventFlagsSet(eventGroup, TASK2_DONE); // Señalar que Task1 terminó
	osThreadExit();
  //}
  /* USER CODE END Mover_Motor_2 */
}

/* USER CODE BEGIN Header_Mover_Motor_3 */
/**
* @brief Function implementing the Mover_Motor3 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Mover_Motor_3 */
void Mover_Motor_3(void *argument)
{
  /* USER CODE BEGIN Mover_Motor_3 */
	char msg[] = "Motor 3 is running\r\n";  // Message specific to Motor 1
  /* Infinite loop */
  //for(;;)
  //{
	HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
	mover_motorq3_mm(200);
	osDelay(1000);
	osEventFlagsSet(eventGroup, TASK3_DONE); // Señalar que Task1 terminó
	osThreadExit();
  //}
  /* USER CODE END Mover_Motor_3 */
}

/* USER CODE BEGIN Header_Mover_Motor_4 */
/**
* @brief Function implementing the Mover_Motor4 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Mover_Motor_4 */
void Mover_Motor_4(void *argument)
{
  /* USER CODE BEGIN Mover_Motor_4 */
	char msg[] = "Motor 4 is running\r\n";  // Message specific to Motor 1
  /* Infinite loop */
  //for(;;)
  //{
	 HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
	 TIM2->CCR4 = radianes_a_valor(1.578); //q4
	 osDelay(1000);
	 osEventFlagsSet(eventGroup, TASK4_DONE); // Señalar que Task1 terminó
	 osThreadExit();
  //}
  /* USER CODE END Mover_Motor_4 */
}

/* USER CODE BEGIN Header_Mover_Motor_5 */
/**
* @brief Function implementing the Mover_Motor5 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Mover_Motor_5 */
void Mover_Motor_5(void *argument)
{
  /* USER CODE BEGIN Mover_Motor_5 */
	char msg[] = "Motor 5 is running\r\n";  // Message specific to Motor 1
  /* Infinite loop */
  //for(;;)
  //{
	HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
	TIM2->CCR2 = radianes_a_valor(1.578); //q5
    osDelay(1000);
    osEventFlagsSet(eventGroup, TASK5_DONE); // Señalar que Task1 terminó
    osThreadExit();

  //}
  /* USER CODE END Mover_Motor_5 */
}

/* USER CODE BEGIN Header_Enviar_datos_confirmaccion */
/**
* @brief Function implementing the Envar_datos_Uar thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Enviar_datos_confirmaccion */
void Enviar_datos_confirmaccion(void *argument)
{
  /* USER CODE BEGIN Enviar_datos_confirmaccion */
	char msg[] = "Sending data request...\r\n";  // Message specific to Motor 1
  /* Infinite loop */
  //for(;;)
  //{
	HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);

    osDelay(1000);
    osEventFlagsSet(eventGroup, TASK6_DONE); // Señalar que Task6 terminó
    osThreadExit();
  //}
  /* USER CODE END Enviar_datos_confirmaccion */
}

/* USER CODE BEGIN Header_Recibir_datos_app */
/**
* @brief Function implementing the Recibir_datos_u thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Recibir_datos_app */
void Recibir_datos_app(void *argument)
{
  /* USER CODE BEGIN Recibir_datos_app */
	char msg[] = "Recibe data management...\r\n";  // Message specific to Motor 1
  /* Infinite loop */
  //for(;;)
  //{
	HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
    osDelay(1000);
    osEventFlagsSet(eventGroup, TASK7_DONE); // Señalar que Task7 terminó
    osThreadExit();
  //}
  /* USER CODE END Recibir_datos_app */
}

/* USER CODE BEGIN Header_Tarea_administradora */
/**
* @brief Function implementing the Tarea_Maestra thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Tarea_administradora */
void Tarea_administradora(void *argument)
{
  /* USER CODE BEGIN Tarea_administradora */
	char msg[] = "Runing manage tasking...\r\n";  // Message specific to Motor 1
	char msg_confirmation[] = "All the tasks finished ....\r\n";
  /* Infinite loop */
  for(;;)
  {

	  HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);

	  Mover_Motor1Handle = osThreadNew(Mover_Motor_1, NULL, NULL);

	  /* creation of Mover_Motor2 */
	  Mover_Motor2Handle = osThreadNew(Mover_Motor_2, NULL, NULL);

	  /* creation of Mover_Motor3 */
	  Mover_Motor3Handle = osThreadNew(Mover_Motor_3, NULL, NULL );

	  /* creation of Mover_Motor4 */
	  Mover_Motor4Handle = osThreadNew(Mover_Motor_4, NULL, NULL);

	  /* creation of Mover_Motor5 */
	  Mover_Motor5Handle = osThreadNew(Mover_Motor_5, NULL, NULL);

	  /* creation of Envar_datos_Uar */
	  Envar_datos_UarHandle = osThreadNew(Enviar_datos_confirmaccion, NULL, NULL);

	  /* creation of Recibir_datos_u */
	  Recibir_datos_uHandle = osThreadNew(Recibir_datos_app, NULL, NULL);

	  osEventFlagsWait(eventGroup, TASK1_DONE | TASK2_DONE | TASK3_DONE | TASK4_DONE | TASK5_DONE | TASK6_DONE | TASK7_DONE , osFlagsWaitAll, osWaitForever);

	  HAL_UART_Transmit(&huart1, (uint8_t*)msg_confirmation, strlen(msg_confirmation), HAL_MAX_DELAY);

	  osDelay(1000);

      //osThreadExit();


  }
  /* USER CODE END Tarea_administradora */
}

/* USER CODE BEGIN Header_Boton_interrupt */
/**
* @brief Function implementing the Boton_system_in thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Boton_interrupt */
void Boton_interrupt(void *argument)
{
  /* USER CODE BEGIN Boton_interrupt */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Boton_interrupt */
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
