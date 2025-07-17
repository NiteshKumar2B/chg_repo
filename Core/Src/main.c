/* USER CODE BEGIN Header */
/** 280225 implementing code for 3 units of bms
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include "measure.h"
#include "utility.h"
#include "math.h"

#include <stdbool.h>

#include<limits.h>
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
ADC_HandleTypeDef hadc1;

FDCAN_HandleTypeDef hfdcan1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim6;

UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
uint8_t ubKeyNumber = 0x0;
uint8_t ubKeyNumberValue = 0x0;
FDCAN_RxHeaderTypeDef RxHeader;
uint8_t RxData[8];
FDCAN_TxHeaderTypeDef TxHeader;
uint8_t TxData[8];
uint16_t set_flag=0,CAN_Delay=0;
uint32_t data=0,can_error=0,soc_value=0,cc=0,cc_current=0,battery_voltage1=5750; //battery_voltage1=6700;
uint16_t data1,data2,data3,data4,data5,data6,data7,data8,Can_comm_fail=0;
uint8_t data_tx[8],flash_set=0;


uint32_t PageError;

/*				Data type defined for all 10ids of BMS_1 JBD 		*/
uint16_t data10,data11,data12,data13,data14,data15,data16,data17; /* RxData1*/
float max_cell_voltage,min_cell_voltage,soc,max_tempbms,total_battery_voltage;


uint16_t data20,data21,data22,data23,data24,data25,chg_start_stop;/* RxData2*/
float max_allow_chg_terminal_voltage,max_allow_chg_current;

uint16_t data30,data31,data32,data33,data34,data35; /* RxData3*/
float max_cell_temp,min_cell_temp,highest_unit_voltage_mv,minimum_unit_voltage_mv;

uint16_t data40,data41,data42,data43,data44,data45,data46,data47; /* RxData4*/
uint16_t data50,data51,data52,data53,data54,data55,data56,data57;/*RxData5*/

uint16_t data64; /*RxData6 */
float battery_health_soh;

uint16_t data70,alarm,data71,data72,data73,battery_status;/*RxData7*/
float power_w;

uint16_t data80,battery_type; /*RxData8*/

uint16_t data90,data91,data92,production_date,
		production_month,data95,data96,production_year; /*RxData9*/
float mos_temp,cell_pressure_difference_mv;

uint16_t percentage_remaining_chg_capacity_soc; /*RxData10*/

uint16_t whilerun = 0,dataReceived=0,t3=0, ccf=0,cf1=0,cf2=0,cf3=0,cf4=0,one_sec_led=0,one_sec_s=0,charging_start_flag=0,battery_present=0,relay_cycle_count = 0;;
unsigned int my_variable = 0xFFFFFFFF;

/********************************************************************/
#define TOTAL_IDS 50  // Number of different IDs

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_FDCAN1_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM3_Init(void);
//static void MX_TIM6_Init(void);
/* USER CODE BEGIN PFP */
static void FDCAN_Config(void);
void init_cal_factors(void);
void flash_page_erase(uint32_t page);
void charging_control (void);
void getTemp(void);
void battery_voltage_measurement (void);
void fan(void);
void led_control (unsigned char led,unsigned char state);
void App_AdcChannelSel(uint32_t channel);

void led_control_1(void);




void ProcessCANMessage(FDCAN_RxHeaderTypeDef *RxHeader, uint8_t *RxData);
void switchfuntion(uint32_t can_id, uint8_t *RxData);
//void update_charging_leds(uint16_t led_battery_voltage, uint16_t led_main_voltage, uint8_t led_charging_started);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
unsigned int gain=0,gain3=0,model_set=0,led_gain=0,micro_sec=0,m_cal=1530,half_sec=0,batt_open_delay=0,fan1=0,fiveamp = 0;
unsigned int four_sec=0,ADC_VAL[5],mains_cnt=0,mains_rms_cal=15,batt_voltage=0,batt_volt12=0;
  int mains_volt_avg=0,prev_batt_volt=0,batt_voltavg=0,avgbattvolt_cnts=0,chg_current_avg,prev_chg_crnt=0,batt_volt11=0;
unsigned char mains_volt_cut=0,fan_on_flag=0,mains_fail=0,chargdeating=0,sw_on=0,hi_cut_deb=0,start_recovery=0;
unsigned char led_presentage=0,tweenty_percent=0,fifty_percent=0,seventy_percent=0,hundred_percent=0,batt_high_flag=0,buzzer_beep=0;
unsigned char charge_flag=0,Battery_full=0, dis_cal=21,batt_cal=25,led_delay=0,batt_charge_delay=0;
unsigned int temp=0,mains_volt=0,i=0,mains_debounce=0,prev_mains_volt=0,delay_on=0,one_min=0,one_sec=0,chg_cnt=0;
unsigned int temp_adc_main=0,  main_cnt=0,temp_value=0,main_voltage=0,charg_offset=0,charging_current=0;
float Batt_set_error=0;
unsigned char can_comm_check=0,start=0;
unsigned char two_sec=0,can_comm=0,cut_count=0,comm_fail=0,batt_open_mode=0,over_temp_protection=0,sett=0,over_current=0,retry_flag=0,batt_high=0;
unsigned int led1=0,led2=0,led3=0,led4=0,led2_50=0,led1_25=0,led3_75,led4_100=0,soft_start=0;
unsigned int main_cut_count=0,charge_delay=0,max_temp=85,chargde=0,avg_counts=0,chg_current=0; //75
unsigned int Ro = 10000, B = 3977; //Nominal resistance 50K, Beta constant
float R=0,r_value=0;// Series resistor 10K
float To = 298.15,Vi=0;
long T=0;
float r_long=0; // Nominal Temperature
int temperature=0,temperature_error=0,temperature_error1=0;
unsigned int chg_cal=130,dischg_cal=30,Batt_recheck=0;
uint32_t address,save_cal_flag,error_cc=0;
uint32_t data;
volatile unsigned int eeprom_add,eeprom_data,delay;
   unsigned int local,sofar=0,local1,local2,s_test=0;
   volatile unsigned int batt_charge_flag;
   static unsigned char reveived_flag=0;
  uint64_t flash_address= 0x08019000;
// uint32_t flash_address[1]= { 0x08008004 };
 // uint32_t flash_address[2]= { 0x08008008 } ;
  uint64_t FData =  0x1111111199999999;


  uint8_t blink_20 = 0, blink_21 = 0, blink_50 = 0, blink_75 = 0, blink_100 = 0,blink_101 = 0, blink_Red_slow = 0,blink_Red_fast = 0, led_on5 = 0, led_count_5 = 0;
  uint8_t blink_fault_slow = 0, blink_fault_fast = 0,led_on = 0,led_on1 = 0,led_on2 = 0,led_on3 = 0, led_on4 = 0, led_count_1 = 0,led_count_2 = 0,led_count_3 = 0,led_count_4 = 0;

  unsigned char led_charging_started = 0;
  uint16_t led_main_voltage = 0,led_battery_voltage = 0;

//      static uint16_t slow_counter = 0;
//      static uint16_t fast_counter = 0;



/****************************************************************************************************************************/

/*     28Feb25   */
  const uint32_t BMS_CAN_IDS[] = {

      0x1800FFF4, 0x1806E5F4, 0x1814EFF4, 0x18F0FDF4, 0x18F0FEF4,
      0x18F101F4, 0x18F110F4, 0x18F1E2F4, 0x18FF28F4, 0x18FFA2F4, //JBD_BMS_ID
  };

  // Function to process and store the received CAN message
  void ProcessCANMessage(FDCAN_RxHeaderTypeDef *RxHeader, uint8_t *RxData)
  {
      for (int i = 0; i < TOTAL_IDS; i++)
      {
          if ((RxHeader->Identifier == BMS_CAN_IDS[i]) &&
              (RxHeader->IdType == FDCAN_EXTENDED_ID) &&
              (RxHeader->DataLength == FDCAN_DLC_BYTES_8))
          {
            //  memcpy(canIdMap[i].dataPtr, RxData, 8);  // Copy data into the corresponding array

              switchfuntion(BMS_CAN_IDS[i], RxData);
              break;  // Stop searching once the matching ID is found
          }
      }
  }
  /* 280225                                                           */
  void switchfuntion(uint32_t can_id, uint8_t *RxData) {


      switch (can_id) {

/*               These are the ids/packets for BMS 1 JBD BMS                      */

          // case 1 id1
          case 0x1800FFF4:
          {
              // Process data for ID 0x1800FFF4
              // Combine two bytes to get voltage (example for max_cell_voltage)

              uint16_t temp;
//              temp = RxData[0] << 8;      // Shift the first byte
//              temp |= RxData[1];          // Add the second byte
//              max_cell_voltage = temp * 0.001;  // Convert to volts

              data10 = RxData[0]<<8;
              data11 = RxData[1];
              max_cell_voltage = (data10+data11)*0.001;




              // For min_cell_voltage
              temp = RxData[2] << 8;
              temp |= RxData[3];
              min_cell_voltage = temp * 0.001;  // Convert to volts

              // State of Charge (SOC)
              temp = RxData[4];
              soc = (temp * 0.4);  // Example scaling factor

              // Maximum temperature of BMS
              temp = RxData[5];
              max_tempbms = temp;  // Assuming a 1:1 scale

              // Total battery voltage (combine two bytes)
              temp = RxData[6] << 8;
              temp |= RxData[7];
              total_battery_voltage = temp * 0.1;  // Example scaling factor //57.6   chg o/p 57.4+-1

          }break;


           // case 2 id2 BMS to Charger
          case 0x1806E5F4:
          {
              data20 = RxData[0] << 8; // Shift first byte
              data21 = RxData[1];      // Second byte
              max_allow_chg_terminal_voltage = (data20 + data21) * 0.1;

              // Combine next two bytes for minimum allowable charging current
              data22 = RxData[2] << 8;
              data23 = RxData[3];
              max_allow_chg_current = (data22 + data23) * 0.1;

              // Combine the following two bytes for charge start/stop flag
              data24 = RxData[4] << 8;
              data25 = RxData[5];
              chg_start_stop = (data24 + data25); /* 0 start charging and 1 stop charging battery proteciton*/
              // Note: RxData[6] and RxData[7] are reserved and not processed.
          }break;

              // case 3 id3
          case 0x1814EFF4:
          {
        	  data30 = RxData[0];
        	  max_cell_temp = data30; // unit degree c

        	  data31 = RxData[1];
        	  min_cell_temp = data31;

        	  data32 = RxData[2];
        	  data32 = data32<<8;
        	  data33 = RxData[3];
        	  highest_unit_voltage_mv = (data32 + data33); //voltage in Mv

        	  data34 = RxData[4];
        	  data34 = data34<<8;
        	  data35 = RxData[5];
        	  minimum_unit_voltage_mv = (data34 + data35); //voltage in Mv
              // Note: RxData[6] and RxData[7] are reserved and not processed.

          }break;

              // case 4 id4
          case 0x18F0FDF4:
          {
        	  data40 = RxData[0]; // byte 0 indicate the frame number and 1-7 BAR_CODE content
        	  data41 = RxData[1];
        	  data42 = RxData[2];
        	  data43 = RxData[3];
        	  data44 = RxData[4];
        	  data45 = RxData[5];
        	  data46 = RxData[6];
        	  data47 = RxData[7];

          }break;

              // case 5 id5
          case 0x18F0FEF4:
          {
        	  data50 = RxData[0]; // byte 0 indicate the frame number and 1-7 BMS_SN
        	  data51 = RxData[1];
        	  data52 = RxData[2];
        	  data53 = RxData[3];
        	  data54 = RxData[4];
        	  data55 = RxData[5];
        	  data56 = RxData[6];
        	  data57 = RxData[7];
          }break;

              // case 6 id6
          case 0x18F101F4:
          {
        	  /*  Process data for ID 0x18F101F4 Battery Health SOH  */
        	  //data60 = RxData[0];
        	  //data61 = RxData[1];
        	  //data62 = RxData[2];
        	  //data63 = RxData[3];
        	  // byte 0 - 3 reserved

        	  data64 = RxData[4];
        	  battery_health_soh = data64;

        	  //data65 = RxData[5];
        	  //data66 = RxData[6];
        	  //data67 = RxData[7];
        	  // byte 5 - 7 reserved

          }break;

              // case 7 id7
          case 0x18F110F4:
          {
        	  alarm  = RxData[0];  // byte0 alram 0->normal 1->alarm

        	  data71 = RxData[1];
        	  data71 = data71<<8;
        	  data72 = RxData[2];
        	  power_w = (data71 + data72);

        	  data73 = RxData[3];
        	  battery_status = data73; // 0 leisure, 1 charging, 2discharge
              // Note: RxData[4] to RxData[7] are reserved and not processed.

          } break;

              // case 8 id8
          case 0x18F1E2F4:
          {
              // Process data for ID 0x18F1E2F4 Battery type

        	  data80 = RxData[0];
        	  battery_type = data80; // 1.LFP, 2.NMC
        	  //byte1 -7 reserved

          }break;

              // case 9 id9
          case 0x18FF28F4:
              // Process data for ID 0x18FF28F4 Big Endian Mode
          {
        	  data90 = RxData[0];
        	  mos_temp = data90;

        	  uint16_t temp;
        	  temp = RxData[1]<<8;
        	  temp |=RxData[2];
        	  cell_pressure_difference_mv = temp;

//        	  data91 = RxData[1];
//        	  data91 = data91<<8;
//        	  data92 = RxData[2];
//        	  cell_pressure_difference_mv = (data91 + data92);

        	  production_date = RxData[3];
        	  production_month = RxData[4];

//        	  data95 = RxData[5];
//        	  data95 = data95<<8;
//        	  data96 = RxData[6];
//        	  production_year = (data95 + data96);

        	  temp = RxData[5]<<8;
        	  temp |= RxData[6];
        	  production_year = temp;

        	  //data97 = RxData9[7]; //reserved

          }break;

              // case 10 id10
          case 0x18FFA2F4:
          {
        	  percentage_remaining_chg_capacity_soc = RxData[4];
        	  //byte 5 -7 reserved
          }  break;
      }
  }



/* USER CODE END 0 */

/***************New Variable declare******************/
uint32_t g_AdcRaw[4];
static uint32_t g_batt_voltage;
static uint32_t g_ac_voltage;
static uint32_t g_chg_current;
static uint32_t g_temperature;
volatile uint16_t chgFaults;

volatile uint32_t g_pwm_gain,OK;


#define LED_SOC_25_OFF  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, 0); // D4 25 %
#define LED_SOC_50_OFF    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, 0); // D4 50 %
#define LED_SOC_75_OFF    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, 0); // D4 75 %
#define	LED_SOC_100_OFF   HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, 0); // D4 100 %
#define	LED_FAULT_OFF  	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, 0); // D5 Red

#define LED_SOC_25_ON   HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, 1); // D4 25 %
#define LED_SOC_50_ON    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, 1); // D4 50 %
#define LED_SOC_75_ON    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, 1); // D4 75 %
#define	LED_SOC_100_ON   HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, 1); // D4 100 %
#define	LED_FAULT_ON  	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, 1); // D5 Red

#define	LED_FAULT_TOGGLE  HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_5); // D5 Red

#define CAN_LED_OFF  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, 0);
#define CAN_LED_ON   HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, 1);

#define RELAY_ON    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, 1); //Relay ON
#define RELAY_OFF   HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, 0); //Relay ON

#define FAN_DRV_ON  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, 1); //Relay ON
#define FAN_DRV_OFF  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, 0); //Relay ON

#define MIN_PACK_VOLT  30000

// Turn ON all SOC LEDs (Full Battery)
#define LED_SOC_FULL    \
  do {                  \
    LED_SOC_25_ON;         \
    LED_SOC_50_ON;         \
    LED_SOC_75_ON;         \
    LED_SOC_100_ON;        \
  } while(0)

// Turn ON all SOC LEDs (Full Battery)
#define ALL_LED_OFF    \
  do {                  \
    LED_SOC_25_OFF;         \
    LED_SOC_50_OFF;         \
    LED_SOC_75_OFF;         \
    LED_SOC_100_OFF;        \
  } while(0)


#define CHG_FAULT_OVERVOLTAGE     (1 << 0)
#define CHG_FAULT_UNDERVOLTAGE    (1 << 1)
#define CHG_FAULT_OVERCURRENT     (1 << 2)
#define CHG_FAULT_OVERTEMP        (1 << 3)
#define CHG_FAULT_HW_FAILURE      (1 << 4)


typedef enum {
    NO_DECISION = 0,
    START_STATE,
	BATT_AVAILABLE,
    SELF_CHECK,
    CC_STATE,
    CV_STATE,
    FULL_CHARGE,
    ERROR_STATE
} ChargerState_e;

ChargerState_e chgState = NO_DECISION;

void App_ChargingState(void);
void App_PulseGeneration(void);
void App_ChgStateMachine(void);
void App_PherpheralInit(void);
void App_GetAllAdcValues(void);
void App_AdcConvertBattVoltage(void);
void App_AdcConvertACMains(void);
void App_AdcConvertChgCurrent(void);
void App_AdcConvertTemperature(void);
void App_FaultLedToggle(void);
void App_PwmDutyCtrl(uint32_t duty_ctrl);
void App_Delay(void (*func)(void), uint32_t delay_ms);




void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) // 250us = psc+1 * arr+1 / clock 32Mhz
{
	static uint8_t l_count=0;
	l_count++;

}


/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
	HAL_Init();

	SystemClock_Config();

	App_PherpheralInit();

	chgState = START_STATE;

	while(1)
	{
		App_GetAllAdcValues();

		App_AdcConvertBattVoltage();

		App_AdcConvertACMains();

		App_AdcConvertChgCurrent();

		App_AdcConvertTemperature();

		App_ChgStateMachine();

		HAL_Delay(100);
	}

	/* USER CODE END 3 */
}

void App_ChgStateMachine(void)
{
	if(chgState == START_STATE)
	{
		if(uwTick < 4000)
		{
			App_PwmDutyCtrl(g_pwm_gain);  // Ensure PWM is zero at entry
		}
		else if(uwTick > 4000 && uwTick < 8000)
		{
			g_pwm_gain = 200;
			App_PwmDutyCtrl(g_pwm_gain);  // Apply PWM while waiting
		}
		else
		{
			RELAY_ON;
			chgState = SELF_CHECK;
		}

		App_Delay(App_FaultLedToggle,400);
	}

	switch (chgState)
	{
	  case SELF_CHECK:
		  App_PulseGeneration();
	   break;

      case CC_STATE:
    	  App_ChargingState();
	   break;

      case NO_DECISION:
	    default:

		break;
	}

}


void App_PulseGeneration(void)
{
    static uint8_t pulse_count = 0;
//    static uint32_t last_pulse_time = 0;
//
    //1) Apply PWl_countM pulse

     while(g_pwm_gain >= 1)
     {
    	 g_pwm_gain -= 1;
    	 App_PwmDutyCtrl(g_pwm_gain);
    	 HAL_Delay(150);
     }

      HAL_Delay(10000);

    // 2) Turn off relay
      RELAY_OFF;
      g_pwm_gain = 200;
      App_PwmDutyCtrl(g_pwm_gain);  // off voc
//
     // 3) Read battery voltage
		App_GetAllAdcValues();
		App_AdcConvertBattVoltage();
		App_AdcConvertACMains();
     if (g_batt_voltage >= MIN_PACK_VOLT)
     {
        chgState = CC_STATE;
        pulse_count = 0;
        return;
    }

////    HAL_Delay(700);
     HAL_Delay(10000);
  // 4) Turn relay back on
      RELAY_ON;
//    g_pwm_gain = 200;
////    HAL_Delay(700);
//
     // 5) Increment pulse count
     pulse_count++;

    if (pulse_count >= 10)
     {
     	 RELAY_OFF;
         chgState = ERROR_STATE;
         pulse_count = 0;
    }

}


void App_ChargingState(void)
{
	RELAY_ON;
	while(g_pwm_gain >= 1)
	{
		g_pwm_gain -= 1;
		App_PwmDutyCtrl(g_pwm_gain);
		HAL_Delay(150);
	}
}

void App_PherpheralInit(void)
{
	MX_GPIO_Init();
	MX_ADC1_Init();

	MX_FDCAN1_Init();
	FDCAN_Config();
	HAL_FDCAN_Start(&hfdcan1);
	MX_TIM3_Init();
	HAL_TIM_Base_Start_IT(&htim3);

	MX_TIM1_Init();
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
}

void App_PwmDutyCtrl(uint32_t duty_ctrl)
{
	TIM1->CCR1 =  duty_ctrl;
}

void App_FaultLedToggle(void)
{
	LED_FAULT_TOGGLE;
}

void App_Delay(void (*func)(void), uint32_t delay_ms)
{
	uint32_t start = HAL_GetTick();
	while ((HAL_GetTick() - start) < delay_ms)
	{
		// wait
	}

	if (func != NULL)
		func();
}

void App_AdcChannelSel(uint32_t channel)
{
	ADC_ChannelConfTypeDef sConfig = {0};
	sConfig.Channel = channel;
	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SamplingTime = ADC_SAMPLINGTIME_COMMON_1;

	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	{
		Error_Handler();
	}
}

void App_GetAllAdcValues(void)
{
	App_AdcChannelSel(ADC_CHANNEL_0);
	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, 1000);
	g_AdcRaw[0] = HAL_ADC_GetValue(&hadc1);
	HAL_ADC_Stop(&hadc1);

	App_AdcChannelSel(ADC_CHANNEL_1);
	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, 1000);
	g_AdcRaw[1] = HAL_ADC_GetValue(&hadc1);   //PA1 Mains on ADC_VAL[1]
	temp_adc_main=g_AdcRaw[1];
	HAL_ADC_Stop(&hadc1);


	App_AdcChannelSel(ADC_CHANNEL_2);
	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, 1000);
	g_AdcRaw[2] = HAL_ADC_GetValue(&hadc1); //PA2 TEMPSENS
	HAL_ADC_Stop(&hadc1);


	App_AdcChannelSel(ADC_CHANNEL_3);
	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, 1000);
	g_AdcRaw[3] = HAL_ADC_GetValue(&hadc1);  //PA3 current on ADC_VAL[3]
	HAL_ADC_Stop(&hadc1);

}

void App_AdcConvertBattVoltage(void)
{
	static unsigned int temp_adc_data;

	temp_adc_data = g_AdcRaw[0];    //PA0 battery_voltage on  g_AdcRaw[0] BAT_V R5
	temp_adc_data = LPF(2,temp_adc_data,&prev_batt_volt);
	temp_adc_data = temp_adc_data*49.5;//10;//100;
	batt_voltavg += (unsigned int)(temp_adc_data/25);//362;batt_cal
	avgbattvolt_cnts++;

	if (avgbattvolt_cnts > 30)
	{
		g_batt_voltage = (batt_voltavg)/30;   //calculated battery voltage
		avgbattvolt_cnts =0;
		batt_voltavg =0;
	}
}


void App_AdcConvertACMains(void)
{
	if(temp_adc_main>1480)
	{
		if(temp_adc_main>prev_mains_volt)
		{
			prev_mains_volt=temp_adc_main;
		}

		else
		{
			mains_volt = prev_mains_volt-m_cal;//1480
		}
	}
	else
	{
		prev_mains_volt=0;
	}

	if(main_cnt > 50)
	{
		mains_volt_avg = mains_volt_avg/main_cnt;
		g_ac_voltage = mains_volt_avg*0.2253;
		mains_volt_avg = 0;main_cnt=0;
	}
	else
	{
		mains_volt_avg=mains_volt+mains_volt_avg;
		main_cnt++;
	}
}



void App_AdcConvertChgCurrent(void)
{
	if (chg_cnt>70)
	{
		chg_current =  chg_current_avg/chg_cnt;
		chg_current =  chg_current+prev_chg_crnt;
		chg_current =  chg_current/dis_cal;
		prev_chg_crnt = chg_current;
		g_chg_current =(chg_current*8);
		chg_current_avg = 0;
		chg_cnt = 0;
	}
	else
	{
		chg_current_avg = chg_current_avg+ g_AdcRaw[3];
		chg_cnt++;
	}
}


void App_AdcConvertTemperature(void)
{
	Vi = g_AdcRaw[2] * (3.3 / 4096.0);
	R = (Vi *2000) / (3.3 - Vi);

	r_long=R/10000;

	T =  1 / ((1 / To) + ((log(r_long)) / B));

	g_temperature = T - 273.15+9;
}



void led_control_1(void)
{
	// blink_20 =  ( batt_voltage < (battery_voltage1*89/100) );  //5117.5
	//blink_50 =   ( batt_voltage >= (battery_voltage1*89/100)  && batt_voltage <= (battery_voltage1*925/1000)); //5318
	//blink_75 =   ( batt_voltage >= (battery_voltage1*925/1000)  && batt_voltage <= (battery_voltage1*941/1000) ); //5410
	//blink_100 =  ( batt_voltage >= (battery_voltage1*941/1000)  && batt_voltage <= (battery_voltage1*976/1000) ); //5612
	//blink_101 =  ( batt_voltage > (battery_voltage1*976/1000) ); //5612




	blink_20 =  ( batt_voltage <= 5300 );
	blink_50 =  ( batt_voltage > 5300 && batt_voltage <= 5350 );
	blink_75 =  ( batt_voltage > 5350 && batt_voltage <= 5400);
	blink_100 = (batt_voltage  > 5400);
	blink_101 = (batt_voltage  > 5610);


	/********CAN_Led_On******/

	if ( reveived_flag == 1)
	{
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, 1); //D6 CAN
	}
	else if (reveived_flag == 0)
	{
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, 0); //D6 CAN

	}



	/******** Batt_voltage < 5117******/

	if( blink_20 == 1 && charging_current>20  && mains_volt_cut==0 &&  over_temp_protection == 0 && relay_cycle_count < 10 && led_count_1++ >100)
	{
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, 0); // D4 50 %
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, 0); // D4 75 %
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, 0); // D4 100 %
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, 0); // D5 Red

		if (led_on == 1)
		{
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, 1); // D4 25 %
			led_on = 0;

		}
		else{
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, 0); // D4 25 %
			led_on = 1;
		}
		led_count_1 = 0;
	}


	/******** Batt_voltage > 5117 &&  Batt_voltage < 5317******/
	if( blink_50 == 1 && charging_current>20 && mains_volt_cut==0  && over_temp_protection == 0 && relay_cycle_count < 10 && led_count_2++>100  )
	{

		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, 0); // D4 75 %
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, 0); // D4 100 %
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, 0); // D5 Red

		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, 1); // D4 25 %

		if (led_on2 == 1)
		{
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, 1); // D4 25 %
			led_on2 = 0;

		}
		else{
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, 0); // D4 25 %
			led_on2 = 1;
		}
		led_count_2 = 0;
	}

	/******** Batt_voltage >5317 &&  Batt_voltage < 5413******/
	if( blink_75 == 1 && charging_current>20  && mains_volt_cut==0  && over_temp_protection == 0 && relay_cycle_count < 10 &&  led_count_3 ++>100)
	{
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, 0); // D4 100 %
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, 0); // D5 Red

		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, 1); // D4 25 %
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, 1); // D4 50 %

		if (led_on3 == 1)
		{
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, 1); // D4 25 %
			led_on3 = 0;

		}
		else{
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, 0); // D4 25 %
			led_on3 = 1;
		}
		led_count_3 = 0;
	}


	/******** Batt_voltage >5413 &&  Batt_voltage < 5610******/
	if( blink_100 == 1 && charging_current>20 && batt_charge_flag==0 &&  mains_volt_cut==0  && over_temp_protection == 0 && relay_cycle_count < 10 && led_count_4 ++>100 )
	{
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, 0); // D5 Red

		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, 1); // D4 25 %
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, 1); // D4 50 %
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, 1); // D4 75 %

		if (led_on4 == 1)
		{
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, 1); // D4 100 %
			led_on4 = 0;

		}
		else{
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, 0); // D4 100 %
			led_on4 = 1;
		}
		led_count_4 = 0;
	}



	/******** Batt_voltage > 5610******/
	if(( blink_101 == 1 && (batt_charge_flag==1 ||Battery_full==1) &&  mains_volt_cut == 0 && over_temp_protection == 0) || relay_cycle_count >= 10 )
	{
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, 0); // D5 Red

		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, 1); // D4 25 %
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, 1); // D4 50 %
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, 1); // D4 75 %
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, 1); // D4 100 %
	}

	///* batt_voltage >=5610 = batt_charge_flag=1 and @ 5462v charge_flag = 1 -> 120min -->Battery_full = 1*/
	//	if( (batt_charge_flag==1 ||Battery_full==1 || relay_cycle_count>=10 ) && mains_volt_cut==0 && over_temp_protection == 0 ) /* Charge_flag=1 means battery 55v and Mains_present blue on constant*/
	//	{
	//		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, 1); // D4 25 %
	//		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, 1); // D4 50 %
	//		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, 1); // D4 75 %
	//		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, 1); // D4 100 %
	//		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, 0); // D5 Red
	//	}


	/* Falut led Red */
	blink_Red_slow = (( main_voltage < 140 && mains_volt_cut == 1) || (charging_current < 20 && mains_volt_cut==0 && relay_cycle_count < 10)); // No_battery

	blink_Red_fast = (( main_voltage > 288 && mains_volt_cut == 1 && relay_cycle_count < 10)) ;

	if(blink_Red_slow == 1 && over_temp_protection == 0 && batt_charge_flag==0 && led_count_5 ++>100 )
	{
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, 0); // D4 25 %
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, 0); // D4 50 %
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, 0); // D4 75 %
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, 0); // D4 100 %

		//          if ( reveived_flag == 1)
		//          {
		//          	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, 1); //D6 CAN
		//          }

		if (led_on5 == 1)
		{
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, 1); // D5 Red
			led_on5 = 0;

		}
		else{
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, 0); // D5 Red
			led_on5 = 1;
		}
		led_count_5 = 0;
	}

	if(blink_Red_fast == 1 && over_temp_protection == 0 && batt_charge_flag==0  && led_count_5 ++>50 )
	{
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, 0); // D4 25 %
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, 0); // D4 50 %
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, 0); // D4 75 %
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, 0); // D4 100 %

		//          if ( reveived_flag == 1)
		//          {
		//          	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, 1); //D6 CAN
		//          }

		if (led_on5 == 1)
		{
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, 1); // D4 25 %
			led_on5 = 0;

		}
		else{
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, 0); // D4 25 %
			led_on5 = 1;
		}
		led_count_5 = 0;
	}
	else if( charging_current < 20 &&  mains_volt_cut == 0  && relay_cycle_count < 10)
	{
		blink_Red_slow = 1;
	}

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
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 8;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV4;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
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

  //ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.LowPowerAutoPowerOff = DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.SamplingTimeCommon1 = ADC_SAMPLETIME_1CYCLE_5;
  hadc1.Init.SamplingTimeCommon2 = ADC_SAMPLETIME_1CYCLE_5;
  hadc1.Init.OversamplingMode = DISABLE;
  hadc1.Init.TriggerFrequencyMode = ADC_TRIGGER_FREQ_HIGH;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief FDCAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_FDCAN1_Init(void)
{

  /* USER CODE BEGIN FDCAN1_Init 0 */

  /* USER CODE END FDCAN1_Init 0 */

  /* USER CODE BEGIN FDCAN1_Init 1 */

  /* USER CODE END FDCAN1_Init 1 */
  hfdcan1.Instance = FDCAN1;
  hfdcan1.Init.ClockDivider = FDCAN_CLOCK_DIV1;
  hfdcan1.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
  hfdcan1.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan1.Init.AutoRetransmission = ENABLE;
  hfdcan1.Init.TransmitPause = ENABLE;
  hfdcan1.Init.ProtocolException = DISABLE;
  hfdcan1.Init.NominalPrescaler = 2;
  hfdcan1.Init.NominalSyncJumpWidth = 16;
  hfdcan1.Init.NominalTimeSeg1 = 47;
  hfdcan1.Init.NominalTimeSeg2 = 16;
  hfdcan1.Init.DataPrescaler = 1;
  hfdcan1.Init.DataSyncJumpWidth = 4;
  hfdcan1.Init.DataTimeSeg1 = 5;
  hfdcan1.Init.DataTimeSeg2 = 4;
  hfdcan1.Init.StdFiltersNbr = 0;
  hfdcan1.Init.ExtFiltersNbr = 1;
  hfdcan1.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN FDCAN1_Init 2 */

  /* USER CODE END FDCAN1_Init 2 */

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
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 64;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 200;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL; //32MHZ
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK) //PA8 voltage control
  {
    Error_Handler();
  }

  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.BreakAFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.Break2AFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 24-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 100-1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
//static void MX_TIM6_Init(void)
//{
//
//  /* USER CODE BEGIN TIM6_Init 0 */
//
//  /* USER CODE END TIM6_Init 0 */
//
//  TIM_MasterConfigTypeDef sMasterConfig = {0};
//
//  /* USER CODE BEGIN TIM6_Init 1 */
//
//  /* USER CODE END TIM6_Init 1 */
//  htim6.Instance = TIM6;
//  htim6.Init.Prescaler = 4;
//  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
//  htim6.Init.Period = 200;
//  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
//  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
//  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
//  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  /* USER CODE BEGIN TIM6_Init 2 */
//
//  /* USER CODE END TIM6_Init 2 */
//
//}

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
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5|GPIO_PIN_7|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6
                          |GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pins : PC14 PC15 */
  GPIO_InitStruct.Pin = GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA7 PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_7|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB4 PB5 PB6
                           PB7 PB8 PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6
                          |GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);


}

/* USER CODE BEGIN 4 */
static void FDCAN_Config(void)
{
  FDCAN_FilterTypeDef sFilterConfig;

  /* Configure Rx filter */
  sFilterConfig.IdType = FDCAN_EXTENDED_ID;
  sFilterConfig.FilterIndex = 0;
  sFilterConfig.FilterType = FDCAN_FILTER_MASK;
  sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
//  sFilterConfig.FilterID1 =  0x1806E5F4; //90
//  sFilterConfig.FilterID2 = 0x18FF50E5;  //0x7ff

  sFilterConfig.FilterID1 = 0x00000000;          // No specific ID
  sFilterConfig.FilterID2 = 0x00000000;          // Mask = 0 (ignores all bits)

  if (HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilterConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /* Configure global filter:
     Filter all remote frames with STD and EXT ID
     Reject non matching frames with STD ID and EXT ID */
  if (HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE) != HAL_OK)
  {
    Error_Handler();
  }

  /* Start the FDCAN module */
  if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK)
  {
    Error_Handler();
  }

  /* Prepare Tx Header */
  TxHeader.Identifier = 0x18FF50E5;
  TxHeader.IdType = FDCAN_EXTENDED_ID;
  TxHeader.TxFrameType = FDCAN_DATA_FRAME;
  TxHeader.DataLength = FDCAN_DLC_BYTES_8;
  TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
  TxHeader.FDFormat = FDCAN_CLASSIC_CAN;
  TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
  TxHeader.MessageMarker = 0;
}



void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
  if((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET)
  {
    if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &RxHeader, RxData) != HAL_OK)
    {
      Error_Handler();
    }
    // Call function to store data, passing CAN ID

        ProcessCANMessage(&RxHeader, RxData);  // Call function to store data
        dataReceived++;
        reveived_flag = 1;

          	//HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, 1); //D6 CAN


  }

}




void PWM_output_control (unsigned int duty_ctrl)
{
	   TIM1->CCR1 =  duty_ctrl;
}

void charging_control (void)
{
	if(over_temp_protection==0 && start_recovery==1  && chargdeating==0 && Battery_full==0 && batt_high==0 && batt_open_mode==0) //6800
	{
		sw_on=1;/* Flag For Fan ON*/
		fiveamp = 0;

		gain3 = gain3 + 8; cf1++;
	}
	else if(batt_voltage <= battery_voltage1 && chargdeating==1 && over_temp_protection==0 && start_recovery==1   && Battery_full==0 && batt_high==0 && batt_open_mode==0) //6800
	{
		cf2++;
		sw_on=1;
		fiveamp = 0;
		if(gain3<=500) { gain3=gain3+8;} else { gain3=500;}

	}
	else
	{  cf4++;
	sw_on=0;
	gain3=0;
	gain=200;
	}

	if(gain<=3) { gain=0;}
	if(gain>200){ gain=200;cf3++;}

	if(gain3<=0) { gain3=0;}
	// if(gain3>900){gain3=900;}
	if(gain3>800){gain3=800;}

	PWM_output_control(gain);
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
