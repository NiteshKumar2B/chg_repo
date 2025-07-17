#include "stm32g0xx.h"
#include "measure.h"


extern volatile int prev_mains_volt1,mains_volt_avg,prev_batt_volt,batt_voltavg,avgbattvolt_cnts
,chg_current_avg,prev_chg_crnt,pv_current_avg,prev_pv_crnt,chg_current;


extern unsigned int ADC_VAL[] ,chg_cnt,m_cal,main_voltage;
extern unsigned int mains_volt,prev_mains_volt,temp_adc_main,  main_cnt,temp_value;
extern char dis_cal,batt_cal,batt_open_mode;
















 void temp_measurement(void)
 {

	 //temp_value=ADC_VAL[2];


 }












