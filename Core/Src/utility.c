#include "stm32g0xx.h"
#include "utility.h"  



//unsigned int led_battery_voltage,led_main_voltage;

 //update_charging_leds( _battery_voltage,  _main_voltage,  _charging_started);

//update_charging_leds(battery_voltage, charging_started, blink_state);


//#define blink_state ^= 1;  // Toggle 0 <-> 1


/*LPF(2,temp_adc_data,&prev_batt_volt);*/

unsigned int LPF (unsigned char coef,unsigned int current_sample, unsigned int *temp)
{
	current_sample = ((current_sample*coef)+((10-coef)*(*temp)));	
	current_sample = (current_sample+5)/10;
	*temp = current_sample;
	return current_sample;	
}



void led_control (unsigned char led,unsigned char state)
{

 switch (led)
	{
//		case Twentypercent_led:
//			if(state == on)
//				 GPIOA->ODR |= 0x00008000;
//			else
//				GPIOA->ODR &= 0xFFFF7FFF;
//		break;
//		case fiftypercent_led:
//			if (state == on)
//				GPIOB->ODR |= 0x00000040;
//			else
//
//			GPIOB->ODR &= 0xFFFFFFBF;
//		break;
//		case seventypercent_led:
//			if (state == on)
//				GPIOB->ODR |= 0x00000080;
//			else
//				 GPIOB->ODR &= 0xFFFFFF7F;
//		break;
//		case hundredpercent_led:
//			if (state == on)
//				GPIOB->ODR |= 0x00000100;
//			else
//			  GPIOB->ODR &= 0xFFFFFEFF;
//
//		break;
//
//		case  mains_led:               //pc15
//				if (state == on)
//					GPIOC->ODR |= 0x00008000;
//				else
//					 GPIOC->ODR &= 0xFFFF7FFF;
//
//			break;
//		case fault_led:
//				if (state == on)
//					 GPIOC->ODR |= 0x00004000;
//				else
//					 GPIOC->ODR &=0xFFFFBFFF;
//
//			break;
//
//
//		case buzzer_led:                    //pc15
//				if (state == on)
//				  GPIOB->ODR |= 0x00000001;
//				else
//					 GPIOB->ODR &= 0xFFFFFFFE;
//
//
//		case all_on_led:                    //pc15
//				if (state == on)
//				  GPIOB->ODR |= 0x00000001;
//				else
//					 GPIOB->ODR &= 0xFFFFFFFE;
//
//
//		case all_off_led:                    //pc15
//				if (state == on)
//				  GPIOB->ODR |= 0x00000001;
//				else
//					 GPIOB->ODR &= 0xFFFFFFFE;


	}
}



//void update_charging_leds(uint16_t voltage, uint8_t charging_started, uint8_t blink_toggle)
//{
//    if (!charging_started)
//    {
//        // Turn off all LEDs if not charging
//        led_control(Twentypercent_led, off);
//        led_control(fiftypercent_led, off);
//        led_control(seventypercent_led, off);
//        led_control(hundredpercent_led, off);
//        return;
//    }
//
//    // First LED
//    if (voltage < 5117)
//        led_control(Twentypercent_led, blink_toggle);
//    else
//        led_control(Twentypercent_led, on);
//
//    // Second LED
//    if (voltage >= 5117 && voltage < 5317)
//        led_control(fiftypercent_led, blink_toggle);
//    else if (voltage >= 5317)
//        led_control(fiftypercent_led, on);
//    else
//        led_control(fiftypercent_led, off);  // before 5117, keep OFF
//
//    // Third LED
//    if (voltage >= 5317 && voltage < 5413)
//        led_control(seventypercent_led, blink_toggle);
//    else if (voltage >= 5413)
//        led_control(seventypercent_led, on);
//    else
//        led_control(seventypercent_led, off); // before 5317, keep OFF
//
//    // Fourth LED
//    if (voltage >= 5413 && voltage < 5610)
//        led_control(hundredpercent_led, blink_toggle);
//    else if (voltage >= 5610)
//        led_control(hundredpercent_led, on);
//    else
//        led_control(hundredpercent_led, off); // before 5413, keep OFF
//}


/*this function call every 500ms in timer */

// You may define these as timers/counters elsewhere in your code
//extern uint8_t blink_toggle_fast;
//extern uint8_t blink_toggle_slow;

//void update_charging_leds(uint16_t battery_voltage, uint8_t charging_started, uint8_t blink_toggle)
//{
//    if (!charging_started)
//    {
//        // If not charging, turn off all LEDs
//        led_control(Twentypercent_led, off);
//        led_control(fiftypercent_led, off);
//        led_control(seventypercent_led, off);
//        led_control(hundredpercent_led, off);
//        led_control(fault_led, off);
//        return;
//    }
//
//    // 20% LED
//    if (battery_voltage >= 5117)
//        led_control(Twentypercent_led, on);
//    else
//        led_control(Twentypercent_led, blink_toggle);
//
//    // 50% LED
//    if (battery_voltage >= 5317)
//        led_control(fiftypercent_led, on);
//    else if (battery_voltage >= 5117)
//        led_control(fiftypercent_led, blink_toggle);
//    else
//        led_control(fiftypercent_led, off);
//
//    // 70% LED
//    if (battery_voltage >= 5413)
//        led_control(seventypercent_led, on);
//    else if (battery_voltage >= 5317)
//        led_control(seventypercent_led, blink_toggle);
//    else
//        led_control(seventypercent_led, off);
//
//    // 100% LED
//    if (battery_voltage >= 5610)
//        led_control(hundredpercent_led, on);
//    else if (battery_voltage >= 5413)
//        led_control(hundredpercent_led, blink_toggle);
//    else
//        led_control(hundredpercent_led, off);



// ------------------------
// Fault LED Logic
// ------------------------
//if (battery_voltage < 4300) {
//	// Fault LED solid ON
//	led_control(fault_led, on);
//} else if (main_voltage < 120) {
//	// Blink slow
//	led_control(fault_led, blink_toggle_slow);
//} else if (main_voltage > 300) {
//	// Blink fast
//	led_control(fault_led, blink_toggle_fast);
//} else {
//	// No fault
//	led_control(fault_led, off);
//}

//}



//// Declare globally
//uint8_t blink_toggle = 0;
//uint8_t blink_toggle_slow = 0;
//uint8_t blink_toggle_fast = 0;
//
//// Toggle these in a timer interrupt (e.g., HAL_TIM_PeriodElapsedCallback)
//void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
//{
//    static uint16_t counter = 0;
//
//    counter++;
//
//    // 1 Hz blinking (toggle every 500ms)
//    if (counter % 1 == 0) blink_toggle ^= 1;
//
//    // 0.5 Hz (slow blink)
//    if (counter % 2 == 0) blink_toggle_slow ^= 1;
//
//    // 2 Hz (fast blink)
//    if (counter % 0 == 0) blink_toggle_fast ^= 1;
//
//    // Reset counter to avoid overflow
//    if (counter >= 1000) counter = 0;
//}
