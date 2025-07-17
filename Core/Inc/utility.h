#ifndef __UTILITY_H
#define __UTILITY_H

extern uint8_t blink_20, blink_50, blink_70, blink_100;
extern uint8_t blink_fault_slow, blink_fault_fast;

//extern uint16_t led_main_voltage,led_battery_voltage;

//extern unsigned char mains_volt_cut;
//extern unsigned char led_charging_started;

unsigned int LPF (unsigned char ,unsigned int ,unsigned int *);

#define	on						1
#define	off						2


#define Twentypercent_led			1
#define	fiftypercent_led	2
#define	seventypercent_led			3
#define	hundredpercent_led	4


#define	fault_led	5
#define	mains_led	6
#define buzzer_led   7

#define all_on_led  8
#define all_off_led  9


#endif
