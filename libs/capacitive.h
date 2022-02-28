/*
 * capacitive.h
 *
 * Created: 25.02.2022 13:51:09
 *  Author: phigu
 */ 


#ifndef CAPACITIVE_H_
#define CAPACITIVE_H_

// Port and Pin definitions for capacitance measurement
#define CAP_PORT_DIR	DDRB
#define CAP_PORT_PIN	PINB
#define CAP_PORT		PORTB
#define CAP_OUT_PIN		PINB2
#define CAP_IN_PIN		PINB4

#define MAX_TIMER1_OVF 1000
// Capacitance between IN_PIN and Ground
// Stray capacitance value will vary from board to board.
// Calibrate this value using known capacitor.
#define IN_STRAY_CAP_TO_GND		24.48
#define IN_CAP_TO_GND			IN_STRAY_CAP_TO_GND

// Pullup resistance will vary depending on board.
// Calibrate this with known capacitor.
#define R_PULLUP				34.8 //kOhm
#define MAX_ADC_VALUE			1023

void capacitive_init(void);
uint16_t get_capacitance(void);
uint16_t get_capacitance_avg(uint8_t n_measures);


#endif /* CAPACITIVE_H_ */