//
//  File: ds18b20.h
//  Project: yafocuser
//
//  Created by Sven Kreiensen on 06.11.19.
//  Copyright © 2019 Sven Kreiensen. All rights reserved.
//
#ifndef DS18B20_H_
#define DS18B20_H_

#include <avr/io.h>

// Temperature sensor DS18B20 pinouts
// Pin 10 supplies ground
// Pin 11 supplies Vcc
// Note a 4.7k pull up resistor is required between pins VCC and 22 (PD4)

#define DS18B20_PIN	PD4
#define DS18B20_PINREG	PIND
#define DS18B20_INPUT() (DDRD &= ~(1<<DS18B20_PIN))
#define DS18B20_OUTPUT() (DDRD |= (1<<DS18B20_PIN))
#define DS18B20_LOW()    (PORTD &= ~(1<<DS18B20_PIN))
#define DS18B20_HIGH()   (PORTD |= (1<<DS18B20_PIN))

#define DS18B20_CMD_CONVERTTEMP		0x44
#define DS18B20_CMD_RSCRATCHPAD		0xBE
#define DS18B20_CMD_WSCRATCHPAD		0x4E
#define DS18B20_CMD_CPYSCRATCHPAD	0x48
#define DS18B20_CMD_RECEEPROM		0xB8
#define DS18B20_CMD_RPWRSUPPLY		0xB4
#define DS18B20_CMD_SEARCHROM		0xF0
#define DS18B20_CMD_READROM		0x33
#define DS18B20_CMD_MATCHROM		0x55
#define DS18B20_CMD_SKIPROM		0xCC
#define DS18B20_CMD_ALARMSEARCH		0xEC

#define DS18B20_RESOLUTION_12BIT    0x7F    // 750ms wait, precision 0.0625 deg.C
#define DS18B20_RESOLUTION_9BIT     0x1F    // 93,75ms wait, precision 0.5 deg.C

#define DS18B20_DECIMAL_STEPS_12BIT 	625 //.0625

//stop any interrupt on read
#define DS18B20_STOPINTERRUPTONREAD 1

//functions
int8_t ds18b20_get_temperature(void);

#endif
