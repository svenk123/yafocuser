//
//  File: ds18b20.h
//  Project: yafocuser
//
//  Created by Sven Kreiensen on 06.11.19.
//  Copyright © 2019 Sven Kreiensen. All rights reserved.
//
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

#include "ds18b20.h"

//
// Initialize DS18B20 sensor
//
uint8_t ds18b20_reset(void) {
    uint8_t i = 0;

    // low for 480us
    DS18B20_LOW();
    DS18B20_OUTPUT();
    _delay_us(480);

    // release line and wait for 60uS
    DS18B20_INPUT();
    _delay_us(60);

    // get value and wait 420us
    i = (DS18B20_PINREG & (1<<DS18B20_PIN));
    _delay_us(420);

    // return the read value, 0=ok, 1=error
    return i;
}

//
// write one bit
//
void ds18b20_write_bit(uint8_t bit) {
    // Pull line low for 1uS
    DS18B20_LOW();
    DS18B20_OUTPUT();
    _delay_us(1);

    // if we want to write 1, release the line (if not will keep low)
    if(bit)
        DS18B20_INPUT();

    // wait 60uS and release the line
    _delay_us(60);
    DS18B20_INPUT();
}

//
// read one bit
//
uint8_t ds18b20_read_bit(void){
    uint8_t bit=0;

    // Pull line low for 1uS
    DS18B20_LOW();
    DS18B20_OUTPUT();
    _delay_us(1);

    // release line and wait for 14uS
    DS18B20_INPUT();
    _delay_us(14);

    // read the value
    if (DS18B20_PINREG & (1<<DS18B20_PIN))
        bit=1;

    // wait 45uS and return read value
    _delay_us(45);
    
    return bit;
}

//
// write one byte
//
void ds18b20_write_byte(uint8_t byte){
    uint8_t i=8;

    while (i--) {
        ds18b20_write_bit(byte & 1);
        byte >>= 1;
    }
}

//
// read one byte
//
uint8_t ds18b20_read_byte(void){
    uint8_t i=8;
    uint8_t n=0;

    while (i--) {
        n >>= 1;
        n |= (ds18b20_read_bit() << 7);
    }

    return n;
}

//
// get temperature
//
int8_t ds18b20_get_temperature(void) {
    uint8_t temperature[2];
    int8_t digit;
    //uint16_t decimal = 0;
    
#if DS18B20_STOPINTERRUPTONREAD == 1
    cli();
#endif
    
    // Reset, skip ROM and write Scratchpad for 9-bit resoltion
    ds18b20_reset();
    ds18b20_write_byte(DS18B20_CMD_SKIPROM);
    ds18b20_write_byte(DS18B20_CMD_WSCRATCHPAD);
    ds18b20_write_byte(0); //TH Register
    ds18b20_write_byte(0); //TL Register
    ds18b20_write_byte(DS18B20_RESOLUTION_9BIT);  //Configuration Register
    
    // Reset, skip ROM and start temperature conversion
    ds18b20_reset();
    ds18b20_write_byte(DS18B20_CMD_SKIPROM);
    ds18b20_write_byte(DS18B20_CMD_CONVERTTEMP);

    // wait until conversion is complete
    while (!ds18b20_read_bit());

    // Reset, skip ROM and send command to read Scratchpad
    ds18b20_reset();
    ds18b20_write_byte(DS18B20_CMD_SKIPROM);
    ds18b20_write_byte(DS18B20_CMD_RSCRATCHPAD);

    //Read Scratchpad (only 2 first bytes)
    temperature[0] = ds18b20_read_byte();
    temperature[1] = ds18b20_read_byte();
    ds18b20_reset();

#if DS18B20_STOPINTERRUPTONREAD == 1
    sei();
#endif

    //Store temperature integer digits and decimal digits
    digit = ((temperature[0] >> 4) | ((temperature[1] & 0x7) << 4));
    
    // Negative temperature
    if ((temperature[1] & 0x80) > 0) {
        digit |= 0x80;  // Set MSB to 1
    }
    
    //Store decimal digits
    //if (temperature[0] & 0x01) {
    //    decimal += 63;  // add 0.0625 deg.C
    //}
    //if (temperature[0] & 0x02) {
    //    decimal += 125;  // add 0.125 deg.C
    //}
    //if (temperature[0] & 0x04) {
    //    decimal += 250;  // add 0.25 deg.C
    //}
    //if (temperature[0] & 0x08) {
    //    decimal += 500;  // add 0.5 deg.C
    //}

    return digit;
}

