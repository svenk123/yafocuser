//
//  File: yafocuser.c
//  Project: yafocuser
//  A Moonlite-compatible stepper controller
//
//  Created by Sven Kreiensen on 06.11.19.
//  Copyright © 2019 Sven Kreiensen. All rights reserved.
//
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>
#include <util/atomic.h>
#include <ctype.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <util/delay.h>
#include <avr/wdt.h>
#include <avr/eeprom.h>

#include "millis.h"
#include "ds18b20.h"
#include "usb_serial.h"

#define FWVERSION "1.2"

#define CPU_PRESCALE(n) (CLKPR = 0x80, CLKPR = (n))
#define CPU_16MHz	0x00
#define CPU_125kHz	0x07
#define HEX(n) (((n) < 10) ? ((n) + '0') : ((n) + 'A' - 10))

//#define USB_ECHO	1
//#define EXTERNAL_TEMPERATURE_SENSOR 1

#define ADC_TIMEOUT 1000 // wait ADC_TIMEOUT clock cycles for a single ADC conversion

#define TIMER_FAST  0

#define LED_ON()	(PORTD |= (1<<6))
#define LED_OFF()	(PORTD &= ~(1<<6))
#define LED_OUTPUT()	(DDRD |= (1<<6))

#define GPIO_MOTOR_PIN1	7	// PC7		// Blue - 28BYJ48 pin 1
#define GPIO_MOTOR_PIN2 6       // PC6		Pink - 28BYJ48 pin 2
#define GPIO_MOTOR_PIN3 3       // PD3			Yellow - 28BYJ48 pin 3
#define GPIO_MOTOR_PIN4 2       // PD2		Orange - 28BYJ48 pin 4

#define GPIO_MOTOR_PIN1_OUTPUT() 	(DDRC |= (1<<GPIO_MOTOR_PIN1))
#define GPIO_MOTOR_PIN2_OUTPUT() 	(DDRC |= (1<<GPIO_MOTOR_PIN2))
#define GPIO_MOTOR_PIN3_OUTPUT() 	(DDRD |= (1<<GPIO_MOTOR_PIN3))
#define GPIO_MOTOR_PIN4_OUTPUT() 	(DDRD |= (1<<GPIO_MOTOR_PIN4))

#define GPIO_MOTOR_PIN1_ON() 	(PORTC |= (1<<GPIO_MOTOR_PIN1))
#define GPIO_MOTOR_PIN1_OFF() 	(PORTC &= ~(1<<GPIO_MOTOR_PIN1))
#define GPIO_MOTOR_PIN2_ON() 	(PORTC |= (1<<GPIO_MOTOR_PIN2))
#define GPIO_MOTOR_PIN2_OFF() 	(PORTC &= ~(1<<GPIO_MOTOR_PIN2))
#define GPIO_MOTOR_PIN3_ON() 	(PORTD |= (1<<GPIO_MOTOR_PIN3))
#define GPIO_MOTOR_PIN3_OFF() 	(PORTD &= ~(1<<GPIO_MOTOR_PIN3))
#define GPIO_MOTOR_PIN4_ON() 	(PORTD |= (1<<GPIO_MOTOR_PIN4))
#define GPIO_MOTOR_PIN4_OFF() 	(PORTD &= ~(1<<GPIO_MOTOR_PIN4))

#define MIN_SPEED 20
#define MAX_SPEED 2
#define MIN_POSITION 0
#define HOME_POSITION 20000
#define DEFAULT_TEMPERATURE 20

#define MAXCOMMAND 8
#define DEFAULT_POLL_INTERVAL	20	// 20ms
#define SAVED_STEPPER_POSITION 0  // EEPROM address for saved stepper position

uint16_t eeFooWord EEMEM = 12345;

//
// Function prototypes
//
void send_str(const char *s);
void send_str_pgm(const char *s);
uint8_t recv_str(char *buf, uint8_t size);
void parse_and_execute_command(const char *buf, uint8_t num);
void motorOff(void);

//
// Global variables
//
volatile uint16_t current_position;
volatile uint16_t target_position;
volatile uint16_t millis_last_move; // Motor timeout to write current position to EEPROM and shut off motor pins
volatile uint8_t speed; // Motor speed
volatile uint8_t is_running;
volatile uint8_t last_position_not_saved;
uint8_t half_step_mode;
const uint8_t hex_table[16] = {'0', '1', '2', '3', '4', '5', '6', '7', '8', '9', 'A', 'B', 'C', 'D', 'E', 'F'};
uint8_t current_temperature;
int8_t temperature_coefficient;

//
// Check if a bit is set in a uint8_t variable
//
uint8_t isBitSet(uint8_t var, uint8_t bit) {
    if ((var) & (1 << (bit)))
        return 1;
    else
        return 0;
}

//
// Convert a 4 char string to int16_t
//
int16_t convert4CharToInt(char c1, char c2, char c3, char c4) {
    int16_t value = 0;
    int i;

    for (i=0; i<16; i++) {
        if(hex_table[i] == c4)
            value |= (int16_t)i;

        if(hex_table[i] == c3)
            value |= ((int16_t)i) << 4;

        if(hex_table[i] == c2)
            value |= ((int16_t)i) << 8;
	
        if(hex_table[i] == c1)
            value |= ((int16_t)i) << 12;
    }

    return value;
}

// Convert a 2 char string to int16_t
int16_t convert2CharToInt(char c1, char c2) {
    int16_t value = 0;
    int i;

    for (i=0; i<16; i++) {
        if(hex_table[i] == c2)
            value |= (int16_t)i;

        if(hex_table[i] == c1)
            value |= ((int16_t)i) << 4;
    }

    return value;
}

//
// Convert signed and unsigned values to a string
//
void convertIntToChar(int16_t value, int nbChar, char *buffer) {
    int i;
    int shift = 0;

    for (i = nbChar-1; i >= 0; i--) {
        buffer[i] = hex_table[(value >> shift) & 0xF];
        shift = shift + 4;
    }
}


//
// Disable all hardware peripherals
//
void disablePeripherals(void) {
        EIMSK = 0;
        PCICR = 0;
        SPCR = 0;
        ACSR = 0;
        EECR = 0;
        ADCSRA = 0;
        TIMSK0 = 0;
        TIMSK1 = 0;
        TIMSK3 = 0;
        TIMSK4 = 0;
        UCSR1B = 0;
        TWCR = 0;
        DDRB = 0;
        DDRC = 0;
        DDRD = 0;
        DDRE = 0;
        DDRF = 0;
        TWCR = 0;
        PORTB = 0;
        PORTC = 0;
        PORTD = 0;
        PORTE = 0;
        PORTF = 0;
}

//
// Reboot the device
//
void reboot_yafocuser(uint8_t bootloader) {
    motorOff();

    send_str_pgm(PSTR("Rebooting...\r\n"));
    
    _delay_ms(2000);
    
    cli();

    // stop watchdog timer, if running
    MCUSR &= ~(1<<WDRF);
    WDTCSR |= (1<<WDCE);
    WDTCSR = 0;

    _delay_ms(5);

    UDIEN = 0;  // Disable USB interrupts
    UDCON = 1;  // Disconnect attach resistor
    USBCON = (1<<FRZCLK); // Shut off USB peripheral

    _delay_ms(5);

    disablePeripherals();

    if (bootloader == 1) {
        // Jump into bootloader
        asm volatile("jmp 0x7000");
    } else {
        // Normal reboot
        asm volatile("jmp 0x0");
    }
        
    //__builtin_unreachable();  // available in gcc 4.5
    while (1) ;
}

//
// Switch the motor wires per phase
//
void setOutput(uint8_t phase) {
    // Lookup table
    uint8_t step_table[8] = { 0b01001, 0b00001, 0b00011, 0b00010, 0b00110, 0b00100, 0b01100, 0b01000 };

    if (isBitSet(step_table[phase], 0) == 1) {
        GPIO_MOTOR_PIN1_ON();
    } else {
        GPIO_MOTOR_PIN1_OFF();
    }

    if (isBitSet(step_table[phase], 1) == 1) {
        GPIO_MOTOR_PIN2_ON();
    } else {
        GPIO_MOTOR_PIN2_OFF();
    }

    if (isBitSet(step_table[phase], 2) == 1) {
        GPIO_MOTOR_PIN3_ON();
    } else {
        GPIO_MOTOR_PIN3_OFF();
    }

    if (isBitSet(step_table[phase], 3) == 1) {
        GPIO_MOTOR_PIN4_ON();
    } else {
        GPIO_MOTOR_PIN4_OFF();
    }
}

//
// Turn of the stepper motor
//
void motorOff(void) {
    GPIO_MOTOR_PIN1_OFF();
    GPIO_MOTOR_PIN2_OFF();
    GPIO_MOTOR_PIN3_OFF();
    GPIO_MOTOR_PIN4_OFF();
}

//
// Timer0 overflow interrupt
//
ISR(TIMER0_OVF_vect) {
    static int16_t distance_to_go = 0;
    static uint8_t mphase = 0;
    static uint8_t wait = 0;

    // Compute remaining distance and move the motor
    distance_to_go = target_position - current_position;

    if (distance_to_go == 0) {
        // Motion is complete
        is_running = 0;
    }
	
    if (is_running){
        if (wait == 0) {
            if (distance_to_go > 0) {
                // Forward step
                current_position++;
                mphase++;
                mphase &= 0x07;
                setOutput(mphase);
            }
		
            if (distance_to_go < 0) {
                // Backward step
                current_position--;
                mphase--;
                mphase &= 0x07;
                setOutput(mphase);
            }

            millis_last_move = millis();
            last_position_not_saved = 1;

            // New wait cycle
            if (speed == 2)
                wait = 0;
            else if (speed == 4)
                wait = 8;
            else if (speed == 8)
                wait = 16;
            else if (speed == 10)
                wait = 32;
            else if (speed == 20)
                wait = 64;
            else
                wait = 4;
        }
        
        if (wait > 0) {
            wait=wait-1;

        }

    }
    
    TCNT0 = 6;
}

//
// Stepper motor initialization
//
void motorInit(void) {
    //setup the motor pins as outputs
    GPIO_MOTOR_PIN1_OUTPUT();
    GPIO_MOTOR_PIN2_OUTPUT();
    GPIO_MOTOR_PIN3_OUTPUT();
    GPIO_MOTOR_PIN4_OUTPUT();

    speed = MIN_SPEED;
    millis_last_move = millis();
    half_step_mode = 0;
    is_running = 0;
    target_position = HOME_POSITION;
    last_position_not_saved = 1;
    
    // Fetch current stepper position from EEPROM
    current_position = eeprom_read_word (&eeFooWord);
    if (current_position == 0xFFFF)
        current_position = HOME_POSITION;

    // Setup an 8-bit timer with 1 ms ticks
    TCNT0 = 6;    // Preload 0
    TCCR0A = 0;    // Normal Counter Mode
    TCCR0B |= (1<<CS01) | (1<<CS00);    // Prescaler 64

    TIMSK0 |= (1 << TOIE0); // Enable overflow interrupt
}

//
// Hand-controller initialization
//
void handControllerInit(void) {
    uint16_t adc_timeout = ADC_TIMEOUT;

    // internal reference
    ADMUX = (1<<REFS1) | (1<<REFS0);

    // Bit ADFR ("free running") in ADCSRA is 0 for single conversion
    ADCSRA = (1<<ADPS1) | (1<<ADPS0);
    ADCSRA |= (1<<ADEN);                  // Enable ADC

    // Dummy readout as warm-up
    ADCSRA |= (1<<ADSC);

    // Wait for measurement
    while ( (ADCSRA & (1<<ADSC)) && ( --adc_timeout > 0 ) ) {
        
    }

    
    (void) ADCW;
}

//
// ADC single measurement
//
uint16_t analogRead(uint8_t channel) {
    uint16_t adc_timeout = ADC_TIMEOUT;

    // Select channel
    ADMUX = (ADMUX & ~(0x1F)) | (channel & 0x1F);

    // Start measurement
    ADCSRA |= (1<<ADSC);
    
    // Wait for measurement
    while ( (ADCSRA & (1<<ADSC)) && ( --adc_timeout > 0 ) ) {
    
    }

    // Measurement timeout
    if (adc_timeout == 0)
        return 0;
    
    return ADCW;
}

//
// Read hand-controller buttons
//
void handControllerRead(void) {
    uint8_t speed_table[16] = {2,2,4,8,10,20,0,0,0,0,20,10,8,4,2,2};
    int8_t go_table[16] = {-1,-1,-1,-1,-1,-1,0,0,0,0,1,1,1,1,1,1};
    uint8_t rADC = analogRead(0); // Channel 0

    if (rADC > 20) {
        // if hand controller is connected
        rADC = rADC>>6;
    
        if(go_table[rADC]) {
            speed = speed_table[rADC];

            ATOMIC_BLOCK(ATOMIC_FORCEON) {
                target_position = target_position + go_table[rADC];
            }

            is_running = 1;
            LED_ON();
        }
    }
}

//
// Return command help
//
void usage(void) {
	// print a nice welcome message
	send_str_pgm(PSTR("\r\nYaFocuser v"));
	send_str_pgm(PSTR(FWVERSION));
	send_str_pgm(PSTR(" a Moonlite-compatible stepper controller,\r\n "
	    "Example Commands\r\n"
        " :GI#      get motor running\r\n"
        " :FG#      start motor \r\n"
        " :FQ#      stop motor\r\n"
        " :GP#      get current position\r\n"
        " :SP?????# set current position\r\n"
        " :GN#      get new target position\r\n"
        " :SN?????# set new target position\r\n"
        " :GC#      get temperature coefficient\r\n"
        " :SC??#    set temperature coefficent\r\n"
        " :C#       start temperature conversion\r\n"
	    " :GT#      get temperature\r\n"
        " :GH#      get half step mode\r\n"
	    " :SH#      set half step mode\r\n"
	    " :SF#      set full step mode\r\n"
        " :GD#      get motor speed\r\n"
	    " :SD??#    set motor speed 02,04,08,10,20\r\n"
        " :PH#      find motor home\r\n"
        " :GB#      get current backlight value\r\n"
        " :GV#      get version\r\n"
	    " :HELP#	print usage\r\n"
	    " :RN#      restart normal\r\n"
	    " :RB#      restart bootloader\r\n"));
}

//
// Main function
//
int main(void) {
    char buf[32];
    uint8_t n;

    CPU_PRESCALE(CPU_125kHz);
    _delay_ms(1); // allow slow power supply startup
    CPU_PRESCALE(CPU_16MHz); // set for 16 MHz clock

    // Activate AVR watchdog
//    CLR_WADORESET();
//    wdt_disable();

    cli();

    LED_OUTPUT();
    LED_ON();

    init_millis(16000000UL);

    motorInit();
    motorOff();

//    handControllerInit();

    sei();

    usb_init();
    
    LED_OFF();

    current_temperature = 0x20; // 16 deg.C.
    temperature_coefficient = 0;

    while (1) {
        //readHandController();

        /* Poll interval */ ;
        _delay_ms(DEFAULT_POLL_INTERVAL);

        // Der Benutzer hat sein Terminalprogramm gestartet, was durch das
        // DTR-Signal detektiert wird. Wir sind nun bereits Zeichen zu
        // empfangen
        if (usb_configured() && (usb_serial_get_control() & USB_SERIAL_DTR)) {
            // discard anything that was received prior.  Sometimes the
            // operating system or other software will send a modem
            // "AT command", which can still be buffered.
            usb_serial_flush_input();

    	    // and then listen for commands and process them
            while (1) {

                n = recv_str(buf, sizeof(buf));
                if (n == 255)
                    break;
                
                //_delay_ms(500);
                
                parse_and_execute_command(buf, n);

                memset(buf, 0, sizeof(buf));

                // Turn off motor if idle time is up
                if (is_running == 0) {
                    if ((millis() - millis_last_move) > 5000) {
                        motorOff();
                        LED_OFF();

                        // Store current position in EEPROM
                        if (last_position_not_saved) {
                            LED_ON();
                            eeprom_update_word(&eeFooWord, current_position);
                            last_position_not_saved = 0;
                            LED_OFF();
                        }
                    }
                }
            }
        }
    }

    // Stop the stepper motor
    motorOff();
}

//
// Output string on USB
//
void send_str(const char *s) {
    // USB not connected
    if (!usb_configured())
        return;
    
    // Users terminal program is not running
    if (!(usb_serial_get_control() && USB_SERIAL_DTR))
        return;
    
    usb_serial_write((uint8_t*)s, strlen(s));
}

//
// Output progmem string on USB
//
void send_str_pgm(const char *s) {
    char c;

    // USB not connected
    if (!usb_configured())
        return;

    // Users terminal program is not running
    if (!(usb_serial_get_control() && USB_SERIAL_DTR))
        return;

    while (1) {
        c = pgm_read_byte(s++);
        if (!c)
            break;
        
        usb_serial_putchar(c);
    }
}

//
// Receive a string from the USB serial port.  The string is stored
// in the buffer and this function will not exceed the buffer size.
// A carriage return or newline completes the string, and is not
// stored into the buffer.
// The return value is the number of characters received, or 255 if
// the virtual serial connection was closed while waiting.
//
uint8_t recv_str(char *buf, uint8_t size) {
    int16_t r;
    uint8_t count=0;

    while (count < size) {
        r = usb_serial_getchar();
        if (r != -1) {
//            if (r == '\r' || r == '\n')
//                return count;

            if (r == '#') {
                *buf++ = r;

#ifdef USB_ECHO
                usb_serial_putchar(r);
#endif

                count++;
		
                return count;
            }

            if (r >= ' ' && r <= '~') {
                *buf++ = r;
#ifdef USB_ECHO
                usb_serial_putchar(r);
#endif
                count++;
            }
        } else {
            if (!usb_configured() || !(usb_serial_get_control() & USB_SERIAL_DTR)) {
                // user no longer connected
                return 255;
            }
            // just a normal timeout, keep waiting
        }
    }

    return count;
}

//
// parse a user command and execute it, or print an error message
//
void parse_and_execute_command(const char *buf, uint8_t num) {
    // Command is too short
    if (num < 3)
        return;

    // Command is too long
    if (num > 9)
        return;

    // Check if it is a command
    if (!(buf[0] == ':'))
        return;

    // Check for terminating character
    if (!(buf[num-1] == '#'))
        return;

    // C command start temperature readout
    if ((buf[1] == 'C') && (num == 3)) {
#ifdef EXTERNAL_TEMPERATURE_SENSOR
        current_temperature=ds18b20GetTemp()*2;
#else
        current_temperature=DEFAULT_TEMPERATURE*2;
#endif
        return;
    }

    // HELP command print usage
    if ((buf[1] == 'H') && (buf[2] == 'E')  && (buf[3] == 'L')  && (buf[4] == 'P') && (num==6)) {
        usage();
        return;
    }
    
    // GB
    if ((buf[1] == 'G') && (buf[2] == 'B') && (num==4))
        return;

    // GP command Get current position
    if ((buf[1] == 'G') && (buf[2] == 'P') && (num==4)) {
        char temp_str[20];
        
        memset(temp_str, 0, sizeof(temp_str));
        ATOMIC_BLOCK(ATOMIC_FORCEON) {
            convertIntToChar(current_position, 4, temp_str);
        }
        send_str(temp_str);
        send_str_pgm(PSTR("#"));
        return;
    }

    // GT command Get Temperature
    if ((buf[1] == 'G') && (buf[2] == 'T') && (num==4)) {
        char temp_str[20];
        memset(temp_str, 0, sizeof(temp_str));
        // The returned temperature is in degrees Celcius * 2 for benefit on MoonLite drivers
        convertIntToChar(current_temperature, 2, temp_str);
        send_str(temp_str);
        send_str_pgm(PSTR("#"));

        return;
    }

    // GI command 01 if motor running, 00 if not
    if ((buf[1] == 'G') && (buf[2] == 'I') && (num==4)) {
        if (is_running != 0) {
            send_str_pgm(PSTR("01#"));
        } else {
            send_str_pgm(PSTR("00#"));
        }
        
        return;
    }
    
    // GB command Get current backlight value, always 00
    if ((buf[1] == 'G') && (buf[2] == 'B') && (num==4)) {
        send_str_pgm(PSTR("00#"));
        return;
    }

    // PH command Find motor home
    if ((buf[1] == 'P') && (buf[2] == 'H') && (num==4)) {
        ATOMIC_BLOCK(ATOMIC_FORCEON) {
            current_position = HOME_POSITION;
        }
        is_running = 1;
        return;
    }

    // GV command Get software version, always 10
    if ((buf[1] == 'G') && (buf[2] == 'V') && (num==4)) {
        send_str_pgm(PSTR("10#"));
        return;
    }

    // GN command Get new (target) position
    if ((buf[1] == 'G') && (buf[2] == 'N') && (num==4)) {
        char temp_str[20];
        memset(temp_str, 0, sizeof(temp_str));
        ATOMIC_BLOCK(ATOMIC_FORCEON) {
            convertIntToChar(target_position, 4, temp_str);
        }
        send_str(temp_str);
        send_str_pgm(PSTR("#"));
        return;
    }

    // GC command Get temerature coefficient, always 2
    if ((buf[1] == 'G') && (buf[2] == 'C') && (num==4)) {
        char temp_str[20];
        memset(temp_str, 0, sizeof(temp_str));
        convertIntToChar(temperature_coefficient, 2, temp_str);
        send_str(temp_str);
        send_str_pgm(PSTR("#"));
        return;
    }

    // SCxx temperature coefficient
    if ((buf[1] == 'S') && (buf[2] == 'C') && (num < 6)) {
        temperature_coefficient = convert2CharToInt(buf[3], buf[4]);
        return;
    }

    // GD command Get motor speed
    if ((buf[1] == 'G') && (buf[2] == 'D') && (num==4)) {
        char temp_str[20];
        memset(temp_str, 0, sizeof(temp_str));

        // Return 02, 04, 08, 20
        ATOMIC_BLOCK(ATOMIC_FORCEON) {
            convertIntToChar(target_position, 2, temp_str);
        }
        send_str(temp_str);
        send_str_pgm(PSTR("#"));
        return;
    }
    
    // SDxx command Set motor speed
    // Set the new stepping delay where XX is a two-digit,
    // unsigned hex number. Valid values to send are 02,
    // 04, 08, 10 and 20, which correspond to a stepping
    // delay of 250, 125, 63, 32 and 16 steps per second
    // respectively.
    if ((buf[1] == 'S') && (buf[2] == 'D') && (num == 6)) {
        uint8_t tspeed = 2;
        if ((buf[3]=='0') && (buf[4]=='4'))
            tspeed = 4;
        if ((buf[3]=='0') && (buf[4]=='8'))
            tspeed = 8;
        if ((buf[3]=='1') && (buf[4]=='0'))
            tspeed = 10;
        if ((buf[3]=='2') && (buf[4]=='0'))
            tspeed = 20;
        ATOMIC_BLOCK(ATOMIC_FORCEON) {
            speed = tspeed;
        }

        return;
    }

    // GH command Get half step mode, always 00
    // 00 = full step
    // FF = half step
    if ((buf[1] == 'G') && (buf[2] == 'H') && (num==4)) {
        if (half_step_mode)
            send_str_pgm(PSTR("FF#"));
        else
            send_str_pgm(PSTR("00#"));

        return;
    }

    // SH set half step mode
    if ((buf[1] == 'S') && (buf[2] == 'H') && (num==4)) {
        half_step_mode = 1;
        
        return;
    }

    // SF set full step mode
    if ((buf[1] == 'S') && (buf[2] == 'F') && (num==4)) {
        half_step_mode = 0;
        
        return;
    }

    // SPxxxx command Set current position
    if ((buf[1] == 'S') && (buf[2] == 'P') && (num < 9)) {
        int16_t pos = convert4CharToInt(buf[3], buf[4], buf[5], buf[6]);
        if (pos == 0)
            pos = 0;

        ATOMIC_BLOCK(ATOMIC_FORCEON) {
            current_position = pos;
        }
        ATOMIC_BLOCK(ATOMIC_FORCEON) {
            millis_last_move = millis(); // reset idle timer
        }
        is_running = 1;

        return;
    }

    // SNxxxx command Set new position
    if ((buf[1] == 'S') && (buf[2] == 'N') && (num < 9)) {
        int16_t pos = convert4CharToInt(buf[3], buf[4], buf[5], buf[6]);
        if (pos == 0)
            pos = 0;

        ATOMIC_BLOCK(ATOMIC_FORCEON) {
            target_position = pos;
        }
        ATOMIC_BLOCK(ATOMIC_FORCEON) {
            millis_last_move = millis(); // reset idle timer
        }
        is_running = 1;

        return;
    }

    // FG command Start motor command
    if ((buf[1] == 'F') && (buf[2] == 'G') && (num==4)) {
        ATOMIC_BLOCK(ATOMIC_FORCEON) {
            millis_last_move = millis(); // reset idle timer
        }
        is_running = 1;

        return;
    }

    // FQ command Stop motor command
    if ((buf[1] == 'F') && (buf[2] == 'Q') && (num==4)) {
        is_running = 0;
        
        return;
    }

    // RN command Reboot normal
    if ((buf[1] == 'R') && (buf[2] == 'N') && (num==4)) {
        reboot_yafocuser(0);
        
        return;
    }

    // RB command Reboot bootloader
    if ((buf[1] == 'R') && (buf[2] == 'B') && (num==4)) {
        reboot_yafocuser(1);
        
        return;
    }

    return;
}
