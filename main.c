/*
 * File:   main.c
 * Author: mattis
 *
 * Created on January 30, 2020, 10:46 PM
 */

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

// PIC12F1571 Configuration Bit Settings

// 'C' source line config statements

// CONFIG1
//  (INTOSC oscillator; I/O function on CLKIN pin)
#pragma config FOSC = INTOSC
// Watchdog Timer Enable (WDT disabled)
#pragma config WDTE = OFF
// Power-up Timer Enable (PWRT disabled)
#pragma config PWRTE = OFF
// MCLR Pin Function Select (MCLR/VPP pin function is MCLR)
#pragma config MCLRE = ON
// Flash Program Memory Code Protection (Program memory code protection is
// disabled)
#pragma config CP = OFF
// Brown-out Reset Enable (Brown-out Reset enabled)
#pragma config BOREN = ON
// Clock Out Enable (CLKOUT function is disabled. I/O or oscillator function on
// the CLKOUT pin)
#pragma config CLKOUTEN = OFF

// CONFIG2
// Flash Memory Self-Write Protection (Write protection off)
#pragma config WRT = OFF
// PLL Enable (4x PLL enabled)
#pragma config PLLEN = ON
// Stack Overflow/Underflow Reset Enable (Stack Overflow or Underflow will cause
// a Reset)
#pragma config STVREN = ON
// Brown-out Reset Voltage Selection (Brown-out Reset Voltage (Vbor), low trip
// point selected.)
#pragma config BORV = LO
// Low Power Brown-out Reset enable bit (LPBOR is disabled)
#pragma config LPBOREN = OFF
// Low-Voltage Programming Enable (Low-voltage programming enabled)
#pragma config LVP = OFF

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

#include <xc.h>

#define TRUE 1
#define FALSE 0

#define ABS_TO_SPEEDO_DIVIDER 2348 // 2,348
#define PWM_MODE_TOGGLE_ON_MATCH 0b10
#define PWM_SC_HFOSC 0b01
#define PWM_PRESCALER_16 0b100
#define PWM_PRESCALER_8 0b011
#define PWM_PRESCALER_4 0b010
#define PWM_PRESCALER_2 0b001
#define START_OUTPUT (PWM1CONbits.EN = 1)
#define STOP_OUTPUT (PWM1CONbits.EN = 0)

#define TRISA_INPUT 1
#define TRISA_OUTPUT 0
#define CLK_8_MHZ 0b1110
#define CLK_SRC_INTOSC 0b10
#define BIT_ON 1
#define BIT_OFF 0
#define TRIGGER_FALLING_EDGE 0
#define T1_CS_CLK_4 0
#define T1_PS_4 0b10

#define TMR1_OVF_STOP_VALUE 255
#define DUTY_CYCLE_50_PERCENT(x) (((x)-1) / 2)
// Input time limit limited by conversion multiplier
#define INPUT_CYCLE_TIME_LIMIT ((UINT16_MAX / ABS_TO_SPEEDO_DIVIDER) * 1000)

typedef enum
{
    RET_OK = 0
} Ret;

// Timer 1 overflow count since last read value
volatile uint8_t gTmr1OverflowCount = 0;
// Latest timestamp of signal edge in timer 1 microseconds
volatile uint16_t gLatestEdgeTimestamp_us = 0;
// Current output time
volatile uint16_t gOutputPeriodTime_us    = 0;
volatile uint16_t gOutputDutyCycleTime_us = 0;

/* Init function */
/* Called before any other function*/
Ret Init();

/* ABS to speedo cycle time conversion */
/* Returns input value times 2,348 */
uint16_t ABSToSpeedo_us(uint16_t inputCycleTime_us);

/* Sets new output frequency */
Ret SetOutputFrequency(uint16_t cycleTime_us);

/*****************************************************************************/
void __interrupt() ISR(void)
{
    if (INTCONbits.INTF)
    {
        PORTAbits.RA4 = !LATAbits.LATA4;
        // Interrupt pin interrupt
        // Clear interrupt flag
        INTCONbits.INTF = BIT_OFF;
        // Take edge timestamp from timer1
        gLatestEdgeTimestamp_us = TMR1;
    }
    else if (PIR1bits.TMR1IF)
    {
        // Timer 1 overflow interrupt
        // Clear interrupt flag
        PIR1bits.TMR1IF = BIT_OFF;
        // Increase overflow count
        if (gTmr1OverflowCount < TMR1_OVF_STOP_VALUE)
        {
            gTmr1OverflowCount++;
        }
    }
    else if (PWM1INTFbits.PRIF)
    {
        // PWM1 period interrupt
        // Clear interrupt flag
        PWM1INTFbits.PRIF = BIT_OFF;

        // When frequency has changed and output is low
        // This ensures clean frequency change
        if (!PWM1OUT)
        {
            // Set new cycle time to PWM1 module
            // No need to clear timer register because we have just
            // gotten a full period
            // Disable pwm
            PWM1EN = BIT_OFF;
            // Period register
            PWM1PR = gOutputPeriodTime_us;
            // Phase register
            PWM1PH = gOutputDutyCycleTime_us;
            // Enable pwm
            PWM1EN = BIT_ON;
            // Disable this interrupt
            PWM1INTEbits.PRIE = BIT_OFF;
        }
    }
}

Ret Init()
{
    /************************************************************************/
    /************************** Clock settings ******************************/
    /************************************************************************/
    // 1110, Set clock to 8 MHz
    OSCCONbits.IRCF = CLK_8_MHZ;
    // Select Internal oscillator
    OSCCONbits.SCS = CLK_SRC_INTOSC;

    /************************************************************************/
    /************************ Port register settings ************************/
    /************************************************************************/
    // Clear registers
    PORTA  = 0;
    TRISA  = 0;
    ANSELA = 0;
    // RA1 PWM 1 output, RA2 for interrupt
    TRISA = (TRISA_INPUT << 2 | TRISA_OUTPUT << 1);

    /************************************************************************/
    /************************** Interrupt settings **************************/
    /************************************************************************/
    // Falling edge triggering
    OPTION_REGbits.INTEDG = TRIGGER_FALLING_EDGE;
    // Timer1 overflow interrupt
    PIE1bits.TMR1IE = BIT_ON;
    // Enable interrupts, INT pin interrupt enable, Peripheral interrupt enable
    INTCONbits.GIE  = BIT_ON;
    INTCONbits.INTE = BIT_ON;
    INTCONbits.PEIE = BIT_ON;

    /************************************************************************/
    /*************************** Timer 1 settings ***************************/
    /************************************************************************/
    // Counts microseconds
    T1CONbits.TMR1ON  = BIT_ON;      // Timer 1 on
    T1CONbits.TMR1CS  = T1_CS_CLK_4; // Using Clk/4
    T1CONbits.T1CKPS  = T1_PS_4;     // Prescaler 1/4
    T1GCONbits.TMR1GE = BIT_OFF;     // Not using as counter

    /************************************************************************/
    /**************************** PWM 1 settings ****************************/
    /************************************************************************/
    PWM1CLKCONbits.CS = PWM_SC_HFOSC; // 16 MHz
    PWM1CLKCONbits.PS = PWM_PRESCALER_16;
    PWM1CONbits.MODE  = PWM_MODE_TOGGLE_ON_MATCH;
    PIE3bits.PWM1IE   = BIT_ON;
    PWM1INTEbits.PRIE = BIT_ON;
    PWM1CONbits.OE    = BIT_ON; // Enable output to pin
    PWM1CONbits.EN    = BIT_ON; // Enable PWM1
    PWM1PR            = 10;
    PWM1PH            = 5;

    return RET_OK;
}

Ret SetOutputFrequency(uint16_t cycleTime_us)
{
    if (cycleTime_us > 1)
    {
        gOutputPeriodTime_us    = cycleTime_us;
        gOutputDutyCycleTime_us = DUTY_CYCLE_50_PERCENT(cycleTime_us);
        // Enable interrupt where new time is set
        PWM1INTEbits.PRIE = BIT_ON;
    }

    return RET_OK;
}

uint16_t ABSToSpeedo_us(uint16_t inputCycleTime_us)
{
    return (uint16_t)(((uint32_t)inputCycleTime_us * ABS_TO_SPEEDO_DIVIDER) /
                      1000);
}

int main(int argc, char **argv)
{
    // Initialize registers
    (void)Init();

    while (TRUE)
    {
        // Timestamp of last read event
        static uint16_t lastEdgeTimestamp_us = 0;
        if (gLatestEdgeTimestamp_us != lastEdgeTimestamp_us)
        {
            uint16_t newEdgeTimeStamp_us = gLatestEdgeTimestamp_us;
            // New event stored
            uint16_t inputCycleTime_us =
                newEdgeTimeStamp_us - lastEdgeTimestamp_us;
            lastEdgeTimestamp_us = newEdgeTimeStamp_us;
            gTmr1OverflowCount   = 0;
            // Calculate new output cycle time
            if (inputCycleTime_us < INPUT_CYCLE_TIME_LIMIT)
            {
                uint16_t outputCycleTime_us = ABSToSpeedo_us(inputCycleTime_us);
                static uint16_t lastSetOutputCycleTime_us = 0;
                // Set new output
                if (outputCycleTime_us != lastSetOutputCycleTime_us)
                {
                    (void)SetOutputFrequency(outputCycleTime_us);
                    lastSetOutputCycleTime_us = outputCycleTime_us;
                }
                START_OUTPUT;
            }
            else
            {
                STOP_OUTPUT;
            }
        }
        else if (gTmr1OverflowCount > 1)
        {
            STOP_OUTPUT;
        }
    }
    return (EXIT_SUCCESS);
}
