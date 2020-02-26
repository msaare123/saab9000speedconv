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
#pragma config PWRTE = ON
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
#define T1_CS_CLK_4 0b00
#define T1_CS_CLK_1 0b01
#define T1_PS_4 0b10
#define T1_PS_2 0b01
#define T1_PS_1 0b00

#define TMR1_OVF_STOP_VALUE 255
#define DUTY_CYCLE_50_PERCENT(x) (((x) + 1) / 2)
// Input time limit limited by conversion multiplier
#define INPUT_CYCLE_TIME_LIMIT ((UINT16_MAX / ABS_TO_SPEEDO_DIVIDER) * 1000)
#define OUTPUT_UPDATE_INTERVAL_US 50000
#define BUFFER_LENGTH 5
#define DIFFERENCE_16BIT(x, y) (abs((int16_t)((x) - (y))))

typedef enum
{
    RET_OK = 0
} Ret;

typedef uint8_t bool8;

// Timer 1 has overflown
volatile bool8 gbStopped = FALSE;
// Current output time
volatile uint16_t gOutputPeriodTime_us = 0;
volatile uint16_t gOutputDutyCycleTime_us = 0;

uint16_t gInputCycleTimeBuffer_us[BUFFER_LENGTH] = {UINT16_MAX};

/* Init function */
/* Called before any other function*/
Ret Init();

/* ABS to speedo cycle time conversion */
/* Returns input value times 2,348 */
uint16_t ABSToSpeedo_us(uint16_t inputCycleTime_us);

/* Sets new output frequency */
Ret SetOutputFrequency(uint16_t cycleTime_us);

/* Calculates average value of buffer */
uint16_t CalculateBufferAverage();

/* Adds value to buffer */
inline void AddValueToBuffer(uint16_t value);
/*****************************************************************************/
void __interrupt() ISR(void)
{
    if (PIR1bits.TMR1GIF)
    {
        // TMR1 gate interrupt. Acquire ready.
        // Clear interrupt flag
        PIR1bits.TMR1GIF = BIT_OFF;
        AddValueToBuffer(TMR1);
        TMR1 = 0;
        // Ready for new cycle time acquisition
        T1GCONbits.T1GGO = BIT_ON;
        gbStopped = FALSE;
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
            if (PWM1TMR >= PWM1PH)
            {
                PWM1TMR = 0;
            }
            // Enable pwm
            PWM1EN = BIT_ON;
            // Disable this interrupt
            PWM1INTEbits.PRIE = BIT_OFF;
        }
    }
    else if (PIR1bits.TMR1IF)
    {
        // Timer 1 overflow interrupt
        // Clear interrupt flag
        PIR1bits.TMR1IF = BIT_OFF;
        gbStopped = TRUE;
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
    PORTA = 0;
    TRISA = 0;
    ANSELA = 0;
    // RA1 PWM 1 output, RA4 for timer gate
    TRISA = (TRISA_INPUT << 4 | TRISA_OUTPUT << 1);

    /************************************************************************/
    /************************** Interrupt settings **************************/
    /************************************************************************/
    // Enable interrupts, Peripheral interrupt enable
    INTCONbits.GIE = BIT_ON;
    PIE1bits.TMR1GIE = BIT_ON; // Timer gate interrupt enable
    PIE1bits.TMR1IE = BIT_ON;  // Timer overflow interrupt enable
    INTCONbits.PEIE = BIT_ON;

    /************************************************************************/
    /*************************** Timer 1 settings ***************************/
    /************************************************************************/
    // Counts microseconds
    T1CONbits.TMR1CS = T1_CS_CLK_4; // Using Clk/4, Clk = 16 Mhz
    T1CONbits.T1CKPS = T1_PS_4;     // Prescaler 1/4
    // Configure to single pulse gate mode
    // Timer 1 source from RA4
    T1GCONbits.T1GSS = 0b00;
    // Timer 1 on
    T1CONbits.TMR1ON = BIT_ON;
    // Gate toggle mode
    T1GCONbits.T1GTM = BIT_ON;
    // Falling edge
    T1GCONbits.T1GPOL = BIT_OFF;
    // Gate enable
    T1GCONbits.TMR1GE = BIT_ON;
    // Gate single mode
    T1GCONbits.T1GSPM = BIT_ON;
    // Reset timer value
    TMR1 = 0;
    // Ready to acquire
    T1GCONbits.T1GGO = BIT_ON;

    /************************************************************************/
    /**************************** PWM 1 settings ****************************/
    /************************************************************************/
    PWM1CLKCONbits.CS = PWM_SC_HFOSC; // 16 MHz
    PWM1CLKCONbits.PS = PWM_PRESCALER_16;
    PWM1CONbits.MODE = PWM_MODE_TOGGLE_ON_MATCH;
    PIE3bits.PWM1IE = BIT_ON;
    PWM1INTEbits.PRIE = BIT_ON;
    PWM1CONbits.OE = BIT_ON; // Enable output to pin
    PWM1CONbits.EN = BIT_ON; // Enable PWM1
    PWM1PR = 10;
    PWM1PH = 5;

    return RET_OK;
}

Ret SetOutputFrequency(uint16_t cycleTime_us)
{
    if (cycleTime_us > 1)
    {
        gOutputPeriodTime_us = cycleTime_us;
        gOutputDutyCycleTime_us = DUTY_CYCLE_50_PERCENT(cycleTime_us);
        // Enable interrupt where new time is set
        PWM1INTEbits.PRIE = BIT_ON;
    }

    return RET_OK;
}

inline uint16_t ABSToSpeedo_us(uint16_t inputCycleTime_us)
{
    return (uint16_t)(((uint32_t)inputCycleTime_us * ABS_TO_SPEEDO_DIVIDER) /
                      1000);
}

inline uint16_t CalculateBufferAverage()
{
    // Disable TMR1 gate interrupt during calculation to ensure that buffer
    // does not change
    PIE1bits.TMR1GIE = BIT_OFF;
    uint16_t ret = 0;
    uint32_t cumulativeValueWithWeight = 0;
    const static uint8_t weights = 15;
    for (uint8_t i = 0; i < BUFFER_LENGTH; i++)
    {
        cumulativeValueWithWeight +=
            (gInputCycleTimeBuffer_us[i] * (BUFFER_LENGTH - i));
    }
    
    ret = (uint16_t)((cumulativeValueWithWeight + (weights / 2)) / weights);
    PIE1bits.TMR1GIE = BIT_ON;
    return ret;
}

inline void AddValueToBuffer(uint16_t value)
{
    // Move buffer forwards
    for (uint8_t i = 1; i < BUFFER_LENGTH; i++)
    {
        gInputCycleTimeBuffer_us[BUFFER_LENGTH - i] =
            gInputCycleTimeBuffer_us[BUFFER_LENGTH - i - 1];
    }

    gInputCycleTimeBuffer_us[0] = value;
}

int main(int argc, char **argv)
{
    // Initialize registers
    (void)Init();

    while (TRUE)
    {
        // Calculate average time from buffer timestamps
        uint16_t averageCycletime = CalculateBufferAverage();

        // Calculate new output cycle time
        if (averageCycletime < INPUT_CYCLE_TIME_LIMIT && !gbStopped)
        {
            uint16_t outputCycleTime_us = ABSToSpeedo_us(averageCycletime);
            static uint16_t lastSetOutputCycleTime_us = 0;
            // Set new output if it differs from last one by 2 %
            uint16_t cycletimeThreshold_us = lastSetOutputCycleTime_us / 200;
            if (DIFFERENCE_16BIT(outputCycleTime_us,
                                 lastSetOutputCycleTime_us) >
                cycletimeThreshold_us)
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
    return (EXIT_SUCCESS);
}
