/*
 * File:   main.c
 * Author: mattis
 *
 * SAAB 9000 ABS Wheel signal to speedometer converter
 *
 * This pulse conversion enables use of ABS wheel signal as a main speed signal
 * instead of signal coming from transmission.
 *
 *                   PIC12F1571
 *                    --------
 *               VDD |o       | VSS
 *    Button 1 - RA5 |        | RA0 - Button 2
 * Pulse input - RA4 |        | RA1 - Pulse output
 *        MCLR - RA3 |        | RA2 - Status LED
 *                    --------
 *
 * Saab 9000 speedometer needs 9828 pulses per kilometer
 * ABS sensor is producing 23080 pulses per kilometer
 * So input signal frequency needs to be divided by 2,348 for output
 *
 * Created on January 30, 2020, 10:46 PM
 */

#include <stdbool.h>
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
#pragma config BORV = HI
// Low Power Brown-out Reset enable bit (LPBOR is disabled)
#pragma config LPBOREN = ON
// Low-Voltage Programming Enable (Low-voltage programming enabled)
#pragma config LVP = ON

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

#include <xc.h>

#define ABS_TO_SPEEDO_DIVIDER 601 // 2,348 * 256
#define OFFSET_STEP
#define PWM_MODE_TOGGLE_ON_MATCH 0b10
#define PWM_SC_HFOSC 0b01
#define PWM_PRESCALER_16 0b100
#define PWM_PRESCALER_8 0b011
#define PWM_PRESCALER_4 0b010
#define PWM_PRESCALER_2 0b001
#define START_OUTPUT (PWM1CONbits.EN = 1)
#define STOP_OUTPUT (PWM1CONbits.EN = 0)
#define STOP_TIMER_MAX_OVERFLOW_COUNT 2

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
#define T1_DIVIDER 2

#define TMR1_OVF_STOP_VALUE 255
#define DUTY_CYCLE_50_PERCENT(x) (((x) + 1) / 2)
// Input time limit limited by conversion multiplier
#define INPUT_CYCLE_TIME_LIMIT                                                 \
    (((UINT16_MAX / ABS_TO_SPEEDO_DIVIDER) * 256) / T1_DIVIDER)
#define OUTPUT_UPDATE_INTERVAL_US 50000
#define BUFFER_LENGTH 5
#define DIFFERENCE_16BIT(x, y) (abs((int16_t)((x) - (y))))

#define BUTTON_UP_PIN !PORTAbits.RA0
#define BUTTON_DOWN_PIN !PORTAbits.RA5
#define STATUS_LED_PIN PORTAbits.RA2

typedef enum
{
    MODE_NORMAL,
    MODE_OFFSET_INPUT
} InputMode;

// Increases about 16 milliseconds if no edge has been measured
volatile uint8_t gTimer0OverFlowCount = 0;
// Timer1 has overflown when pulse length is too long
volatile bool gbTooLongPulse = false;
// Current output time
volatile uint16_t gOutputPeriodTime_us = 0;
volatile uint16_t gOutputDutyCycleTime_us = 0;
volatile uint16_t gLastCapturedValue_us = 0;
int16_t gOffset_us = 0;

/* Init function */
/* Called before any other function*/
void Init();

/* ABS to speedo cycle time conversion */
/* Returns input value times 2,348 */
uint16_t ABSToSpeedo_us(uint16_t inputCycleTime_us);

/* Sets new output frequency */
void SetOutputFrequency(uint16_t cycleTime_us);

/* Checks if up/down button is pressed and then released */
void CheckButtonStates(bool *pbUpReleased, bool *pbDownReleased);

/* Checks if offset mode is requested by pressing both buttons for 3 seconds */
bool OffsetModeIsRequested();

/* Update status led state */
void UpdateStatusLed(InputMode currentMode);

/* Check if no pulses have received for some while */
bool IsStopped();

/* Handle user offset setting, buttons and led */
void HandleUserInterface();

/* Calculate new output frequency based on last received input */
void CalculateAndSetNewOutput();

/*****************************************************************************/
void __interrupt() ISR(void)
{
    if (PWM1INTFbits.PRIF)
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
            if (PWM1TMR >= PWM1PR)
            {
                PWM1TMR = 0;
            }
            // Enable pwm
            PWM1EN = BIT_ON;
            // Disable this interrupt
            PWM1INTEbits.PRIE = BIT_OFF;
        }
    }

    if (PIR1bits.TMR1GIF)
    {
        // TMR1 gate interrupt. Acquire ready.
        // Clear interrupt flag
        PIR1bits.TMR1GIF = BIT_OFF;
        gLastCapturedValue_us = (TMR1 / T1_DIVIDER);
        TMR1 = 0;
        // Ready for new cycle time acquisition
        T1GCONbits.T1GGO = BIT_ON;
        // Reset stoptimer (TMR0)
        gTimer0OverFlowCount = 0;
        // Reset too long pulse flag
        gbTooLongPulse = false;
        TMR0 = 0;
    }

    if (PIR1bits.TMR1IF)
    {
        // Timer 1 overflow interrupt
        // Clear interrupt flag
        PIR1bits.TMR1IF = BIT_OFF;
        gbTooLongPulse = true;
    }

    if (INTCONbits.TMR0IF)
    {
        // Timer 0 overflow interrupt
        // Clear interrupt flag
        INTCONbits.TMR0IF = BIT_OFF;
        if (gTimer0OverFlowCount < STOP_TIMER_MAX_OVERFLOW_COUNT)
        {
            gTimer0OverFlowCount++;
        }
    }
}

void Init()
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
    WPUA = (BIT_ON << 5 | BIT_ON << 0);
    // RA1 PWM 1 output, RA4 for timer gate, RA2 LED, RA5 button, RA0 Button
    TRISA = (TRISA_INPUT << 4 | TRISA_OUTPUT << 1 | TRISA_OUTPUT << 2 |
             TRISA_INPUT << 5 | TRISA_INPUT << 0);

    /************************************************************************/
    /************************** Interrupt settings **************************/
    /************************************************************************/
    // Enable interrupts, Peripheral interrupt enable
    INTCONbits.GIE = BIT_ON;
    PIE1bits.TMR1GIE = BIT_ON; // Timer gate interrupt enable
    PIE1bits.TMR1IE = BIT_ON;  // Timer overflow interrupt enable
    INTCONbits.PEIE = BIT_ON;

    /************************************************************************/
    /*************************** Timer 0 settings ***************************/
    /************************************************************************/
    OPTION_REGbits.TMR0CS = BIT_OFF; // Use Clk/4
    OPTION_REGbits.PS = 0b111;
    OPTION_REGbits.PSA = BIT_OFF; // Use prescaler above
    INTCONbits.TMR0IE = BIT_ON;   // OF-interrupt enable

    /************************************************************************/
    /*************************** Timer 1 settings ***************************/
    /************************************************************************/
    // Counts microseconds
    T1CONbits.TMR1CS = T1_CS_CLK_4; // Using Clk/4, Clk = 16 Mhz
    T1CONbits.T1CKPS = T1_PS_2;     // Prescaler 1/2
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
    /*************************** Timer 2 settings ***************************/
    /************************************************************************/
    T2CONbits.T2CKPS = 0b10; // 1/64 prescaler 11=1/256 10=1/128 01=1/64
    TMR2 = 0;
    T2CONbits.TMR2ON = BIT_ON; // Timer2 on

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
}

void SetOutputFrequency(uint16_t cycleTime_us)
{
    if (cycleTime_us > 1)
    {
        gOutputPeriodTime_us = cycleTime_us;
        gOutputDutyCycleTime_us = DUTY_CYCLE_50_PERCENT(cycleTime_us);
        // Enable interrupt where new time is set
        PWM1INTEbits.PRIE = BIT_ON;
    }
}

inline uint16_t ABSToSpeedo_us(uint16_t inputCycleTime_us)
{
    return (uint16_t)(((uint32_t)inputCycleTime_us * (ABS_TO_SPEEDO_DIVIDER + gOffset_us)) /
                      256);
}

bool IsStopped()
{
    bool ret = false;
    if (gTimer0OverFlowCount >= STOP_TIMER_MAX_OVERFLOW_COUNT)
    {
        // T0 has not been cleared. No pulses received for a while
        ret = true;
    }
    else if (gbTooLongPulse)
    {
        // Input pulse length counter overflown. Too long pulse.
        ret = true;
    }
    else
    {
        // Still running
        if (gLastCapturedValue_us >= INPUT_CYCLE_TIME_LIMIT)
        {
            // Too high captured frequency. Stop output.
            ret = true;
        }
    }

    return ret;
}

void CalculateAndSetNewOutput()
{
    // Calculate new output cycle time
    static uint16_t lastSetValue_us = 0;
    if (lastSetValue_us != gLastCapturedValue_us)
    {
        INTCONbits.GIE = BIT_OFF;
        uint16_t outputCycleTime_us = ABSToSpeedo_us(gLastCapturedValue_us);
        SetOutputFrequency(outputCycleTime_us);
        lastSetValue_us = gLastCapturedValue_us;
        INTCONbits.GIE = BIT_ON;
    }
}

void HandleUserInterface()
{
    if (TMR2 >= 61)
    {
        // Not time critical, so no interrupt used
        TMR2 = 0;
        static uint8_t millisecondCounter = 0;
        millisecondCounter++;

        static InputMode inputMode = MODE_NORMAL;
        static bool bIdleForLastSecond = true;
        bool bButtonUpActive = false;
        bool bButtonDownActive = false;

        if (millisecondCounter % 10)
        {
            // 10 ms branch
            if (inputMode == MODE_OFFSET_INPUT)
            {
                CheckButtonStates(&bButtonUpActive, &bButtonDownActive);
                if (bButtonUpActive)
                {
                    gOffset_us++;
                    bIdleForLastSecond = false;
                }
                else if (bButtonDownActive)
                {
                    gOffset_us--;
                    bIdleForLastSecond = false;
                }
                else
                {
                    bIdleForLastSecond = true;
                }
            }
            else
            {
                // Do nothing when on normal operation
            }
        }

        if (millisecondCounter >= 100)
        {
            // 100 ms branch
            millisecondCounter = 0;

            static bool bOffsetModeRequested = false;

            if (!bOffsetModeRequested)
            {
                bOffsetModeRequested = OffsetModeIsRequested();
            }

            if ((inputMode == MODE_NORMAL) && bOffsetModeRequested)
            {
                // Turn led of to signal user that mode has changed but
                // don't actually change mode before both buttons have been
                // released
                STATUS_LED_PIN = BIT_OFF;
                if (BUTTON_UP_PIN && BUTTON_DOWN_PIN)
                {
                    inputMode = MODE_OFFSET_INPUT;
                    bOffsetModeRequested = false;
                }
            }

            static uint8_t secondCounter = 0;
            secondCounter++;

            if (secondCounter >= 10)
            {
                secondCounter = 0;
                // 1 s branch
                // Return to normal operation mode after 10 seconds idle
                static uint8_t idleCounter_s = 0;
                if (bIdleForLastSecond && (inputMode == MODE_OFFSET_INPUT))
                {
                    idleCounter_s++;
                    if (idleCounter_s >= 10)
                    {
                        inputMode = MODE_NORMAL;
                        idleCounter_s = 0;
                    }
                }
                else
                {
                    idleCounter_s = 0;
                }

                UpdateStatusLed(inputMode);
            }
        }
    }
}

void CheckButtonStates(bool *pbUpReleased, bool *pbDownReleased)
{
    // Check button state
    static bool bLastButtonUpState = false;
    static bool bLastButtonDownState = false;
    bool bButtonUpState = BUTTON_UP_PIN;
    bool bButtonDownState = BUTTON_DOWN_PIN;

    if (!bButtonUpState && bLastButtonUpState)
    {
        *pbUpReleased = true;
    }
    else
    {
        *pbUpReleased = false;
    }

    if (!bButtonDownState && bLastButtonDownState)
    {
        *pbDownReleased = true;
    }
    else
    {
        *pbDownReleased = false;
    }

    bLastButtonUpState = bButtonUpState;
    bLastButtonDownState = bButtonDownState;
}

bool OffsetModeIsRequested()
{
    bool ret = false;
    static uint8_t modeChangeTimer = 0;
    if (BUTTON_UP_PIN && BUTTON_DOWN_PIN)
    {
        modeChangeTimer++;
        // Called on every 100 ms
        if (modeChangeTimer > 30)
        {
            // Both buttons have been pressed for 3 seconds
            ret = true;
        }
    }
    else
    {
        modeChangeTimer = 0;
    }

    return ret;
}

void UpdateStatusLed(InputMode currentMode)
{
    // Called on every second
    if (currentMode == MODE_OFFSET_INPUT)
    {
        static bool ledState = BIT_OFF;
        ledState = !ledState;
        STATUS_LED_PIN = ledState;
    }
    else
    {
        STATUS_LED_PIN = BIT_ON;
    }
}

int main(int argc, char **argv)
{
    // Initialize registers
    Init();

    while (true)
    {
        if (!IsStopped())
        {
            CalculateAndSetNewOutput();
            START_OUTPUT;
        }
        else
        {
            STOP_OUTPUT;
        }

        HandleUserInterface();
    }

    return (EXIT_SUCCESS);
}
