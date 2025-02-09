/*  Fall 24 - EEL-4746 Class Design Project - Light Sensor
 * Group ID# 4
 * Primary Author:
 *  Keila Souriac
 * Secondary Authors:
 * Grace Parnas
 * Ashlynn Quintana
****************************************************************************************
 * Function Name: Main
 * Description: Reads in the Input from Pushbuttons or Light Sensor
 * Inputs: PBS1, PBS2, Light Sensor Blocked, Flashlight on Light Sensor
 * Returns: LED1 and LED2 on/off/toggle
 * Primary Author: Keila Souriac
 * Secondary Author: Ashlynn Quintana and Grace Parnas
 **************************************************************************************
 * Function Name: ISR
 * Description: This is the Interrupt Service Routine which serves as the timer/delay
 * for the motorDriver function, which controls the motor moving direction.
 * Input: interrupt, delay value (high or low speed)
 * Returns: delays the motor's movement
 * Primary Author: Keila Souriac
 * Secondary Author: Ashlynn Quintana
 * ************************************************************************************
 * Function Name: FR5994_I2C_init
 * Description: connects the pins for the launchpad to the boosterpack, and identifies
 * if the pins are being used for the temperature sensor or light sensor. For this lab
 * we are using the light sensor.
 * Input: None
 * Returns: Activates the Light Sensor
 * Primary Author: Keila Souriac
 * Secondary Author: Ashlynn Quintana and Grace Parnas
 * ************************************************************************************
 * Function Name: config_mkII
 * Description: Configures mkII Pins, PBS1 and PBS2, and LEDs on launchpad
 * Input: None
 * Returns: None
 * Primary Author: Keila Souriac
 * Secondary Author: Ashlynn Quintana and Grace Parnas
 * ************************************************************************************
 * Function Name: myPatternMode
 * Description: Outlines how the motor should be moving for each step of Pattern 3
 * Input: patternSeq
 * Returns: None
 * Primary Author: Keila Souriac
 * Secondary Author: Ashlynn Quintana and Grace Parnas
 * ************************************************************************************
 * Function Name: configTimerA
 * Description: Configures Parameters for TimerA, using SMCLK
 * Input: delayValue, clockDividerValue
 * Returns: None
 * Primary Author: Keila Souriac
 * Secondary Author: Ashlynn Quintana and Grace Parnas
 * ************************************************************************************
 * Function Name: myTimerADelay
 * Description: Hardware Timer Delay Function using polling with Timer A
 * Input: delayValue, clockDividerValue
 * Returns: None
 * Primary Author: Keila Souriac
 * Secondary Author: Ashlynn Quintana and Grace Parnas
 * ************************************************************************************
 * Function Name: config_PWM
 * Description: Configures PWM Channel TB0.6
 * Input: timerPeriod, timerDivider
 * Returns: None
 * Primary Author: Keila Souriac
 * Secondary Author: Ashlynn Quintana and Grace Parnas
 * ************************************************************************************
 * Function Name: ADC_init
 * Description: Configures ADC to use Light Sensor Inputs
 * Input: None
 * Returns: None
 * Primary Author: Keila Souriac
 * Secondary Author: Ashlynn Quintana and Grace Parnas
 * ************************************************************************************
 * Function Name: myStepperDriver
 * Description: Steps the cases for the motor to spin, by identifying which poles(pins)
 * are active(high) and which are not
 * Input: stepSeq
 * Returns: None
 * Primary Author: Keila Souriac
 * Secondary Author: Ashlynn Quintana and Grace Parnas
 * ************************************************************************************
 * Function Name: myMotorDriver
 * Description: Defines what the motor should be doing, depending on the mode it is in
 * either standby (hold), clockwise rotation, or counterclockwise rotation
 * Input: motorSeq
 * Returns: None
 * Primary Author: Keila Souriac
 * Secondary Author: Ashlynn Quintana and Grace Parnas
 * ***********************************************************************************
 */

// Includes
#include <stdint.h>
#include <stdio.h>
#include  "LcdDrivermsp430/Crystalfontz128x128_ST7735.h"
#include  "LcdDrivermsp430/HAL_MSP_EXP430FR5994_Crystalfontz128x128_ST7735.h"
#include "HAL_FR5994_OPT3001.h"
#include "HAL_FR5994_I2CLIB.h"
#include "HAL_SDCard.h"
#include "grlib.h"
#include "driverlib.h"
#include "SDCard_API.h"
#include "HAL_UART_4746.h" //UART


// Initialize File status with default values
char buffer[48];
char asciiData[64];

//global variables
uint8_t pat;
uint16_t motorSeq, patternSeq, stepSeq, ccwcount, count, noon, move;
uint32_t tempspeed;
float stepcount;
int16_t x, g, period;
Timer_B_outputPWMParam MyTimerB;
Timer_A_initUpModeParam MyTimerA;

//  Needed to configure I2C bus
extern EUSCI_B_I2C_initMasterParam i2cConfig;


// Defines
#define OPTADDRESS 0x44
#define DELAYTIME 500000
#define HIGH 1400
#define LOW 2664

//Function Prototypes
void FR5994_I2C_init();
void config_mkII();
void myMotorDriver();
void myPatternMode();
void configTimerA(uint16_t,uint16_t);
void myTimerADelay(uint16_t,uint16_t);
void config_PWM(uint16_t,uint16_t);
void ADC_init(void);
void myStepperDriver();
void pwnOff(void);


//Main Function
void main (void)
{
     char buffer[100];
     volatile uint32_t value, luxCounter;
     volatile uint32_t ii,result;
     volatile uint8_t PBS1, PBS2, y;

    //Stop WDT
    WDT_A_hold(WDT_A_BASE);

    // Unlock PMM
    PMM_unlockLPM5();

    //UART
    UART_initGPIO();
    UART_init();

    //Disable Global Interrupts
    __disable_interrupt();

    //Configure mkII & launchpad
    config_mkII();

     //Initialize I2C interface

       FR5994_I2C_init(); //enable I2C bus
       OPT3001_init(OPTADDRESS); //initialize OPT3001
       value = OPT3001_getLux(OPTADDRESS);

       motorSeq=0;
       patternSeq = 0;
       ccwcount = 0;
       count = 0;
       stepSeq = 0;
       stepcount = 0;

       while(1)
       {
       // read light level
       value = OPT3001_getLux(OPTADDRESS);

       PBS1 = GPIO_getInputPinValue(GPIO_PORT_P4, GPIO_PIN3);
       PBS2 = GPIO_getInputPinValue(GPIO_PORT_P4, GPIO_PIN2);

        if(PBS1 == GPIO_INPUT_PIN_HIGH && PBS2 == GPIO_INPUT_PIN_HIGH)
        {
                //Table 1 conditions
            if(value > 0x11E && value < 0x62D) //background light
            {
                stepcount = stepcount;
                //led1 off and led 2 toggle
                GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN0); //red launchpad
                GPIO_toggleOutputOnPin(GPIO_PORT_P1, GPIO_PIN1); //green launchpad
                myTimerADelay(65000,TIMER_A_CLOCKSOURCE_DIVIDER_64); //THIS WORKS!!!
                patternSeq = 0;

                motorSeq = 0;
                if(ccwcount == 1) //if standby
                {
                   count++;
                }
                ccwcount = 0;

            }
            if(value > 0x62D) //cellphone light clockwise
            {
                patternSeq = 0;
                tempspeed = HIGH;
                //led 1 off and led 2 on
                GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN0); //Red launchpad
                GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN1); //green launchpad
                ccwcount = 0;
                count = 0;
                motorSeq = 1;
                stepcount += 2.1; //2.2
                myMotorDriver(motorSeq);

            }

            if(value < 0x11E) //no light counterclockwise
            {
                patternSeq = 0;

                tempspeed = LOW;
                //led 1 on and led 2 off
                GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN0); //Red launchpad
                GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN1); //green launchpad
                motorSeq = 2;
                stepcount -= 1;

                if(ccwcount == 0)
                {
                    ccwcount++;
                }
                myMotorDriver(motorSeq);


            }

            if(count == 3) //display pattern mode if 3 con. ccw
            {
                //led 1 & led 2 on
                GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN0); //Red launchpad
                GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN1); //green launchpad

                stepcount = stepcount;

                while(stepcount < 0)
                {
                   if(stepSeq == 8)
                   {
                       stepSeq = 0;
                   }
                   else
                   {
                       stepSeq++;

                   }
                      stepcount++;
                      myStepperDriver(stepSeq);
                      myTimerADelay(2664,TIMER_A_CLOCKSOURCE_DIVIDER_20); // this for high speed

                  }
                while (stepcount > 0)
                {
                if(stepSeq == 0)
                              {
                                  stepSeq = 8;
                              }
                 else
                              {
                                  stepSeq--;
                              }
                    stepcount--;
                   myStepperDriver(stepSeq);
                   myTimerADelay(2664,TIMER_A_CLOCKSOURCE_DIVIDER_20); // this for high speed

               }
                  __delay_cycles(DELAYTIME);

                for(patternSeq = 0; patternSeq < 11; patternSeq++)
                {
                    myPatternMode(patternSeq);
                    __delay_cycles(DELAYTIME);
                }
                 count = 0;
            }
        }
        if(PBS1 == GPIO_INPUT_PIN_HIGH && PBS2 == GPIO_INPUT_PIN_LOW) //clockwise
        {
            //led 1 off and led 2 off
            GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN0); //Red launchpad
            GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN1); //green launchpad
            patternSeq = 0;

            tempspeed = HIGH;
            motorSeq = 1;

            stepcount += 2.95;

            sprintf(buffer,"stepSeq value: %u\r\n",stepSeq);
             UART_transmitString(buffer);
              myMotorDriver(motorSeq);
        }
        if(PBS1 == GPIO_INPUT_PIN_LOW && PBS2 == GPIO_INPUT_PIN_HIGH)
        {
            //led 1 off and led 2 off
            GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN0); //Red launchpad
            GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN1); //green launchpad
            patternSeq = 0;

            tempspeed = LOW;
            motorSeq = 2;
            stepcount -= 1;
            myMotorDriver(motorSeq);
        }
        if(PBS1 == GPIO_INPUT_PIN_LOW && PBS2 == GPIO_INPUT_PIN_LOW) //pattern Seq
        {
            //led 1 off and led 2 off
            GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN0); //Red launchpad
            GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN1); //green launchpad
            while (stepcount > 0)
                 {
                  if(stepSeq == 0)
                                {
                                    stepSeq = 8;
                                }
                   else
                                {
                                    stepSeq--;
                                }
                      stepcount--;
                     myStepperDriver(stepSeq);
                     myTimerADelay(2664,TIMER_A_CLOCKSOURCE_DIVIDER_20); // this for high speed
                 }

            while(stepcount < 0)
            {
                if(stepSeq == 8)
                {
                  stepSeq = 0;
                 }
                 else
                 {
                   stepSeq++;

                  }
                stepcount++;
                myStepperDriver(stepSeq);
                myTimerADelay(2664,TIMER_A_CLOCKSOURCE_DIVIDER_20); // this for high speed

            }
            __delay_cycles(DELAYTIME);

                tempspeed = HIGH;

                sprintf(buffer,"patternSeq value: %u\r\n",patternSeq);
                                              UART_transmitString(buffer);

                           myPatternMode(patternSeq);
                           __delay_cycles(DELAYTIME);

              if(patternSeq >= 10)
               {
                   patternSeq = 0;
               }
               else
               {
                   patternSeq++;
               }
        }
      sprintf(buffer,"tempspeed value : %u\r\n",tempspeed);
                     UART_transmitString(buffer);

       configTimerA(tempspeed,20); // Configure the timer parameters
        Timer_A_initUpMode(TIMER_A0_BASE,&MyTimerA); // Initialize the timer

        Timer_A_enableInterrupt(TIMER_A0_BASE);      // Enable Timer A0 interrupt
        Timer_A_startCounter(TIMER_A0_BASE,TIMER_A_UP_MODE);  // Start Timer
        __enable_interrupt();     // Enable Global Interrupts
       }
}

#pragma vector=TIMER0_A1_VECTOR
 __interrupt void myTimerISR(void)
 {
     myMotorDriver(motorSeq);
     Timer_A_clearTimerInterrupt(TIMER_A0_BASE);   // Reset Interrupt Flag
 }

void FR5994_I2C_init()
{
    //Init I2C GPIO

        GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P7, GPIO_PIN1,GPIO_PRIMARY_MODULE_FUNCTION);  //SCL
        GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P7, GPIO_PIN0,GPIO_PRIMARY_MODULE_FUNCTION);  //SDA

        //  Disable I2C module to make changes
            EUSCI_B_I2C_disable(EUSCI_B2_BASE);

        // Initialize USCI_B1 device
            EUSCI_B_I2C_initMaster(EUSCI_B2_BASE, &i2cConfig);

        // Enable I2C Module to start operations
            EUSCI_B_I2C_enable(EUSCI_B2_BASE);

}

void config_mkII()
{
   //config red. green, blue leds on mkII
    GPIO_setAsOutputPin(GPIO_PORT_P3,GPIO_PIN7);
    GPIO_setAsOutputPin(GPIO_PORT_P3,GPIO_PIN6); //Red
    GPIO_setAsOutputPin(GPIO_PORT_P3,GPIO_PIN5); //green
    GPIO_setAsOutputPin(GPIO_PORT_P3,GPIO_PIN4); //blue


    //config S1 and S2 on MKII
    GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P4, GPIO_PIN3); //S1
    GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P4, GPIO_PIN2); //S2

    //Config leds on launchpad
    GPIO_setAsOutputPin(GPIO_PORT_P1,GPIO_PIN0); //Red LED
    GPIO_setAsOutputPin(GPIO_PORT_P1,GPIO_PIN1); //Green LED

    PMM_unlockLPM5();

    //set LEDs off
    GPIO_setOutputLowOnPin(GPIO_PORT_P3,GPIO_PIN7);
    GPIO_setOutputLowOnPin(GPIO_PORT_P3, GPIO_PIN6); //Red MKII
    GPIO_setOutputLowOnPin(GPIO_PORT_P3, GPIO_PIN5); //green MKII
    GPIO_setOutputLowOnPin(GPIO_PORT_P3, GPIO_PIN4); //blue MKII
    GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN0); //Red launchpad
    GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN1); //green launchpad
}

void myPatternMode(patternSeq)
{
    uint16_t i;
    switch(patternSeq)
    {
        case 0: //go from 12 to 1
            pat = 0;
            for(i=0; i<=36; i++) //36
                       {
                           if(stepSeq == 8)
                           {
                               stepSeq = 0;
                           }
                           else
                           {
                               stepSeq++;
                           }
                           myTimerADelay(2664,TIMER_A_CLOCKSOURCE_DIVIDER_20); // this for high speed
                          myStepperDriver(stepSeq);
                       }
                break;
        case 1: //1 to 7
            pat = 1;
            for(i=0; i<=225; i++)
                       {
                           if(stepSeq == 8)
                           {
                               stepSeq = 0;
                           }
                           else
                           {
                               stepSeq++;
                           }
                           myTimerADelay(2664,TIMER_A_CLOCKSOURCE_DIVIDER_20); // this for high speed
                          myStepperDriver(stepSeq);
                       }
                break;
        case 2: //7 to 10
            pat = 2;
            for(i=0; i<=117; i++) //increassed by 1 "9 steps"
                       {
                           if(stepSeq == 8)
                           {
                               stepSeq = 0;
                           }
                           else
                           {
                               stepSeq++;
                           }
                           myTimerADelay(2664,TIMER_A_CLOCKSOURCE_DIVIDER_20); // this for high speed
                          myStepperDriver(stepSeq);
                       }
                break;
        case 3: //10 to 4
            pat = 3;
            for(i=0; i<=216; i++)
                       {
                           if(stepSeq == 8)
                           {
                               stepSeq = 0;
                           }
                           else
                           {
                               stepSeq++;
                           }
                           myTimerADelay(2664,TIMER_A_CLOCKSOURCE_DIVIDER_20); // this for high speed
                          myStepperDriver(stepSeq);
                       }
                break;
        case 4: //4 to 8
            pat = 4;
            for(i=0; i<=153; i++)
                       {
                           if(stepSeq == 8)
                           {
                               stepSeq = 0;
                           }
                           else
                           {
                               stepSeq++;
                           }
                           myTimerADelay(2664,TIMER_A_CLOCKSOURCE_DIVIDER_20); // this for high speed
                          myStepperDriver(stepSeq);
                       }
                break;
        case 5: //8 to 2
            pat = 5;
            for(i=0; i<=216; i++)
                       {
                           if(stepSeq == 8)
                           {
                               stepSeq = 0;
                           }
                           else
                           {
                               stepSeq++;
                           }
                           myTimerADelay(2664,TIMER_A_CLOCKSOURCE_DIVIDER_20); // this for high speed
                          myStepperDriver(stepSeq);
                       }
                break;
        case 6: // 2 to 10
            pat = 6;
            for(i=0; i<=306; i++) //changed by 2 "18 steps"
                       {
                           if(stepSeq == 8)
                           {
                               stepSeq = 0;
                           }
                           else
                           {
                               stepSeq++;
                           }
                           myTimerADelay(2664,TIMER_A_CLOCKSOURCE_DIVIDER_20); // this for high speed
                          myStepperDriver(stepSeq);
                       }
                break;
        case 7: //10 to 7
            pat = 7;
            for(i=0; i<=333; i++) //changed by 1 "9 steps"
                       {
                           if(stepSeq == 8)
                           {
                               stepSeq = 0;
                           }
                           else
                           {
                               stepSeq++;
                           }
                           myTimerADelay(2664,TIMER_A_CLOCKSOURCE_DIVIDER_20); // this for high speed
                          myStepperDriver(stepSeq);
                       }
                break;
        case 8: //7 to 6
            pat = 8;
            for(i=0; i<=414; i++) //changed by 2 "18 steps"
                       {
                           if(stepSeq == 8)
                           {
                               stepSeq = 0;
                           }
                           else
                           {
                               stepSeq++;
                           }
                           myTimerADelay(2664,TIMER_A_CLOCKSOURCE_DIVIDER_20); // this for high speed
                          myStepperDriver(stepSeq);
                       }
                break;
        case 9: //6 to 12
            pat = 9;
            for(i=0; i<=216; i++)
                       {
                           if(stepSeq == 8)
                           {
                               stepSeq = 0;
                           }
                           else
                           {
                               stepSeq++;
                           }
                           myTimerADelay(2664,TIMER_A_CLOCKSOURCE_DIVIDER_20); // this for high speed
                          myStepperDriver(stepSeq);
                       }
                break;
    }
}

void configTimerA(uint16_t delayValue, uint16_t clockDividerValue)
{
    MyTimerA.clockSource = TIMER_A_CLOCKSOURCE_SMCLK;
    MyTimerA.clockSourceDivider = clockDividerValue;
    MyTimerA.timerPeriod = delayValue;
    MyTimerA.timerClear = TIMER_A_DO_CLEAR;
    MyTimerA.startTimer = false;
}

void myTimerADelay(uint16_t delayValue, uint16_t clockDividerValue)
{

   configTimerA(delayValue,clockDividerValue);  // Configure the timer parameters
   Timer_A_initUpMode(TIMER_A0_BASE,&MyTimerA); // Initialize the timer
   Timer_A_startCounter(TIMER_A0_BASE,TIMER_A_UPDOWN_MODE);  // Start Timer//
   while((TA0CTL&TAIFG) == 0);                   // Wait for TAIFG to become Set
   Timer_A_stop(TIMER_A0_BASE);                  // Stop timer
   Timer_A_clearTimerInterrupt(TIMER_A0_BASE);   // Reset TAIFG to Zero
}

void config_PWM(uint16_t timerPeriod, uint16_t timerDivider){

    MyTimerB.clockSource = TIMER_B_CLOCKSOURCE_SMCLK;
    MyTimerB.clockSourceDivider = timerDivider;
    MyTimerB.timerPeriod = timerPeriod;
    MyTimerB.compareRegister = TIMER_B_CAPTURECOMPARE_REGISTER_6;
    MyTimerB.compareOutputMode = TIMER_B_OUTPUTMODE_RESET_SET;
    MyTimerB.dutyCycle = timerPeriod / 2;
    Timer_B_outputPWM(TIMER_B0_BASE,&MyTimerB);     // Initialize Timer
}

void ADC_init(){

    //Initialize the ADC12B Module
    ADC12_B_initParam initParam = {0};
    initParam.sampleHoldSignalSourceSelect = ADC12_B_SAMPLEHOLDSOURCE_SC;
    initParam.clockSourceSelect = ADC12_B_CLOCKSOURCE_ADC12OSC;
    initParam.clockSourceDivider = ADC12_B_CLOCKDIVIDER_1;
    initParam.clockSourcePredivider = ADC12_B_CLOCKPREDIVIDER__1;
    initParam.internalChannelMap = ADC12_B_NOINTCH;
    ADC12_B_init(ADC12_B_BASE, &initParam);

    //Enable the ADC12B module
    ADC12_B_enable(ADC12_B_BASE);
    ADC12_B_setupSamplingTimer(ADC12_B_BASE,
      ADC12_B_CYCLEHOLD_16_CYCLES,
      ADC12_B_CYCLEHOLD_4_CYCLES,
      ADC12_B_MULTIPLESAMPLESENABLE);


    // Clear Interrupt
    ADC12_B_clearInterrupt(ADC12_B_BASE,0,ADC12_B_IFG1);
}


void myStepperDriver(stepSeq){  //makes motor spin
    switch(stepSeq){

    case 0:
        GPIO_setOutputHighOnPin(GPIO_PORT_P3,GPIO_PIN7); // A RED LP
        // B RED EB
        GPIO_setOutputLowOnPin(GPIO_PORT_P3,GPIO_PIN6);
        // ABAR BLUE EB
        GPIO_setOutputLowOnPin(GPIO_PORT_P3,GPIO_PIN5);
        // BBAR GREEN EB
        GPIO_setOutputLowOnPin(GPIO_PORT_P3,GPIO_PIN4);
        break;

    case 1:
        // A RED LP
        GPIO_setOutputHighOnPin(GPIO_PORT_P3,GPIO_PIN7);
        // B RED EB
        GPIO_setOutputHighOnPin(GPIO_PORT_P3,GPIO_PIN6);
        // ABAR BLUE EB
        GPIO_setOutputLowOnPin(GPIO_PORT_P3,GPIO_PIN5);
        // BBAR GREEN EB
        GPIO_setOutputLowOnPin(GPIO_PORT_P3,GPIO_PIN4);
        break;
    case 2:
        // A RED LP
        GPIO_setOutputLowOnPin(GPIO_PORT_P3,GPIO_PIN7);
        // B RED EB
        GPIO_setOutputHighOnPin(GPIO_PORT_P3,GPIO_PIN6);
        // ABAR BLUE EB
        GPIO_setOutputLowOnPin(GPIO_PORT_P3,GPIO_PIN5);
        // BBAR GREEN EB
        GPIO_setOutputLowOnPin(GPIO_PORT_P3,GPIO_PIN4);
        break;
    case 3:
        // A RED LP
        GPIO_setOutputLowOnPin(GPIO_PORT_P3,GPIO_PIN7);
        // B RED EB
        GPIO_setOutputHighOnPin(GPIO_PORT_P3,GPIO_PIN6);
        // ABAR BLUE EB
        GPIO_setOutputHighOnPin(GPIO_PORT_P3,GPIO_PIN5);
        // BBAR GREEN EB
        GPIO_setOutputLowOnPin(GPIO_PORT_P3,GPIO_PIN4);
        break;
    case 4:
        // A RED LP
        GPIO_setOutputLowOnPin(GPIO_PORT_P3,GPIO_PIN7);
        // B RED EB
        GPIO_setOutputLowOnPin(GPIO_PORT_P3,GPIO_PIN6);
        // ABAR BLUE EB
        GPIO_setOutputHighOnPin(GPIO_PORT_P3,GPIO_PIN5);
        // BBAR GREEN EB
        GPIO_setOutputLowOnPin(GPIO_PORT_P3,GPIO_PIN4);
        break;
    case 5:
        // A RED LP
        GPIO_setOutputLowOnPin(GPIO_PORT_P3,GPIO_PIN7);
        // B RED EB
        GPIO_setOutputLowOnPin(GPIO_PORT_P3,GPIO_PIN6);
        // ABAR BLUE EB
        GPIO_setOutputHighOnPin(GPIO_PORT_P3,GPIO_PIN5);
        // BBAR GREEN EB
        GPIO_setOutputHighOnPin(GPIO_PORT_P3,GPIO_PIN4);
        break;
    case 6:
        // A RED LP
        GPIO_setOutputLowOnPin(GPIO_PORT_P3,GPIO_PIN7);
        // B RED EB
        GPIO_setOutputLowOnPin(GPIO_PORT_P3,GPIO_PIN6);
        // ABAR BLUE EB
        GPIO_setOutputLowOnPin(GPIO_PORT_P3,GPIO_PIN5);
        // BBAR GREEN EB
        GPIO_setOutputHighOnPin(GPIO_PORT_P3,GPIO_PIN4);
        break;
    case 7:
        // A RED LP
        GPIO_setOutputHighOnPin(GPIO_PORT_P3,GPIO_PIN7);
        // B RED EB
        GPIO_setOutputLowOnPin(GPIO_PORT_P3,GPIO_PIN6);
        // ABAR BLUE EB
        GPIO_setOutputLowOnPin(GPIO_PORT_P3,GPIO_PIN5);
        // BBAR GREEN EB
        GPIO_setOutputHighOnPin(GPIO_PORT_P3,GPIO_PIN4);
        break;

    default:
        // A RED LP
        GPIO_setOutputLowOnPin(GPIO_PORT_P3,GPIO_PIN7);
        // B RED EB
        GPIO_setOutputLowOnPin(GPIO_PORT_P3,GPIO_PIN6);
        // ABAR BLUE EB
        GPIO_setOutputLowOnPin(GPIO_PORT_P3,GPIO_PIN5);
        // BBAR GREEN EB
        GPIO_setOutputLowOnPin(GPIO_PORT_P3,GPIO_PIN4);
        break;

    }
}

void myMotorDriver(motorSeq) //controls direction it move
{
   uint8_t a;
    switch(motorSeq)
    {
        case 0: //hold
            motorSeq = motorSeq; //hold/standby
            break;
        case 1: //clockwise
                if(stepSeq == 8)
                {
                    stepSeq = 0;
                }
                else
                {
                    stepSeq++;

                }
                myStepperDriver(stepSeq);
                break;
        case 2: // counter clockwise
                if(stepSeq == 0)
                {
                    stepSeq = 8;
                }
                else
                {
                    stepSeq--;
                }
               myStepperDriver(stepSeq);
                break;
    }
}
