#define TARGET_IS_TM4C123_RA1

#include "inc/tm4c123gh6pm.h"
#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include <math.h>
#include "driverlib/gpio.h"
#include "inc/hw_memmap.h"
#include "inc/hw_gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/pwm.h"
#include "driverlib/sysctl.h"
#include "driverlib/pin_map.h"
#include "inc/hw_types.h"
#include "driverlib/interrupt.h"
#include "driverlib/timer.h"
#include "driverlib/systick.h"
#include "driverlib/qei.h"
#include "driverlib/uart.h"
#include "driverlib/rom.h"
#include "driverlib/fpu.h"



void pwmInit(void);
void intIni(void);
void magnetInit(void);
void qeiInit(void);
void uartInit(void);

// CONSTANTS -------------------------------------------------------------------------
float kp = 26.5, kd = 525, initialAngle = 90, finalAngle = 45;

// VARIABLES -------------------------------------------------------------------------
float lastError = 0, currentAngle = 0, error, integral = 0, degrees, p, d;
float flag = 0, object = 0, output, qeiPosition = 0, qeiRotation = 0, qeiDegrees = 0;

int j = 0; // used for allowing settling before output is turned off
int M = 10; // moving average size
int MA_flag = 0; // when moving average array is filled, set to 1
int MA_count = 0; // moving average count
int data = 0; // data to send over UART
float output_array[10]; // used to store controller output values

float filter(float *array, int M) { //filter for the motor
   
    int n;
    float average = 0;
    for (n=0; n<M; n++){
        average =  average + array[n];
    }
    average = average / M;
    return average;
}
//*****************************************************************************
//
// Send a string to the UART.
//
//*****************************************************************************
void
UARTSend(const uint8_t *pui8Buffer, uint32_t ui32Count)
{
    //
    // Loop while there are more characters to send.
    //
    while(ui32Count--)
    {
        //
        // Write the next character to the UART.
        //
        ROM_UARTCharPutNonBlocking(UART0_BASE, *pui8Buffer++);
    }
}

void UARTIntHandler(void)
{
    uint32_t ui32Status;

    //
    // Get the interrrupt status.
    //
    ui32Status = ROM_UARTIntStatus(UART0_BASE, true);

    //
    // Clear the asserted interrupts.
    //
    ROM_UARTIntClear(UART0_BASE, ui32Status);

    //
    // Loop while there are characters in the receive FIFO.
    //
    while(ROM_UARTCharsAvail(UART0_BASE))
    {
        //
        // Read the next character from the UART and write it back to the UART.
        //
        ROM_UARTCharPutNonBlocking(UART0_BASE,
                                   ROM_UARTCharGetNonBlocking(UART0_BASE));

        //
        // Blink the LED to show a character transfer is occuring.
        //
        GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, GPIO_PIN_2);

        //
        // Delay for 1 millisecond.  Each SysCtlDelay is about 3 clocks.
        //
        SysCtlDelay(SysCtlClockGet() / (1000 * 3));

        //
        // Turn off the LED
        //
        GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, 0);

    }
}

void Timer0IntHandler(void){

    TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT); //clear interrupt timer
    qeiPosition = QEIPositionGet(QEI0_BASE); // get position from the QEI 
    qeiRotation = QEIDirectionGet(QEI0_BASE); // get the direction from QEI
    qeiDegrees = qeiPosition*360.0/12606.0; //degrees from 0 to 360
    degrees = qeiDegrees; // setting new variable

    data = (int) degrees;
    flag = 3;
    ROM_UARTCharPutNonBlocking(UART0_BASE, data);


    if (degrees > 180.0) {
        degrees = (360.0 - degrees)*-1.0; // angles range from -180 to 180 
    }

   if (object == 0) { // magnet has not been picked up
       error = initialAngle - degrees; // the initial angle from start minus the current motor position
       if( error >= -1.5 && error <= 1.5){ // error is between the limits
           j++; // small delay
           if (j > 5){
               object = 1; //object has been picked up
               GPIOPinTypeGPIOOutput(GPIO_PORTB_BASE, GPIO_PIN_5 | GPIO_PIN_7);
               GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_5, 0x00); // turning pwm off
               GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_7, 0x00); // turning pwm off
               GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_0, GPIO_PIN_0); // turn on magnet
               _delay_cycles((100*16000)-1); // delay for 16000-1 = 1 millisecond
               j = 0; // exit statement
           }
       }
   }

   else {  // if object has been picked up
      error = finalAngle - degrees; // the final angle from start reference minus the current motor position
      if( error >= -2 && error <= 2){ // error is between the limits
          j++; // small delay
          if (j > 8) {
            GPIOPinTypeGPIOOutput(GPIO_PORTB_BASE, GPIO_PIN_5 | GPIO_PIN_7);
            GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_5, 0x00); // turning pwm off
            GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_7, 0x00); // turning pwm off
            GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_0, 0x00); // turn off magnet
            flag = 3; // program will exit
            output = 0; // motor stops
          }
      }
   }

     if(flag !=3){
         p = kp * error; // position 
         d = kd * (error - lastError); //derivative
         output = p + d; // minimum PWM should be 80 to properly power motor
         lastError = error; // setting last error for derivative term
         if (MA_count < M){ // filter for motor
             output_array[MA_count] = output;
             MA_count++;
         }
         else if (MA_count == M){
             MA_flag = 1;
             MA_count = 0;
         }
         if(MA_flag == 1) output = filter(output_array, M);
     }

     if(output < 0){ 
         output = (-1.0*output); // make output positive
         flag = 0; // go CCW
     }
     else if (output > 0) flag = 1; // go CW

     if (output >= 950) output = 950; // Setting max output duty cycle, so it never hits 100%.

     else if( output < 80 && output > 30) output = 80; //setting min output duty cycle b/c doesn't work below 80

     if(flag==0) // CCW
     {
         GPIOPinTypeGPIOOutput(GPIO_PORTB_BASE, GPIO_PIN_5); // make pin 5 output for Q1/Q4 (CC)
         GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_5, 0x00); // turning pwm off

         GPIOPinTypePWM(GPIO_PORTB_BASE, GPIO_PIN_7); //set pin back to PWM for Q2/Q3 (CCW)
         PWMPulseWidthSet(PWM0_BASE, PWM_OUT_1, output); //CCW
     }

     if(flag==1){ // CW
            GPIOPinTypeGPIOOutput(GPIO_PORTB_BASE, GPIO_PIN_7);
            GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_7, 0x00); // turning pwm off for Q2/Q3

            GPIOPinTypePWM(GPIO_PORTB_BASE, GPIO_PIN_5); //set pin back to PWM for Q1/Q4 (CW)
            PWMPulseWidthSet(PWM0_BASE, PWM_OUT_3, output); //CW

         if(degrees > 90.0){ //CCW
             GPIOPinTypeGPIOOutput(GPIO_PORTB_BASE, GPIO_PIN_5);
             GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_5, 0x00); // turning pwm off for Q1/Q4
             }
         }
 }

void gpioInit(void){

    GPIOPadConfigSet(GPIO_PORTB_BASE,GPIO_PIN_5,GPIO_STRENGTH_8MA,GPIO_PIN_TYPE_STD_WPD); // weak pull down resistor
    GPIOPadConfigSet(GPIO_PORTB_BASE,GPIO_PIN_7,GPIO_STRENGTH_8MA,GPIO_PIN_TYPE_STD_WPD); // weak pull down resistor
    GPIOPinTypeGPIOOutput(GPIO_PORTB_BASE, GPIO_PIN_5 | GPIO_PIN_7); // make pin 5 & 7 output
    GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_5, 0x00); // turning pwm off / setting to low
    GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_7, 0x00); // turning pwm off / setting to low
}

void main(void) {

        SysCtlClockSet(SYSCTL_SYSDIV_5 | SYSCTL_USE_OSC |   SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ); // clock

        magnetInit();
        pwmInit();
        intIni();
        qeiInit();
        gpioInit();
        uartInit();

       while(1){
        }
}

void pwmInit(){
         SysCtlPWMClockSet(SYSCTL_PWMDIV_32); //Configure PWM Clock to match system
         SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0); // enable PWM
         SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB); // enable port B for GPIO
          while(! SysCtlPeripheralReady(SYSCTL_PERIPH_PWM0)) {} // wait until port is ready
          GPIODirModeSet(GPIO_PORTB_BASE, GPIO_PIN_5  | GPIO_PIN_7, GPIO_DIR_MODE_OUT); // set pins to output direction
          GPIOPinTypePWM(GPIO_PORTB_BASE, GPIO_PIN_5 | GPIO_PIN_7 ); // configure pins as PWM
          GPIOPinConfigure(GPIO_PB5_M0PWM3); // configure pins as PWM
          GPIOPinConfigure(GPIO_PB7_M0PWM1); // configure pins as PWM
          PWMGenConfigure(PWM0_BASE, PWM_GEN_0, PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC); // gen 0 initilization
          PWMGenConfigure(PWM0_BASE, PWM_GEN_1, PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC); // gen 1 initialization
          PWMGenPeriodSet(PWM0_BASE, PWM_GEN_0, 1000); // setting pwm period
          PWMGenPeriodSet(PWM0_BASE, PWM_GEN_1, 1000); // setting pwm period
          PWMOutputState(PWM0_BASE, (PWM_OUT_1_BIT | PWM_OUT_3_BIT), true); // putting pwm as output
          PWMGenEnable(PWM0_BASE, PWM_GEN_0); // enable gen 0
          PWMGenEnable(PWM0_BASE, PWM_GEN_1); // enable gen 1
}

void magnetInit(void){ //magnet initiilzation
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB); //enable port b
    while (! SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOB)) {} // wait to be initialized
    GPIOPinTypeGPIOOutput(GPIO_PORTB_BASE, GPIO_PIN_0); //set the gpio pin to output PB-O for magnet
    GPIOPadConfigSet(GPIO_PORTB_BASE,GPIO_PIN_0,GPIO_STRENGTH_2MA,GPIO_PIN_TYPE_OD); // weak pull up and 8ma current
    GPIODirModeSet(GPIO_PORTB_BASE, GPIO_PIN_0, GPIO_DIR_MODE_OUT); // gpio pin output
    GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_0, 0x00); // setting the magnet to off
}

void intIni(void){ //interupt initilization
    uint32_t ui32Period; // period integer 
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0); // enable timer 0 
    TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC); // configure timer as periodic 
    ui32Period = (SysCtlClockGet() /100); // period is 16MHZ/(5*100) = 32000 Hz
    TimerLoadSet(TIMER0_BASE, TIMER_A, ui32Period - 1); // load the timer with the calculated period
    IntEnable(INT_TIMER0A); // enable timer 0A for intterupt
    TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT); // enable the interrupt for timer
    IntMasterEnable(); // enable interrupt
    TimerEnable(TIMER0_BASE, TIMER_A); // enable timer A
}

void qeiInit(void){ // QEI Initilization
    SysCtlPeripheralEnable (SYSCTL_PERIPH_GPIOD); // enable port D
    SysCtlPeripheralEnable (SYSCTL_PERIPH_QEI0); // enable quadature encoder interface 0 
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_QEI0)) {} // wait for it to be ready 
    HWREG(GPIO_PORTD_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY; // // unlocking pin D7
    HWREG(GPIO_PORTD_BASE + GPIO_O_CR) |= 0x80; // unlocking pin D7
    HWREG(GPIO_PORTD_BASE + GPIO_O_LOCK) = 0; // unlocking pin D7
    HWREG(QEI0_BASE + QEI0_CTL_R) |= QEI_CTL_FILTEN; // enables digital filter
    GPIOPinConfigure(GPIO_PD6_PHA0); // A
    GPIOPinConfigure(GPIO_PD7_PHB0); // B
    GPIOPinConfigure(GPIO_PD3_IDX0); // Index
    GPIOPinTypeQEI(GPIO_PORTD_BASE, GPIO_PIN_3 | GPIO_PIN_6 |  GPIO_PIN_7); // enable pins as QEI inputs
    QEIDisable(QEI0_BASE); // disable QEI
    QEIIntDisable(QEI0_BASE,QEI_INTERROR | QEI_INTDIR | QEI_INTTIMER | QEI_INTINDEX); // disables QEI interrupts
    QEIConfigure(QEI0_BASE, (QEI_CONFIG_CAPTURE_A_B  | QEI_CONFIG_NO_RESET  | QEI_CONFIG_QUADRATURE | QEI_CONFIG_NO_SWAP), 12606); //127990
    QEIEnable(QEI0_BASE); // enable QEI
}

// Initialize UART module
void uartInit(void)
{

    // Enable lazy stacking for interrupt handlers.  This allows floating-point
    // instructions to be used within interrupt handlers, but at the expense of
    // extra stack usage.
    //
    ROM_FPUEnable();
    ROM_FPULazyStackingEnable();

    //
    // Set the clocking to run directly from the crystal.
    //
    ROM_SysCtlClockSet(SYSCTL_SYSDIV_1 | SYSCTL_USE_OSC | SYSCTL_OSC_MAIN |
                       SYSCTL_XTAL_16MHZ);

    //
    // Enable the GPIO port that is used for the on-board LED.
    //
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);

    //
    // Enable the GPIO pins for the LED (PF2).
    //
    ROM_GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_2);

    //
    // Enable the peripherals used by this example.
    //
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

    //
    // Enable processor interrupts.
    //
    //ROM_IntMasterEnable();

    //
    // Set GPIO A0 and A1 as UART pins.
    //
    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);
    ROM_GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    //
    // Configure the UART for 115,200, 8-N-1 operation.
    //
    ROM_UARTConfigSetExpClk(UART0_BASE, ROM_SysCtlClockGet(), 115200,
                            (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
                             UART_CONFIG_PAR_NONE));


     //Enable the UART interrupt

    ROM_IntEnable(INT_UART0);
    ROM_UARTIntEnable(UART0_BASE, UART_INT_RX | UART_INT_RT);

}
