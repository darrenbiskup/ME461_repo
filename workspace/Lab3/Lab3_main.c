//#############################################################################
// FILE:   LABstarter_main.c
//
// TITLE:  Lab Starter
//#############################################################################

// Included Files
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <string.h>
#include <math.h>
#include <limits.h>
#include "F28x_Project.h"
#include "driverlib.h"
#include "device.h"
#include "F28379dSerial.h"
#include "LEDPatterns.h"
#include "song.h"
#include "dsp.h"
#include "fpu32/fpu_rfft.h"

#define PI          3.1415926535897932384626433832795
#define TWOPI       6.283185307179586476925286766559
#define HALFPI      1.5707963267948966192313216916398
// The Launchpad's CPU Frequency set to 200 you should not change this value
#define LAUNCHPAD_CPU_FREQUENCY 200

// Interrupt Service Routines predefinition
__interrupt void cpu_timer0_isr(void);
__interrupt void cpu_timer1_isr(void);
__interrupt void cpu_timer2_isr(void);
__interrupt void SWI_isr(void);

// Count variables
uint32_t numTimer0calls = 0;
uint32_t numSWIcalls = 0;
extern uint32_t numRXA;
uint16_t UARTPrint = 0;
uint16_t LEDdisplaynum = 0;
int16_t updown = 1;
int16_t motorupdown = 1; //amp dbc either 0 or one to let us know ehn to count up or down
uint16_t songindex = 0;

void setEPWM2A(float controleffort);
void setEPWM2B(float controleffort);
void setEPWM8A_RCServo(float angle); //amp dbc predefine functions to prevent errors
void setEPWM8B_RCServo(float angle);

//amp dbc define global control effort for motor and servo control
float ctrlEffort = 0;

void main(void)
{
    // PLL, WatchDog, enable Peripheral Clocks
    // This example function is found in the F2837xD_SysCtrl.c file.
    InitSysCtrl();

    InitGpio();

    // Blue LED on LaunchPad
    GPIO_SetupPinMux(31, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(31, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPASET.bit.GPIO31 = 1;

    // Red LED on LaunchPad
    GPIO_SetupPinMux(34, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(34, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPBSET.bit.GPIO34 = 1;

    // LED1 and PWM Pin
    GPIO_SetupPinMux(22, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(22, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPACLEAR.bit.GPIO22 = 1;

    // LED2
    GPIO_SetupPinMux(94, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(94, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPCCLEAR.bit.GPIO94 = 1;

    // LED3
    GPIO_SetupPinMux(95, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(95, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPCCLEAR.bit.GPIO95 = 1;

    // LED4
    GPIO_SetupPinMux(97, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(97, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPDCLEAR.bit.GPIO97 = 1;

    // LED5
    GPIO_SetupPinMux(111, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(111, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPDCLEAR.bit.GPIO111 = 1;

    // LED6
    GPIO_SetupPinMux(130, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(130, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPECLEAR.bit.GPIO130 = 1;

    // LED7
    GPIO_SetupPinMux(131, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(131, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPECLEAR.bit.GPIO131 = 1;

    // LED8
    GPIO_SetupPinMux(25, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(25, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPACLEAR.bit.GPIO25 = 1;

    // LED9
    GPIO_SetupPinMux(26, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(26, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPACLEAR.bit.GPIO26 = 1;

    // LED10
    GPIO_SetupPinMux(27, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(27, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPACLEAR.bit.GPIO27 = 1;

    // LED11
    GPIO_SetupPinMux(60, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(60, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPBCLEAR.bit.GPIO60 = 1;

    // LED12
    GPIO_SetupPinMux(61, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(61, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPBCLEAR.bit.GPIO61 = 1;

    // LED13
    GPIO_SetupPinMux(157, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(157, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPECLEAR.bit.GPIO157 = 1;

    // LED14
    GPIO_SetupPinMux(158, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(158, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPECLEAR.bit.GPIO158 = 1;

    // LED15
    GPIO_SetupPinMux(159, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(159, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPECLEAR.bit.GPIO159 = 1;

    // LED16
    GPIO_SetupPinMux(160, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(160, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPFCLEAR.bit.GPIO160 = 1;

    //WIZNET Reset
    GPIO_SetupPinMux(0, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(0, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPASET.bit.GPIO0 = 1;

    //ESP8266 Reset
    GPIO_SetupPinMux(1, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(1, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPASET.bit.GPIO1 = 1;

    //SPIRAM  CS  Chip Select
    GPIO_SetupPinMux(19, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(19, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPASET.bit.GPIO19 = 1;

    //DRV8874 #1 DIR  Direction
    GPIO_SetupPinMux(29, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(29, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPASET.bit.GPIO29 = 1;

    //DRV8874 #2 DIR  Direction
    GPIO_SetupPinMux(32, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(32, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPBSET.bit.GPIO32 = 1;

    //DAN28027  CS  Chip Select
    GPIO_SetupPinMux(9, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(9, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPASET.bit.GPIO9 = 1;

    //MPU9250  CS  Chip Select
    GPIO_SetupPinMux(66, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(66, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;

    //WIZNET  CS  Chip Select
    GPIO_SetupPinMux(125, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(125, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPDSET.bit.GPIO125 = 1;

    //PushButton 1
    GPIO_SetupPinMux(4, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(4, GPIO_INPUT, GPIO_PULLUP);

    //PushButton 2
    GPIO_SetupPinMux(5, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(5, GPIO_INPUT, GPIO_PULLUP);

    //PushButton 3
    GPIO_SetupPinMux(6, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(6, GPIO_INPUT, GPIO_PULLUP);

    //PushButton 4
    GPIO_SetupPinMux(7, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(7, GPIO_INPUT, GPIO_PULLUP);

    //Joy Stick Pushbutton
    GPIO_SetupPinMux(8, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(8, GPIO_INPUT, GPIO_PULLUP);

    // Clear all interrupts and initialize PIE vector table:
    // Disable CPU interrupts
    DINT;

    // Initialize the PIE control registers to their default state.
    // The default state is all PIE interrupts disabled and flags
    // are cleared.
    // This function is found in the F2837xD_PieCtrl.c file.
    InitPieCtrl();

    // Disable CPU interrupts and clear all CPU interrupt flags:
    IER = 0x0000;
    IFR = 0x0000;

    // Initialize the PIE vector table with pointers to the shell Interrupt
    // Service Routines (ISR).
    // This will populate the entire table, even if the interrupt
    // is not used in this example.  This is useful for debug purposes.
    // The shell ISR routines are found in F2837xD_DefaultIsr.c.
    // This function is found in F2837xD_PieVect.c.
    InitPieVectTable();

    // Interrupts that are used in this example are re-mapped to
    // ISR functions found within this project
    EALLOW;
    // This is needed to write to EALLOW protected registers
    PieVectTable.TIMER0_INT = &cpu_timer0_isr;
    PieVectTable.TIMER1_INT = &cpu_timer1_isr;
    PieVectTable.TIMER2_INT = &cpu_timer2_isr;
    PieVectTable.SCIA_RX_INT = &RXAINT_recv_ready;
    PieVectTable.SCIB_RX_INT = &RXBINT_recv_ready;
    PieVectTable.SCIC_RX_INT = &RXCINT_recv_ready;
    PieVectTable.SCID_RX_INT = &RXDINT_recv_ready;
    PieVectTable.SCIA_TX_INT = &TXAINT_data_sent;
    PieVectTable.SCIB_TX_INT = &TXBINT_data_sent;
    PieVectTable.SCIC_TX_INT = &TXCINT_data_sent;
    PieVectTable.SCID_TX_INT = &TXDINT_data_sent;

    PieVectTable.EMIF_ERROR_INT = &SWI_isr;
    EDIS;
    // This is needed to disable write to EALLOW protected registers

    // Initialize the CpuTimers Device Peripheral. This function can be
    // found in F2837xD_CpuTimers.c
    InitCpuTimers();

    // Configure CPU-Timer 0, 1, and 2 to interrupt every given period:
    // 200MHz CPU Freq,                       Period (in uSeconds)
    ConfigCpuTimer(&CpuTimer0, LAUNCHPAD_CPU_FREQUENCY, 10000);
    ConfigCpuTimer(&CpuTimer1, LAUNCHPAD_CPU_FREQUENCY, 125000*4); // amp dbc make the period larger for the song to play at the right tempo
    ConfigCpuTimer(&CpuTimer2, LAUNCHPAD_CPU_FREQUENCY, 500);

    // Enable CpuTimer Interrupt bit TIE
    CpuTimer0Regs.TCR.all = 0x4000;
    CpuTimer1Regs.TCR.all = 0x4000;
    CpuTimer2Regs.TCR.all = 0x4000;

    init_serialSCIA(&SerialA, 115200);

    // amp dbc PWM12A setup
    EPwm12Regs.TBCTL.bit.FREE_SOFT = 0x3; // 11 in bit
    EPwm12Regs.TBCTL.bit.CTRMODE = 0x0; //00 in bit
    EPwm12Regs.TBCTL.bit.PHSEN = 0x0; //0 in bit
    EPwm12Regs.TBCTL.bit.HSPCLKDIV = 0x0; //000 in bit

    EPwm12Regs.TBCTR = 0x0; //000 in bit
    EPwm12Regs.TBPRD = 2500; //(1/20000)/(1/50000000)
    EPwm12Regs.CMPA.bit.CMPA = 0;

    EPwm12Regs.AQCTLA.bit.CAU = 1; //clearwhen TBCTR = CMPA on up count

    EPwm12Regs.AQCTLA.bit.ZRO = 2; //set when TBCTR = 0

    EPwm12Regs.TBPHS.bit.TBPHS = 0; //set phase to 0

    GPIO_SetupPinMux(22, GPIO_MUX_CPU1, 5); //GPIO PinName, CPU, Mux Index
    //set gpio 22 to EPWM12A

    // amp dbc PWM2A setup
    EPwm2Regs.TBCTL.bit.FREE_SOFT = 0x3; // 11 in bit
    EPwm2Regs.TBCTL.bit.CTRMODE = 0x0; //00 in bit
    EPwm2Regs.TBCTL.bit.PHSEN = 0x0; //0 in bit
    EPwm2Regs.TBCTL.bit.HSPCLKDIV = 0x0; //000 in bit

    EPwm2Regs.TBCTR = 0x0; //000 in bit
    EPwm2Regs.TBPRD = 2500; //(1/20000)/(1/50000000)
    EPwm2Regs.CMPA.bit.CMPA = 0;
    EPwm2Regs.CMPB.bit.CMPB = 0;

    EPwm2Regs.AQCTLA.bit.CAU = 1; //clearwhen TBCTR = CMPA on up count
    EPwm2Regs.AQCTLB.bit.CBU = 1; //clearwhen TBCTR = CMPB on up count

    EPwm2Regs.AQCTLA.bit.ZRO = 2; //set when TBCTR = 0
    EPwm2Regs.AQCTLB.bit.ZRO = 2; //set when TBCTR = 0

    EPwm2Regs.TBPHS.bit.TBPHS = 0; //set phase to 0

    GPIO_SetupPinMux(2, GPIO_MUX_CPU1, 1); //GPIO PinName, CPU, Mux Index
    GPIO_SetupPinMux(3, GPIO_MUX_CPU1, 1); //GPIO PinName, CPU, Mux Index
    //set gpio 2 to EPWM2Af
    //set gpio 3 to EPWM2B

    // amp dbc PWM8A setup
    EPwm8Regs.TBCTL.bit.FREE_SOFT = 0x3; // 11 in bit
    EPwm8Regs.TBCTL.bit.CTRMODE = 0x0; //00 in bit
    EPwm8Regs.TBCTL.bit.PHSEN = 0x0; //0 in bit
    EPwm8Regs.TBCTL.bit.CLKDIV = 0x4; //100 in bit

    EPwm8Regs.TBCTR = 0x0; //000 in bit
    EPwm8Regs.TBPRD = 62500; //50*10^6/(clkdiv*50)
    EPwm8Regs.CMPA.bit.CMPA = 62500 * 0.08;
    EPwm8Regs.CMPB.bit.CMPB = 62500 * 0.08; // amp dbc 8% duty cycle is position 0 on the servo

    EPwm8Regs.AQCTLA.bit.CAU = 1; //clearwhen TBCTR = CMPA on up count
    EPwm8Regs.AQCTLB.bit.CBU = 1; //clearwhen TBCTR = CMPB on up count

    EPwm8Regs.AQCTLA.bit.ZRO = 2; //set when TBCTR = 0
    EPwm8Regs.AQCTLB.bit.ZRO = 2; //set when TBCTR = 0

    EPwm8Regs.TBPHS.bit.TBPHS = 0; //set phase to 0

    GPIO_SetupPinMux(14, GPIO_MUX_CPU1, 1); //GPIO PinName, CPU, Mux Index
    GPIO_SetupPinMux(15, GPIO_MUX_CPU1, 1); //GPIO PinName, CPU, Mux Index
    //set gpio 14 to EPWM8A
    //set gpio 15 to EPWM8B

    // amp dbc PWM9A setup
    EPwm9Regs.TBCTL.bit.FREE_SOFT = 0x3; // 11 in bit
    EPwm9Regs.TBCTL.bit.CTRMODE = 0x0; //00 in bit
    EPwm9Regs.TBCTL.bit.PHSEN = 0x0; //0 in bit
    EPwm9Regs.TBCTL.bit.CLKDIV = 0x1; //000 in bit

    EPwm9Regs.TBCTR = 0x0; //000 in bit
    EPwm9Regs.TBPRD = 2500; //(1/20000)/(1/50000000)
    //EPwm9Regs.CMPA.bit.CMPA = 0;

    EPwm9Regs.AQCTLA.bit.CAU = 0; //clearwhen TBCTR = CMPA on up count
    EPwm9Regs.AQCTLA.bit.ZRO = 3; //set when TBCTR = 0

    EPwm9Regs.TBPHS.bit.TBPHS = 0; //set phase to 0

    GPIO_SetupPinMux(16, GPIO_MUX_CPU1, 5); //GPIO PinName, CPU, Mux Index
    //set gpio 16 to EPWM9A

    EALLOW;
    // Below are protected registers
    GpioCtrlRegs.GPAPUD.bit.GPIO2 = 1; // For EPWM2A
    GpioCtrlRegs.GPAPUD.bit.GPIO3 = 1; // For EPWM2B
    GpioCtrlRegs.GPAPUD.bit.GPIO14 = 1; // For EPWM8A
    GpioCtrlRegs.GPAPUD.bit.GPIO15 = 1; // For EPWM8B
    GpioCtrlRegs.GPAPUD.bit.GPIO16 = 1; // For EPWM9A
    GpioCtrlRegs.GPAPUD.bit.GPIO22 = 1; // For EPWM12A
    EDIS;

    // Enable CPU int1 which is connected to CPU-Timer 0, CPU int13
    // which is connected to CPU-Timer 1, and CPU int 14, which is connected
    // to CPU-Timer 2:  int 12 is for the SWI.  
    IER |= M_INT1;
    IER |= M_INT8;  // SCIC SCID
    IER |= M_INT9;  // SCIA
    IER |= M_INT12;
    IER |= M_INT13;
    IER |= M_INT14;

    // Enable TINT0 in the PIE: Group 1 interrupt 7
    PieCtrlRegs.PIEIER1.bit.INTx7 = 1;
    // Enable SWI in the PIE: Group 12 interrupt 9
    PieCtrlRegs.PIEIER12.bit.INTx9 = 1;

//    init_serialSCIB(&SerialB, 115200);
    init_serialSCIC(&SerialC, 115200);
    init_serialSCID(&SerialD, 115200);
    // Enable global Interrupts and higher priority real-time debug events
    EINT;
    // Enable Global interrupt INTM
    ERTM;
    // Enable Global realtime interrupt DBGM

    // IDLE loop. Just sit and loop forever (optional):
    while (1)
    {
        if (UARTPrint == 1)
        {
            serial_printf(&SerialA, "Num Timer2:%ld Num SerialRX: %ld\r\n",
                          CpuTimer2.InterruptCount, numRXA);
            UARTPrint = 0;
        }
    }
}

// SWI_isr,  Using this interrupt as a Software started interrupt
__interrupt void SWI_isr(void)
{

    // These three lines of code allow SWI_isr, to be interrupted by other interrupt functions
    // making it lower priority than all other Hardware interrupts.
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP12;
    asm("       NOP");
    // Wait one cycle
    EINT;
    // Clear INTM to enable interrupts

    // Insert SWI ISR Code here.......

    numSWIcalls++;

    DINT;

}

// cpu_timer0_isr - CPU Timer0 ISR
__interrupt void cpu_timer0_isr(void)
{
    CpuTimer0.InterruptCount++;

    numTimer0calls++;

    //    if ((numTimer0calls%50) == 0) {
    //        PieCtrlRegs.PIEIFR12.bit.INTx9 = 1;  // Manually cause the interrupt for the SWI
    //    }

    if ((numTimer0calls % 25) == 0)
    {
        //displayLEDletter(LEDdisplaynum);
        LEDdisplaynum++;
        if (LEDdisplaynum == 0xFFFF)
        {  // prevent roll over exception
            LEDdisplaynum = 0;
        }
    }

    if ((numTimer0calls % 50) == 0)
    {
        // Blink LaunchPad Red LED
        GpioDataRegs.GPBTOGGLE.bit.GPIO34 = 1;
    }

    // Acknowledge this interrupt to receive more interrupts from group 1
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}

// cpu_timer1_isr - CPU Timer1 ISR
__interrupt void cpu_timer1_isr(void)
{

    //amp dbc this part of the code reads the array from song.h
    //from the values of each elem in the array we set the TBPRD to make different pitches
    //this is using a sqare wave so the duty cycle is fixed but the frequency/period is modulated
    EPwm9Regs.TBPRD = songarray[songindex];  //song array val at current index
    if (songindex < SONG_LENGTH) //amp dbc prevents us from going above the array
        songindex++;
    else
    {
        GPIO_SetupPinMux(16, GPIO_MUX_CPU1, 0);
        GpioDataRegs.GPACLEAR.bit.GPIO16 = 1;
        //amp dbc this turns the buzzer pin off when the song ends
    }
    CpuTimer1.InterruptCount++;
}

// cpu_timer2_isr CPU Timer2 ISR
__interrupt void cpu_timer2_isr(void)
{
    // Blink LaunchPad Blue LED
    GpioDataRegs.GPATOGGLE.bit.GPIO31 = 1;

    CpuTimer2.InterruptCount++;

    if ((CpuTimer2.InterruptCount % 2) == 0)
    {
        //amp dbc motor control up and down control
//        if (motorupdown)
//        {
//            ctrlEffort += 0.01;
//        }
//        else
//        {
//            ctrlEffort -= 0.01;
//        }
//        if (ctrlEffort > 10)
//        {
//            motorupdown = 0;
//        }
//        if (ctrlEffort < -10)
//        {
//            motorupdown = 1;
//        }
//        setEPWM2A(ctrlEffort);
//        setEPWM2B(-ctrlEffort);

        // amp dbc increment servo up down
        if (motorupdown)
        {
            ctrlEffort += 0.05; //step by 0.05
        }
        else
        {
            ctrlEffort -= 0.05;
        }
        if (ctrlEffort > 90)
        {
            motorupdown = 0;
        }
        if (ctrlEffort < -90)
        {
            motorupdown = 1;
        }
        setEPWM8A_RCServo(ctrlEffort); //amp dbc calls the set servo functions
        setEPWM8B_RCServo(-ctrlEffort);

        //amp dbc gradually increase the led brightness from 0 to 100% and back down forever at 1ms
        if (updown)
        {
            EPwm12Regs.CMPA.bit.CMPA++;
        }
        else
        {
            EPwm12Regs.CMPA.bit.CMPA--;
        }
        if (EPwm12Regs.CMPA.bit.CMPA == EPwm12Regs.TBPRD)
        {
            updown = 0;
        }
        if (EPwm12Regs.CMPA.bit.CMPA == 0)
        {
            updown = 1;
        }
        //UARTPrint = 1;
    }

}

//amp dbc saturated the motor input and sets the motor velocity
void setEPWM2A(float controleffort)
{
    //amp dbc saturate controleffort if too big or too small
    if (controleffort > 10)
    {
        controleffort = 10;
    }
    if (controleffort < -10)
    {
        controleffort = -10;
    }
    // this is a linear mapping
    EPwm2Regs.CMPA.bit.CMPA = ((controleffort / 20.0) + 0.5) * EPwm2Regs.TBPRD; //note the float division
}

//amp dbc saturated the motor input and sets the motor velocity
void setEPWM2B(float controleffort)
{
    //amp dbc saturate controleffort if too big or too small
    if (controleffort > 10)
    {
        controleffort = 10;
    }
    if (controleffort < -10)
    {
        controleffort = -10;
    }

    EPwm2Regs.CMPB.bit.CMPB = ((controleffort / 20.0) + 0.5) * EPwm2Regs.TBPRD;
}

//amp dbc saturated the servo input and sets the servo position
void setEPWM8A_RCServo(float angle)
{
    if (angle > 90)
        angle = 90;
    if (angle < -90)
        angle = -90;

    angle = (angle + 90) / 180.0; //brings angle between 0 and 1 amp dbc
    angle = angle * 8.0 + 4; //between 4 and 12

    EPwm8Regs.CMPA.bit.CMPA = 62500 * angle / 100.0;

}

//amp dbc saturated the servo input and sets the servo position
void setEPWM8B_RCServo(float angle)
{
    if (angle > 90)
        angle = 90;
    if (angle < -90)
        angle = -90;

    angle = (angle + 90) / 180.0; //brings angle between 0 and 1 amp dbc
    angle = angle * 8.0 + 4; //between 4 and 12

    EPwm8Regs.CMPB.bit.CMPB = 62500 * angle / 100.0;
}
