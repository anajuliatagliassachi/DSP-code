// TI File $Revision: /main/1 $
// Checkin $Date: October 6, 2010   14:42:31 $
//###########################################################################
//
// FILE:    Example_2802xEPwmRealTimeInt.c
//
// TITLE:   DSP2802x ePWM Real-Time Interrupt example.
//
// ASSUMPTIONS:
//
//    This program requires the DSP2802x header files.
//
//    Other then boot mode configuration, no other hardware configuration
//    is required.
//
//    As supplied, this project is configured for "boot to SARAM"
//    operation.  The 2802x Boot Mode table is shown below.
//    For information on configuring the boot mode of an eZdsp,
//    please refer to the documentation included with the eZdsp.
//
//    $Boot_Table
//    While an emulator is connected to your device, the TRSTn pin = 1,
//    which sets the device into EMU_BOOT boot mode. In this mode, the
//    peripheral boot modes are as follows:
//
//      Boot Mode:   EMU_KEY        EMU_BMODE
//                   (0xD00)         (0xD01)
//      ---------------------------------------
//      Wait         !=0x55AA        X
//      I/O          0x55AA          0x0000
//      SCI          0x55AA          0x0001
//      Wait         0x55AA          0x0002
//      Get_Mode     0x55AA          0x0003
//      SPI          0x55AA          0x0004
//      I2C          0x55AA          0x0005
//      OTP          0x55AA          0x0006
//      Wait         0x55AA          0x0007
//      Wait         0x55AA          0x0008
//      SARAM        0x55AA          0x000A   <-- "Boot to SARAM"
//      Flash        0x55AA          0x000B
//      Wait         0x55AA          Other
//
//   Write EMU_KEY to 0xD00 and EMU_BMODE to 0xD01 via the debugger
//   according to the Boot Mode Table above. Build/Load project,
//   Reset the device, and Run example
//
//   $End_Boot_Table
//
//
// Description:
//
//    This example configures the ePWM1 Timer and increments
//    a counter each time an interrupt is taken. ePWM interrupt can
//    be configured as time critical to demonstrate real-time mode
//    functionality and real-time interrupt capability
//
//    ControlCard LED2 (GPIO31) toggled in main loop
//    ControlCard LED3 (GPIO34) toggled in ePWM1 Timer Interrupt
//
//    FREE_SOFT bits and DBBIER.INT3 bit must be set to enable ePWM1
//    interrupt to be time critical and operational in real time mode
//    after halt command
//
//    As supplied:
//
//    ePWM1 is initalized
//    ePWM1 is cleared at period match and set at Compare-A match
//    Compare A match occurs at half period
//
//    GPIOs for LED2 and LED3 are initialized
//
//    Free_Soft bits and DBGIER are cleared
//
//    An interrupt is taken on a zero event for the ePWM1 timer
//
//    Watch Variables:
//       EPwm1TimerIntCount
//       EPwm1Regs.TBCTL.bit.FREE_SOFT
//       EPwm1Regs.TBCTR
//       DBGIER.INT3
//
//###########################################################################
// $TI Release: 2802x C/C++ Header Files and Peripheral Examples V1.29 $
// $Release Date: January 11, 2011 $
//###########################################################################
// Select the global Q value to use:
#define GLOBAL_Q    20
long GlobalQ = GLOBAL_Q;      // Used for legacy GEL & Graph Debug.

// Include The Following Definition Files:
#include <stdio.h>
#include <stdlib.h>
#include "IQmathLib.h"
#include "DSP28x_Project.h"     // Device Headerfile and Examples Include File


// Configure if ePWM timer interrupt is enabled at the PIE level:
// 1 = enabled,  0 = disabled
#define PWM1_INT_ENABLE  1


// Configure the period for the timer
#define PWM1_TIMER_TBPRD   0x1388


// Prototype statements for functions found within this file.
interrupt void epwm1_timer_isr(void);
void scia_echoback_init(void);
void scia_fifo_init(void);
void scia_xmit(int a);
void scia_msg(char *msg);

void InitEPwmTimer(void);

// Global variables used in this example
Uint32  EPwm1TimerIntCount;     //counts entries into PWM1 Interrupt
Uint16  LEDcount;       //creates delay for LED3 toggling


int16 sine[200] = { 0, 64, 128, 192, 256, 319, 383, 446, 508,
         570,   631,  692,  752,  812,  870,  928,  985, 1040, 1095, 1149, 1202, 1253,
         1303, 1352, 1399, 1446, 1490, 1533, 1575, 1615, 1654, 1691, 1726, 1760,
         1792, 1822, 1850, 1876, 1901, 1924, 1944, 1963, 1980, 1995, 2008, 2019,
         2028, 2035, 2040, 2043, 2045, 2043, 2040, 2035, 2028, 2019, 2008, 1995,
         1980, 1963, 1944, 1924, 1901, 1876, 1850, 1822, 1792, 1760, 1726, 1691,
         1654, 1615, 1575, 1533, 1490, 1446, 1399, 1352, 1303, 1253, 1202, 1149,
         1095, 1040, 985, 928, 870, 812, 752, 692, 631, 570, 508, 446, 383, 319,
         256, 192, 128, 64, 0,-65,-129,-193,-257,-320,-384,-447,-509,-571,-632,-693,-753
,-813,-871,-929,-986,-1041,-1096,-1150,-1203,-1254,-1304,-1353,-1400,-1447,-1491,-1534,-1576,-1616
,-1655,-1692,-1727,-1761,-1793,-1823,-1851,-1877,-1902,-1925,-1945,-1964,-1981,-1996,-2009,-2020,-2029,-2036
,-2041,-2044,-2045,-2044,-2041,-2036,-2029,-2020,-2009,-1996,-1981,-1964,-1945,-1925,-1902,-1877,-1851,-1823,-1793,-1761
,-1727,-1692,-1655,-1616,-1576,-1534,-1491,-1447,-1400,-1353,-1304,-1254,-1203,-1150,-1096,-1041,-986,-929,-871,-813,-753
,-693,-632,-571,-509,-447,-384,-320,-257,-193,-129,-65};


_iq sine_teste[200];

Uint16 ReceivedChar;
char *msg;

void main(void)
{
   int i;

// Step 1. Initialize System Control:
// PLL, WatchDog, enable Peripheral Clocks
// This example function is found in the DSP2802x_SysCtrl.c file.
   InitSysCtrl();
   InitSciaGpio();

// Step 2. Initalize GPIO:
// This example function is found in the DSP2802x_Gpio.c file and
// illustrates how to set the GPIO to it's default state.
// InitGpio();  // Skipped for this example - LEDs set up in main code
    EALLOW;

//  GPIO-31 - PIN FUNCTION = LED2 on controlCARD
    GpioCtrlRegs.GPAMUX2.bit.GPIO31 = 0;    // 0=GPIO,  1=CANTX-A,  2=Resv,  3=Resv
    GpioCtrlRegs.GPADIR.bit.GPIO31 = 1;     // 1=OUTput,  0=INput
    GpioDataRegs.GPACLEAR.bit.GPIO31 = 1;   // uncomment if --> Set Low initially
//  GpioDataRegs.GPASET.bit.GPIO31 = 1;     // uncomment if --> Set High initially

//  GPIO-34 - PIN FUNCTION = LED3 on controlCARD
    GpioCtrlRegs.GPBMUX1.bit.GPIO34 = 0;    // 0=GPIO,  1=Resv,  2=Resv,  3=Resv
    GpioCtrlRegs.GPBDIR.bit.GPIO34 = 1;     // 1=OUTput,  0=INput
//  GpioDataRegs.GPBCLEAR.bit.GPIO34 = 1;   // uncomment if --> Set Low initially
    GpioDataRegs.GPBSET.bit.GPIO34 = 1;     // uncomment if --> Set High initially

    EDIS;

// Step 3. Clear all interrupts and initialize PIE vector table:
// Disable CPU interrupts
   DINT;

// Initialize the PIE control registers to their default state.
// The default state is all PIE interrupts disabled and flags
// are cleared.
// This function is found in the DSP2802x_PieCtrl.c file.
   InitPieCtrl();

// Disable CPU interrupts and clear all CPU interrupt flags:
   IER = 0x0000;
   IFR = 0x0000;

// Initialize the PIE vector table with pointers to the shell Interrupt
// Service Routines (ISR).
// This will populate the entire table, even if the interrupt
// is not used in this example.  This is useful for debug purposes.
// The shell ISR routines are found in DSP2802x_DefaultIsr.c.
// This function is found in DSP2802x_PieVect.c.
   InitPieVectTable();

// Interrupt that is used in this example is re-mapped to
// ISR function found within this file.
   EALLOW;  // This is needed to write to EALLOW protected registers
   PieVectTable.EPWM1_INT = &epwm1_timer_isr;
   EDIS;    // This is needed to disable write to EALLOW protected registers

// Step 4. Initialize all the Device Peripherals:
// This function is found in DSP2802x_InitPeripherals.c
// InitPeripherals();  // Not required for this example
   InitEPwmTimer();    // For this example, only initialize the ePWM Timer

// Step 5. User specific code, enable interrupts:

// Initalize counters:
   //EPwm1TimerIntCount = 0;

   LEDcount=0;

// Enable CPU INT3 which is connected to EPWM1-6 INT:
   IER |= M_INT3;

// Enable EPWM INTn in the PIE: Group 3 interrupt 1-6
   PieCtrlRegs.PIEIER3.bit.INTx1 = PWM1_INT_ENABLE;

// Initially disable time-critical interrupts
   SetDBGIER(0x0000);   //PIE groups time-critical designation

// Enable global Interrupts and higher priority real-time debug events:
   EINT;   // Enable Global interrupt INTM
   ERTM;   // Enable Global realtime interrupt DBGM

// Step 6. IDLE loop. Just sit and loop forever (optional):

   scia_fifo_init();      // Initialize the SCI FIFO
   scia_echoback_init();  // Initalize SCI for echoback

   msg = "\r\n\n\nHello World!\0";
   scia_msg(msg);

   msg = "\r\nYou will enter a character, and the DSP will echo it back! \n\0";
   scia_msg(msg);

   for(;;)
   {
       asm("          NOP");
       for(i=1;i<=100;i++)
       {
            GpioDataRegs.GPATOGGLE.bit.GPIO31 = 1;  //toggle LED2 on the controlCARD
       }

       if(SciaRegs.SCIFFRX.bit.RXFFST ==1){
       // Get character
       ReceivedChar = SciaRegs.SCIRXBUF.all & 0x00FF;
       //ReceivedChar = ((ReceivedChar & 0x00FF) << 8) | (SciaRegs.SCIRXBUF.all & 0x00FF);
       //SciaRegs.SCIFFRX.bit.RXFFINTCLR = 1;

       // Echo character back
       //msg = "  You sent: \0";
       //scia_msg(msg);
       //scia_xmit(ReceivedChar);

       }
   }
}

void InitEPwmTimer()
{

   EALLOW;
   SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC = 0;      // Stop all the TB clocks
   EDIS;

   // Disable Sync
   EPwm1Regs.TBCTL.bit.SYNCOSEL = 11;  // Pass through

   // Initally disable Free/Soft Bits
   EPwm1Regs.TBCTL.bit.FREE_SOFT = 0;

   EPwm1Regs.TBPRD = PWM1_TIMER_TBPRD;          // Set up PWM1 Period
   EPwm1Regs.TBCTL.bit.CTRMODE = TB_COUNT_UP;    // Count up mode
   EPwm1Regs.ETSEL.bit.INTSEL = ET_CTR_ZERO;     // Select INT on Zero event
   EPwm1Regs.ETSEL.bit.INTEN = PWM1_INT_ENABLE;  // Enable INT
   EPwm1Regs.ETPS.bit.INTPRD = ET_1ST;           // Generate INT on 1st event
   EPwm1Regs.TBCTR = 0x0000;                    // Clear timer counter
   EPwm1Regs.CMPA.half.CMPA = PWM1_TIMER_TBPRD/2;   //CompareA event at half of period
   EPwm1Regs.AQCTLA.all = 0x0024;               // Action-qualifiers, Set on CMPA, Clear on PRD

   EALLOW;
   SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC = 1;         // Start all the timers synced
   EDIS;

}

// Interrupt routines uses in this example:
interrupt void epwm1_timer_isr(void)
{
    _iq sine_ref;
    sine_ref=_IQmpyIQX(_IQ(sine[LEDcount]),20,_IQ30(0.00048899755501222493887530562347188),30);

    sine_teste[LEDcount]=_IQmpy(sine_ref,_IQ(ReceivedChar));

   //EPwm1TimerIntCount++;
   LEDcount++;

   // Clear INT flag for this timer
   EPwm1Regs.ETCLR.bit.INT = 1;

   if (LEDcount==199) {
    GpioDataRegs.GPBTOGGLE.bit.GPIO34 = 1;  //turn on/off LED3 on the controlCARD
    LEDcount=0;
    }


   // Acknowledge this interrupt to receive more interrupts from group 3
   PieCtrlRegs.PIEACK.all = PIEACK_GROUP3;
}

// Test 1,SCIA  DLB, 8-bit word, baud rate 0x000F, default, 1 STOP bit, no parity
void scia_echoback_init()
{
    // Note: Clocks were turned on to the SCIA peripheral
    // in the InitSysCtrl() function

    SciaRegs.SCICCR.all =0x0007;   // 1 stop bit,  No loopback
                                   // No parity,8 char bits,
                                   // async mode, idle-line protocol
    SciaRegs.SCICTL1.all =0x0003;  // enable TX, RX, internal SCICLK,
                                   // Disable RX ERR, SLEEP, TXWAKE
    SciaRegs.SCICTL2.all =0x0003;
    SciaRegs.SCICTL2.bit.TXINTENA =1;
    SciaRegs.SCICTL2.bit.RXBKINTENA =1;

    // SCI BRR = LSPCLK/(SCI BAUDx8) - 1
    #if (CPU_FRQ_60MHZ)
        SciaRegs.SCIHBAUD    =0x0000;  // 9600 baud @LSPCLK = 15MHz (60 MHz SYSCLK).
        SciaRegs.SCILBAUD    =0x00C2;
    #endif

    SciaRegs.SCICTL1.all =0x0023;  // Relinquish SCI from Reset
}

// Transmit a character from the SCI
void scia_xmit(int a)
{
    while (SciaRegs.SCIFFTX.bit.TXFFST != 0) {}
    SciaRegs.SCITXBUF=a;

}

void scia_msg(char * msg)
{
    int i;
    i = 0;
    while(msg[i] != '\0')
    {
        scia_xmit(msg[i]);
        i++;
    }
}

// Initalize the SCI FIFO
void scia_fifo_init()
{
    SciaRegs.SCIFFTX.all=0xE040;
    SciaRegs.SCIFFRX.all=0x2044;
    SciaRegs.SCIFFCT.all=0x0;

}

//===========================================================================
// No more.
//===========================================================================
