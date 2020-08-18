/****************************************************************************
 Module
   AccelSerial.c

 Revision
   1.0

 Description
   This is a module to provide SPI initilization and communication for the
   accelerometer.

****************************************************************************/
/*----------------------------- Include Files -----------------------------*/
#include "serial.h"
#include <stdint.h>
#include "inc/hw_memmap.h"
#include "inc/hw_gpio.h"
#include "inc/hw_sysctl.h"
#include "inc/hw_types.h"
#include "inc/hw_ssi.h"
#include "inc/hw_nvic.h"
#include "bitdefs.h"
#include "termio.h"

#include "ES_Configure.h"
#include "ES_Framework.h"
#include "AccelService.h"


/*----------------------------- Module Defines ----------------------------*/
#define BitsPerNibble 4
#define SSI_PRESCALE 0x04
#define SCR_VALUE 0x1800

#define TWO_COMPLEMENT_CONVERT 0xFC00

#define Y_OFFSET -15 // Empirically determined
/*---------------------------- Module Functions ---------------------------*/


/*---------------------------- Module Variables ---------------------------*/
static int16_t FilteredxAccel;
static int16_t FilteredyAccel;
static int16_t LastyAccel;

/*------------------------------ Module Code ------------------------------*/
/****************************************************************************
 Function
     InitSerial

 Parameters
     None

 Returns
     None
     
 Description
     Initializes SPI 0 and relevant interrupts.

****************************************************************************/
void InitAccelSerial(void)
{
  // Initialize acceleration variables
  FilteredxAccel = 0;
  FilteredyAccel = 0;
  LastyAccel = 0;
  // Enable the clock to the GPIO port
  HWREG(SYSCTL_RCGCGPIO) |= SYSCTL_RCGCGPIO_R5;
  // Enable the clock to the SSI module
  HWREG(SYSCTL_RCGCSSI) |= SYSCTL_RCGCGPIO_R1;
  // Wait for the GPIO port to be ready
  while ((HWREG(SYSCTL_PRGPIO) & SYSCTL_PRGPIO_R5) != SYSCTL_PRGPIO_R5)
  {}
  // Program the GPIO to use the alternate functions on the SSI pins
  HWREG(GPIO_PORTF_BASE + GPIO_O_AFSEL) |= (BIT0HI | BIT1HI | 
    BIT2HI | BIT3HI);
  // Set mux position in GPIOCTL to select the SSI use of the pins
  HWREG(GPIO_PORTF_BASE + GPIO_O_PCTL) |=
      (HWREG(GPIO_PORTF_BASE + GPIO_O_PCTL) & 0xffff0000) +
      ((2 << (0*BitsPerNibble)) + (2 << (1*BitsPerNibble)) +
      (2 << (2*BitsPerNibble)) + (2 << (3*BitsPerNibble)));
  // Program the port lines for digital I/O
  HWREG(GPIO_PORTF_BASE + GPIO_O_DEN) |= (BIT0HI | BIT1HI | BIT2HI | BIT3HI);
  // Program the required data directions on the port lines
  // PF2: clock (out), PF3: SS (out), PF0: MISO (in), PF1: MOSI (out)
  HWREG(GPIO_PORTF_BASE + GPIO_O_DIR) |= (BIT1HI | BIT2HI | BIT3HI);
  HWREG(GPIO_PORTF_BASE + GPIO_O_DIR) &= BIT0LO;
  // Using SPI mode 3, so program the pull-up on the clock line
   HWREG(GPIO_PORTF_BASE + GPIO_O_PUR) |= BIT2HI;
  // Wait for the SSI1 to be ready
  while ((HWREG(SYSCTL_PRSSI) & SYSCTL_PRSSI_R1) != SYSCTL_PRSSI_R1)
  {}
  // Make sure that the SSI is disable before programming mode bits
  HWREG(SSI1_BASE + SSI_O_CR1) &= ~SSI_CR1_SSE;
  // Select master mode (MS) & TXRIS indicating end of transmit
  HWREG(SSI1_BASE + SSI_O_CR1) &= ~SSI_CR1_MS;
  HWREG(SSI1_BASE + SSI_O_CR1) |= SSI_CR1_EOT;
  // Configure the SSI clock source to the system clock
  HWREG(SSI1_BASE + SSI_O_CC) = SSI_CC_CS_SYSPLL;
  // Configure the clock pre-scaler. We choose a rate of 1MHz.
    // Set SCR = 0, so CPSDVSR = 40.
  HWREG(SSI1_BASE + SSI_O_CPSR) &= 0xffffff00;
  HWREG(SSI1_BASE + SSI_O_CPSR) |= SSI_PRESCALE;
  // Configure clock rate (SCR), phase and polarity (SPH, SPO),
  // mode (FRF), data size (DSS)
  // SCR = 0, SPH = 1, SPO = 1
  HWREG(SSI1_BASE + SSI_O_CR0) &= 0xffff00ff;
  HWREG(SSI1_BASE + SSI_O_CR0) |= SCR_VALUE;
  HWREG(SSI1_BASE + SSI_O_CR0) |= (SSI_CR0_SPH | SSI_CR0_SPO);
  HWREG(SSI1_BASE + SSI_O_CR0) &= ~SSI_CR0_FRF_M;
  HWREG(SSI1_BASE + SSI_O_CR0) |= SSI_CR0_DSS_8;
  // Locally enable interrupts (TXIM in SSIIM)
  HWREG(SSI1_BASE + SSI_O_IM) |= (SSI_IM_TXIM);
  // Make sure that the SSI is enable for operation
  HWREG(SSI1_BASE + SSI_O_CR1) |= SSI_CR1_SSE;
  // Enable the NVIC interrupt for the SSI when starting to transmit
  // Interrupt number 34
  HWREG(NVIC_EN1) = BIT2HI;
  printf("SPI1 Configured!\r\n");
}

/****************************************************************************
 Function
     TransmitAccelSerial

 Parameters
     mesage: byte packet to transmit via serial

 Returns
     None

 Description
     Transmits message via SPI

****************************************************************************/
void TransmitAccelSerial(uint8_t message)
{
  // Disable interrupt
  HWREG(SSI1_BASE + SSI_O_IM) &= ~SSI_IM_TXIM;
  // Load message into data register
  HWREG(SSI1_BASE + SSI_O_DR) = message;
  // Re-enable interrupt
  HWREG(SSI1_BASE + SSI_O_IM) |= SSI_IM_TXIM;
}

/****************************************************************************
 Function
    ReceiveISR

 Parameters
   None

 Returns
   message, most recent packet received over serial

 Description
   Returns most recent packet received from SPI

****************************************************************************/
void ReceiveAccelSerial(void)
{
  // Disable interrupt
  HWREG(SSI1_BASE + SSI_O_IM) &= ~SSI_IM_TXIM;
  
  // Read junk value from data register
  uint8_t Ignored_Byte = HWREG(SSI1_BASE + SSI_O_DR);
  
  // Read acceleration values
  uint8_t AX0 = HWREG(SSI1_BASE + SSI_O_DR);
  uint8_t AX1 = HWREG(SSI1_BASE + SSI_O_DR);
  uint8_t AY0 = HWREG(SSI1_BASE + SSI_O_DR);
  uint8_t AY1 = HWREG(SSI1_BASE + SSI_O_DR);
  
  // Process 10bit x acceleration with two's complement
  int16_t xAccel = AX1 << 2*BitsPerNibble;
  xAccel |= AX0;
  if (xAccel & BIT9HI) {
    xAccel |= TWO_COMPLEMENT_CONVERT;
  } else {
    xAccel &= ~(TWO_COMPLEMENT_CONVERT);
  }
  
  // Process 10bit x acceleration with two's complement
  int16_t yAccel = AY1 << 2*BitsPerNibble;
  yAccel |= AY0;
  if (yAccel & BIT9HI) {
    yAccel |= TWO_COMPLEMENT_CONVERT;
  } else {
    yAccel &= ~(TWO_COMPLEMENT_CONVERT);
  }
  LastyAccel = yAccel;
  
  // Simple low pass filter for measuring values
  FilteredxAccel = 0.3*xAccel + 0.7*FilteredxAccel;
  FilteredyAccel = 0.3*yAccel + 0.7*FilteredyAccel;
  
  // Post final received value to SPUD service
  ES_Event_t ThisEvent;
  ThisEvent.EventType = BYTE_RECEIVED;
  PostAccelService(ThisEvent);
}

/****************************************************************************
 Function
    GetxAccel

 Parameters
   None

 Returns
   int16_t: low-pass filtered x acceleration value (in 1/256 g)

 Description
   Returns low-pass filtered x acceleration value (in 1/256 g)

****************************************************************************/
int16_t GetxAccel(void) {
  return FilteredxAccel;
}

/****************************************************************************
 Function
    GetyAccel

 Parameters
   None

 Returns
   int16_t: low-pass filtered y acceleration value (in 1/256 g)

 Description
   Returns low-pass filtered y acceleration value (in 1/256 g)

****************************************************************************/
int16_t GetyAccel(void) {
  return FilteredyAccel - Y_OFFSET;
}

/****************************************************************************
 Function
    GetRawyAccel

 Parameters
   None

 Returns
   int16_t: most recent acceleration value (in 1/256 g)

 Description
   Returns most recent y acceleration value (in 1/256 g)

****************************************************************************/
int16_t GetRawyAccel(void) {
  return LastyAccel - Y_OFFSET;
}
/***************************************************************************
 private functions
 ***************************************************************************/

/*------------------------------- Footnotes -------------------------------*/
/*------------------------------ End of file ------------------------------*/

